import importlib.util
import json
from pathlib import Path
import struct
import sys
import tempfile
import unittest

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT/'scripts/flash'))
spec = importlib.util.spec_from_file_location('calibration_backup', ROOT/'scripts/flash/calibration.py')
cal = importlib.util.module_from_spec(spec);spec.loader.exec_module(cal)
from test_calibration_dump import bank

UID = '003300373432471234373230'
ENTRY = dict(uid=UID, serial='51FF66064982565324552187')


def payload(schema=6, right=False):
    p = bytearray(cal.SIZES[schema])
    struct.pack_into('<18I', p, 412, 1, 3, 47000, 94000, 167000, 179000,
                     91000, 92000, 400, 131000, 132000, 200, 84500, 85500, 0, 81500, 83500, 1000)
    struct.pack_into('<5I', p, 484, 1, 47000, 94000, 167000, 179000)
    for i, speed in enumerate((40, 80, 120)):
        struct.pack_into('<9I', p, 504+36*i, speed, 65000, 65500, 500, 500, 300, 400, 6, 6)
    for side in range(2 if right else 1):
        offset = 612+316*side
        struct.pack_into('<7I', p, offset, 1, side, 173000, 47000, 94000, 167000, 179000)
        for i in range(6):
            struct.pack_into('<2I8i2I', p, offset+28+i*48, (40, 120, 220)[i % 3], 3,
                             -12000, 15000, -11000, 14000, -11500, 15500, -11500, 13500, 1000, 1500)
    struct.pack_into('<5i', p, 4680, 470, 940, 167, 179, 173)
    if schema == 6:
        struct.pack_into('<2I', p, 4700, 3000, 8400)
    return p


def flash(a, b=None):
    raw = bytearray(b'\xff'*0x8000)
    raw[:len(a)] = a
    if b:
        raw[0x4000:0x4000+len(b)] = b
    return bytes(raw)


class CalibrationBackupTest(unittest.TestCase):
    def test_restore_only_calibrations_and_keep_missing_right(self):
        source = payload();target = payload(right=True)
        target[:412] = bytes([0x55])*412  # Distinct robot settings / active maze.
        target[1244:4680] = bytes([0xaa])*3436  # Distinct maze library / learned flag.
        current = flash(bank(6, 9, target), bank(6, 8, target))
        chosen, pending, final = cal.plan(current, flash(bank(6, 2, source)))
        self.assertEqual(chosen, 1)
        merged = final[32:]
        self.assertEqual(merged[:412], target[:412])
        self.assertEqual(merged[1244:4680], target[1244:4680])
        self.assertEqual(merged[cal.BUNDLE], source[cal.BUNDLE])
        self.assertEqual(struct.unpack_from('<I', merged, 928)[0], 0)
        self.assertEqual(pending[24:28], b'\xff'*4)
        self.assertEqual(final[24:28], struct.pack('<I', cal.COMMIT))

    def test_power_loss_before_commit_preserves_previous_snapshot(self):
        old = payload(right=True);current = flash(bank(6, 20, old), bank(6, 19, old))
        chosen, pending, final = cal.plan(current, flash(bank(6, 1, payload())))
        for written in (0, 24, 28, 32, 400, len(pending)):
            torn = bytearray(current);torn[0x4000:] = b'\xff'*0x4000
            torn[0x4000:0x4000+written] = pending[:written]
            self.assertEqual(cal.selected(torn)['payload'], old)
            self.assertEqual(cal.selected(torn)['bank'], 0)
        done = bytearray(torn);done[0x4000:0x4000+len(final)] = final
        self.assertEqual(cal.selected(done)['bank'], chosen)

    def test_sequence_wrap_and_firmware_schema_priority(self):
        current = flash(bank(6, 0xffffffff, payload()), bank(5, 0, payload(5)))
        self.assertEqual(cal.selected(current)['bank'], 0)
        self.assertEqual(cal.decode(current)['selected_bank'], 'A')
        chosen, _, final = cal.plan(current, flash(bank(6, 1, payload())))
        self.assertEqual(chosen, 1)
        self.assertEqual(struct.unpack_from('<I', final, 8)[0], 1)

    def test_v5_roundtrip_and_v5_to_v6_preserve_battery_reference(self):
        source = flash(bank(5, 1, payload(5)))
        for schema in (5, 6):
            _, _, final = cal.plan(flash(bank(schema, 1, payload(schema))), source)
            self.assertEqual(len(final), 32+cal.SIZES[schema])
            if schema == 6:
                self.assertEqual(final[-8:], struct.pack('<2I', 3000, 8400))
        with self.assertRaisesRegex(ValueError, 'Target snapshot v5'):
            cal.plan(source, flash(bank(6, 1, payload())))

    def test_reject_bad_crc_or_unsupported_schema(self):
        good = flash(bank(6, 1, payload()))
        bad = bytearray(good);bad[100] ^= 1
        for image in (bad, flash(bank(7, 1, payload()))):
            with self.assertRaises(ValueError):
                cal.plan(good, image)

    def test_reject_invalid_measurements_even_with_valid_crc(self):
        good = flash(bank(6, 1, payload()))
        for offset, value in ((412, 2), (444, 100000), (504, 41), (612+4, 1),
                              (612+28+4, 1), (612+28+8, 70000), (4700, 4095)):
            broken = payload();struct.pack_into('<I', broken, offset, value)
            with self.assertRaises(ValueError, msg=str(offset)):
                cal.plan(good, flash(bank(6, 1, broken)))

    def test_uid_and_archive_hash_guard(self):
        raw = flash(bank(6, 1, payload()))
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory)/'archive.json';obj = cal.archive(raw, ENTRY)
            cal.write_json(path, obj)
            self.assertEqual(cal.read_archive(path, UID)[1], raw)
            with self.assertRaisesRegex(ValueError, 'another STM32'):
                cal.read_archive(path, '1'*24)
            with self.assertRaises(FileExistsError):
                cal.write_json(path, obj)
            obj['sha256'] = '0'*64;path.write_text(json.dumps(obj))
            with self.assertRaisesRegex(ValueError, 'Corrupt'):
                cal.read_archive(path)

    def test_transaction_sector_bounds_backup_and_verification(self):
        with tempfile.TemporaryDirectory() as directory:
            session = Path(directory)
            current = flash(bank(6, 1, payload(right=True)))
            (session/'identity.json').write_text(json.dumps(ENTRY))
            (session/'before.bin').write_bytes(current)
            cal.write_json(session/'source.json', cal.archive(flash(bank(6, 1, payload())), ENTRY))
            cal.prepare(session)
            self.assertEqual(cal.read_archive(session/'before.calibration.json', UID)[1], current)
            config = (session/'program.cfg').read_text()
            self.assertIn('flash erase_sector 0 2 2', config)
            self.assertNotIn('write_image erase', config)
            self.assertNotIn('0x08010000', config)
            self.assertLess(config.index('verify_image'), config.index('commit.bin'))
            final = (session/'final.bin').read_bytes()
            after = bytearray(current);after[0x4000:] = b'\xff'*0x4000
            after[0x4000:0x4000+len(final)] = final
            (session/'after.bin').write_bytes(after);cal.verify(session)
            after[128] ^= 1;(session/'after.bin').write_bytes(after)
            with self.assertRaisesRegex(ValueError, 'verification failed'):
                cal.verify(session)
