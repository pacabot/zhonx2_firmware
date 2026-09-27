"""Check whole-flash comparison, including variable-size erase boundaries."""
import importlib.util
from pathlib import Path
import tempfile
import struct
import json
import subprocess
import sys
import unittest

spec = importlib.util.spec_from_file_location(
    "flash_session", Path(__file__).resolve().parents[1] / "scripts/flash/flash_session.py")
flash = importlib.util.module_from_spec(spec)
spec.loader.exec_module(flash)


class FlashSessionTest(unittest.TestCase):
    def test_boot_vectors_reject_monolithic_and_damaged_images(self):
        vectors = [0x20020000] + [0x08000045] * 15
        for i in (7, 8, 9, 10, 13):
            vectors[i] = 0
        self.assertTrue(flash.check_boot_vectors(struct.pack('<16I', *vectors))[
            'core_vectors_in_boot_sector'])
        for index, value in ((0, 0x2001ffff), (1, 0x08018959),
                             (1, 0xffffffff), (1, 0x08000044), (3, 0x0800d1b9)):
            with self.subTest(index=index, value=value):
                bad = vectors.copy()
                bad[index] = value
                with self.assertRaisesRegex(ValueError, '--all'):
                    flash.check_boot_vectors(struct.pack('<16I', *bad))

    def test_backup_rejects_app_update_but_keeps_recovery_dump(self):
        for fast in (False, True):
            with self.subTest(fast=fast), tempfile.TemporaryDirectory() as directory:
                p = Path(directory)
                size = 0x40000 if fast else flash.FLASH_SIZE
                dump = struct.pack('<II', 0x2001ffff, 0x08018959) + b'\xff' * (size-8)
                (p/'pre-flash.bin').write_bytes(dump)
                (p/'report.json').write_text(json.dumps(dict(mode='app', backup_size=size)))
                result = subprocess.run([sys.executable, str(spec.origin), 'backup', str(p)],
                                        capture_output=True, text=True)
                self.assertNotEqual(result.returncode, 0)
                self.assertIn('--all', result.stderr)
                report = json.loads((p/'report.json').read_text())
                self.assertEqual(report['status'], 'preflight_rejected')
                self.assertEqual(report['pre_flash_sha256'], flash.digest(dump))
                self.assertEqual((p/'pre-flash.bin').read_bytes(), dump)
                self.assertTrue((p/'pre-flash.bin.sha256').exists())

    def test_selection(self):
        self.assertEqual([name for name, _ in flash.images_for('app')],
                         ['image-manifest.bin', 'application.ota.bin'])
        self.assertEqual(flash.images_for('boot'), [('bootloader.bin', 0)])

    def test_app_and_boot_preserve_other_image(self):
        for mode in ('app', 'boot'):
            with self.subTest(mode=mode), tempfile.TemporaryDirectory() as directory:
                session = Path(directory)
                before = b'\xa5' * flash.FLASH_SIZE
                (session / 'pre-flash.bin').write_bytes(before)
                expected = bytearray(before)
                if mode == 'app':
                    expected[0xC000:0x20000] = b'\xff' * 0x14000
                else:
                    expected[:0x4000] = b'\xff' * 0x4000
                for name, offset in flash.images_for(mode):
                    data = b'\x42' * (32 if name == 'image-manifest.bin' else 100)
                    (session / name).write_bytes(data)
                    expected[offset:offset + len(data)] = data
                (session / 'post-flash.bin').write_bytes(expected)
                self.assertTrue(flash.verify(session, mode)['untouched_flash_verified'])
                other = 0 if mode == 'app' else 0xC000
                expected[other] ^= 1
                (session / 'post-flash.bin').write_bytes(expected)
                with self.assertRaises(ValueError):
                    flash.verify(session, mode)

    def test_readback_and_preserved_sectors(self):
        for length, erase_end in [(0xD6D4, 0x20000), (0x10004, 0x40000),
                                  (0x60000, 0x80000)]:
            with self.subTest(length=length), tempfile.TemporaryDirectory() as directory:
                session = Path(directory)
                before = b'\xa5' * flash.FLASH_SIZE
                (session / 'pre-flash.bin').write_bytes(before)
                (session / 'bootloader.bin').write_bytes(b'\x12' * 3632)
                (session / 'image-manifest.bin').write_bytes(b'\x34' * 32)
                (session / 'application.ota.bin').write_bytes(b'\x56' * length)
                expected = bytearray(before)
                expected[:0x4000] = b'\xff' * 0x4000
                expected[0xC000:erase_end] = b'\xff' * (erase_end - 0xC000)
                for name, offset in flash.images_for('all'):
                    data = (session / name).read_bytes()
                    expected[offset:offset + len(data)] = data
                (session / 'post-flash.bin').write_bytes(expected)
                self.assertTrue(flash.verify(session)['untouched_flash_verified'])
                # A changed byte in an image, persistence, stage, reserved or
                # erased padding must all reject the readback.
                for offset in (0, 0x4000, 0x8000, 0x10000, 0x80000, 0xE0000, 0x3FFF):
                    corrupted = expected.copy()
                    corrupted[offset] ^= 1
                    (session / 'post-flash.bin').write_bytes(corrupted)
                    with self.assertRaises(ValueError):
                        flash.verify(session)
                (session / 'post-flash.bin').write_bytes(expected[:-1])
                with self.assertRaises(ValueError):
                    flash.verify(session)


class FastFlashTest(unittest.TestCase):
    def test_fast_checks_persistence_images_and_completion(self):
        with tempfile.TemporaryDirectory() as directory:
            p = Path(directory)
            before = bytes(range(256))*1024
            (p/'pre-flash.bin').write_bytes(before)
            (p/'post-calibration.bin').write_bytes(before[0x4000:0xC000])
            report = dict(mode='app', backup_size=len(before), images={})
            markers = []
            for name, _ in flash.images_for('app'):
                (p/name).write_bytes(b'firmware')
                report['images'][name] = dict(sha256=flash.digest(b'firmware'))
                markers.append('IMAGE_VERIFIED:'+name)
            (p/'program.log').write_text('\n'.join(markers)+'\n')
            result = flash.verify_fast(p, report)
            self.assertTrue(result['persistence_verified'])
            self.assertFalse(result['untouched_flash_verified'])
            (p/'program.log').write_text(markers[0]+'\n')
            with self.assertRaises(ValueError): flash.verify_fast(p, report)
            (p/'program.log').write_text('\n'.join(markers)+'\n')
            bad = bytearray(before[0x4000:0xC000]); bad[17] ^= 1
            (p/'post-calibration.bin').write_bytes(bad)
            with self.assertRaises(ValueError): flash.verify_fast(p, report)


class OptionalVerificationTest(unittest.TestCase):
    @staticmethod
    def prepare_fixture(root, session, mode, fast=True, verifying=False):
        from unittest.mock import patch
        build = root/'build'/'release'
        build.mkdir(parents=True, exist_ok=True)
        app = struct.pack('<II', 0x20010000, 0x08010009) + b'\x42' * (0x11000-8)
        boot = struct.pack('<II', 0x20020000, 0x08000009) + b'\x42' * 120
        for name, data in [('application.ota.bin', app), ('image-manifest.bin', flash.manifest(app)),
                           ('bootloader.bin', boot), ('ZHONX_II_M4.elf', b'test ELF')]:
            (build/name).write_bytes(data)
        symbols = b'20000100 B os_context\n20000200 B scan\n20000300 B fault\n20000400 B active\n'
        with patch.object(flash, 'identity', return_value={'serial':'51FF66064982565324552187', 'uid':'001100223344556677889900'}), \
                patch.object(flash, 'ROOT', root), patch.object(flash.subprocess, 'check_output', return_value=symbols.decode()):
            return flash.prepare(session, mode, 'release', fast=fast, verify_images=verifying)

    def test_generated_commands_for_all_modes(self):
        for mode in ('app', 'boot', 'all'):
            for fast in (False, True):
                for verifying in (False, True):
                    with self.subTest(mode=mode, fast=fast, verifying=verifying), tempfile.TemporaryDirectory() as directory:
                        root = Path(directory); session = root/'session'; session.mkdir()
                        report = self.prepare_fixture(root, session, mode, fast, verifying)
                        self.assertEqual(report['backup_size'], 0xC000 if fast else flash.FLASH_SIZE)
                        self.assertEqual(report['verification_requested'], verifying)
                        self.assertFalse(report['flash_verified'])
                        self.assertEqual(report['robot'], 'zhonx2')
                        for name in ('backup.cfg', 'program.cfg', 'startup.cfg'):
                            guarded = (session/name).read_text()
                            self.assertIn('adapter serial 51FF66064982565324552187', guarded)
                            self.assertIn('0x1fff7a10 32 3', guarded)
                            self.assertLess(guarded.index('ROBOT_IDENTITY_OK:'), guarded.index('reset halt'))
                        cfg = (session/'program.cfg').read_text()
                        self.assertLess(cfg.index('ROBOT_IDENTITY_OK:'), cfg.index('flash write_image erase'))
                        self.assertEqual('verify_image ' in cfg, verifying and fast)
                        self.assertEqual('dump_image ' in cfg, verifying)
                        self.assertEqual(cfg.count('flash write_image erase '), len(flash.images_for(mode)))
                        self.assertNotIn('mass_erase', cfg)
                        for start,end in flash.erase_ranges(session, mode):
                            self.assertFalse(start<0xC000 and end>0x4000)
                        self.assertIn('CFSR_HFSR', (session/'startup.cfg').read_text())
                        if verifying:
                            self.assertIn('post-calibration.bin' if fast else 'post-flash.bin', cfg)

    def test_unverified_success_needs_all_writes_and_does_not_claim_verification(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory); p=root/'session';p.mkdir()
            report=self.prepare_fixture(root,p,'all')
            data=b'\xa5'*report['backup_size'];(p/'pre-flash.bin').write_bytes(data)
            report.update(status='backup_verified', pre_flash_sha256=flash.digest(data))
            markers=['IMAGE_WRITTEN:'+name for name,_ in flash.images_for('all')]
            (p/'program.log').write_text('\n'.join(markers)+'\n')
            result=flash.programmed(p,report)
            self.assertEqual(result['status'], 'flash_written')
            self.assertFalse(result['flash_verified'])
            self.assertFalse(result['persistence_verified'])
            self.assertFalse((p/'post-flash.bin').exists())
            (p/'program.log').write_text('\n'.join(markers[:-1])+'\n')
            with self.assertRaisesRegex(ValueError, 'incomplète'): flash.programmed(p,report)
            (p/'program.log').write_text('\n'.join(markers)+'\n')
            (p/'bootloader.bin').write_bytes(b'changed')
            with self.assertRaisesRegex(ValueError, 'Image modifiée'): flash.programmed(p,report)

    def test_verify_checks_written_images_and_persistent_bytes(self):
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory);p=root/'session';p.mkdir()
            report=self.prepare_fixture(root,p,'app',verifying=True)
            data=b'\xa5'*report['backup_size'];(p/'pre-flash.bin').write_bytes(data)
            report.update(status='backup_verified', pre_flash_sha256=flash.digest(data))
            markers=[prefix+name for name,_ in flash.images_for('app')
                     for prefix in ('IMAGE_WRITTEN:','IMAGE_VERIFIED:')]
            (p/'program.log').write_text('\n'.join(markers)+'\n')
            (p/'post-calibration.bin').write_bytes(data[0x4000:0xC000])
            result=flash.programmed(p,report)
            self.assertTrue(result['flash_verified'] and result['persistence_verified'])
            corrupted=bytearray(data[0x4000:0xC000]);corrupted[0]^=1
            (p/'post-calibration.bin').write_bytes(corrupted)
            with self.assertRaisesRegex(ValueError, 'persistantes modifiées'): flash.programmed(p,report)
