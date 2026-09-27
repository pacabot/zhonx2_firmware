#!/usr/bin/env python3
"""Back up / restore ZHONX II calibrations; preserve firmware, maps and settings."""
import argparse
import base64
import hashlib
import json
import math
from pathlib import Path
import struct
import subprocess
import sys
import tempfile
import time
import zlib

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT/'tools'))
from calibration_dump import decode
from robot_guard import identity, guarded_config

SECTOR = 0x4000
ADDRESS = 0x08004000
COMMIT = 0x434f4d54
SIZES = {5: 4700, 6: 4708}
BUNDLE = slice(412, 1244)
GEOMETRY = slice(4680, 4700)


def records(raw):
    if len(raw) != 2*SECTOR:
        raise ValueError('Persistence dump must be exactly 32 KiB')
    result = []
    for bank in range(2):
        offset = bank*SECTOR
        magic, schema, seq, size, crc, hcrc, commit, _ = struct.unpack_from('<8I', raw, offset)
        if (magic == 0x3254535a and commit == COMMIT and 0 < size <= SECTOR-32 and not size % 4
                and zlib.crc32(raw[offset:offset+20]) == hcrc
                and zlib.crc32(raw[offset+32:offset+32+size]) == crc):
            result.append(dict(bank=bank, schema=schema, seq=seq,
                               payload=raw[offset+32:offset+32+size]))
    return result


def newest(items):
    if not items:
        raise ValueError('No committed snapshot with valid CRC; no restoration performed')
    chosen = items[0]
    for item in items[1:]:
        if 0 < ((item['seq']-chosen['seq']) & 0xffffffff) < 0x80000000:
            chosen = item
    return chosen


def selected(raw):
    items = records(raw)
    if any(i['schema'] not in SIZES or len(i['payload']) != SIZES[i['schema']] for i in items):
        raise ValueError('Supported snapshot formats: 5 and 6 only; incompatible bank present')
    # Match firmware load priority: schema 6 first, then schema 5, then sequence.
    schema = max((i['schema'] for i in items), default=0)
    return newest([i for i in items if i['schema'] == schema])


def geometry(v):
    nose, width, inner, pitch = v
    radius = math.isqrt(nose*nose+width*width//4)
    if radius*radius < nose*nose+width*width//4:
        radius += 1
    clearance = ((radius+999)//1000)*1000+2000
    if not (10000 <= nose <= 75000 and 40000 <= width <= 140000 and
            140000 <= inner <= 190000 and inner < pitch <= 210000 and clearance+1000 < inner//2):
        raise ValueError('Invalid calibration geometry')
    return clearance


def validate(payload):
    """Same calibration validity bounds as fw_calibration.c / fw_cal_extra.c."""
    if len(payload) not in SIZES.values():
        raise ValueError('Unsupported payload size')
    wall = struct.unpack_from('<18I', payload, 412)
    rot = struct.unpack_from('<32I', payload, 484)
    corners = [struct.unpack_from('<79I', payload, 612+316*i) for i in range(2)]
    common = None
    for words, geom in [(wall, wall[2:6]), (rot, rot[1:5])] + [(c, c[3:7]) for c in corners]:
        if words[0] not in (0, 1):
            raise ValueError('Invalid calibration validity flag')
        if words[0]:
            geometry(geom)
            if common and common != geom:
                raise ValueError('Inconsistent geometry across calibrations')
            common = geom
    if wall[0]:
        clearance = geometry(wall[2:6])
        if wall[1] != 3:
            raise ValueError('Invalid wall repetition count')
        for i in range(2):
            on, off, spread = wall[6+3*i:9+3*i]
            near, far, side_spread = wall[12+3*i:15+3*i]
            if not (wall[2] <= on <= off <= wall[4] and spread <= 2000 and
                    clearance <= near < far <= wall[4]-clearance and
                    far-near <= 1000+side_spread and side_spread <= 2000):
                raise ValueError('Invalid wall measurement')
    if rot[0]:
        for i, speed in enumerate((40, 80, 120)):
            p = rot[5+9*i:14+9*i]
            if (p[0] != speed or any(not 47000 <= n <= 87000 for n in p[1:3]) or
                    any(n > 8000 for n in p[3:5]) or any(n > 2000 for n in p[5:7]) or
                    any(0 < n < 4 for n in p[7:9])):
                raise ValueError('Invalid rotation measurement')
    for side, c in enumerate(corners):
        if not c[0]:
            continue
        if c[1] != side or not c[5]//2+50000 <= c[2] <= 250000:
            raise ValueError('Invalid corner fixture / post')
        for i in range(6):
            p = c[7+12*i:19+12*i]
            if p[0] != (40, 120, 220)[i % 3] or not p[1] & 2 or p[1] & ~3:
                raise ValueError('Invalid corner profile')
            for sensor in range(2):
                if p[1] & (1 << sensor):
                    signed = [n if n < 0x80000000 else n-0x100000000 for n in p[2+sensor:10:2]]
                    if any(abs(n) > 60000 for n in signed) or p[10+sensor] > 5000:
                        raise ValueError('Invalid corner measurement')
    n, w, inner, pitch, post = struct.unpack_from('<5i', payload, 4680)
    geometry((n*100, w*100, inner*1000, pitch*1000))
    if not 100 <= post <= 250:
        raise ValueError('Invalid configured post position')
    if len(payload) == 4708:
        raw, mv = struct.unpack_from('<2I', payload, 4700)
        if not ((raw == mv == 0) or (256 <= raw <= 4080 and 6000 <= mv <= 8500)):
            raise ValueError('Invalid battery reference')


def archive(raw, entry):
    return dict(format='zhonx2-calibrations-v1', robot='zhonx2', uid=entry['uid'],
                probe=entry['serial'], created_utc=time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()),
                sha256=hashlib.sha256(raw).hexdigest(), persistence_base64=base64.b64encode(raw).decode(),
                report=decode(raw))


def read_archive(path, uid=None):
    obj = json.loads(Path(path).read_text())
    if obj.get('format') != 'zhonx2-calibrations-v1' or obj.get('robot') != 'zhonx2':
        raise ValueError('Not a ZHONX II calibration archive')
    if uid is not None and obj.get('uid') != uid:
        raise ValueError('Archive belongs to another STM32. Restoration refused.')
    raw = base64.b64decode(obj['persistence_base64'], validate=True)
    if len(raw) != 2*SECTOR or hashlib.sha256(raw).hexdigest() != obj.get('sha256'):
        raise ValueError('Corrupt archive (length / SHA-256)')
    return obj, raw


def plan(current, saved):
    source, target = selected(saved), selected(current)
    validate(source['payload']); validate(target['payload'])
    merged = bytearray(target['payload'])
    merged[BUNDLE] = source['payload'][BUNDLE]
    merged[GEOMETRY] = source['payload'][GEOMETRY]
    if source['schema'] == 6:
        reference = source['payload'][4700:4708]
        if target['schema'] == 6:
            merged[4700:4708] = reference
        elif any(reference):
            raise ValueError('Target snapshot v5: save settings with current firmware before restoring battery calibration')
    validate(merged)
    # Preserve the snapshot actually loaded by the firmware, even with mixed schemas.
    bank = 1-target['bank']
    sequence = (newest(records(current))['seq']+1) & 0xffffffff
    prefix = struct.pack('<5I', 0x3254535a, target['schema'], sequence, len(merged), zlib.crc32(merged))
    final = prefix+struct.pack('<3I', zlib.crc32(prefix), COMMIT, 0xffffffff)+merged
    pending = bytearray(final);pending[24:28] = b'\xff'*4
    return bank, bytes(pending), bytes(final)


def quoted(value):
    # Tcl literal, no variable/command substitutions.
    value = str(value)
    if any(c in value for c in '{}\\\r\n'):
        raise ValueError('Unsupported character in tool path')
    return '{'+value+'}'


def write_json(path, value):
    with Path(path).open('x') as out:
        json.dump(value, out, indent=2);out.write('\n')


def prepare(session):
    """Called by OpenOCD while target is halted, before the first erase."""
    entry = json.loads((session/'identity.json').read_text())
    current = (session/'before.bin').read_bytes()
    write_json(session/'before.calibration.json', archive(current, entry))
    _, saved = read_archive(session/'source.json', entry['uid'])
    bank, pending, final = plan(current, saved)
    (session/'pending.bin').write_bytes(pending)
    (session/'final.bin').write_bytes(final)
    (session/'commit.bin').write_bytes(struct.pack('<I', COMMIT))
    (session/'plan.json').write_text(json.dumps(dict(bank=bank)))
    address = ADDRESS+bank*SECTOR
    (session/'program.cfg').write_text(f'''flash erase_sector 0 {bank+1} {bank+1}
flash write_image {quoted(session/'pending.bin')} {address:#x} bin
verify_image {quoted(session/'pending.bin')} {address:#x} bin
flash write_image {quoted(session/'commit.bin')} {address+24:#x} bin
verify_image {quoted(session/'final.bin')} {address:#x} bin
dump_image {quoted(session/'after.bin')} {ADDRESS:#x} {2*SECTOR:#x}
''')


def verify(session):
    before = (session/'before.bin').read_bytes();after = (session/'after.bin').read_bytes()
    bank = json.loads((session/'plan.json').read_text())['bank']
    final = (session/'final.bin').read_bytes()
    other = slice((1-bank)*SECTOR, (2-bank)*SECTOR)
    if (len(after) != len(before) or before[other] != after[other] or
            after[bank*SECTOR:bank*SECTOR+len(final)] != final):
        raise ValueError('Restoration verification failed')
    selected_after = selected(after)
    if selected_after['bank'] != bank:
        raise ValueError('Restored bank would not be loaded by firmware')


def run(action, filename):
    entry = identity('zhonx2')
    if action == 'restore':
        obj, saved = read_archive(filename, entry['uid'])
        validate(selected(saved)['payload'])  # Reject incompatible archive before any target access.
    directory = ROOT/'backups/flash-sessions';directory.mkdir(parents=True, exist_ok=True)
    session = Path(tempfile.mkdtemp(prefix=time.strftime('%Y%m%dT%H%M%SZ-cal-'), dir=directory))
    write_json(session/'identity.json', entry)
    if action == 'restore':
        write_json(session/'source.json', obj)
    else:
        filename = Path(filename) if filename else ROOT/'backups/calibrations'/f'{session.name}.json'
        if filename.exists():
            raise ValueError('Backup already exists; choose a new filename')
        filename.parent.mkdir(parents=True, exist_ok=True)
    worker = f'{quoted(sys.executable)} {quoted(Path(__file__).resolve())}'
    # Refuse a running robot. Test again after halt to close the race window.
    config = guarded_config('zhonx2', entry)+'''if {([lindex [read_memory 0x40020014 32 1] 0] & 0x10c) != 0x100} {error "Stop motors before calibration backup/restore"}
halt
if {[catch {
if {([lindex [read_memory 0x40020014 32 1] 0] & 0x10c) != 0x100} {error "Motors active: operation refused"}
'''
    config += f'dump_image {quoted(session/"before.bin")} {ADDRESS:#x} {2*SECTOR:#x}\n'
    if action == 'restore':
        config += f'exec {worker} _prepare {quoted(session)}\n'
        config += f'script {quoted(session/"program.cfg")}\nexec {worker} _verify {quoted(session)}\n'
    config += '} error_message]} {resume; error $error_message}\n'
    config += 'reset run\n' if action == 'restore' else 'resume\n'
    config += 'shutdown\n'
    (session/'access.cfg').write_text(config)
    print(f'Robot zhonx2 | ST-Link {entry["serial"]} | UID {entry["uid"]}', flush=True)
    print(f'Session: {session}', flush=True)
    with (session/'openocd.log').open('w') as log:
        result = subprocess.run(['openocd', '-f', str(session/'access.cfg')], cwd=ROOT,
                                stdout=log, stderr=subprocess.STDOUT, timeout=180)
    if result.returncode:
        raise ValueError(f'OpenOCD failed; inspect {session}/openocd.log and the captured backup before retrying.')
    if action == 'backup':
        raw = (session/'before.bin').read_bytes()
        write_json(filename, archive(raw, entry))
        print(f'Backup: {filename}')
        print(json.dumps(decode(raw).get('assessment', []), indent=2))
    else:
        verify(session)
        print(f'Calibrations restored. Automatic previous-state backup: {session}/before.calibration.json')


def main():
    if len(sys.argv) == 3 and sys.argv[1] in ('_prepare', '_verify'):
        (prepare if sys.argv[1] == '_prepare' else verify)(Path(sys.argv[2]));return
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='action', required=True)
    sub.add_parser('backup', help='Read 32 KiB; motors must be stopped').add_argument('file', nargs='?')
    sub.add_parser('restore', help='Merge calibrations into one data sector; verify; restart').add_argument('file')
    sub.add_parser('inspect', help='Read archive report offline, no probe needed').add_argument('file')
    args = parser.parse_args()
    if args.action == 'inspect':
        obj, raw = read_archive(args.file)
        print(f'Robot {obj["robot"]} | UID {obj["uid"]}')
        print(json.dumps(decode(raw), indent=2))
    else:
        run(args.action, args.file)


if __name__ == '__main__':
    try:
        main()
    except (ValueError, KeyError, OSError, subprocess.SubprocessError) as error:
        sys.exit(str(error))
