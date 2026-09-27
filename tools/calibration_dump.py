#!/usr/bin/env python3
"""Decode STM32F405 ARM-ABI calibration snapshots; never writes to the target."""
import argparse
import hashlib
import json
import math
from pathlib import Path
import struct
import zlib

BASE_SIZE = 412  # robot_settings (64), nm_map_t (324), six uint32 fields
WALL_SIZE, ROTATION_SIZE, CORNER_SIZE = 72, 128, 316


def decode(data):
    if len(data) not in (0x100000, 0x8000):
        raise ValueError('Expected full 1 MiB flash or the 32 KiB persistence area')
    regions = (0x4000, 0x8000) if len(data) == 0x100000 else (0, 0x4000)
    banks = []
    for bank, offset in enumerate(regions):
        h = struct.unpack_from('<8I', data, offset)
        magic, schema, sequence, size, crc, hcrc, committed, _ = h
        valid = (magic == 0x3254535a and committed == 0x434f4d54 and
                 0 < size <= 0x4000-32 and size % 4 == 0 and
                 zlib.crc32(data[offset:offset+20]) == hcrc and
                 zlib.crc32(data[offset+32:offset+32+size]) == crc)
        banks.append(dict(bank='AB'[bank], schema=schema, sequence=sequence,
                          size=size, crc_valid=valid, offset=offset))
    valid = [b for b in banks if b['crc_valid']]
    report = dict(sha256=hashlib.sha256(data).hexdigest(), banks=banks)
    if not valid:
        report['error'] = 'No committed snapshot with valid CRC'; return report
    chosen = valid[0]
    for b in valid[1:]:
        delta = (b['sequence']-chosen['sequence']) & 0xffffffff
        if 0 < delta < 0x80000000: chosen = b
    report['selected_bank'] = chosen['bank']
    if chosen['schema'] not in (3, 4, 5, 6) or chosen['size'] < BASE_SIZE+WALL_SIZE:
        report['error'] = 'Snapshot has no supported calibration payload'; return report
    off = chosen['offset']+32+BASE_SIZE
    def ints(n, signed=False):
        nonlocal off
        v = struct.unpack_from('<'+('i' if signed else 'I')*n, data, off); off += 4*n
        return v
    def geometry(v):
        return dict(zip(('nose_mm', 'width_mm', 'clear_mm', 'pitch_mm'), [x/1000 for x in v]))
    w = ints(18)
    report['wall'] = dict(valid=w[0] == 1, repetitions=w[1], geometry=geometry(w[2:6]),
        front=[dict(zip(('on_mm', 'off_mm', 'spread_mm'), [x/1000 for x in w[i:i+3]])) for i in (6, 9)],
        side=[dict(zip(('near_mm', 'far_mm', 'spread_mm'), [x/1000 for x in w[i:i+3]])) for i in (12, 15)])
    if chosen['schema'] == 3:
        report['assessment'] = assess(report); return report
    if chosen['size'] < BASE_SIZE+WALL_SIZE+ROTATION_SIZE+2*CORNER_SIZE:
        raise ValueError('Truncated calibration bundle')
    r = ints(5)
    rotation = dict(valid=r[0] == 1, geometry=geometry(r[1:]), profiles=[])
    for _ in range(3):
        v = ints(9)
        rotation['profiles'].append(dict(speed=v[0], quarter_mm=[x/1000 for x in v[1:3]],
            effective_track_mm=[x/1000*4/math.pi for x in v[1:3]],
            spread_mm=[x/1000 for x in v[3:5]], error_deg=[x/1000 for x in v[5:7]], checks=list(v[7:9])))
    report['rotation'] = rotation
    report['corners'] = []
    for side in range(2):
        c = ints(7)
        corner = dict(valid=c[0] == 1, fixture=c[1], post_mm=c[2]/1000, geometry=geometry(c[3:]), profiles=[])
        for facing in range(2):
            for _ in range(3):
                speed, mask = ints(2); v = ints(8, True); spread = ints(2)
                corner['profiles'].append(dict(facing_out=facing, physical_side=side^facing,
                    speed=speed, mask=mask, raw_open_mm=[x/1000 for x in v[:2]],
                    raw_close_mm=[x/1000 for x in v[2:4]], open_mm=[x/1000 for x in v[4:6]],
                    close_mm=[x/1000 for x in v[6:]], spread_mm=[x/1000 for x in spread]))
        report['corners'].append(corner)
    if chosen['schema'] == 6:
        if chosen['size'] != 4708:
            raise ValueError('Unexpected battery snapshot size')
        raw, mv = struct.unpack_from('<II', data, chosen['offset']+32+chosen['size']-8)
        report['battery_reference'] = dict(raw=raw, pack_mv=mv)
    report['assessment'] = assess(report)
    return report


def assess(report):
    findings = []
    def note(level, text): findings.append(dict(level=level, message=text))
    wall = report.get('wall', {})
    if wall.get('valid'):
        g = wall['geometry']; centre = g['clear_mm']/2
        if abs(g['nose_mm']-47) > 0.1 or abs(g['width_mm']-94) > 0.1:
            note('warning', 'Geometry differs from the supplied 47/94 mm robot')
        for i,p in enumerate(wall['front']):
            if not g['nose_mm'] <= p['on_mm'] <= p['off_mm'] <= g['clear_mm'] or p['spread_mm']>2:
                note('error', f'Front {i}: invalid threshold/hysteresis/spread')
        for i,p in enumerate(wall['side']):
            if not 0 < p['far_mm']-p['near_mm'] <= 1+p['spread_mm'] or p['spread_mm']>2:
                note('error', f'Side {i}: invalid detection interval')
            note('info', f'Side {i}: centre {centre:.1f} mm; detection bracket {p["near_mm"]:.1f}..{p["far_mm"]:.1f} mm')
    else: note('warning', 'No saved wall calibration')
    r = report.get('rotation', {})
    if r.get('valid'):
        for p in r['profiles']:
            if any(not 47<=q<=87 for q in p['quarter_mm']) or max(p['spread_mm'])>8 or max(p['error_deg'])>2:
                note('error', f'Rotation {p["speed"]}: outside firmware validity bounds')
            if not all(p['checks']): note('warning', f'Rotation {p["speed"]}: no independent 90-degree check in at least one direction')
            tracks=p['effective_track_mm']
            if min(tracks)>0 and abs(tracks[0]-tracks[1])/min(tracks)>0.05:
                note('warning', f'Rotation {p["speed"]}: CW/CCW effective tracks differ by more than 5%')
    else: note('warning', 'No saved rotation calibration')
    for side,c in enumerate(report.get('corners', [])):
        if not c['valid']:
            note('warning', f'No saved corner fixture {side}'); continue
        for p in c['profiles']:
            for channel in range(2):
                if not p['mask'] & (1<<channel): continue
                fields=('raw_open_mm','raw_close_mm','open_mm','close_mm')
                if any(abs(p[k][channel])>60 for k in fields) or p['spread_mm'][channel]>5:
                    note('error', f'Corner {side}, {p["speed"]}: offset/spread outside firmware bounds')
                if p['open_mm'][channel]+0.2<p['raw_open_mm'][channel] or p['close_mm'][channel]-0.2>p['raw_close_mm'][channel]:
                    note('warning', f'Corner {side}, {p["speed"]}: inspect unexpected raw/filtered transition order')
    note('info', 'CRC and numerical checks do not validate physical accuracy or absence of wheel slip')
    return findings


if __name__ == '__main__':
    p = argparse.ArgumentParser(description=__doc__); p.add_argument('dump', type=Path)
    p.add_argument('--output', type=Path); a = p.parse_args()
    report = json.dumps(decode(a.dump.read_bytes()), indent=2)+'\n'
    if a.output: a.output.write_text(report)
    else: print(report, end='')
