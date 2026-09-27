#!/usr/bin/env python3
"""Shared robot/probe registry. No automatic selection or association.

Registry is per workstation; the ZHONX_III workspace may have its own guard.
UID words: STM32F405 RM0090, 96-bit unique device ID at 0x1FFF7A10.
"""
import argparse
import fcntl
import json
import os
from pathlib import Path
import re
import subprocess
import sys
import tempfile

ROBOTS = ('zhonx2', 'zhonx3')


def registry_path():
    return Path(os.environ.get('XDG_CONFIG_HOME', Path.home()/'.config'))/'pacabot/robots.json'


def hex_id(value, label):
    if not isinstance(value, str) or not re.fullmatch(r'[0-9A-Fa-f]{24}', value):
        raise ValueError(f'{label} : 24 chiffres hexadécimaux requis')
    value = value.upper()
    if value in ('0'*24, 'F'*24):
        raise ValueError(f'{label} invalide')
    return value


def validate_registry(data):
    if not isinstance(data, dict) or data.get('schema') != 1 or not isinstance(data.get('robots'), dict):
        raise ValueError('Registre robots invalide')
    serials, uids = set(), set()
    for robot, entry in data['robots'].items():
        if robot not in ROBOTS or not isinstance(entry, dict):
            raise ValueError('Robot inconnu dans le registre')
        serial = hex_id(entry.get('serial'), 'ST-Link')
        uid = hex_id(entry.get('uid'), 'UID STM32')
        if serial in serials or uid in uids:
            raise ValueError('Association ambiguë : sonde ou STM32 attribué à deux robots')
        serials.add(serial); uids.add(uid)
        entry.update(serial=serial, uid=uid)
    return data


def load_registry():
    path = registry_path()
    if not path.exists():
        return {'schema': 1, 'robots': {}}
    return validate_registry(json.loads(path.read_text()))


def identity(robot):
    entry = load_registry()['robots'].get(robot)
    if not entry:
        raise ValueError(f'{robot} non associé : utiliser robot_guard.py associate {robot} --serial NUMERO '
                         'après identification physique du robot. Aucun accès à la cible.')
    override = os.environ.get('STLINK_SERIAL', '')
    if override and hex_id(override, 'STLINK_SERIAL') != entry['serial']:
        raise ValueError(f'STLINK_SERIAL ne correspond pas à {robot}. Flash refusé.')
    return dict(entry)


def base_config(serial):
    serial = hex_id(serial, 'ST-Link')
    speed = os.environ.get('SWD_KHZ', '1000')
    if not re.fullmatch(r'[1-9][0-9]*', speed):
        raise ValueError('SWD_KHZ invalide')
    return f'''source [find interface/stlink.cfg]
adapter serial {serial}
transport select hla_swd
source [find target/stm32f4x.cfg]
adapter speed {speed}
stm32f4x.cpu configure -event reset-start {{adapter speed {speed}}}
gdb_port disabled
tcl_port disabled
telnet_port disabled
init
if {{([lindex [read_memory 0xe0042000 32 1] 0] & 0xfff) != 0x413}} {{ error "MCU inattendu" }}
if {{[lindex [read_memory 0x1fff7a22 16 1] 0] != 1024}} {{ error "Flash attendue : 1 Mio" }}
'''


def guarded_config(robot, entry):
    if robot not in ROBOTS:
        raise ValueError('Robot inconnu')
    uid = hex_id(entry['uid'], 'UID STM32')
    config = base_config(entry['serial'])
    config += 'set robot_uid [read_memory 0x1fff7a10 32 3]\n'
    for i in range(3):
        config += (f'if {{[lindex $robot_uid {i}] != 0x{uid[i*8:i*8+8]}}} '
                   f'{{ error "Mauvais robot : UID incompatible avec {robot}. Aucune ecriture." }}\n')
    return config + f'echo ROBOT_IDENTITY_OK:{robot}:{uid}\n'


def inspect_probe(serial):
    """Only debug identification reads: no erase, write, halt or reset command."""
    config = base_config(serial) + '''set uid [read_memory 0x1fff7a10 32 3]
echo [format "ROBOT_UID:%08X%08X%08X" [lindex $uid 0] [lindex $uid 1] [lindex $uid 2]]
shutdown
'''
    with tempfile.TemporaryDirectory(prefix='pacabot-identify-') as directory:
        path = Path(directory)/'identify.cfg'
        path.write_text(config)
        result = subprocess.run(['openocd', '-f', str(path)], text=True, capture_output=True, timeout=30)
    output = result.stdout+'\n'+result.stderr
    matches = re.findall(r'^ROBOT_UID:([0-9A-Fa-f]{24})\s*$', output, re.M)
    if result.returncode or len(matches) != 1:
        raise ValueError('Lecture identité impossible.\n'+output[-4000:])
    return hex_id(matches[0], 'UID STM32')


def associate(robot, serial):
    serial = hex_id(serial, 'ST-Link')
    path = registry_path(); path.parent.mkdir(parents=True, exist_ok=True)
    with (path.parent/'robots.lock').open('a') as lock:
        fcntl.flock(lock, fcntl.LOCK_EX)
        data = load_registry()
        for name, entry in data['robots'].items():
            if name != robot and entry['serial'] == serial:
                raise ValueError(f'Cette sonde est déjà associée à {name}')
        old = data['robots'].get(robot)
        if old and old['serial'] != serial:
            raise ValueError('Association déjà enregistrée : modification manuelle du registre requise')
        uid = inspect_probe(serial)
        entry = {'serial': serial, 'uid': uid}
        if old and old != entry:
            raise ValueError('Le STM32 a changé derrière cette sonde. Association conservée, opération refusée.')
        data['robots'][robot] = entry
        validate_registry(data)
        with tempfile.NamedTemporaryFile(mode='w', dir=path.parent, delete=False) as out:
            json.dump(data, out, indent=2); out.write('\n'); temp = out.name
        os.replace(temp, path)
    return entry


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest='action', required=True)
    sub.add_parser('show', help='Afficher les associations enregistrées, sans accès matériel')
    for action in ('check', 'verify', 'config', 'associate'):
        child = sub.add_parser(action)
        child.add_argument('robot', choices=ROBOTS)
        if action == 'associate':
            child.add_argument('--serial', required=True,
                               help='Sonde dont le raccordement physique au robot a été confirmé')
    args = parser.parse_args()
    if args.action == 'show':
        print(json.dumps(load_registry(), indent=2)); return
    entry = associate(args.robot, args.serial) if args.action == 'associate' else identity(args.robot)
    print(f"Robot {args.robot} | ST-Link {entry['serial']} | UID {entry['uid']}", file=sys.stderr)
    if args.action == 'config':
        print(guarded_config(args.robot, entry), end='')
    elif args.action == 'verify':
        with tempfile.TemporaryDirectory(prefix='pacabot-check-') as directory:
            path = Path(directory)/'check.cfg'
            path.write_text(guarded_config(args.robot, entry)+'shutdown\n')
            subprocess.run(['openocd', '-f', str(path)], check=True, timeout=30)


if __name__ == '__main__':
    try:
        main()
    except (ValueError, OSError, subprocess.SubprocessError) as error:
        sys.exit(str(error))
