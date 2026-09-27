#!/usr/bin/env bash
# Read-only flash capture and calibration report. Resets into the idle menu afterwards.
set -euo pipefail
cd "$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
case ${1:-} in
    --help|-h) echo 'Usage: scripts/flash/dump.sh [--calibration-only]'; exit 0 ;;
    --calibration-only) address=0x08004000; size=0x8000 ;;
    '') address=0x08000000; size=0x100000 ;;
    *) echo 'Unknown option' >&2; exit 2 ;;
esac
(( $# <= 1 )) || exit 2
export PATH="$PATH:/opt/homebrew/bin:/usr/local/bin"
[[ ${SWD_KHZ:-1000} =~ ^[1-9][0-9]*$ ]] || exit 2
[[ ${STLINK_SERIAL:-} =~ ^[[:alnum:]]*$ ]] || exit 2
mkdir -p backups/flash-sessions
session=$(mktemp -d "backups/flash-sessions/$(date -u +%Y%m%dT%H%M%SZ)-dump-XXXXXX")
python3 - "$session" "$address" "$size" <<'PY'
import os, sys
from pathlib import Path
p, address, size = Path(sys.argv[1]), sys.argv[2], sys.argv[3]
sys.path.insert(0, 'scripts/flash')
from robot_guard import identity, guarded_config
config = guarded_config('zhonx2', identity('zhonx2')) + f'''reset halt
dump_image {p}/flash.bin {address} {size}
reset run
shutdown
'''
(p/'read.cfg').write_text(config)
PY
openocd -f "$session/read.cfg" 2>&1 | tee "$session/read.log"
python3 tools/calibration_dump.py "$session/flash.bin" --output "$session/calibrations.json"
python3 - "$session" <<'PY'
import hashlib,sys
from pathlib import Path
p=Path(sys.argv[1]);h=hashlib.sha256((p/'flash.bin').read_bytes()).hexdigest()
(p/'flash.bin.sha256').write_text(h+'  flash.bin\n')
print('Dump and calibration report: '+str(p))
PY
