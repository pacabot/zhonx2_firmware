#!/usr/bin/env bash
# Flash capture and guarded ADC diagnostics. ADC test temporarily uses injected conversions.
set -euo pipefail
cd "$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
diagnostics=0
adc_test=0
case ${1:-} in
    --help|-h)
        echo 'Usage: scripts/flash/dump.sh [--calibration-only | --diagnostics | --adc-test]'
        echo 'Sans option : dump flash complet ; --calibration-only : 32 Kio de données.'
        echo '--diagnostics : lecture GPIO/ADC/DMA sans reset ni écriture.'
        echo '--adc-test : moteurs arrêtés, suspend le CPU, mesure PA4 et VREFINT'
        echo 'par conversions injectées sans DMA, restaure le séquenceur et reprend le CPU.'
        echo 'Aucune écriture en flash ; contrôle sonde/UID dans tous les modes.'
        exit 0 ;;
    --adc-test) diagnostics=1; adc_test=1; address=0; size=0 ;;
    --diagnostics) diagnostics=1; address=0; size=0 ;;
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
python3 - "$session" "$address" "$size" "$adc_test" <<'PY'
import os, sys
from pathlib import Path
p, address, size = Path(sys.argv[1]), sys.argv[2], sys.argv[3]
sys.path.insert(0, 'scripts/flash')
from robot_guard import identity, guarded_config
config = guarded_config('zhonx2', identity('zhonx2'))
adc_test = sys.argv[4] == '1'
if address == '0':
    config += r'''echo "GPIOA_MODER [read_memory 0x40020000 32 1]"
echo "GPIOA_PUPDR [read_memory 0x4002000c 32 1]"
echo "ADC1_SR_CR1_CR2 [read_memory 0x40012000 32 3]"
echo "ADC1_SMPR1_SMPR2 [read_memory 0x4001200c 32 2]"
echo "ADC1_SQR1_SQR2_SQR3 [read_memory 0x4001202c 32 3]"
echo "ADC_COMMON_CCR [read_memory 0x40012304 32 1]"
echo "DMA2_STREAM0 [read_memory 0x40026410 32 6]"
set adc_buffer [lindex [read_memory 0x4002641c 32 1] 0]
if {$adc_buffer >= 0x20000000 && $adc_buffer <= 0x2001fffc} {
    for {set sample 0} {$sample < 4} {incr sample} {
        echo "ADC_SAMPLES [read_memory $adc_buffer 16 2]"
        sleep 250
    }
}
echo "VREFINT_CAL [read_memory 0x1fff7a2a 16 1]"
'''
    if adc_test:
        # Temporary injected conversions bypass DMA. No GPIO or flash writes.
        # Require an idle robot and restore the injected sequencer before resume.
        config += r'''if {([lindex [read_memory 0x40020014 32 1] 0] & 0x10c) != 0x100} {error "Stop motors before ADC test"}
halt
if {([lindex [read_memory 0x40020014 32 1] 0] & 0x10c) != 0x100} {resume; error "Motors active"}
set adc_cr1 [lindex [read_memory 0x40012004 32 1] 0]
set adc_cr2 [lindex [read_memory 0x40012008 32 1] 0]
set adc_jsqr [lindex [read_memory 0x40012038 32 1] 0]
if {($adc_cr1 & 0x00401480) != 0 || ($adc_cr2 & 0x00300001) != 1 || $adc_jsqr != 0} {resume; error "Injected ADC already in use or ADC disabled"}
set failed [catch {
    foreach channel {4 17 4 17 4 17} {
        write_memory 0x40012038 32 [list [expr {$channel << 15}]]
        write_memory 0x40012000 32 {0xfffffffb}
        write_memory 0x40012008 32 [list [expr {$adc_cr2 | 0x00400000}]]
        sleep 2
        if {([lindex [read_memory 0x40012000 32 1] 0] & 4) == 0} {error "Injected ADC timeout"}
        echo "ADC_INDEPENDENT channel=$channel value=[read_memory 0x4001203c 32 1]"
    }
} diagnostic_error]
write_memory 0x40012038 32 [list $adc_jsqr]
write_memory 0x40012000 32 {0xfffffff3}
resume
if {$failed} {error $diagnostic_error}
'''
    config += 'shutdown\n'
else:
    config += f'''reset halt
dump_image {p}/flash.bin {address} {size}
reset run
shutdown
'''
(p/'read.cfg').write_text(config)
PY
openocd -f "$session/read.cfg" 2>&1 | tee "$session/read.log"
if (( diagnostics )); then echo "Diagnostic terminé, sans écriture en flash : $session/read.log"; exit 0; fi
python3 tools/calibration_dump.py "$session/flash.bin" --output "$session/calibrations.json"
python3 - "$session" <<'PY'
import hashlib,sys
from pathlib import Path
p=Path(sys.argv[1]);h=hashlib.sha256((p/'flash.bin').read_bytes()).hexdigest()
(p/'flash.bin.sha256').write_text(h+'  flash.bin\n')
print('Dump and calibration report: '+str(p))
PY
