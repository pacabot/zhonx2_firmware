#!/usr/bin/env bash
# Build and flash through SWD, including older ST-Link/V2 probes (HLA).
set -euo pipefail
cd "$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"

usage() {
    cat <<'EOF'
Usage : .scripts/build/build.sh [--build-only] [--test] [--clean]
Sans option : compile, sauvegarde la Flash, programme et vérifie le démarrage.
  --build-only  Compile sans accéder au robot.
  --test        Exécute aussi les tests logiciels avant programmation.
  --clean       Recompile entièrement (les sauvegardes restent conservées).
Variables : JOBS=4, SWD_KHZ=1000, STLINK_SERIAL=<numéro optionnel>.
Prérequis : GNU Arm Embedded, make, Python 3 et OpenOCD pour flasher.
Le robot doit être alimenté et relié à la sonde ST-Link.
EOF
}
build_only=0
run_tests=0
clean=0
for arg in "$@"; do
    case "$arg" in
        --build-only) build_only=1 ;;
        --test) run_tests=1 ;;
        --clean) clean=1 ;;
        -h|--help) usage; exit 0 ;;
        *) printf 'Option inconnue : %s\n' "$arg" >&2; usage >&2; exit 2 ;;
    esac
done
# Standard macOS installations; the caller's PATH keeps priority.
export PATH="$PATH:/usr/local/arm-gnu-toolchain/bin:/opt/homebrew/bin:/usr/local/bin"
for command in make python3 arm-none-eabi-gcc arm-none-eabi-objcopy arm-none-eabi-size; do
    command -v "$command" >/dev/null || { echo "Outil manquant : $command" >&2; exit 1; }
done
[[ ${JOBS:-4} =~ ^[1-9][0-9]*$ ]] || { echo 'JOBS doit être positif.' >&2; exit 2; }
if (( ! build_only )); then
    command -v openocd >/dev/null || { echo 'Outil manquant : openocd' >&2; exit 1; }
    command -v arm-none-eabi-nm >/dev/null || exit 1
    [[ ${SWD_KHZ:-1000} =~ ^[1-9][0-9]*$ ]] || { echo 'SWD_KHZ doit être positif.' >&2; exit 2; }
    [[ ${STLINK_SERIAL:-} =~ ^[[:alnum:]]*$ ]] || { echo 'Numéro ST-Link invalide.' >&2; exit 2; }
fi
if (( clean )); then make clean; fi
make -j"${JOBS:-4}"
if (( run_tests )); then make test; fi
if (( build_only )); then echo 'Compilation terminée (aucun accès au robot).'; exit 0; fi

mkdir -p backups/flash-sessions
session=$(mktemp -d "backups/flash-sessions/$(date -u +%Y%m%dT%H%M%SZ)-XXXXXX")
trap 'echo "Échec : consulter les journaux et sauvegardes dans $session. Le flash peut être incomplet ; aucun redémarrage supplémentaire ne sera demandé." >&2' ERR
helper=.scripts/build/flash_session.py
python3 "$helper" prepare "$session"
echo "Sauvegarde complète avant flash : $session/pre-flash.bin"
openocd -f "$session/backup.cfg" 2>&1 | tee "$session/backup.log"
python3 "$helper" backup "$session"
echo 'Programmation et relecture complète…'
openocd -f "$session/program.cfg" 2>&1 | tee "$session/program.log"
python3 "$helper" verify "$session"
echo 'Redémarrage et contrôle de fonctionnement au repos…'
openocd -f "$session/startup.cfg" 2>&1 | tee "$session/startup.log"
python3 "$helper" complete "$session"
echo "Flash vérifié, robot redémarré. Sauvegarde et rapport : $session"
