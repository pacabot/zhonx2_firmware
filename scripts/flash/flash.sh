#!/usr/bin/env bash
# Build selected image(s), preserve full flash, program and verify over ST-Link SWD.
set -euo pipefail
cd "$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"

usage() {
    cat <<'HELP'
Usage : scripts/flash/flash.sh [--bootloader | --all] [--debug | --release] [--test] [--clean]
Sans option : compile et flashe uniquement l'application en mode debug.
  --bootloader  Compile et flashe uniquement le bootloader.
  --all         Compile et flashe le bootloader, le manifeste et l'application.
  --debug       Optimisation -Og, symboles -g3 (défaut).
  --release     Optimisation -O3, sans symboles de debug.
  --test        Exécute aussi les tests logiciels avant programmation.
  --clean       Recompile entièrement le profil choisi.
  --help        Affiche cette aide.
La Flash est sauvegardée avant écriture et relue après. Les réglages sont préservés.
Variables facultatives : JOBS=4, SWD_KHZ=1000, STLINK_SERIAL=<numéro>.
Robot alimenté et sonde ST-Link branchée ; OpenOCD requis.
HELP
}
mode=app
profile=debug
profile_given=0
for arg in "$@"; do
    case "$arg" in
        --bootloader) [[ $mode == app ]] || { echo 'Choisir --bootloader ou --all.' >&2; exit 2; }; mode=boot ;;
        --all) [[ $mode == app ]] || { echo 'Choisir --bootloader ou --all.' >&2; exit 2; }; mode=all ;;
        --debug|--release)
            (( profile_given == 0 )) || { echo 'Choisir une seule fois --debug ou --release.' >&2; exit 2; }
            profile=${arg#--}; profile_given=1 ;;
        --test|--clean) ;;
        -h|--help) usage; exit 0 ;;
        *) printf 'Option inconnue : %s\n' "$arg" >&2; usage >&2; exit 2 ;;
    esac
done
export PATH="$PATH:/usr/local/arm-gnu-toolchain/bin:/opt/homebrew/bin:/usr/local/bin"
for tool in openocd python3 arm-none-eabi-nm; do
    command -v "$tool" >/dev/null || { echo "Outil manquant : $tool" >&2; exit 1; }
done
[[ ${SWD_KHZ:-1000} =~ ^[1-9][0-9]*$ ]] || { echo 'SWD_KHZ doit être positif.' >&2; exit 2; }
[[ ${STLINK_SERIAL:-} =~ ^[[:alnum:]]*$ ]] || { echo 'Numéro ST-Link invalide.' >&2; exit 2; }
.scripts/build/build.sh "$@"
mkdir -p backups/flash-sessions
session=$(mktemp -d "backups/flash-sessions/$(date -u +%Y%m%dT%H%M%SZ)-XXXXXX")
trap 'echo "Échec : consulter $session. Le flash peut être incomplet." >&2' ERR
helper=scripts/flash/flash_session.py
python3 "$helper" prepare "$session" "$mode" "$profile"
echo "Sauvegarde avant programmation : $session/pre-flash.bin"
openocd -f "$session/backup.cfg" 2>&1 | tee "$session/backup.log"
python3 "$helper" backup "$session"
echo 'Programmation et relecture complète…'
openocd -f "$session/program.cfg" 2>&1 | tee "$session/program.log"
python3 "$helper" verify "$session"
echo 'Redémarrage et contrôle au repos…'
openocd -f "$session/startup.cfg" 2>&1 | tee "$session/startup.log"
python3 "$helper" complete "$session"
echo "Flash vérifié. Sauvegarde et rapport : $session"
