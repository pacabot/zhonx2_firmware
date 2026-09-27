#!/usr/bin/env bash
# Build selected images, preserve persistent data and program over ST-Link SWD.
set -euo pipefail
cd "$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"

usage() {
    cat <<'HELP'
Usage : scripts/flash/flash.sh [--bootloader | --all] [--debug | --release] [--test] [--clean] [--fast | --full-backup] [--verify]
Sans option : compile et flashe uniquement l'application en mode debug.
  --bootloader  Compile et flashe uniquement le bootloader.
  --all         Compile et flashe le bootloader, le manifeste et l'application.
  --debug       Optimisation -Og, symboles -g3 (défaut).
  --release     Optimisation -O3, sans symboles de debug.
  --test        Exécute aussi les tests logiciels avant programmation.
  --clean       Recompile entièrement le profil choisi.
  --fast        Mode par défaut : sauvegarde 48 Kio (bootloader et données).
  --full-backup  Sauvegarde toute la flash (1 Mio) avant écriture.
  --verify      Vérifie les images et les données après écriture ; avec
                --full-backup, compare aussi toute la flash après écriture.
  --check       Vérifie uniquement sonde + UID, sans compilation ni flash.
  --help        Affiche cette aide.
La compilation est incrémentale, y compris avec --fast : seuls les fichiers
à reconstruire sont compilés dans le profil demandé avant tout accès à la cible.
Sans --verify, aucune vérification ni relecture de flash après écriture.
Les secteurs de calibration/réglages/labyrinthes ne sont jamais effacés.
Le contrôle de démarrage au repos est conservé dans tous les modes.
Le mode application exige un bootloader en secteur 0 ; sinon utiliser --all.
Association robot/sonde obligatoire : robot_guard.py associate zhonx2 --serial NUMERO
Le numéro de sonde et l’UID STM32 sont contrôlés avant tout effacement.
Variables facultatives : JOBS=4, SWD_KHZ=1000. STLINK_SERIAL ne peut pas
sélectionner une sonde différente de celle enregistrée pour zhonx2.
Robot alimenté et sonde ST-Link branchée ; OpenOCD requis.
HELP
}
if [[ ${1:-} == --check ]]; then
    (( $# == 1 )) || { echo '--check doit être utilisé seul.' >&2; exit 2; }
    export PATH="$PATH:/opt/homebrew/bin:/usr/local/bin"
    python3 scripts/flash/robot_guard.py verify zhonx2
    exit 0
fi
mode=app
profile=debug
profile_given=0
backup_mode=fast
backup_given=0
verification=no-verify
build_args=()
for arg in "$@"; do
    case "$arg" in
        --fast|--full-backup)
            (( backup_given == 0 )) || { echo 'Choisir --fast ou --full-backup.' >&2; exit 2; }
            backup_given=1; backup_mode=fast
            [[ $arg != --full-backup ]] || backup_mode=full
            continue ;;
        --verify) verification=verify; continue ;;
    esac
    build_args+=("$arg")
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
printf 'Mise à jour incrémentale des images : profil %s, cible %s…\n' "$profile" "$mode"
if ! ./scripts/build/build.sh ${build_args[@]+"${build_args[@]}"}; then
    echo 'Échec de compilation : programmation annulée, aucun accès à la cible.' >&2
    exit 1
fi
printf 'Images %s à jour ; préparation du flash.\n' "$profile"
mkdir -p backups/flash-sessions
session=$(mktemp -d "backups/flash-sessions/$(date -u +%Y%m%dT%H%M%SZ)-XXXXXX")
failure='Aucune écriture en flash effectuée.'
trap 'echo "Échec : consulter $session. $failure" >&2' ERR
helper=scripts/flash/flash_session.py
python3 "$helper" prepare "$session" "$mode" "$profile" "$backup_mode" "$verification"
echo "Sauvegarde avant programmation : $session/pre-flash.bin"
openocd -f "$session/backup.cfg" 2>&1 | tee "$session/backup.log"
python3 "$helper" backup "$session"
if [[ $verification == verify ]]; then echo 'Programmation et vérification…'; else echo 'Programmation sans vérification de flash…'; fi
failure='La programmation peut être incomplète.'
openocd -f "$session/program.cfg" 2>&1 | tee "$session/program.log"
python3 "$helper" programmed "$session"
failure='Programmation terminée, mais contrôle du démarrage non validé.'
echo 'Redémarrage et contrôle au repos…'
openocd -f "$session/startup.cfg" 2>&1 | tee "$session/startup.log"
python3 "$helper" complete "$session"
if [[ $verification == verify ]]; then
    echo "Flash vérifié et démarrage contrôlé. Sauvegarde et rapport : $session"
else
    echo "Flash terminé sans vérification de flash, démarrage contrôlé. Sauvegarde et rapport : $session"
fi
