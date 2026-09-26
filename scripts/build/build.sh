#!/usr/bin/env bash
# Build only. Flashing is handled by scripts/flash/flash.sh.
set -euo pipefail
cd "$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"

usage() {
    cat <<'HELP'
Usage : scripts/build/build.sh [--bootloader | --all] [--debug | --release] [--test] [--clean]
Sans option : compile uniquement l'application en mode debug (-Og -g3).
  --bootloader  Compile uniquement le bootloader.
  --all         Compile l'application et le bootloader, puis le paquet complet.
  --debug       Optimisation -Og, symboles -g3 (défaut).
  --release     Optimisation -O3, sans symboles de debug.
  --test        Exécute aussi les tests logiciels.
  --clean       Efface la sortie du profil choisi avant compilation.
  --help        Affiche cette aide.
Sorties : build/debug/ ou build/release/. Variable facultative : JOBS=4.
HELP
}

target=app
profile=debug
profile_given=0
run_tests=0
clean=0
for arg in "$@"; do
    case "$arg" in
        --bootloader) [[ $target == app ]] || { echo 'Choisir --bootloader ou --all.' >&2; exit 2; }; target=boot ;;
        --all) [[ $target == app ]] || { echo 'Choisir --bootloader ou --all.' >&2; exit 2; }; target=all ;;
        --debug|--release)
            (( profile_given == 0 )) || { echo 'Choisir une seule fois --debug ou --release.' >&2; exit 2; }
            profile=${arg#--}; profile_given=1 ;;
        --test) run_tests=1 ;;
        --clean) clean=1 ;;
        -h|--help) usage; exit 0 ;;
        *) printf 'Option inconnue : %s\n' "$arg" >&2; usage >&2; exit 2 ;;
    esac
done
export PATH="$PATH:/usr/local/arm-gnu-toolchain/bin:/opt/homebrew/bin:/usr/local/bin"
for tool in make python3 arm-none-eabi-gcc arm-none-eabi-objcopy arm-none-eabi-size; do
    command -v "$tool" >/dev/null || { echo "Outil manquant : $tool" >&2; exit 1; }
done
[[ ${JOBS:-4} =~ ^[1-9][0-9]*$ ]] || { echo 'JOBS doit être positif.' >&2; exit 2; }
if [[ $profile == release ]]; then opt=-O3; debug_flags=-g0; else opt=-Og; debug_flags=-g3; fi
if (( clean )); then rm -rf -- "build/$profile"; fi
make -j"${JOBS:-4}" "BUILD=build/$profile" "OPT=$opt" "DEBUG_FLAGS=$debug_flags" "$target"
if (( run_tests )); then make test; fi
echo "Compilation $profile ($target) terminée : build/$profile/"
