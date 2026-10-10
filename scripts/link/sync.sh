#!/usr/bin/env bash
# Copie tels quels les fichiers Ariane Link partagés. Ne jamais les modifier à la
# main : corriger la source puis relancer ce script.
#  - ariane-tools/link : schéma généré, tramage C, vecteurs de test ;
#  - ZHONX-III_firmware : modules purs (anneaux, signal lumineux, paramètres,
#    ligne de console) et leurs tests hôte.
# link_port.h est propre à ZHONX II (CMSIS de la SPL) et n'est pas copié.
# Usage : scripts/link/sync.sh [ariane-tools/link] [ZHONX-III_firmware]
set -euo pipefail
root=$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)
link=${1:-"$root/../ariane-tools/link"}
z3=${2:-"$root/../ZHONX-III_firmware"}
for f in "$link/schema/link.yaml" "$z3/Src/middleware/link/link_ring.c"; do
    [[ -f $f ]] || { printf 'introuvable : %s\n' "$f" >&2; exit 1; }
done
inc=$root/firmware/include/middleware/link
cp "$link/generated/c/ariane_link_schema.h" "$link/c/link_frame.h" "$inc/"
cp "$link/c/link_frame.c" "$root/firmware/link/"
cp "$link/tests/vectors.json" "$root/tests/link/"
for m in link_ring link_led link_params link_line; do
    cp "$z3/Inc/middleware/link/$m.h" "$inc/"
    cp "$z3/Src/middleware/link/$m.c" "$root/firmware/link/"
done
cp "$z3/tests/test_link.c" "$z3/tests/test_link_frame.c" "$root/tests/"
printf 'Synchronisé depuis %s et %s\n' "$link" "$z3"
grep -h "LINK_SCHEMA_CRC\|LINK_PROTO_VERSION" "$inc/ariane_link_schema.h"
