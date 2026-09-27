#!/usr/bin/env bash
set -euo pipefail
cd "$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
export PATH="$PATH:/opt/homebrew/bin:/usr/local/bin"
exec python3 scripts/flash/calibration.py "$@"
