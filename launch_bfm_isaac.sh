#!/usr/bin/env bash
# Production Isaac launch; prerequisites are built/installed separately.
set -euo pipefail
PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
[[ -x /isaac-sim/isaac-sim.sh ]] || { echo 'ERROR: Isaac Sim not found at /isaac-sim' >&2; exit 1; }
source "$PROJECT_DIR/tools/collection/environment.bash"
exec /isaac-sim/isaac-sim.sh --ext-folder "$PROJECT_DIR/exts" --enable bfm_collection
