#!/usr/bin/env bash
set -euo pipefail
PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$PROJECT_DIR/tools/collection/environment.bash"
export PYTHONPATH="$PROJECT_DIR/exo2isaacsim/src/isaac_eff_control${PYTHONPATH:+:$PYTHONPATH}"
exec python3 "$PROJECT_DIR/tools/evaluation/record.py" "$@"
