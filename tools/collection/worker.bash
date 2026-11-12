#!/usr/bin/env bash
# Internal tmux worker, not another user-facing launch command.
set -euo pipefail
PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
source "$PROJECT_DIR/tools/collection/environment.bash"
cd "$PROJECT_DIR"
if [[ "$1" == arm_control_node || "$1" == eff_control_node ]]; then
    exec 9>"${XDG_RUNTIME_DIR:-/tmp}/synthetic-control-${UID}.lock"
    flock -sn 9 || { echo 'ERROR: policy control is running' >&2; exit 1; }
fi
exec python3 "$PROJECT_DIR/tools/collection/worker.py" "$1"
