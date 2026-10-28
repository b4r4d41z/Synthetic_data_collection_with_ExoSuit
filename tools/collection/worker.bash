#!/usr/bin/env bash
# Internal tmux worker, not another user-facing launch command.
set -euo pipefail
PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
source "$PROJECT_DIR/tools/collection/environment.bash"
cd "$PROJECT_DIR"
exec python3 "$PROJECT_DIR/tools/collection/worker.py" "$1"
