#!/usr/bin/env bash
# Run alongside Isaac and the external BFM runtime, instead of start_isaac_control.sh.
set -euo pipefail
PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$PROJECT_DIR/tools/collection/environment.bash"
# BFM_COLLECTION_MODE=1 selects /bfm/command/hand_target in eff_control_node.
# External BFM owns arm targets and /bfm/action; only execute hands here.
exec ros2 run isaac_eff_control eff_control_node
