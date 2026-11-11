#!/usr/bin/env bash
# Run alongside Isaac and the external BFM runtime, instead of start_isaac_control.sh.
set -euo pipefail
PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$PROJECT_DIR/tools/collection/environment.bash"
# External policy owns only /bfm/action; same executor as collection.
ros2 run isaac_eff_control bfm_action_executor_node &
EXECUTOR_PID=$!
HAND_PID=''
trap 'kill -INT "$EXECUTOR_PID" ${HAND_PID:+"$HAND_PID"} 2>/dev/null || true; wait || true' EXIT
ros2 run isaac_eff_control eff_control_node &
HAND_PID=$!
wait -n "$EXECUTOR_PID" "$HAND_PID"
