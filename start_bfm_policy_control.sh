#!/usr/bin/env bash
# Run alongside Isaac and the external BFM runtime, instead of start_isaac_control.sh.
set -euo pipefail
PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$PROJECT_DIR/tools/collection/environment.bash"
# Exclusive against the shared locks held by collection's control workers.
exec 9>"${XDG_RUNTIME_DIR:-/tmp}/synthetic-control-${UID}.lock"
flock -n 9 || { echo 'ERROR: collection or policy control is already running' >&2; exit 1; }
export HAND_INPUT_TOPIC=/bfm/command/hand_target
# External policy owns /bfm/action; direct translation, no trajectory or ACK.
ros2 run isaac_eff_control bfm_policy_bridge_node &
BRIDGE_PID=$!
HAND_PID=''
trap 'kill -INT "$BRIDGE_PID" ${HAND_PID:+"$HAND_PID"} 2>/dev/null || true; wait || true' EXIT
ros2 run isaac_eff_control eff_control_node &
HAND_PID=$!
wait -n "$BRIDGE_PID" "$HAND_PID"
