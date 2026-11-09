#!/usr/bin/env bash
# Production Isaac launch; prerequisites are built/installed separately.
set -euo pipefail
PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
[[ $# -le 1 ]] || { echo 'ERROR: expected at most one scenario argument' >&2; exit 1; }
if [[ $# == 1 && -n ${SCENARIO+x} && "$1" != "$SCENARIO" ]]; then
    echo 'ERROR: positional scenario conflicts with SCENARIO' >&2; exit 1
fi
export SCENARIO="${1-${SCENARIO-sorting_experimental}}"
REGISTRY="$PROJECT_DIR/exo2isaacsim/src/isaac_eff_control/isaac_eff_control/scenarios.py"
# Validate before sourcing ROS or starting Isaac. A fixed registry rejects paths.
SELECTION="$(python3 "$REGISTRY" "$SCENARIO" "$PROJECT_DIR")" || exit 1
mapfile -t SELECTED <<< "$SELECTION"
[[ -x /isaac-sim/isaac-sim.sh ]] || { echo 'ERROR: Isaac Sim not found at /isaac-sim' >&2; exit 1; }
source "$PROJECT_DIR/tools/collection/environment.bash"
# environment.bash clears BFM_*; set this only after its fixed ROS setup.
export BFM_SCENE_CONFIG="${SELECTED[1]}"
printf '[BFM] Scenario: %s\n[BFM] Scene: %s\n[BFM] Configuration: %s\n[BFM] Starting Isaac Sim...\n' \
    "$SCENARIO" "${SELECTED[0]}" "$BFM_SCENE_CONFIG"
# Isaac's startup script opens the stage without starting the timeline.
exec /isaac-sim/isaac-sim.sh --ext-folder "$PROJECT_DIR/exts" --enable bfm_collection \
    --/isaac/startup/create_new_stage=false \
    --exec "\"$PROJECT_DIR/tools/collection/open_sorting_stage.py\""
