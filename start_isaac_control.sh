#!/usr/bin/env bash
set -euo pipefail
PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
command -v tmux >/dev/null || { echo 'ERROR: tmux is not installed' >&2; exit 1; }
command -v python3 >/dev/null || { echo 'ERROR: python3 is not installed' >&2; exit 1; }
source "$PROJECT_DIR/tools/collection/environment.bash"
SESSION=isaac_control
if tmux has-session -t "$SESSION" 2>/dev/null; then
    # Workers forward Ctrl-C and stay alive until their nodes finish cleanup.
    while read -r pane dead; do
        [[ "$dead" == 1 ]] || tmux send-keys -t "$pane" C-c
    done < <(tmux list-panes -s -t "$SESSION" -F '#{pane_id} #{pane_dead}')
    for ((attempt=0; attempt<40; attempt++)); do
        if ! tmux list-panes -s -t "$SESSION" -F '#{pane_dead}' | grep -q '^0$'; then
            break
        fi
        sleep 1
    done
    if tmux list-panes -s -t "$SESSION" -F '#{pane_dead}' | grep -q '^0$'; then
        echo 'ERROR: previous collection nodes are still finalizing; session preserved' >&2
        exit 1
    fi
    tmux kill-session -t "$SESSION"
fi
# Create idle panes first so remain-on-exit is set before any worker can fail.
PANE_ARM=$(tmux new-session -d -s "$SESSION" -n main -P -F '#{pane_id}')
tmux set-option -t "$SESSION:main" remain-on-exit on
PANE_HAND=$(tmux split-window -h -t "$PANE_ARM" -P -F '#{pane_id}')
PANE_RECORDER=$(tmux split-window -h -t "$PANE_HAND" -P -F '#{pane_id}')
tmux select-layout -t "$SESSION:main" even-horizontal
tmux set-option -t "$SESSION" pane-border-status top
tmux set-option -t "$SESSION" pane-border-format '#{pane_title}'
tmux select-pane -t "$PANE_ARM" -T 'ARM / JOINTS'
tmux select-pane -t "$PANE_HAND" -T 'HAND / GRIPPERS'
tmux select-pane -t "$PANE_RECORDER" -T 'RECORDING'
PANE_GATEWAY=$(tmux new-window -d -t "$SESSION" -n backend -P -F '#{pane_id}')
tmux set-option -t "$SESSION:backend" remain-on-exit on
PANE_LOG=$(tmux split-window -v -t "$PANE_GATEWAY" -P -F '#{pane_id}')
tmux select-pane -t "$PANE_GATEWAY" -T 'BFM gateway'
tmux select-pane -t "$PANE_LOG" -T 'rosbag2 details'
# Each worker re-sources the fixed environment, including in an old tmux server.
start_worker() {
    local command
    printf -v command 'exec bash %q %q' "$PROJECT_DIR/tools/collection/worker.bash" "$2"
    tmux send-keys -t "$1" "$command" C-m
}
start_worker "$PANE_ARM" arm_control_node
start_worker "$PANE_HAND" eff_control_node
start_worker "$PANE_RECORDER" sim_recorder_node
start_worker "$PANE_GATEWAY" bfm_action_gateway_node
printf -v LOG_COMMAND 'exec tail -F -- %q' "$PROJECT_DIR/records/rosbag2.log"
tmux send-keys -t "$PANE_LOG" "$LOG_COMMAND" C-m
tmux select-window -t "$SESSION:main"
tmux select-pane -t "$PANE_ARM"
tmux attach-session -t "$SESSION"
