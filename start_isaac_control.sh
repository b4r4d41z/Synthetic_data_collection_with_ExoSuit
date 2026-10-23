#!/bin/bash

SESSION="isaac_control"

PROJECT_DIR="/workspace/Synthetic_data_collection_with_ExoSuit"
ROS2_WS="$PROJECT_DIR/exo2isaacsim"

# Opt-in only: the existing direct teleoperation mode remains the default.
BFM_COLLECTION_MODE="${BFM_COLLECTION_MODE:-0}"
case "$BFM_COLLECTION_MODE" in
  0|1) ;;
  *) echo "BFM_COLLECTION_MODE must be 0 or 1" >&2; exit 1 ;;
esac
# Existing tmux servers may retain an older environment. Pass this mode and
# optional source/task-frame configuration explicitly into every node process.
BFM_ENV="BFM_COLLECTION_MODE=$BFM_COLLECTION_MODE"
for key in BFM_STATE_FRAME BFM_REFERENCE_PRIM_PATH BFM_EEF_PRIM_PATH \
           BFM_OBJECT_PRIM_PATH BFM_GOAL_PRIM_PATH ARM_INPUT_TOPIC ARM_OUTPUT_TOPIC \
           ARM_INPUT_IS_DEGREES HAND_INPUT_TOPIC; do
  if [[ -v $key ]]; then
    printf -v quoted_value '%q' "${!key}"
    BFM_ENV+=" $key=$quoted_value"
  fi
done

tmux kill-session -t $SESSION 2>/dev/null

PANE_HAND=$(tmux new-session -d -s $SESSION -n main -P -F "#{pane_id}")
PANE_ARM=$(tmux split-window -h -t $PANE_HAND -P -F "#{pane_id}")
PANE_RECORDER=$(tmux split-window -h -t $PANE_ARM -P -F "#{pane_id}")

tmux select-layout -t $SESSION:main even-horizontal

tmux set-option -t $SESSION pane-border-status top
tmux set-option -t $SESSION pane-border-format "#{pane_title}"

tmux select-pane -t $PANE_HAND -T "Isaac hand controller"
tmux select-pane -t $PANE_ARM -T "Isaac arm controller"
tmux select-pane -t $PANE_RECORDER -T "Simulation data recorder"

if [[ "$BFM_COLLECTION_MODE" == "1" ]]; then
  PANE_BFM=$(tmux split-window -v -t $PANE_RECORDER -P -F "#{pane_id}")
  tmux select-pane -t $PANE_BFM -T "BFM 10 Hz action gateway"
  tmux select-layout -t $SESSION:main tiled
  tmux send-keys -t $PANE_BFM "
cd $ROS2_WS
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
unset ROS_STATIC_PEERS
env $BFM_ENV ros2 run isaac_eff_control bfm_action_gateway_node
" C-m
fi

# --- HAND CONTROLLER ---
tmux send-keys -t $PANE_HAND "
cd $ROS2_WS

source /opt/ros/jazzy/setup.bash
source install/setup.bash

export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
unset ROS_STATIC_PEERS

env $BFM_ENV ros2 run isaac_eff_control eff_control_node
" C-m

# --- ARM CONTROLLER ---
tmux send-keys -t $PANE_ARM "
cd $ROS2_WS

source /opt/ros/jazzy/setup.bash
source install/setup.bash

export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
unset ROS_STATIC_PEERS

env $BFM_ENV ros2 run isaac_eff_control arm_control_node
" C-m

# --- SIMULATION DATA RECORDER ---
tmux send-keys -t $PANE_RECORDER "
cd $ROS2_WS

source /opt/ros/jazzy/setup.bash
source install/setup.bash

export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
unset ROS_STATIC_PEERS

env $BFM_ENV ros2 run isaac_eff_control sim_recorder_node
" C-m

tmux select-pane -t $PANE_HAND
tmux attach -t $SESSION