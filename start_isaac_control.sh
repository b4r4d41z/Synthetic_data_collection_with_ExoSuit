#!/bin/bash

SESSION="isaac_control"

PROJECT_DIR="/workspace/Synthetic_data_collection_with_ExoSuit"
ROS2_WS="$PROJECT_DIR/exo2isaacsim"

tmux kill-session -t $SESSION 2>/dev/null

PANE_HAND=$(tmux new-session -d -s $SESSION -n main -P -F "#{pane_id}")
PANE_ARM=$(tmux split-window -h -t $PANE_HAND -P -F "#{pane_id}")

tmux select-layout -t $SESSION:main even-horizontal

tmux set-option -t $SESSION pane-border-status top
tmux set-option -t $SESSION pane-border-format "#{pane_title}"

tmux select-pane -t $PANE_HAND -T "Isaac hand controller"
tmux select-pane -t $PANE_ARM -T "Isaac arm controller"

# --- HAND CONTROLLER ---
tmux send-keys -t $PANE_HAND "
cd $ROS2_WS

source /opt/ros/jazzy/setup.bash
source install/setup.bash

export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
unset ROS_STATIC_PEERS

ros2 run isaac_eff_control eff_control_node
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

ros2 run isaac_eff_control arm_control_node
" C-m

tmux select-pane -t $PANE_HAND
tmux attach -t $SESSION