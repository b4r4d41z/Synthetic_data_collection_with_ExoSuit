#!/bin/bash

SESSION="isaac_bridge"
PROJECT_DIR="$HOME/Desktop/ivan/Synthetic_data_collection_with_ExoSuit"
ROS2_WS="$PROJECT_DIR/exo2isaacsim"
BRIDGE_DIR="$PROJECT_DIR/docker/exo_ros_bridge"

tmux kill-session -t $SESSION 2>/dev/null

PANE_BRIDGE=$(tmux new-session -d -s $SESSION -n main -P -F "#{pane_id}")

PANE_ROS2=$(tmux split-window -h -t $PANE_BRIDGE -P -F "#{pane_id}")
PANE_CONTROL=$(tmux split-window -v -t $PANE_BRIDGE -P -F "#{pane_id}")
PANE_ARM_CONTROL=$(tmux split-window -v -t $PANE_ROS2 -P -F "#{pane_id}")

tmux select-layout -t $SESSION:main tiled

tmux set-option -t $SESSION pane-border-status top
tmux set-option -t $SESSION pane-border-format "#{pane_title}"

tmux select-pane -t $PANE_BRIDGE -T "ROS1 bridge docker"
tmux select-pane -t $PANE_ROS2 -T "ROS2 publisher"
tmux select-pane -t $PANE_CONTROL -T "Isaac hand controller"
tmux select-pane -t $PANE_ARM_CONTROL -T "Isaac arm controller"

# --- PANE 1: ROS1 BRIDGE DOCKER ---
tmux send-keys -t $PANE_BRIDGE "
cd $BRIDGE_DIR &&
docker compose up -d --force-recreate --no-deps ros1_bridge &&
docker logs -f exo_ros1_bridge
" C-m

# --- PANE 2: ROS2 PUBLISHER ---
tmux send-keys -t $PANE_ROS2 "
conda deactivate 2>/dev/null || true

source /opt/ros/jazzy/setup.bash
source $ROS2_WS/install/setup.bash

export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
unset ROS_STATIC_PEERS

/usr/bin/python3 $BRIDGE_DIR/ros2/bridge/ros2_publisher.py
" C-m

# --- PANE 3: ISAAC HAND CONTROLLER ---
tmux send-keys -t $PANE_CONTROL "
conda deactivate 2>/dev/null || true
cd $ROS2_WS

source /opt/ros/jazzy/setup.bash
source install/setup.bash

export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
unset ROS_STATIC_PEERS

ros2 run isaac_eff_control eff_control_node
" C-m

# --- PANE 4: ISAAC ARM CONTROLLER ---
tmux send-keys -t $PANE_ARM_CONTROL "
conda deactivate 2>/dev/null || true
cd $ROS2_WS

source /opt/ros/jazzy/setup.bash
source install/setup.bash

export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
unset ROS_STATIC_PEERS

ros2 run isaac_eff_control arm_control_node
" C-m

tmux select-pane -t $PANE_BRIDGE
tmux attach -t $SESSION