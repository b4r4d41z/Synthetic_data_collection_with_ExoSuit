#!/bin/bash

SESSION="isaac_bridge"
PROJECT_DIR="$HOME/Desktop/ivan/Synthetic_data_collection_with_ExoSuit"
ROS2_WS="$PROJECT_DIR/exo2isaacsim"
BRIDGE_DIR="$PROJECT_DIR/docker/exo_ros_bridge"

# удалить старую сессию если есть
tmux kill-session -t $SESSION 2>/dev/null

# создать новую сессию с одним окном main
PANE_BRIDGE=$(tmux new-session -d -s $SESSION -n main -P -F "#{pane_id}")

# разделить экран на 4 панели
PANE_ROS2=$(tmux split-window -h -t $PANE_BRIDGE -P -F "#{pane_id}")
PANE_CONTROL=$(tmux split-window -v -t $PANE_BRIDGE -P -F "#{pane_id}")
PANE_CHECK=$(tmux split-window -v -t $PANE_ROS2 -P -F "#{pane_id}")

# выровнять панели в сетку 2x2
tmux select-layout -t $SESSION:main tiled

# подписи панелей
tmux set-option -t $SESSION pane-border-status top
tmux set-option -t $SESSION pane-border-format "#{pane_title}"

tmux select-pane -t $PANE_BRIDGE -T "ROS1 bridge docker"
tmux select-pane -t $PANE_ROS2 -T "ROS2 publisher"
tmux select-pane -t $PANE_CONTROL -T "Isaac controller"
tmux select-pane -t $PANE_CHECK -T "Checks"

# --- PANE 1: ROS1 BRIDGE (docker) ---
tmux send-keys -t $PANE_BRIDGE "
cd $BRIDGE_DIR &&
docker compose up -d --no-deps ros1_bridge &&
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

# --- PANE 4: DEBUG / CHECKS ---
tmux send-keys -t $PANE_CHECK "
conda deactivate 2>/dev/null || true
cd $ROS2_WS

source /opt/ros/jazzy/setup.bash
source install/setup.bash

export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=0
export ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
unset ROS_STATIC_PEERS

echo 'Useful checks:'
echo 'ros2 topic echo /control_robot_hand_position --qos-reliability best_effort'
echo 'ros2 topic echo /hand_joint_command --qos-reliability reliable'
echo ''
bash
" C-m

# выбрать первую панель
tmux select-pane -t $PANE_BRIDGE

# attach к сессии
tmux attach -t $SESSION