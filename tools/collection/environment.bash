# Internal production environment shared by both launchers and tmux workers.
ROS2_WS="$PROJECT_DIR/exo2isaacsim"
[[ -f /opt/ros/jazzy/setup.bash ]] || { echo 'ERROR: ROS2 Jazzy not found' >&2; exit 1; }
[[ -f "$ROS2_WS/install/setup.bash" ]] || { echo 'ERROR: ROS2 workspace is not built' >&2; exit 1; }
# ROS/colcon setup scripts may reference unset variables.
set +u
source /opt/ros/jazzy/setup.bash
source "$ROS2_WS/install/setup.bash"
set -u
# Ignore caller/tmux-server overrides in the single production workflow.
for key in ${!BFM_@}; do unset "$key"; done
unset ARM_OUTPUT_TOPIC ROS_STATIC_PEERS
export BFM_COLLECTION_MODE=1
export ARM_INPUT_TOPIC=/robot/command/arm_joint_target
export ARM_INPUT_IS_DEGREES=1
export HAND_INPUT_TOPIC=/robot/command/hand_target
export ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET
export RCUTILS_CONSOLE_OUTPUT_FORMAT='{message}'
