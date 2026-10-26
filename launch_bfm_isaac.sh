#!/usr/bin/env bash
# Launch in the existing Isaac environment. Does not launch controllers or build.
set -eo pipefail
PROJECT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
: "${ISAAC_SIM_PATH:?Set ISAAC_SIM_PATH to the existing Isaac Sim installation directory}"
source /opt/ros/jazzy/setup.bash
source "$PROJECT_DIR/exo2isaacsim/install/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-0}"
export ROS_AUTOMATIC_DISCOVERY_RANGE="${ROS_AUTOMATIC_DISCOVERY_RANGE:-SUBNET}"
exec "$ISAAC_SIM_PATH/isaac-sim.sh" --ext-folder "$PROJECT_DIR/exts" --enable bfm_collection "$@"
