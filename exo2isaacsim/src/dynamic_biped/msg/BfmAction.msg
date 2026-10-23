# One absolute high-level decision held until the next 10 Hz policy tick.
# header.stamp uses the gateway ROS clock. Cross-topic synchronization uses
# rosbag2/MCAP recorded-message timestamps, not a universal header clock.
std_msgs/Header header

# Canonical order: zarm_r1_joint, ..., zarm_r7_joint (after existing side mapping).
string[7] right_arm_joint_names
# Absolute joint-position targets, radians.
float64[7] right_arm_target
# Unchanged RobotHandPosition.right_hand_position[0..5], controller units 0..100.
# These are six logical Sharpa targets, not joint angles or a binary gripper.
float64[6] right_hand_target
