# One absolute high-level decision held until the next 10 Hz policy tick.
# header.stamp uses the selected Isaac /clock simulation-time sample. Cross-topic synchronization uses
# rosbag2/MCAP recorded-message timestamps, not a universal header clock.
std_msgs/Header header

# Canonical flattening: left_arm[7], left_hand[6], right_arm[7], right_hand[6] (26D).
# Absolute mapped arm joint-position targets, radians, in l1..l7 / r1..r7 order.
string[7] left_arm_joint_names
float64[7] left_arm_target
# Six logical Sharpa controller targets, 0..100; not physical finger joint angles.
float64[6] left_hand_target

string[7] right_arm_joint_names
float64[7] right_arm_target
float64[6] right_hand_target
