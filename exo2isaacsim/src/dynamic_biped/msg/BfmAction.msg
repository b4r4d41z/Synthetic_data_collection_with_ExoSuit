# Passive operator snapshot in collection; policy producers use the same 26D targets.
# header.stamp uses the selected Isaac /clock simulation-time sample. Cross-topic synchronization uses
# rosbag2/MCAP recorded-message timestamps, not a universal header clock.
# Exact input association uses the provenance identity/digest below, not receive-time matching.
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

# v4 identity: IDs remain monotonic across simulation resets.
string producer_session_id
uint64 action_id

# 1: exact selected Gateway inputs in /bfm/source/input_provenance (String JSON).
# SHA-256 of that String.data's exact UTF-8 bytes, joined by producer_session_id/action_id.
# 0 + empty digest: no operator provenance claimed (policy/default).
# Historical schemas lack these fields and cannot claim v1 evidence.
# Observational metadata only, never execution acknowledgement; target fields are unchanged.
uint32 source_provenance_version
string source_provenance_sha256
