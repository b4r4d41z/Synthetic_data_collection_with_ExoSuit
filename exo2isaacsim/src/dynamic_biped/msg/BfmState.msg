# Measured simulator state only, expressed in the static header.frame_id basis.
# Positions use meters; object_velocity uses meters/second.
# header.stamp is simulation time; synchronization uses MCAP record timestamps.
std_msgs/Header header
float64[3] right_eef_position
float64[3] object_position
float64[3] object_velocity
float64[3] goal_position
