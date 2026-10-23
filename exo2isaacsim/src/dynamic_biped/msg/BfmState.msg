# Measured simulator state only, expressed in the static header.frame_id basis.
# Positions and half extents: m. Linear velocity: m/s. Angular velocity: rad/s.
# header.stamp is simulation time; synchronization uses MCAP record timestamps.
std_msgs/Header header
float64[3] right_eef_position
# Object position and velocity are measured at its live PhysX center of mass.
float64[3] object_position
float64[3] object_velocity
# Moving usable interior OBB, derived from the container's live rigid motion.
float64[3] goal_center
# Fixed lengths along goal-LOCAL X/Y/Z axes, not a world-axis AABB.
float64[3] goal_half_extents
# Unit quaternion [x, y, z, w], goal-local axes relative to header.frame_id.
float64[4] goal_orientation
# Velocity at goal_center, including omega cross the offset from the body origin.
float64[3] goal_linear_velocity
float64[3] goal_angular_velocity
