# Measured simulator state only, expressed in the static header.frame_id basis.
# Positions and half extents: m. Linear velocity: m/s. Angular velocity: rad/s.
# header.stamp is simulation time; synchronization uses MCAP record timestamps.
std_msgs/Header header
float64[3] left_eef_position
float64[3] right_eef_position
# Measured articulation DOFs in l1..l7 / r1..r7 order; rad and rad/s.
# No action targets are used as measurements. Measured finger joints remain
# in /robot/state/joint_states; logical 6D hand commands belong to BfmAction.
float64[7] left_arm_position
float64[7] right_arm_position
float64[7] left_arm_velocity
float64[7] right_arm_velocity
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
