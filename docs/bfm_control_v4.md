# BFM control contract v4

The entry point in both collection and policy operation is `/bfm/action` at
15 Hz **simulation time**. Its ordering, radians, mechanical arm limits and
logical hand range remain unchanged. This intentionally replaces v3's direct
source forwarding; old recordings retain their original contract.

Collection: ExoSuit bridge -> existing arm mapper / raw hands -> gateway buffers
-> canonical action -> common executor -> 100 Hz arm trajectory and accepted
logical hand targets -> existing 50 Hz finger mapping. Inference launches the
same executor/finger implementation; external policy owns only `/bfm/action`.
It must no longer publish `/robot_arm_joint_command` or
`/bfm/command/hand_target`. No BFM-repository changes are included here.

Producers must rebuild against the appended BfmAction fields, generate a unique
nonempty producer_session_id at process startup and use positive uint64 action_id
values that increase across scene resets. Publish canonical names in order and
simulation decision stamps. The executor locks a producer until scene reset;
accepted-ID watermarks survive resets. Duplicate, backward, future, stale,
malformed and out-of-bound decisions are rejected without changing targets.

`/bfm/execution_status` confirms acceptance, **not physical attainment**. Match
(session, action_id), accepted=true, simulation stamp and configuration hash to
recorded actions. Rejections do not constitute executed training decisions.
Use successive accepted events and low-level target streams to establish actual
execution intervals; do not infer acceptance from the presence of an action.
ROS messages are not a reliable physical application acknowledgement.

`bfm_executor_config.json` is installed with the package and SHA-256 hashed by
both recorder and executor. Its versioned algorithm is:

    desired_v = clip((target_q - commanded_q) / response_time, +/-max_velocity)
    commanded_v = clip(desired_v, previous_v +/- max_acceleration * 0.01)
    commanded_q = clip(commanded_q + commanded_v * 0.01, mechanical_joint_bounds)

The initial configuration uses response_time=0.2 s, each arm velocity cap=1 rad/s,
and acceleration cap=2 rad/s². These are conservative execution settings requiring
robot/runtime validation, not asserted mechanical limits. Mechanical bounds remain
authoritative model XML values. Retargeting preserves commanded q/v. Invalid or
stale measured positions inhibit commands and clear trajectory state; recovery
requires fresh valid measurements and a new accepted action. This emergency reset
of commanded velocity is a safety exception to normal acceleration continuity.
The simulator then holds its last published bounded position target.

Finger mapping, thumb gate and logical rate limits are unchanged. The existing
integer logical message rounds fractional canonical hand targets to nearest integer
(ties to even). The recorded accepted hand target exposes that quantization.
The finger controller ticks on /clock rather than a wall timer in BFM mode;
legacy non-BFM mode retains its wall-time timer and direct inputs.

Physics uses the existing PhysicsScene's PhysxSchema timeStepsPerSecond=300,
authored in the session layer before PLAY. No saved scene, materials or clock
ActionGraph architecture changes. At 300 Hz there are 20 physics steps/decision,
3/arm update and 6/finger update. Rational integer deadlines tolerate one ns of
ROS stamp quantization. Frozen stamps produce no work; rollback rebases; missing
samples produce one current update, never a catch-up burst. Missing low-level
callbacks therefore slow trajectory progress rather than inventing updates.
STOP/new boundaries clear buffered sources/trajectories; PAUSE freezes updates.
Gateway source freshness uses a 0.5 s monotonic receive watchdog (not a policy
clock), so a disconnected source cannot keep generating new decisions. An already
accepted target is held until superseded or reset, as with any target controller.

## Rebuild and deployment

Rebuild both interface and controller packages; restart Isaac and all ROS nodes
so no v3 and v4 message definitions coexist:

```bash
cd /workspace/Synthetic_data_collection_with_ExoSuit/exo2isaacsim
source /opt/ros/jazzy/setup.bash
colcon build --packages-select dynamic_biped isaac_eff_control --symlink-install
source install/setup.bash
cd ..
bash launch_bfm_isaac.sh
# Collection in a separate terminal:
bash start_isaac_control.sh
# OR policy execution in a separate terminal (never both):
bash start_bfm_policy_control.sh
```

The existing ExoSuit ROS1/ROS2/ZMQ bridge remains an external prerequisite exactly
as before; no duplicate bridge is started. The common executor must be the only
publisher of arm and logical-hand execution topics. It fails on detected competing
publishers; discovery is asynchronous, not a DDS ownership lock.

The external BFM runtime must independently adopt v4 identities and 15 Hz
simulation scheduling, and relinquish direct execution topics. A v3 converter
must not silently accept these recordings: join accepted execution statuses and
check configuration identity. This task does not modify that other repository.

## Validation on the Isaac machine

After manual PLAY, check publisher ownership and wall-delivery rates:

```bash
ros2 topic info /robot_arm_joint_command --verbose
ros2 topic info /bfm/command/hand_target --verbose
ros2 topic hz /clock
ros2 topic hz /bfm/action
ros2 topic hz /robot_arm_joint_command
ros2 topic hz /left_hand_joint_command
ros2 topic echo /bfm/execution_status
ros2 topic echo /bfm/action --field header.stamp
ros2 bag info /absolute/path/to/episode/bag
```

Use simulation stamps to verify 300/15/100/50 Hz; CLI wall Hz scales with RTF.
PAUSE must stop updates; PLAY resumes without bursts. STOP/new episode must
reinitialize from valid measured arm positions, with increasing producer IDs.
Disconnect each ExoSuit source: new canonical decisions must stop within the
receive watchdog. Inject duplicate/out-of-order IDs and invalid q: require explicit
rejections/inhibition. Check every accepted event hash against episode metadata.
Record nonzero action, status, controller, state, cameras, clock and lifecycle
streams. Compare collection and policy low-level trajectories under identical
accepted actions, initial measured state and clock samples. Live tests, generated
ROS interfaces and actual 300 Hz PhysX stepping remain required; host tests do
not establish physical tracking or rendering capacity at that frequency.
