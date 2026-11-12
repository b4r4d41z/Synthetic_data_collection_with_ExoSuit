# V4 — Decoupled Teleoperation & Data Collection

`contract_version=4`, `control_architecture=decoupled_teleoperation_v1` identifies
this revision. It is **not** the earlier executor-based V4 contract. Historical
recordings and their metadata are unchanged. BfmAction retains its existing fields,
26D `[left_arm7, left_hand6, right_arm7, right_hand6]` ordering, radians, logical
0..100 hands, `producer_session_id` and monotonically increasing `action_id`.
BfmState remains the same 56D measured physical/task state.

## Teleoperation and sampling

```
ROS1 ExoSuit -> ZMQ/ROS2 raw commands
  arm -> arm_control_node -> named/radian pre-clamp source
                         -> mechanical clamp -> /robot_arm_joint_command
  hand -> eff_control_node -> /left_hand_joint_command, /right_hand_joint_command
                          -> /bfm/command/hand_target (validated logical input copy)

mapped arm + raw logical hand -> passive BfmActionGateway -> /bfm/action
                                  15 Hz simulation time; never controls the robot
```

Arms forward at valid input rate without added interpolation, velocity/acceleration
limits or a measurement/freshness gate. The existing XML mechanical bounds remain
strict command bounds; malformed names, missing/duplicate joints and nonfinite
values are rejected. Original side mapping and degree-to-radian conversion remain.
The pre-clamp source is recorded separately from bounded controller targets.
Control is published first; passive source/diagnostic hand copies use best-effort
QoS so a missing/slow sampler is not a reliable-delivery barrier. No explicit
downsampling occurs; available message loss must still be checked in real MCAP.

The existing finger mapping, rates of change and thumb gate remain unchanged.
Its 50 Hz timer uses an explicit steady wall clock, independently of `/clock`,
PAUSE and the sampler. Logical inputs must contain six finite values in 0..100
per hand. ROS header stamps remain native node/source stamps; wall scheduling
is not a claim that every physical update occurs at 50 Hz.

PhysX remains 300 Hz, authored into the session layer by the existing runtime
configuration. The ONDEMAND physics `/clock` graph and camera pipeline are intact.
The rational 15 Hz sampler retains latest valid targets and publishes only in
playing episodes. No source-age watchdog is imposed: an unchanged/latest target
may be sampled repeatedly after input stops. This is explicit in metadata and
must be distinguished from fresh input using the independently recorded streams.
Boundary/reset events clear snapshots and rebase the clock, never reset the
producer's increasing action counter or recreate control handles. Frozen simulation
time produces no action ticks; missed samples do not produce catch-up bursts.

There is no ActionExecutor, trajectory configuration, live execution ACK, or
`/bfm/execution_status` recorder requirement. BfmExecutionStatus.msg remains
registered solely for historical interface compatibility; no live endpoint uses it.
A sampled action and a published controller target are neither physical attainment
nor a complete description of all commands between two sampled decisions.

## State, success and clocks

Task-state post-update callbacks use `SimulationManager.get_simulation_time()`
(completed physics time) for both publication deduplication and reset/settling
progress. They no longer depend on `timeline.get_current_time()`, which can repeat
when a composed stage root has no usable time range. The ambidextrous root also
explicitly retains the stable base's 0..1000000 time-code range.

The Nov11 evidence establishes strict measured-bound inhibition in the removed
executor and zero recorded state/success messages. A composed ambidextrous root
was statically confirmed as 0..0 before this change. It is not possible to prove
from those MCAPs alone that timeline looping was the only runtime cause. The host
regression exercises an advancing completed-physics clock with a frozen timeline;
live Isaac verification remains required. Exceptions identify publication phase,
selected object/container and recovery action instead of publishing zero state.

Success still uses the existing selected-object collision-mesh containment,
simulation-time residence and episode latch. No geometry/velocity thresholds or
terminal/success semantics changed. Restoring measured state publication also
restores its associated success evaluation; invalid/unavailable results remain
unknown/review-required. No state is reconstructed from targets.

`CLOCK_DOMAINS` in episode metadata distinguishes:

- Raw bridge headers: source-native ROS1/ExoSuit clock, not assumed hardware or Isaac time.
- Mapped arm/controller headers: mapper ROS clock in teleoperation; copied policy
  decision header in policy mode; publication, not application.
- Logical hand controller headers: copied raw source header in collection; policy decision in policy mode.
- Physical finger command headers: node ROS clock at wall-scheduled publication.
- Action headers: selected physics-driven `/clock` sample, not execution confirmation.
- State/success: completed PhysX simulation time; retain epoch/reset distinctions from `/clock`.
- Images/joint states: existing sensor/simulation headers, independently sampled.
- MCAP log time: unchanged rosbag system receive time, without `--use-sim-time`.

The recorder retains raw arm/hand, mapped pre-clamp arm, bounded controller arm,
logical hand input, both physical finger command streams, measured joints, all
three RGB cameras, `/clock`, `/bfm/action`, `/bfm/state`, `/bfm/success` and existing
lifecycle/outcome channels. Nothing is downsampled to the action cadence. Required
streams must be verified in the real bag before dataset admission.

## Independent ambidextrous pool

`models/stages/v1_manipulation_ambidextrous/things_pool.usda` owns exactly
`lemon1`, `orange2`, `lunchbag`, `ketchup_bottle` and four static remote cradles.
Individual source assets are referenced unchanged. Existing body promotion,
source mass/inertia, geometry, material and collider margins are reused.
All four manipulation bodies exist before PhysX starts; episodes reset five live
actors including the container through the existing mechanism. Unselected bodies
are parked with zero initial velocity. No per-reset creation/re-reference occurs.

The scenario still references the stable V1 environment/robot and reuses the
existing bidirectional A/B placement algorithm, regions, clearance/support checks
and seed behavior. Its configuration identity is `ambidextrous_v4`; the physical
prim namespace remains `/World/Task/Things`. Experimental five-object and legacy
fruit pools/layers/configurations are unchanged.

## Launch and rebuild

Rebuild ROS package registration; retain historical message registration:

```bash
cd /workspace/Synthetic_data_collection_with_ExoSuit/exo2isaacsim
source /opt/ros/jazzy/setup.bash
colcon build --packages-select dynamic_biped isaac_eff_control --symlink-install
source install/setup.bash
cd ..
```

If using the Docker bridge, rebuild its ROS2 image for the added per-stream
input diagnostics (existing EXO_IP/ISAAC_PC_IP deployment setup still applies):

```bash
docker compose -f docker/exo_ros_bridge/docker-compose.yml build ros2_bridge
docker compose -f docker/exo_ros_bridge/docker-compose.yml up -d --no-deps ros2_bridge
```

Restart Isaac and control nodes; do not leave old executor binaries running.
In separate terminals:

```bash
bash launch_bfm_isaac.sh v1_manipulation_ambidextrous
bash start_isaac_control.sh
```

PLAY/PAUSE/STOP remain manual. Existing default/other scenario selection is intact.
Workers hold shared `flock` control locks throughout their lifetime, including
after tmux detaches. Policy startup takes the exclusive lock, and collection
refuses to start against it. Thus the supported launchers cannot run both control
sources concurrently. Locks are process-scoped and release on exit; do not delete
an active lock file. This is not a DDS ownership mechanism for arbitrary manual
publishers; inspect topic publishers and stop competing manually launched nodes.

Policy mode, **instead of** collection:

```bash
bash start_bfm_policy_control.sh
```

This launches only `bfm_policy_bridge_node` and the existing finger controller
(listening to `/bfm/command/hand_target`). The external policy publishes canonical
`/bfm/action`; the bridge validates its structure/bounds/identity and immediately
translates into direct arm and logical hand targets. No interpolation, local-clock
freshness gate or ACK is introduced. Duplicate/backward IDs within a producer
session are rejected; simulation timestamp rollback has no age deadline.

### Downstream BFM changes required (not implemented here)

The external inference runtime must support this exact architecture discriminator,
stop requiring execution_status/ACK timeouts and old controller-config hashes, and
publish only canonical actions when using the supplied bridge. It must not also
publish arm/hand targets. The previous executor-based V4 converter must reject this
new variant until adapted: no acceptance join or 100 Hz trajectory reconstruction
is valid. Use full-rate controller/source streams and explicit causal timing;
a latest-target snapshot is not a zero-order-held executed action over the next
interval. Preserve all actual within-interval commands/action history or define
appropriate new derived training horizons. Do not silently reinterpret old V3 or
executor V4 data, rewrite historical samples, or substitute measured q for actions.

## Live acceptance checks

```bash
ros2 topic info /robot_arm_joint_command --verbose
ros2 topic info /bfm/command/hand_target --verbose
ros2 topic hz /robot/command/arm_joint_target
ros2 topic hz /bfm/source/arm_joint_target_rad
ros2 topic hz /robot_arm_joint_command
ros2 topic hz /left_hand_joint_command
ros2 topic hz /bfm/action
ros2 topic echo /bfm/action --field header.stamp
ros2 topic hz /bfm/state
ros2 topic echo /bfm/success
ros2 topic hz /clock
ros2 bag info /absolute/path/to/record/bag
```

Check one controller publisher, direct arm source-rate forwarding, 50 wall Hz
finger publication and 15 sim Hz actions (wall Hz depends on RTF). PAUSE stops
sim-time sampling; fingers remain wall-scheduled. Repeat STOP/PLAY; verify new
selected-object identity and measured state/success, all four objects and both
layouts across seeds, unchanged native contacts, smoothness and complete MCAP.
Compare RTF under identical scene/render/recording load; no performance gain is
claimed from host tests. All objects/textures resolve locally, but remote warehouse
and table payloads need the normal Isaac asset service and live placement checks.

For arm interruption, the mapper reports raw input silence and ROS2 bridge logs
independent ZMQ arm/hand received counts on health transitions. These diagnostics
never gate messages. If arms stop while hand counts advance, compare ROS1
`/kuavo_arm_traj`, bridge input and ROS2 raw topic before blaming the sampler.
The existing recordings cannot localize the upstream cause more precisely; no
transport parameters or hardware protocol were changed speculatively.
