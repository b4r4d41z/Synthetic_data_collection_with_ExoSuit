# Simulation recording control

The existing ROS1 bridge forwards `std_msgs/Bool` on
`/data_collection/recording` over its shared ZMQ connection as
`{"kind": "recording", "enabled": true}` (or `false`). The ROS2 bridge container
republishes `std_msgs/msg/Bool` with reliable, transient-local,
keep-last/depth-1 QoS. Hand and arm topics retain their best-effort QoS.
`ROS1_RECORDING_TOPIC` and `ROS2_RECORDING_TOPIC` override the bridge topic names.

`sim_recorder_node` runs beside the controllers in the existing Isaac Sim
container, subscribes with the same recording-control QoS, and starts one
`ros2 bag record` subprocess on a false-to-true transition. A retained true also
starts recording when the node joins late. Repeated states do nothing.
Transient-local history survives subscriber reconnects while the ROS2 publisher
is alive; after a bridge publisher restart, the upstream state must arrive again
(for example through a latched ROS1 publisher).

False or node shutdown sends SIGINT to the bag's process group and waits up to
15 seconds for finalization. If needed, it escalates to SIGTERM and SIGKILL,
waiting five seconds at each step and logging warnings. Ctrl-C, SIGTERM, and tmux
hangup go through cleanup. SIGKILL cannot run cleanup. Unexpected bag exits are
logged; toggle false then true to retry.

## ROS2 topic contract

The ROS1 receiver keeps the external ExoSuit topic names and payloads unchanged.
The ROS2 bridge applies the semantic names at its publisher boundary:

| Previous topic | Synthetic Data Collection ROS2 topic |
| --- | --- |
| `/kuavo_arm_traj` | `/robot/command/arm_joint_target` |
| `/control_robot_hand_position` | `/robot/command/hand_target` |
| `/isaac/joint_states` | `/robot/state/joint_states` |
| `/head_camera/image_raw` | `/sensors/camera/head/image_raw` |
| `/left_hand_camera/image_raw` | `/sensors/camera/left_wrist/image_raw` |
| `/right_hand_camera/image_raw` | `/sensors/camera/right_wrist/image_raw` |
| `/clock` | `/clock` |
| `/data_collection/recording` | `/data_collection/recording` |

Arm targets remain `sensor_msgs/msg/JointState` with the original ExoSuit units
(degrees by default). Hand targets remain `dynamic_biped/msg/RobotHandPosition`.
`arm_control_node` and `eff_control_node` consume the semantic command topics in
the Isaac environment. They retain the existing conversion and mapping, then
publish `/robot_arm_joint_command`, `/left_hand_joint_command`, and
`/right_hand_joint_command` for the saved Articulation Controllers. These internal
compatibility topics carry mapped joint angles; connecting the raw bridge
messages directly to the Articulation Controllers would bypass that conversion.

Open `models/robots/s40_sharpa/assembly/s40_sharpa_two_cameras.usd` for the current
scene. Its `Control_ActionGraph` camera helpers publish the three semantic image
topics; `Data_collection_ActionGraph` publishes `/robot/state/joint_states`.
Only their topic-name attributes change; `/clock`, graph wiring, camera frames,
optics, timestamps, and robot parameters are preserved. Reopen the saved scene
after updating it; an already-open stage may retain its previous layer.

The existing launch scripts use these new defaults without ROS1 changes.
`ROS2_ARM_TOPIC`/`ROS2_HAND_TOPIC` configure bridge outputs;
`ARM_INPUT_TOPIC`/`HAND_INPUT_TOPIC` configure matching controller inputs.
For a temporary legacy deployment, explicitly set both ends to the old names
(or use ROS2 `--ros-args -r NEW_TOPIC:=OLD_TOPIC` on both nodes). Configure the
recorder's `topics` parameter to match that temporary deployment as well.
Aliases are opt-in so duplicate command publishers are not introduced.

The standalone `control/ros*_hand_command_subscriber.py` scripts and
`tests/test_left_hand_ros2_publisher.py` intentionally retain the old topic as
legacy `Int32MultiArray` mapper smoke tests. They are not launched by the
production scripts and must not publish that different type on the semantic
`RobotHandPosition` topic. No rosbag-to-Zarr converter or BFM training code is
currently present in this repository; the table also defines the name mapping
for historical bags when converters are added.

The default recorded topics include raw inputs, executed commands, optional
BFM decisions/task state, recording state, and Isaac Sim observations:

```text
/robot/command/hand_target
/robot/command/arm_joint_target
/data_collection/recording
/robot/state/joint_states
/sensors/camera/head/image_raw
/sensors/camera/left_wrist/image_raw
/sensors/camera/right_wrist/image_raw
/clock
/robot_arm_joint_command
/right_hand_joint_command
/bfm/action
/bfm/state
```

All twelve topics are recorded independently at their native rates using the
explicit MCAP storage backend (`ros2 bag record --storage mcap`). The complete
robot JointState (`name`, `position`, `velocity`, and `effort`) is retained.
Public arm targets retain ExoSuit-side degrees; only the existing arm controller
converts its internal joint commands to radians. Historical bags need no migration.

## Timestamp contract

**Cross-topic synchronization uses the rosbag2 recorded message timestamp.**
The future BFM rosbag-to-Zarr converter must use the timestamp returned with
each stored message by rosbag2 (the record/log timestamp), rather than treating
message headers as a shared clock.

**`message.header.stamp` remains source-native and is preserved unchanged.**
The public ExoSuit command headers retain their ROS1 source timestamps, often
Unix-like seconds (for example, approximately `1792639761`). Isaac observation
headers use simulation time (for example, approximately `9` seconds). This
domain difference is expected. `/clock` is recorded unchanged; Bool recording
control messages have no header. Neither headers nor record timestamps are
rewritten or normalized by this node, and no simulation-clock override is added
to rosbag2. Synchronization/resampling belongs in the BFM converter.

Every `episode.json` explicitly records:

```json
"time": {
  "synchronization_source": "rosbag_record_timestamp",
  "header_stamp": "source_native"
}
```

## Episode metadata and camera configuration

Each new recording is self-contained under the existing `output_root`:

```text
records/record_YYYYMMDD_HHMMSS_microseconds_random/
  episode.json
  bag/
    metadata.yaml
    *.mcap
```

rosbag2 creates `bag/` itself; its output directory is initially absent so the
sidecar does not conflict with rosbag2's nonempty-output-directory checks.
`episode.json` is saved before starting rosbag2. Metadata errors prevent a bag
from starting. A failed startup may leave an episode directory with metadata
but no completed bag; errors are logged and require a false/true cycle to retry.

The episode records `source`, `task_id`, `instruction`, `success`, `robot`,
`scene`, the timestamp policy, exact recorded topic list, and MCAP storage ID.
`instruction` is the episode's language task instruction for future BFM/VLA
conditioning, not a per-frame ROS topic or an encoded embedding.
`task_id` is a stable machine-readable identifier. Defaults are:

```json
{
  "source": "sim",
  "task_id": "sorting_food",
  "instruction": "Sort the food items into their corresponding containers.",
  "success": null,
  "robot": "kuavo_s40_sharpa",
  "scene": "s40_sharpa_two_cameras.usd"
}
```

Override the task defaults for each actual demonstration. The string fields
are ROS2 parameters, read at the start of each episode; changes during a bag do
not rewrite that episode's metadata. `source` also accepts identifiers such as
`real` for later use. Annotate `success` in the sidecar after recording; it starts
as `null`. Folder timestamps and `created_at` are wall-clock identifiers, not
cross-topic synchronization inputs.

The packaged [camera_metadata.json](isaac_eff_control/camera_metadata.json)
describes all three cameras from the saved repository USD, including its manual
camera adjustments. Each episode embeds the snapshot's `cameras` and
`camera_configuration` provenance, with repository commit and scene/model hashes.
The saved graph provides 640x480 resolution, virtual perspective optics, and
rigid poses relative to `zhead_2_link`, `zarm_l7_link`, or `zarm_r7_link`.
Extrinsics describe the USD camera frame (+X right, +Y up, -Z forward) relative
to that link, using translation in meters and a `wxyz` camera-to-link quaternion.
They imply no alignment to a ROS optical frame.

These are saved simulation parameters, not measured physical calibration or a
live-stage query (`runtime_verified` is false). ROS frame IDs, calibrated camera
matrices, and distortion parameters are unavailable and explicitly `null`.
The ideal focal lengths in pixels are derived from the saved focal length,
apertures, and render resolution; no principal point or distortion is invented.
In particular, the head camera's original MJCF FOV was an uncalibrated default.
Supply a matching JSON snapshot through `camera_metadata_path` when the active
scene, session overrides, or physical calibration differs. Preserve its
`provenance` object and three camera entries with the current ROS topic names;
unknown calibration fields may remain `null`. The recorder copies this data
without altering the USD, optics, graph, images, or timestamps.

## Build and launch in the existing Isaac Sim container

The package declares `std_msgs`, `ros2bag`, `rosbag2_transport`,
`rosbag2_storage_default_plugins`, and `rosbag2_storage_mcap` dependencies.
Ensure these are installed in the existing Isaac container. If the MCAP plugin
is missing, install `ros-jazzy-rosbag2-storage-mcap` there.

```bash
cd /workspace/Synthetic_data_collection_with_ExoSuit/exo2isaacsim
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install \
  --packages-select dynamic_biped isaac_eff_control
source install/setup.bash
ros2 bag record --help
cd ..
bash start_isaac_control.sh
```

The launcher opens hand-controller, arm-controller, and recorder panes with the
same ROS domain/discovery settings. BFM mode adds one gateway pane. It does not
build or start a container.
Build both selected packages: `isaac_eff_control` depends on the local
`dynamic_biped` interfaces, and building only the former can leave an unknown
`dynamic_biped` package at runtime.
Stop an active recording before replacing a tmux session, allowing metadata to
finish writing before launching another recorder.

Startup parameters can also be supplied when running the recorder directly
(stop the existing recorder first). Omitting `topics` uses all twelve defaults
above; setting `topics` replaces the complete list:

```bash
ros2 run isaac_eff_control sim_recorder_node --ros-args \
  -p output_root:=/workspace/Synthetic_data_collection_with_ExoSuit/records \
  -p task_id:=sorting_food \
  -p 'instruction:=Sort the food items into their corresponding containers.' \
  -p source:=sim \
  -p robot:=kuavo_s40_sharpa \
  -p scene:=s40_sharpa_two_cameras.usd \
  -p stop_timeout_sec:=15.0
```

While stopped, task parameters can also be changed for the next episode:

```bash
ros2 param set /sim_recorder task_id sorting_food
ros2 param set /sim_recorder instruction 'Sort the food items into their corresponding containers.'
```

Each bag uses a new `record_YYYYMMDD_HHMMSS_microseconds_random` output name;
timestamps use the container's local clock. The default `records/` directory is
Git-ignored. Customize `topics` to change the selection. To change the control
topic, use
`-r /data_collection/recording:=/your/control/topic` and update the recorded topic
list accordingly.

## Verify the current Docker bridge

The current deployment is ROS1 bridge container -> ZMQ -> ROS2 bridge container
-> DDS -> Isaac container. The bridge code and architecture are unchanged by
these recorder additions. From a sourced ROS2 terminal in the same domain:

```bash
ros2 topic echo /data_collection/recording \
  --qos-reliability reliable --qos-durability transient_local
```

Radxa X-button transitions should
appear as `data: false`, `data: true`, `data: false`, and so on.

`start_isaac_bridge.sh` is a historical host-installed-Isaac helper, retained
locally and added to `.gitignore`; the current Docker deployment does not use
it. Ignore rules do not untrack an already tracked script. To stop tracking it
without deleting the local file, optionally run
`git rm --cached start_isaac_bridge.sh` yourself.

For an isolated recorder test, with the bridge stopped to avoid competing state
publishers, run each command from a sourced ROS2 terminal in the same domain:

```bash
ros2 topic pub --once --qos-reliability reliable --qos-durability transient_local \
  /data_collection/recording std_msgs/msg/Bool '{data: true}'
ros2 topic pub --once --qos-reliability reliable --qos-durability transient_local \
  /data_collection/recording std_msgs/msg/Bool '{data: false}'
```

True should log the child PID, topics, and new bag path. Repeated true must not
start a second process. False should log a clean exit and leave `metadata.yaml`
and MCAP files in `bag/`, with `episode.json` alongside it. Inspect the path
printed by the recorder using the standard rosbag2 command:

```bash
ros2 bag info /workspace/Synthetic_data_collection_with_ExoSuit/records/record_NAME/bag
```

To test retained state, leave the bridge running with state true, restart only
the recorder, and confirm it starts without another button press.

## Tests without ROS or Isaac Sim

From the repository root:

```bash
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover \
  -s tests -p test_recording_pipeline.py -v
PYTHONDONTWRITEBYTECODE=1 python3 -m unittest discover \
  -s tests -p test_bfm_goal_region.py -v
```

These tests mock ROS, ZMQ, and subprocesses. Real DDS durability and rosbag
metadata finalization require the manual checks above.
The camera/saved-graph and task-sampler checks run when OpenUSD (`pxr`) is
available; otherwise those checks are skipped. PhysX views are mocked in the
sampler test; no host-only test claims an Isaac runtime check.

## Optional BFM demonstration collection

The default `BFM_COLLECTION_MODE=0` retains the existing direct teleoperation
pipeline, semantic topic names, side mapping, degree conversion, and 50 Hz
Sharpa interpolation. Enable the additional mode after rebuilding both
`dynamic_biped` and `isaac_eff_control` with the build command above:

```bash
cd /workspace/Synthetic_data_collection_with_ExoSuit
BFM_COLLECTION_MODE=1 bash start_isaac_control.sh
```

The bridge already publishes the semantic raw topics; no bridge, ROS1, USD,
camera, or ActionGraph change is needed. Stop an active bag before replacing
the tmux session. The launcher supplies the same mode to both controllers,
the recorder, and the gateway, including with an already running tmux server.
Set `BFM_COLLECTION_MODE=0` to return to direct control. Existing legacy topic
overrides still work in normal mode; the gateway also honors `HAND_INPUT_TOPIC`
for a configured raw hand source.

```text
ROS1 -> existing ZMQ bridge -> raw ROS2 arm + hand targets
  arm -> existing arm_control_node (side mapping, degrees -> radians)
      -> /bfm/source/arm_joint_target_rad --------+
  raw RobotHandPosition ------------------------+-> gateway @ 10 Hz
                                                       |
                      +--------------------------------+-------------------+
                      v                                v                   v
             /robot_arm_joint_command      /bfm/command/hand_target     /bfm/action
                      |                                |
             existing Isaac graph          existing eff_control_node @ 50 Hz
                                                       |
                                         /{left,right}_hand_joint_command
                                                       |
                                               existing Isaac graph
```

Source callbacks only update buffers. Every 100 ms a timer snapshots the
latest valid full arm and full hand targets, forwards them, and publishes the
same right-side values as `/bfm/action`. Between policy ticks the high-level
targets stay held; the low-level hand trajectory continues. In BFM mode the
arm mapper always publishes to the intermediate topic and the hand controller
always consumes the held topic, preventing a direct-source bypass even if
`ARM_OUTPUT_TOPIC`/`HAND_INPUT_TOPIC` were configured for normal mode. Stop any
other direct controller instances before collecting; ROS remaps or a competing
publisher can still bypass this routing.

The held hand publisher/subscriber use reliable QoS in BFM mode; the bridge's
high-rate raw arm/hand streams and normal hand input retain best-effort QoS.

No decision is forwarded until both valid sources exist. Invalid/incomplete
input is logged and blocks new decisions until replaced by valid input; the
previous target remains held. There is no added source timeout/watchdog.
Separate ROS publications are not an atomic transaction or a physics execution
acknowledgement. The gateway is the actual command path, not a logging mirror.

**BFM policy rate = 10 Hz does not mean Isaac simulation rate = 10 Hz.**
Physics, joint states, cameras, task state, and the hand controller retain
their native/faster rates. Nothing here resamples or aligns those streams.

| Role | Topic | Payload |
| --- | --- | --- |
| Raw arm/provenance | `/robot/command/arm_joint_target` | JointState, ExoSuit degrees by default |
| Raw hand/provenance | `/robot/command/hand_target` | RobotHandPosition, six 0..100 channels per hand |
| Canonical action | `/bfm/action` | dynamic_biped/BfmAction, 13D |
| Measured robot state | `/robot/state/joint_states` | Full simulator JointState, including measured Sharpa joints |
| Measured task state | `/bfm/state` | dynamic_biped/BfmState, configured frame |
| Executed arm/debug | `/robot_arm_joint_command` | Full mapped radians, held at 10 Hz in BFM mode |
| Low-level hand/debug | `/right_hand_joint_command` | Interpolated physical joint targets, not the canonical action |

`BfmAction` has `std_msgs/Header header`, `string[7] right_arm_joint_names`,
`float64[7] right_arm_target`, and `float64[6] right_hand_target`. Arm values
are absolute joint-position targets, extracted by name **after existing side
mapping** in the explicit order `zarm_r1_joint` through `zarm_r7_joint`.
Hand values preserve `right_hand_position[0..5]` in controller units `0..100`.
The existing controller maps channel 0 to thumb MCP/IP, 1 to thumb CMC FE/AA,
and 2..5 to index/middle/ring/pinky. Metadata uses the precise array-index
order. No hand deg2rad, binary gripper, or [-1,1] normalization is introduced.
Left arm/hand values are held too, but are not part of the canonical 13D action.

New action headers use the gateway ROS clock (wall time by default, simulation
time if `use_sim_time` is explicitly enabled). Forwarded command headers are
preserved, including the existing mapped-arm timestamp behavior. Raw input
headers are unchanged. Task-state headers use simulation time. The timestamp
contract above remains authoritative: cross-topic synchronization uses the
rosbag2/MCAP recorded-message timestamp, not a shared header clock.

### Moving sorting goal: scene contract

The measured state uses the packaged
[sorting_scene.json](isaac_eff_control/sorting_scene.json). It was measured from
`models/stages/stage_sorting_v1.usd` composed at commit
`f6f8dbd4c8aa96a41fa388e71dd8ea4b849d3eb2`, including referenced assets and
scene-level child transforms. Exact runtime paths are:

```text
robot root: /World/s40_sharpa_two_cameras
right EEF:  /World/s40_sharpa_two_cameras/Geometry/base_link/zarm_r1_link/zarm_r2_link/zarm_r3_link/zarm_r4_link/zarm_r5_link/zarm_r6_link/zarm_r7_link/zarm_r7_end_effector/right_hand_C_MC
object:     /World/orange1
container:  /World/container_b04
reference:  /                         (state_frame = world)
```

These are active Xforms; object/container have RigidBodyAPI in the composed
stage. The EEF is the physical right palm, not its visual mesh. It has
RigidBodyAPI in the robot's existing `Physics=physx` variant, inspected in an
anonymous session layer without saving a USD. A plain OpenUSD load does not
select that variant automatically. The runtime sampler requires actual rigid
bodies in the active Isaac scene and never selects or authors variants itself.
Use the existing working PhysX import/variant workflow before starting it.
Root/world is stationary, Z-up, with metersPerUnit=1 in this scene.

No scene, mesh, collider, drive, or ActionGraph file is changed. No prim paths
are discovered by fuzzy names at runtime. All three exact physics views must
resolve to **one** live body each; runtime errors/zero or multiple matches
stop publication. USD schema checks here do not prove live PhysX resolution.
The check and the resolved counts are logged on the first successful sample.

```text
orange PhysX state -----------+
                              |
right palm PhysX state -------+--> /bfm/state (native simulation updates)
                              |
container PhysX pose/twist     |
      +                       |
goal local OBB ---------------+
```

### Interior OBB geometry and object centre

The goal is an **oriented bounding box of allowed orange-skin centre positions**,
not the container's exterior bounds or a world AABB. Inspection found the
inner floor at mesh-local Z=0.010474558 m with a flat rectangle approximately
X=+/-0.169402 m, Y=+/-0.118981 m, and the rim at Z=0.119978175 m. A conservative
interior box was chosen at X=+/-0.160 m, Y=+/-0.110 m, Z=[0.011,0.118] m.
None of the 5,776 triangulated mesh faces has an AABB intersecting that box.
Thus the box avoids walls, floor, rounded edges, and rim rather than treating
external mesh extents as free space.

The OrangeSkin enclosing radius measured from its vertices is 0.038224021 m.
Round up to 0.039 m and add a 0.003 m margin, then erode the horizontal
faces and ceiling by 0.042 m. For the lower centre height use inner floor plus
the skin's minimum support radius (0.034265208 m from its convex hull), minus
0.003 m contact tolerance. A full sphere inset at the bottom would incorrectly
exclude an orange resting on the floor. The upright resting centre is about
0.047823 m, inside the selected goal region. The composed container mesh has translation -0.08494214117424082 m
on body-local X and identity rotation. Applying it gives these fixed SI values
in the container rigid-body frame:

```text
goal_local_center      = [-0.08494214117424082, 0, 0.05886988293797252] m
goal_half_extents      = [0.118, 0.068, 0.017130117062027467] m
goal_local_orientation = [0, 0, 0, 1]                      xyzw
```

The local orientation is identity because the inspected mesh/body axes agree.
The usable centre height is about 0.041740..0.076 m above the container body origin.
Mesh dimensions, transforms, hashes, clearance, and method are stored in the
JSON provenance. Runtime code contains no sorting-specific geometry numbers.
This is a geometric region, not a live collision/success certificate: the
asset uses convex decomposition, whose cooked cavity must still be checked
inside Isaac.

The scene translates OrangeSkin beneath `/World/orange1`, so its rigid-body
origin is not the fruit centre. `object_position` measures the live PhysX
centre of mass, obtained from the live body pose and `get_coms()`;
`object_velocity` is the measured COM linear velocity. No fixed object pose or
local centre is configured. Object orientation is used internally to transform
the measured COM offset, but no object orientation/angular-velocity fields are
published. The separately offset stem already in the asset is excluded from
the skin-based goal clearance; its geometry/collision setup is not repaired.

### State schema, motion, frames, and configuration

`dynamic_biped/BfmState` now contains:

```text
std_msgs/Header header
float64[3] right_eef_position
float64[3] object_position
float64[3] object_velocity
float64[3] goal_center
float64[3] goal_half_extents
float64[4] goal_orientation
float64[3] goal_linear_velocity
float64[3] goal_angular_velocity
```

Positions/half extents use m; linear velocities use m/s; angular velocity uses
rad/s. `goal_orientation` is a normalized **[x,y,z,w]** quaternion mapping the
goal-local axes into `header.frame_id`. Half extents stay along those local
axes; they are unchanged as the container rotates. Header time remains
simulation time, while **MCAP record timestamp remains authoritative for
cross-topic synchronization**. Raw input timestamps and action semantics are
unchanged. The old `goal_position` field is removed; rebuild message users.

For each native update the sampler reads EEF, orange, and container poses and
orange/container twists from PhysX tensors. For a body-local goal offset c:

```text
r_world    = R_container * c
goal_center_world = p_container + r_world
q_goal_ref = q_ref_from_world * q_container * q_goal_local
v_goal_world = v_container_origin + omega_world cross r_world
```

PhysX linear velocities are at the centre of mass. The implementation reads
`get_coms()` and first calculates
`v_origin = v_COM - omega cross R_container*COM_local`. This matters because
this container's auto-computed COM need not equal its body origin. The orange uses its live COM position and linear velocity directly. World vectors
and orientation are then rotated into the configured **static** reference;
reference translation affects positions only. Angular velocity is never
scaled as a linear length. Half extents are already physical meters even if
another stage uses centimeters.

The reference must be right-handed and unit-scale, with no animated/rigid-body
ancestors. Moving reference frames are unsupported. Live dynamic poses are
never read from authored USD transforms or commands, so Fabric disabling USD
pose writeback does not substitute initial poses for measurements.

Configuration precedence is packaged scene JSON, then environment, then ROS
parameter overrides. Set the same configuration for the recorder and the
in-process publisher:

| Environment | ROS parameter | Default |
| --- | --- | --- |
| `BFM_SCENE_CONFIG` | `scene_config_path` | packaged `sorting_scene.json` |
| `BFM_STATE_FRAME` | `state_frame` | `world` |
| `BFM_REFERENCE_PRIM_PATH` | `reference_prim_path` | `/` |
| `BFM_EEF_PRIM_PATH` | `eef_prim_path` | exact right-palm path above |
| `BFM_OBJECT_PRIM_PATH` | `object_prim_path` | `/World/orange1` |
| `BFM_CONTAINER_PRIM_PATH` | `container_prim_path` | `/World/container_b04` |
| `BFM_GOAL_LOCAL_CENTER` | `goal_local_center` | local OBB centre above |
| `BFM_GOAL_HALF_EXTENTS` | `goal_half_extents` | local half extents above |
| `BFM_GOAL_LOCAL_ORIENTATION` | `goal_local_orientation` | identity xyzw |

Vector environment overrides are JSON arrays; none are needed for this scene.
ROS vectors use double arrays. All local geometry is validated; zero
quaternions, non-finite values, and nonpositive half extents fail clearly.
`BFM_GOAL_PRIM_PATH`/`goal_prim_path` are rejected with a migration error: use a
container rigid-body path plus local OBB geometry. A static old goal frame is
not silently reinterpreted as a moving container.

The launcher forwards this configuration to the recorder. The active Isaac
process must inherit it or receive matching function/ROS parameter overrides.
Scene files are read at node startup; restart both publisher and recorder after
changing JSON. The publisher snapshots its ROS parameters at startup, while
the recorder snapshots its values per episode. Keep them identical. Metadata
records configured geometry and provenance; it does not claim live validation.
The existing top-level `scene` episode parameter remains configurable; the
BFM task-state provenance explicitly identifies the composed sorting stage.

### Build, launch, and inspect inside Isaac

This is a breaking `BfmState` type update, not an action-contract change. Stop
the old state publisher (`bfm_state_handle.stop()`), rebuild both packages,
and source the new interfaces in the existing Isaac environment:

```bash
cd /workspace/Synthetic_data_collection_with_ExoSuit/exo2isaacsim
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --packages-select dynamic_biped isaac_eff_control
source install/setup.bash
ros2 interface show dynamic_biped/msg/BfmState
cd ..
BFM_COLLECTION_MODE=1 bash start_isaac_control.sh
```

Isaac's Python needs the same built packages and compatible Jazzy rclpy. If it
already loaded the old generated message, restart that Isaac process so it
loads the new type. Use the same ROS domain/discovery environment as before.
No container is created by the launcher.

Load the current sorting stage and use its working PhysX variant. In Script
Editor, with default scene paths (no initial world XYZ to enter):

```python
from isaac_eff_control.bfm_task_state import start_task_state_publisher

# If already running: bfm_state_handle.stop()
bfm_state_handle = start_task_state_publisher(bfm_collection_mode=True)
```

Supply `scene_config_path` or individual parameter keywords for a different
scene/reference. The publisher runs inside the active Isaac process, using
its own rclpy context and a native Kit post-update callback while Play advances.
It is independent of the 10 Hz action timer. Stop/Play recreates physics views;
invalid roles, failed reads or a changed stage disable publication with an
error, never fabricated zeros. Call `stop()` before re-running the helper.
After Play, inspect the live resolution (all three counts must equal one):

```python
print(bfm_state_handle.node.sampler.resolved_body_counts)
```

This runtime hook is started each Isaac session and changes neither persistent
camera preparation nor USD/ActionGraph wiring. Its live checks cannot be
certified by host-only OpenUSD or mocked-PhysX tests.

### BFM episode metadata and runtime checks

Existing task/instruction, camera snapshot/provenance, timestamp policy, MCAP
backend, episode directory layout, and clean-stop behavior are preserved.
Every episode adds `bfm.enabled`, `policy_hz=10`, `policy_dt=0.1`, `action_dim=13`,
explicit action names/order/units, and task-state paths/frame/units, xyzw
convention, moving-OBB local geometry, selected scene-config hash/provenance,
and the explicit object COM convention. Normal-mode episodes have `bfm.enabled=false`.
Task-state configuration is provenance,
not proof of runtime validity: `configured` indicates supplied paths and
`runtime_verified` remains false. No synthetic observations are generated when
configuration or sampling is invalid. The twelve-topic default retains all
eight original topics; absent optional BFM topics contribute no messages. `topics` remains
configurable. Nothing filters measured JointState or modifies camera streams.

Verify on the actual Isaac/ROS2 environment:

```bash
ros2 topic hz /bfm/action
ros2 topic echo /bfm/action
ros2 topic echo /robot_arm_joint_command
ros2 topic echo /bfm/command/hand_target
ros2 topic hz /right_hand_joint_command
ros2 topic echo /bfm/state --qos-reliability best_effort
ros2 topic info /robot_arm_joint_command --verbose
```

Expect approximately 10 Hz canonical decisions with matching executed
right-side targets, 50 Hz low-level hand output, and native-rate task state
while Play advances. Confirm the gateway is the sole arm command publisher
and the hand controller consumes only the held hand topic. Move the object
and check measured position/velocity. Translate the container: goal centre
must follow. Rotate it: goal centre (because offset) and orientation must follow,
with appropriate linear/angular velocity; half extents must stay fixed.
Move the robot hand and verify EEF position. Start/stop via the existing recording
Bool and inspect `ros2 bag info record_NAME/bag` and `episode.json`. Real
Isaac PhysX/DDS/MCAP execution requires these checks; mocks cannot certify it.
