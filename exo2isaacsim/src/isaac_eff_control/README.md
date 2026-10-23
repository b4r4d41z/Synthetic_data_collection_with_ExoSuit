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

### Measured task state inside Isaac

The recorder does not guess which scene objects are the manipulated object
and goal. No generic object/goal paths are configured by default. Start the
runtime publisher inside the active Isaac Sim process with the built packages
on Isaac's Python path and compatible Jazzy rclpy; a separate external ROS
process cannot read that simulator's live physics. It uses a separate rclpy
context and does not change the existing ROS2 bridge context or ActionGraph.

Use matching configuration for the recorder and the Isaac-side publisher:

| Environment variable | ROS parameter | Default |
| --- | --- | --- |
| `BFM_COLLECTION_MODE` | `bfm_collection_mode` | false |
| `BFM_STATE_FRAME` | `state_frame` | `world` |
| `BFM_REFERENCE_PRIM_PATH` | `reference_prim_path` | `/` (stage world) |
| `BFM_EEF_PRIM_PATH` | `eef_prim_path` | empty, must configure |
| `BFM_OBJECT_PRIM_PATH` | `object_prim_path` | empty, must configure |
| `BFM_GOAL_PRIM_PATH` | `goal_prim_path` | empty, must configure |

The launcher passes these environment values to the recorder. Isaac must
inherit the same values when launched, or receive the matching parameter
overrides in the function below. Set all three role paths to inspected prims
from the actual loaded scene, without inventing coordinates. EEF and object
must be real rigid-body prims in the active PhysX variant. EEF/object position
is the selected body's origin; object velocity is PhysX's measured linear
velocity. Goal may be a rigid body or a static Xform. The reference must be
stationary, right-handed, unit-scale, and have no animated/rigid-body ancestors.
Moving reference frames are explicitly unsupported.

The saved robot has `Physics` variants: sampling validates the currently active
runtime stage. It does not select variants or change the model. An inactive
physics variant or a visual-only mesh path fails clearly. PhysX tensor views
read live positions/velocity even when Fabric disables USD pose writeback;
only static reference/goal transforms are read from USD. Positions and linear
velocities are converted from stage units to m and m/s in the same reference
basis. No command target is used as an observation.

After configuring the shared environment, run in Isaac's Script Editor:

```python
from isaac_eff_control.bfm_task_state import start_task_state_publisher

# On a repeated run first call bfm_state_handle.stop().
bfm_state_handle = start_task_state_publisher(bfm_collection_mode=True)
```

Alternatively pass `eef_prim_path`, `object_prim_path`, `goal_prim_path`,
`reference_prim_path`, and `state_frame` as keyword ROS parameter overrides;
use identical values for the recorder parameters. Empty/invalid paths fail
setup, without publishing fabricated zeros. Keep the handle alive. It samples
once per native Kit post-update with new simulation time while Play advances,
independently of the 10 Hz gateway. Stop/Play recreates physics views; changing
the stage or encountering runtime read errors disables publication with an
error until restarted. Call `bfm_state_handle.stop()` when finished. This hook
is started each Isaac session; camera preparation and saved USD are unchanged.

`BfmState` contains `std_msgs/Header header` and four `float64[3]` arrays:
`right_eef_position`, `object_position`, `object_velocity`, and `goal_position`.
All are measured simulator state, in SI units and the `header.frame_id` basis.

### BFM episode metadata and runtime checks

Existing task/instruction, camera snapshot/provenance, timestamp policy, MCAP
backend, episode directory layout, and clean-stop behavior are preserved.
Every episode adds `bfm.enabled`, `policy_hz=10`, `policy_dt=0.1`, `action_dim=13`,
explicit action names/order/units, and task-state paths/frame/units. Normal-mode
episodes have `bfm.enabled=false`. Task-state configuration is provenance,
not proof of runtime validity: `configured` indicates supplied paths and
`runtime_verified` remains false. No synthetic observations are generated when
task state is unconfigured. The twelve-topic default retains all eight original
topics; absent optional BFM topics contribute no messages. `topics` remains
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
and check measured position/velocity. Start/stop via the existing recording
Bool and inspect `ros2 bag info record_NAME/bag` and `episode.json`. Real
Isaac PhysX/DDS/MCAP execution requires these checks; mocks cannot certify it.
