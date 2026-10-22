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

The default recorded topics include actions, recording state, and Isaac Sim
observations:

```text
/robot/command/hand_target
/robot/command/arm_joint_target
/data_collection/recording
/robot/state/joint_states
/sensors/camera/head/image_raw
/sensors/camera/left_wrist/image_raw
/sensors/camera/right_wrist/image_raw
/clock
```

All eight topics are recorded independently at their native rates using the
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
same ROS domain/discovery settings. It does not build or start a container.
Build both selected packages: `isaac_eff_control` depends on the local
`dynamic_biped` interfaces, and building only the former can leave an unknown
`dynamic_biped` package at runtime.
Stop an active recording before replacing a tmux session, allowing metadata to
finish writing before launching another recorder.

Startup parameters can also be supplied when running the recorder directly
(stop the existing recorder first). Omitting `topics` uses all eight defaults
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
The saved-graph topic test runs when OpenUSD (`pxr`) is available, as in Isaac
Sim; otherwise that single test is skipped.
