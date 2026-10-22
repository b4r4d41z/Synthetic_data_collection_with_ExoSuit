# Simulation recording control

The existing ROS1 bridge forwards `std_msgs/Bool` on
`/data_collection/recording` over its shared ZMQ connection as
`{"kind": "recording", "enabled": true}` (or `false`). The existing host-side
ROS2 publisher republishes `std_msgs/msg/Bool` with reliable, transient-local,
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

Messages retain their existing simulation timestamps. Synchronization and
resampling are deferred to the later rosbag2-to-Zarr conversion. This exact
topic selection is passed to rosbag2, including when its storage backend is MCAP;
recording lifecycle and storage configuration are unchanged.

## Build and launch in the existing Isaac Sim container

The package declares `std_msgs`, `ros2bag`, `rosbag2_transport`, and
`rosbag2_storage_default_plugins` dependencies. Ensure these are installed in
that environment.

```bash
cd /workspace/Synthetic_data_collection_with_ExoSuit/exo2isaacsim
source /opt/ros/jazzy/setup.bash
colcon build --packages-select isaac_eff_control
source install/setup.bash
ros2 bag record --help
cd ..
bash start_isaac_control.sh
```

The launcher opens hand-controller, arm-controller, and recorder panes with the
same ROS domain/discovery settings. It does not build or start a container.
Stop an active recording before replacing a tmux session, allowing metadata to
finish writing before launching another recorder.

Startup parameters can also be supplied when running the recorder directly
(stop the existing recorder first). Omitting `topics` uses all eight defaults
above; setting `topics` replaces the complete list:

```bash
ros2 run isaac_eff_control sim_recorder_node --ros-args \
  -p output_root:=/workspace/Synthetic_data_collection_with_ExoSuit/records \
  -p stop_timeout_sec:=15.0
```

Each bag uses a new `record_YYYYMMDD_HHMMSS_microseconds_random` output name;
timestamps use the container's local clock. The default `records/` directory is
Git-ignored. Customize `topics` to change the selection. To change the control
topic, use
`-r /data_collection/recording:=/your/control/topic` and update the recorded topic
list accordingly.

## Update and verify the existing bridge

The ROS1 image copies its Python script rather than bind-mounting it. On the
bridge host, from the repository root, update the existing container without
creating a replacement:

```bash
docker cp docker/exo_ros_bridge/ros1/bridge/ros1_receiver.py \
  exo_ros1_bridge:/bridge/ros1_receiver.py
docker restart exo_ros1_bridge
```

Restart the Python publisher in its existing host ROS2 publisher pane, after
stopping the previous process. Keep the sourced ROS environment and discovery
settings from `start_isaac_bridge.sh`, and run:

```bash
/usr/bin/python3 docker/exo_ros_bridge/ros2/bridge/ros2_publisher.py
ros2 topic echo /data_collection/recording \
  --qos-reliability reliable --qos-durability transient_local
```

Run the echo in a separate sourced terminal. Radxa X-button transitions should
appear as `data: false`, `data: true`, `data: false`, and so on.

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
and storage files in that path. Validate the exact path printed by the recorder:

```bash
ros2 bag info /workspace/Synthetic_data_collection_with_ExoSuit/records/record_NAME
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
