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

The default recorded topics include actions, recording state, and Isaac Sim
observations:

```text
/control_robot_hand_position
/kuavo_arm_traj
/data_collection/recording
/isaac/joint_states
/head_camera/image_raw
/left_hand_camera/image_raw
/right_hand_camera/image_raw
/clock
```

Messages retain their existing simulation timestamps. Synchronization and
resampling are deferred to the later rosbag2-to-Zarr conversion.

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
