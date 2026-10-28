# Synthetic Data Collection with ExoSuit

This project is part of my Master's thesis on behavior foundation models for humanoid manipulation.

The goal of this project is to use a physical ExoSuit to control a humanoid robot in NVIDIA Isaac Sim and collect synthetic manipulation data.

## Normal collection launch

In the existing Isaac container, with Jazzy, the built ROS2 workspace, tmux and
Isaac Sim installed at `/isaac-sim`, run these two commands in separate terminals.
No environment prefixes or manual sourcing are required. Build dependencies once
using the [setup guide](exo2isaacsim/src/isaac_eff_control/README.md#build-and-launch-in-the-existing-isaac-sim-container).

```bash
# Terminal 1
cd /workspace/Synthetic_data_collection_with_ExoSuit
bash launch_bfm_isaac.sh
```

```bash
# Terminal 2 (launch last)
cd /workspace/Synthetic_data_collection_with_ExoSuit
bash start_isaac_control.sh
```

Open `models/stages/stage_sorting_v1.usd` while STOPPED and wait for the
extension's plan-ready message before PLAY. The existing ROS1/ZMQ/ROS2 bridge
must be running in its deployment; these scripts consume its semantic topics.
They do not create another bridge or rebuild the workspace.

The attached tmux `main` window has three equal horizontal panes:
**ARM / JOINTS**, **HAND / GRIPPERS**, **RECORDING**, in that order. The actual
arm controller, hand controller and recorder each run once in these panes.
ARM/HAND show WAITING, ACTIVE on valid input, STOPPED after one second without
input, and ERROR for malformed input. Only transitions are printed. HAND watches
the raw semantic bridge input for status, because held gateway targets continue
when input stops; interpolation remains at 50 Hz. ACTIVE describes command flow,
not measured robot motion. Recording shows WAITING, RECORDING with the episode
name, STOPPED after finalization/sidecar save, or ERROR on failure.

Use normal tmux navigation to reach `backend`: the existing bounded BFM gateway
runs there, beside detailed rosbag output (`records/rosbag2.log`). Warnings and
errors remain visible. Ctrl-C in a worker pane forwards SIGINT and waits for
cleanup; the recorder finalizes its MCAP and outcome. Relaunching the control
script interrupts all old workers and waits up to 40 seconds before replacing
the session; if cleanup is still running it preserves the session and fails.
Unexpected worker exits are retained and announce ERROR in the main window.

Both scripts internally fix collection mode to enabled, semantic inputs,
degree conversion, domain 0 and SUBNET discovery, ignoring inherited production
overrides. Scene defaults and episode-generated descriptors remain authoritative.
Lower-level configuration APIs remain available for debugging outside these
production scripts. Contract v3, bounded action semantics, cameras, physics and
task behavior are unchanged.

## ExoSuit Control

The physical ExoSuit is used to control the humanoid robot in real time.

![ExoSuit robot control](assets/2attempt_isaacsim.gif)

The same control system is used inside a simulated manipulation environment in Isaac Sim.

![Robot control in Isaac Sim](assets/attempt_isaacsim.gif)

## ExoSuit to Isaac Sim

The physical ExoSuit sends arm and hand commands through ROS 1. These commands are transferred through a ROS1-to-ROS2 bridge and then sent to the control nodes connected to the robot model in Isaac Sim.

Arm commands from the ExoSuit are converted into joint targets for the simulated humanoid. Hand commands are mapped to the corresponding finger joints of the simulated robotic hands.

This allows the physical ExoSuit to control the humanoid model in real time and provides a simple way to collect manipulation demonstrations in simulation.

## System Architecture

The Synthetic Data Collection ROS2 interface uses `/robot/command/...` for
ExoSuit commands, `/robot/state/...` for measured state, and `/sensors/...` for
camera observations. External ROS1 topic names are preserved. See the
[topic contract and recording setup](exo2isaacsim/src/isaac_eff_control/README.md#ros2-topic-contract)
for the full mapping, internal controller topics, and legacy configuration.

![ExoSuit to Isaac Sim pipeline](assets/Frame%205.png)