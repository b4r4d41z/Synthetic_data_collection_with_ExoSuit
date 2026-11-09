# Diagnostic MCAP replay and BFM rollout capture

Requires the existing built Jazzy workspace, rosbag2 MCAP plugin, and this
checkout in the existing Isaac container. No rebuild, new package or Docker
image is required. Both new shell scripts reuse `tools/collection/environment.bash`.
They run source tools directly; existing launchers and collection nodes are unchanged.

## Command ownership

| Workflow | Command publishers |
| --- | --- |
| ExoSuit collection | Existing `start_isaac_control.sh`, unchanged |
| External policy + rollout capture | `start_bfm_policy_control.sh` executes hands; external BFM owns arm/action/logical hands; recorder only subscribes |
| Direct replay | Replay alone owns arm and both executed-hand joint topics |

Never run the policy, ExoSuit/control launcher, or hand executor alongside replay.
Replay checks for conflicting command publishers before and during publication;
DDS discovery is asynchronous, so this is detection, not an ownership lock.

## Replay

Terminal 1: `bash launch_bfm_isaac.sh` (manual PLAY/PAUSE/STOP).
Terminal 2, from the repository root:

```bash
bash replay_bfm_bag.sh /absolute/path/to/record_directory --validate-only
bash replay_bfm_bag.sh /absolute/path/to/record_directory
```

Accepts either `episode.json` + `bag/` or `capture.json` + `bag/` (split MCAPs
supported). Full command payloads are validated before publishing. Required
nonempty streams are `/robot_arm_joint_command`, `/left_hand_joint_command`,
`/right_hand_joint_command` (`sensor_msgs/msg/JointState`, radians), and `/clock`.
No logical/raw-command fallback is provided. Arm bounds use the existing model
contract; hand names match the current executed mapping. Recorded values,
ordering within each message and headers are not modified or clamped.

Replay waits for the existing simulator `/bfm/episode` playing status, advancing
live `/clock`, and controller subscribers. It does not inspect or modify fruit,
container or task identities. The stage must expose the existing lifecycle channel
as well as the expected robot/control interfaces. Recorded result events on
`/bfm/episode` are ignored. No simulator or additional controller is started.

Timing uses a causal join: each command's MCAP system receive timestamp maps to
the preceding recorded `/clock` sample. Before/after clock coverage, the nearest
endpoint is used. Timing is therefore quantized by recorded clock delivery and
DDS receive jitter; source/header clocks are not assumed compatible. Equal mapped
times retain MCAP receive order. Commands during recorded clock freezes can share
a deadline; no values are silently dropped. Live physics time paces deadlines,
not wall sleeps or real-time factor. PAUSE freezes progress; resume continues;
STOP, epoch change or backward clock terminates. A missed command deadline over
0.5 simulation seconds aborts instead of sending a stale catch-up sequence.

Exactly one live clock publisher is required. Recorded clock, images, measured
states, raw commands and `/bfm/action` are never republished. Playback ends without
sending a new zero/reset target; existing controllers hold their last target.
No execution acknowledgement is implied. Preflight stores command messages in
memory, so memory grows with command count (images are filtered out).

Replay does not restore recorded initial robot/object state or task randomization.
Set a suitable initial pose manually. Identical commands do not guarantee identical
contacts or trajectories. This is command replay, not a deterministic world replay.

## Interactive policy-rollout recording

Run Isaac, `bash start_bfm_policy_control.sh`, and the external policy normally.
In another interactive terminal at the Synthetic root:

```bash
bash record_bfm_rollout.sh
# Optional metadata and output directory:
bash record_bfm_rollout.sh --checkpoint /path/to/checkpoint --task-z /path/to/task_z --output /path/to/evaluation
```

**Space** arms a new capture on the next advancing physics sample while PLAYING;
Space again stops/finalizes (or cancels an armed start). **q** finalizes and quits.
No Enter is needed. PAUSE keeps the same recording open. STOP/failed/closed or a
changed lifecycle boundary finalizes automatically. A backward clock finalizes
with `clock_reset`. Subsequent PLAY permits another manual capture. Old retained
STOP/result messages cannot start recording. Ctrl-C/SIGTERM finalize and restore
the terminal. Closing the entire container/SIGKILL cannot guarantee finalization.

Outputs are separate from demonstrations:
`records/evaluation/rollout_YYYYMMDD_HHMMSS_RANDOM/{bag/,capture.json,rosbag2.log}`.
Directory creation is exclusive. A managed `ros2 bag record --storage mcap --all
--include-hidden-topics --exclude-regex '^/bfm/recorder_control$'` records
discoverable ROS topics, including topics that
appear later through normal rosbag2 discovery. The private recorder-control topic is excluded because the extension counts its
subscribers and requires their STOP acknowledgements; rosbag must not add a
non-acknowledging subscriber. Public `/bfm/episode` remains recorded.
Very short-lived topics or messages
before discovery/subscription may be missed. It also records unrelated discovered
ROS topics; consider load/privacy before using on a shared ROS domain.

This includes available BFM action/raw arm/logical hands, actual arm/finger targets,
joint states, BFM state/success, all three camera streams, clock, episode and raw
inputs/diagnostics. Nothing unpublished inside PhysX or the external policy is
invented or captured. Recording all topics adds storage/CPU/I/O load.
MCAP log times remain rosbag2 system receive time; message headers stay unchanged.
Capture metadata saves actual discovered/recorded types and counts, start/end
wall and simulation times, reason, software provenance and optional user-supplied
checkpoint/task-z identities (not verified or inferred).

Shutdown first sends SIGINT and allows 20 seconds to flush, then SIGTERM/5 seconds,
then SIGKILL/5 seconds only on failure. Escalation/nonzero exit/missing bag metadata
is reported as failed and potentially incomplete. Inspect `rosbag2.log`. Metadata
is atomically finalized; replay refuses missing essential streams. Complete
captures can be passed directly to `replay_bfm_bag.sh`.

## Validation on the Isaac machine

1. Validate a known demonstration, then replay with only Isaac running. Compare
   live `/robot/state/joint_states` to the three replay command topics. Under low
   RTF replay must progress in simulation time. Test PAUSE/resume, STOP and reset;
   check there are no recorded image/state/clock publishers.
2. With external policy and hand executor running, start the interactive recorder;
   press Space, PAUSE/resume, STOP. Repeat with Space/q and Ctrl-C/SIGTERM.
3. Run `ros2 bag info /path/to/rollout/bag` and inspect `capture.json`/`rosbag2.log`.
   Require nonzero controller, observation, clock, action and camera counts for
   the available streams. Stop all policy/hand publishers and replay that capture.
4. Host tests: `python3 -m pytest -q tests/test_bfm_evaluation_tools.py`.
   Unit tests mock ROS and rosbag processes; they do not prove live DDS delivery,
   MCAP serialization, controller behavior or physical reproducibility.
