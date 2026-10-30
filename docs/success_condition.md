# Sorting success: collision_mesh_residence_latched_v2

`/bfm/success` labels **placement achieved during this episode**, independently
of `terminal` and recording stop. It uses the selected episode fruit and the
intended container's live PhysX rigid-body poses. `/bfm/state` is unchanged.

## Spatial condition

Transform every enabled fruit collision-mesh vertex into container-body space:
`p_container = p_body * fruit_world * inverse(container_world)` (Gf row vectors).
Child transforms/scales are incorporated when the vertices are cached. Test the
actual vertices, not synthetic body/world AABB corners containing empty space.
For a convex region, containing the vertices also contains all their triangles
and their convex hull. This supports the existing convex-decomposition assets.
It does not claim to reconstruct PhysX's cooked contact offsets/hulls.

Retain the inspected container's conservative convex floor-hull extrusion to its
lowest rim, the existing 3 mm side/rim inset and 3 mm floor-contact tolerance.
Every vertex must satisfy those planes. Missing/changed calibration fails closed.
This rejects objects beside, above, below, substantially protruding, or in another
container. The conservative extrusion may reject placements at sloping walls;
no geometry or tolerance was expanded to accommodate a recording.

## Qualification and episode outcome

The existing Kit post-update callback samples completed physics time through
`SimulationManager.get_simulation_time()`. It is not guaranteed to run on every
physics step. State publication, state timestamps, `/clock` and policy timing
are unchanged. The success timestamp explicitly names its physics clock domain.

The state machine converts simulation time to integer nanoseconds:

- First contained observation starts pending residence at zero.
- Further contained observations accumulate elapsed simulation time. There is
  no maximum callback-gap cutoff or assumed sampling frequency.
- At 300,000,000 ns, latch success and save `first_success_sim_sec`.
- Identical times add nothing. PAUSE does not evaluate or add wall time.
- Observed exit or invalid measurement breaks **pending** residence.
- Episode/physics reset or backward simulation time clears residence and latch.
- After qualification, later exit/motion/invalid measurement does not erase
  historical success. Invalid data cannot create success: before qualification
  it publishes `success=null`; after qualification it retains true and explicitly
  marks `measurement_valid=false`, `contained=null`, `stable=null`.

Skipped intervals between valid contained observations count as residence.
This is sampled evidence, not proof of unobserved continuous contact: departure
and re-entry entirely between samples cannot be detected. A brief observed
passage under 0.3 s never qualifies. No contact/grasp sensor is introduced;
this label does not certify hand release. A fruit held completely inside for the
required duration qualifies. Linear/angular speed norms remain finite-validated
telemetry in m/s and rad/s, without a magnitude gate.

`contained` describes current geometry. `stable` means the current observed
residence streak has qualified, not instantaneous staticness.
`stable_duration_sim_sec` measures that streak, even after success latches.
`success_latched` and `first_success_sim_sec` describe historical achievement;
`sample_interval_sim_sec` exposes actual callback spacing. No latch is inferred
from `contained` alone. The detector is created once per episode, retained on
PAUSE and normal updates, and explicitly reset with the publisher/physics epoch.

The recorder keeps the latest matching diagnostic and freezes it before waiting
for rosbag shutdown. A v2 latched success is historical evidence and does not
expire solely due to a wall-time STOP delivery delay. Unqualified/legacy samples
still require freshness; identity mismatches, malformed diagnostics and abnormal
bag finalization still require review. No extra recorder latch is introduced:
a rollback diagnostic replaces the earlier success. STOP/reset does not publish
a false diagnostic overwriting the completed episode. Manual stop remains
`terminal=false, end_reason=user_stop`. Old recordings are not rewritten.

## Failure mechanism and research

The baseline restarts residence whenever adjacent sample times differ by more
than 0.05 s. Post-update callbacks have no such upper bound: rendering and
physics rates differ, multiple physics steps can occur per update, and callbacks
can be skipped. Thus even contained zero-velocity samples every 0.06 s produce
zero dwell indefinitely. Using timeline time for physics measurements adds a
second clock assumption. There is no normal-operation detector recreation loop;
the recorder merely freezes its last received diagnostic. The reported message
counts demonstrate active publication, not the individual timestamp differences.
The newer failing recording is not in this baseline, so its exact failing
intervals were not replayed. Host tests reproduce the mechanism generically.

Primary sources consulted:

- [Isaac Sim simulation fundamentals](https://docs.isaacsim.omniverse.nvidia.com/latest/physics/simulation_fundamentals.html): physics, timeline and rendering cadences can differ.
- [SimulationManager API](https://docs.isaacsim.omniverse.nvidia.com/latest/py/source/extensions/isaacsim.core.simulation_manager/docs/index.html): completed simulation time and physics-step count. Time deltas support varying update rates without assuming a fixed number of steps.
- [ManiSkill apple-in-bowl](https://maniskill.readthedocs.io/en/latest/_modules/mani_skill/envs/tasks/humanoid/humanoid_pick_place.html): task-specific spatial placement and hand clearance, not universal absolute speed limits. Its point threshold is not copied for differently sized fruits.
- [ManiSkill episode metrics](https://github.com/mani-skill/ManiSkill/blob/main/mani_skill/utils/wrappers/gymnasium.py): explicitly distinguishes success-once from success-at-end. The former fits this episode outcome label; it need not terminate recording.
- [RLBench groceries task](https://github.com/stepjam/RLBench/blob/master/rlbench/tasks/put_groceries_in_cupboard.py): combines target proximity detection and release. That grasp-aware simulator mechanism is not assumed available for this ExoSuit hand pipeline.
- [Isaac Lab data collection](https://isaac-sim.github.io/IsaacLab/main/source/features/imitation-learning/teleop_imitation.html): task-specific success checks and post-success recording buffer. This implementation retains operator-controlled stopping.

## Live validation still required

Rebuild/restart using the existing workflow (two launch terminals):

```bash
source /opt/ros/jazzy/setup.bash
cd exo2isaacsim
colcon build --packages-select isaac_eff_control --symlink-install
cd ..
bash launch_bfm_isaac.sh
```

```bash
bash start_isaac_control.sh
```

In a ROS-configured diagnostic terminal:

```bash
ros2 topic echo /bfm/success
ros2 topic hz /bfm/success
```

Place each supported fruit completely inside; confirm advancing physics
`simulation_seconds`, increasing dwell, and `success_latched=true` after 0.3 s.
Repeat under rendering load and with PAUSE/PLAY: frozen time adds nothing,
callback gaps do not erase valid residence. Test above-rim, side, protruding and
brief pass-through negatives. After success, subsequent motion must retain
success while current containment can change. STOP/new episode must clear it.
Inspect the new `episode.json`: true outcome after confirmed placement with
`terminal=false`; invalid unqualified data must require review. Check recorded
`/bfm/success` with `ros2 bag info /path/to/new/recording/bag`.

Host tests use actual local USD meshes with synthetic live tensor poses and fake
Kit/ROS events. They are not live PhysX tests or MCAP replay. Isaac/ROS runtime is unavailable here. The baseline MCAP is an LFS pointer in
this clone; the full recording was not downloaded or replayed.
