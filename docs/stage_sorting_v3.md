# Stage Sorting V3

Baseline: `main`, `d4f666eac0ea939dedfa526c2f0a8863e49fb52e`.

Open `models/stages/stage_sorting_v3.usda` as a stage. It directly sublayers
`stage_sorting_v2.usda` and adds `things_pool.usda`. V2, V1, `fruit_pool.usda`,
all source assets, the robot, cameras, graphs and table remain unchanged.
Only V3's initial composition disables the inherited fruit bodies/cradles.
No body is activated, deleted, re-referenced or created during episode resets.

## Pool and physics

Three bodies exist before simulation starts:

| Body under `/World/Task/Things` | Referenced source | Mass |
| --- | --- | --- |
| `apple_02` | `models/addons/objects/objaverse/apple_02.usd` | 0.16602 kg |
| `lunchbag` | `models/addons/objects/objaverse/lunchbag.usd` | 0.28447 kg |
| `ketchup_bottle` | `models/addons/objects/hope/ketchup_bottle.usd` | 0.5 kg |

Apple and lunchbag have a child rigid body in their source USD. The pool layer
promotes that body to its reference root, disabling only the child body/mass
APIs. It preserves geometry transforms, source mass and inertia, and transforms
the centre of mass and principal axes into the root frame. There are no nested
bodies. Ketchup already has a root body. Original metre/Z-up dimensions,
colliders, convex-decomposition settings, visual materials and texture paths
remain referenced. No geometry or textures are copied or rescaled.

Gravity is enabled, initial velocities are zero, and the existing fruit contact
settings are reused: friction 0.6/0.5, restitution 0, contact/rest offsets
0.002/0 m. Unselected bodies use three references to the existing static remote
parking cradles. Runtime reset still uses the existing live PhysX pose/velocity
setters, preflight, readback and parking-settle checks, now for three things plus
the existing container.

## Layouts and reproducibility

V3 uses `sorting_scene_v3.json`; the existing default configuration is unchanged.
The extension recognizes the actual current V2 filename as well as V3. V3
selects a thing and A/B layout once per plan using the existing seeded RNG:

* A: thing on the right, container on the left.
* B: thing on the left, container on the right.

Right-side footprint centres use the existing rectangle: x `[0.95, 1.025]`,
y `[3.99, 4.12]` m. Left-side centres use the captured container collision
footprint centre plus the existing x/y offsets `[-0.06, 0.06]` m. These are
candidate-centre regions, not new table boundaries. Using the footprint centre
also accounts for the container's offset mesh origin.

Both roles are sampled independently within their assigned regions. The
existing rotated object bounds, support height, 25 mm edge margin, 30 mm
obstacle clearance and 5 mm object release clearance are retained. Container
orientation/Z remain the template values. V3 also rejects environmental
colliders outside the support table and the task bodies/cradles. Placement
fails explicitly after the configured attempt limit, preserving the previous
plan; it never accepts an intersecting fallback.

The episode descriptor remains the authority for the selected physical path.
It additionally records `scenario`, `layout`, both `placement_regions`, and the
V3 `scene_configuration` (configuration digest and pool/scene provenance).
Existing episode serialization carries these into recordings; recorder and
state/action message formats are unchanged. UUID episode IDs remain unique;
with the same seed and scene geometry, layout and pose sequences repeat.

## Validation

Host tests load the actual V3 composition, sources, robot and container, check
texture resolution, body ownership, mass frames, both layouts, seeded
randomization, rejected overlaps/support failures, unchanged inherited scene
content, extension attachment and the existing lifecycle/live-reset interface.
They use synthetic tabletop triangles and fake PhysX actors. The NVIDIA remote
table/environment payloads and Isaac physics are unavailable on the host, so
these tests do not certify real support geometry, contacts or settling.

In the Isaac container, rebuild the existing ROS package and restart Isaac so
the new packaged configuration is installed:

```bash
cd /workspace/Synthetic_data_collection_with_ExoSuit/exo2isaacsim
source /opt/ros/jazzy/setup.bash
colcon build --packages-select isaac_eff_control --symlink-install
cd ..
bash launch_bfm_isaac.sh
```

The launcher still opens V2. While STOPPED, use File > Open to open
`models/stages/stage_sorting_v3.usda`. Start the unchanged
`bash start_isaac_control.sh` in the second terminal. Confirm `[BFM PLAN READY]`,
then PLAY manually. Expect `things pool resolved: 3/3; container resolved: 1`
and `reset verified`. Inspect the authoritative layout/identity/pose metadata:

```bash
ros2 topic echo /bfm/episode --field data --qos-durability transient_local
```

Repeat STOP/PLAY across at least 20 episodes. Check all three objects and both
layouts, randomized positions, supported/nonoverlapping placement, gravity,
table/robot contacts, parking settling, arm/hand control and recording startup.
PAUSE/PLAY must retain the same episode. Open V2 separately to confirm its
original fruit selection and right-object/left-container behavior. These live
checks remain required; no Isaac simulation validation is claimed.
