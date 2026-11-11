# S40 / Sharpa self-collision V2 audit and validation

Baseline: `cd300b91730bcaf9581f5494f12b361854712a27` (`main`, the GitHub
remote default branch, fetched at the start of this task).
Audit completed before implementation. No live Isaac/PhysX execution is claimed.

## Composition and confirmed cause

The working scene is `models/stages/v1_manipulation_one_side_right/stage_sorting_v1.usd`. It references
`models/robots/s40_sharpa/assembly/s40_sharpa_two_cameras.usd`, which composes
`s40_sharpa_two_cameras/payloads/base.usda`, `robot.usda`, `instances.usda`,
`geometries.usd` and `materials.usda`. The `Physics=physx` variant loads
`payloads/Physics/physx.usda`, whose sublayer is `physics.usda` (all paths under
the same assembly directory). Both the composed scene and instance-proxy
colliders were inspected with OpenUSD, not just the text payloads.

The saved V1 robot has **no Physics variant selection**. Its unselected stage
has collision shapes but no robot rigid bodies/articulation. The intended active
variant is `physx`, also required by `sorting_scene.json` and existing tests.
The running operator/session may select it; this is not saved by the current
launcher or extension. Inspecting only the unopened/unselected asset would miss
its physics settings. The audit selected `physx` in an anonymous session layer.

With that selection, the sole robot articulation root is:

`/World/s40_sharpa_two_cameras/fix_base_joint`

It is a world-fixed joint with `PhysicsArticulationRootAPI` and the authored
`NewtonArticulationRootAPI` declaration. Its `physics:body1` targets
`/World/s40_sharpa_two_cameras/Geometry/base_link`. The strongest authored
self-collision opinion is `newton:selfCollisionEnabled=false`, from
`payloads/Physics/physics.usda`; neither `PhysxArticulationAPI` nor an explicit
`physxArticulation:enabledSelfCollisions` is authored. There is no stronger
self-collision override in the saved sorting scene.

This is a concrete disabling configuration, not a missing cross-hand filter.
Current NVIDIA documentation explains that PhysX consumes authored Newton
self-collision as a fallback when an explicit PhysX setting is absent. An explicit
PhysX setting takes priority. A raw PhysX schema default of true is therefore not
evidence that this imported robot self-collides. Runtime engine selection and
actual contact generation still need confirmation on the installed Isaac build.
The project uses `omni.physx`, PhysX-specific joint attributes, the SimulationManager
physics tensor view, and existing articulation controllers; PhysX is intended.

## Collider coverage and filtering

The selected variant has **81 enabled dynamic rigid bodies, 81 joints (72
revolute, 9 fixed), 127 enabled collision shapes**. All bodies are connected by
80 body-to-body edges plus the world joint. No second robot articulation or
collision group was found. No robot body is kinematic/disabled.

| Components | Existing collision geometry |
| --- | --- |
| Torso/base | One Cube proxy on `Geometry/base_link/Box` |
| Both legs | 12 primitive proxies, retained |
| Shoulder/arm links 1-4, each side | Cube, Cylinder, Capsule, Capsule (8 total) |
| Arm links 5-7, each side | No directly owned colliders; neighbouring proxies cover only part of their visual volume |
| Camera mounts, heads, end-effector spacer links | No directly owned colliders |
| Palms `left_hand_C_MC`, `right_hand_C_MC` | 32 existing convex mesh pieces each |
| Thumbs | Four primitive colliders per hand; small MCP spacer has no collider |
| Index/middle/ring/pinky articulations | Four primitive colliders per finger |
| Pinky metacarpals | One convex mesh each |

There are 66 Mesh colliders using `physics:approximation=convexHull`, 38 Capsules,
18 Cylinders and 5 Cubes. All mesh points/indices were present, finite and in
range; primitive dimensions were positive. Each collider resolves to an enabled
rigid-body owner. Instance-proxy meshes are included. Existing transforms,
scales and collision/visual geometry are retained; cube scaling is intentional.
This verifies composition/geometry validity, not cooked PhysX hull fidelity.

The proxies are simplified rather than exact visual surfaces. For example,
base-local torso collision bounds are approximately `[-.10,-.12,0]` to
`[.10,.12,.30]` m, whereas the visual mesh spans `[-.145,-.186,-.126]` to
`[.132,.186,.460]` m. No directly owned shape exists on either wrist's links
5-7. Visual-shell penetration can therefore remain even when all authored
colliders respond correctly. Adding visual convex hulls/de-instancing wrists or
inflating the torso without collision-pair testing could introduce unavoidable
housing intersections; this patch does not guess new geometry. Full visual
coverage is a known limitation, not a claim of acceptance.

There are **52 explicit filtered pairs on 34 bodies**, entirely within the same
hand. They exclude neighbouring finger/palm mechanisms and selected neighbouring
MCP housings. None excludes left versus right, arm versus torso, or the two arms.
Keep them: deleting these exclusions can introduce constraints between geometry
that is mechanically intended to overlap. PhysX also filters directly connected
parent/child articulation links automatically. No broad collision-group filter
needs removal. Opposite-hand finger contacts become eligible under the enabled
root without touching their existing shapes.

## Contact, solver and controller audit

- No authored PhysX contact/rest offsets, articulation solver iteration overrides,
  or PhysX timestep on the composed `/PhysicsScene`. Its authored attribute set
  is empty. Stage timecodes are 60/s; that is not proof of runtime physics dt.
  Query the live scene before comparing simulations. No global physics tuning
  is changed here.
- Twelve imported PhysicsMaterial entries have static/dynamic friction 1.0.
  Newton contact stiffness/damping and rolling/torsional values exist too;
  these must not be mistaken for measured PhysX compliant-contact behavior.
- Authored body masses range from 0.00008 to 17.6816 kg; explicit inertias are
  positive, ranging down to about 2.1e-9 kg m^2. Camera/spacer bodies without
  explicit mass use solver-derived/default properties. The large mass/inertia
  ratios warrant live stability checks; no evidence justifies changing them now.
- Arm angular drive stiffness is 500/500/300/300/150/150/150, damping 20 on both
  sides. Hand drives have stiffness about 0.9-13.2 and damping 0.1. Drives are
  force-type; no finite maxForce is authored (schema fallback is infinity).
  Joint limits, armature and friction remain exactly as imported. The base is
  world-fixed; no root placement/control topology change is made.
- The ActionGraph receives arm JointState targets on `/robot_arm_joint_command`
  and hand JointState targets on `/left_hand_joint_command` and
  `/right_hand_joint_command`, wired to the existing articulation controllers.
  The gateway clamps named absolute arm targets; the hand executor converts
  logical 0..100 targets to mapped joint positions at its existing cadence.
  These are joint targets, not Cartesian collision-aware commands. Controllers
  can continue pushing into an obstacle; enabling contacts cannot certify safe
  commands, zero penetration at arbitrary speed, or bounded contact force.

No gains, force limits, masses, timestep, solver iterations or offsets are tuned
without live evidence. Large target jumps and unlimited drives remain risks.

## Implemented change and V1 preservation

`models/stages/sorting_experimental/stage_sorting_v2.usda` is a small independent scene root,
sublayering V1 with relative paths. It selects `Physics=physx` and authors only:

- `prepend apiSchemas=["PhysxArticulationAPI"]` on the existing fixed-base root;
- `physxArticulation:enabledSelfCollisions=true` there;
- `newton:selfCollisionEnabled=true` there, avoiding a contradictory fallback.

It does not duplicate geometry, add bodies/joints/scenes, change filtering or
alter the ActionGraphs. V1 and every shared USD dependency are unchanged. The
V2 flags are stronger than the payload's false opinion. No simulation topology
changes are performed during resets.

The collection extension previously accepted only V1's exact filename. Its
allowlist now also accepts this exact V2 path, so cameras, physics clock, measured
state, randomization and episode lifecycle continue through the same code. This
is the only necessary connected runtime change. V1 remains the launch default.
V2 episodes add `physics_scene_override` (relative layer path, SHA-256, enabled
flag) through existing episode serialization; no recorder or ROS message format
is changed. V1 episode metadata is unchanged. Existing base-scene provenance is
retained; the new field identifies the effective V2 override explicitly.

Static regression tests compare the complete composed authored attributes,
relationships, prim paths/types/instancing, joints/limits/drives, cameras, graph
connections and topics against PhysX-selected V1. Only the two self-collision
attributes, root API declaration and variant selection are intended differences.
These checks cannot prove live articulation indices or physical contact forces.

## Local application and host tests

In a clean local checkout of the baseline (do not discard unrelated changes):

```bash
git rev-parse HEAD
# Must print cd300b91730bcaf9581f5494f12b361854712a27
git status --short
git apply --check self_collision_v2.patch
git apply self_collision_v2.patch
python3 -m pytest -q tests/test_self_collision_v2.py
```

The static tests require OpenUSD Python (`usd-core` or Isaac's Python) and pytest.
The existing full host suite also uses NumPy and pytest-subtests. No dependency
is added by this patch. The local robot/fruit/container assets resolve in host
OpenUSD. Three pre-existing NVIDIA remote environment references/payloads (desk,
warehouse and extras) cannot load in this host; they require the existing Isaac
asset access for full scene/runtime validation. No replacement assets are added.

## Required live Isaac validation (not executed here)

1. Start the existing `bash launch_bfm_isaac.sh`. While STOPPED use File > Open
   to open `models/stages/sorting_experimental/stage_sorting_v2.usda`. Do not add it as another
   reference to the already-open scene. Verify the collection extension attaches
   and prepares one episode. V1 remains independently openable.
2. Start the usual `bash start_isaac_control.sh` for teleoperation, OR the existing
   policy-control launcher with the external policy; do not run both command
   sources. PLAY/PAUSE/STOP remain manual.
3. Before PLAY, run this read-only Script Editor check:

```python
import omni.usd
from pxr import PhysxSchema, UsdPhysics
s = omni.usd.get_context().get_stage()
r = s.GetPrimAtPath('/World/s40_sharpa_two_cameras')
a = s.GetPrimAtPath('/World/s40_sharpa_two_cameras/fix_base_joint')
assert r.GetVariantSet('Physics').GetVariantSelection() == 'physx'
assert a.HasAPI(UsdPhysics.ArticulationRootAPI)
assert a.HasAPI(PhysxSchema.PhysxArticulationAPI)
assert PhysxSchema.PhysxArticulationAPI(a).GetEnabledSelfCollisionsAttr().Get()
assert a.GetAttribute('newton:selfCollisionEnabled').Get()
```

4. Confirm the engine in Isaac's physics UI/log is PhysX. Enable Physics Debug
   collision-mesh and contact visualization. After PLAY, inspect the existing
   SimulationManager view in Script Editor (read only):

```python
from isaacsim.core.simulation_manager import SimulationManager
v = SimulationManager.get_physics_simulation_view()
assert v is not None
valid = v.is_valid
assert valid() if callable(valid) else valid
robot = v.create_articulation_view('/World/s40_sharpa_two_cameras/fix_base_joint')
assert robot.count == 1
print(robot.shared_metatype.dof_names)
print('physics dt:', SimulationManager.get_physics_dt())
```

5. Compare V1 (manually selected `physx`) and V2 from the same initial pose,
   commands and runtime dt. Wait motionless first; then slowly test left arm/torso,
   right arm/torso, arm/arm, palm/palm, opposite-hand fingertips, grasp and bimanual
   handover. Test limits gradually using the existing controls. Inspect actual
   collision shapes/contact points, not just visual meshes. Stop on instability;
   do not force targets through contact. Automatic parent-child and existing
   filtered pairs should remain quiet. Use the Robot Self-Collision Detector
   to diagnose unexpected non-adjacent housing overlaps rather than disabling
   whole-arm or cross-hand contact.
6. Record max collider penetration/separation from contact debugging, resting
   jitter, joint excursions/tracking error, grasp/transfer outcome and physics
   performance/RTF. Repeat slow and normal command rates, pause/resume and STOP
   resets. Apply a consistent documented penetration tolerance in physical
   metres before declaring acceptance; this patch does not invent a passed value.
7. Check `ros2 topic hz /robot/state/joint_states`, `/bfm/state`, `/clock`, each
   camera topic and the controller command topics while moving. Confirm live DOF
   names AND ordering against V1. If recording, inspect episode.json and
   `/bfm/episode` for the V2 override identity. No old recordings are rewritten.

**All physical acceptance scenarios remain unverified**: static stability,
arm/torso and arm/arm response, palm/finger contact, grasp, handover, joint-limit
behavior under load, performance and live controller tracking. This is a corrected
self-collision configuration for validation, not a collision-free-control system
or a certification that data collection is ready for real-robot transfer.

## Sources

- [NVIDIA Newton-schema consumption and precedence](https://docs.omniverse.nvidia.com/kit/docs/omni_physics/latest/dev_guide/schemas/newtonSchema.html)
- [PhysxArticulationAPI population/fallback contract](https://nvidia-omniverse.github.io/PhysX/ovphysx/0.7.0/population/PhysxArticulationAPI.html)
- [Articulation stability and automatic parent/child filtering](https://docs.omniverse.nvidia.com/kit/docs/omni_physics/110.0/dev_guide/guides/articulation_stability_guide.html)
- [Isaac collision-pair inspection](https://docs.isaacsim.omniverse.nvidia.com/6.0.0/openusd_tuning_tutorials/tutorial_04_collider_pairs.html)

Documentation versions describe the configuration mechanism; installed-version
parser behavior and the effective live backend must still be checked in Isaac.
