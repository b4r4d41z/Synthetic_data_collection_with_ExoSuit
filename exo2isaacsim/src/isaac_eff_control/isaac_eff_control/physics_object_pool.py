"""Live pose/velocity reset of existing actors, borrowing the manager's view.

Isaac Sim v6.1.0 RigidPrim calls set_transforms(pose, indices) and
set_velocities(twist, indices). Raw poses use xyzw, stage-length units;
twists use stage-length/s followed by world angular velocity in rad/s.
No SimulationView, articulation, graph, USD opinion or actor is created here.
"""
import math


def host_row(values, size, label):
    """Transfer Warp/Torch to host before indexing any backend array."""
    if hasattr(values, 'detach'):
        values = values.detach()
    if hasattr(values, 'cpu'):
        values = values.cpu()
    if hasattr(values, 'numpy'):
        values = values.numpy()
    row = [float(v) for v in values[0]]
    if len(row) != size or not all(math.isfinite(v) for v in row):
        raise ValueError(f'Invalid live {label}: {row}')
    return row


def native_tensor(template, data, *, indices=False):
    """Use the getter's frontend/device; do not change manager backend settings."""
    module = type(template).__module__.split('.')[0]
    if module == 'warp':
        import warp as wp
        return wp.array(data, dtype=wp.int32 if indices else wp.float32, device=template.device)
    if module == 'torch':
        import torch
        return torch.tensor(data, dtype=torch.int32 if indices else torch.float32,
                            device=template.device).contiguous()
    if module == 'numpy':
        import numpy as np
        return np.ascontiguousarray(data, dtype=np.int32 if indices else np.float32)
    raise TypeError(f'Unsupported physics tensor frontend: {type(template)}')


class PhysicsObjectPool:
    """Own only five local body views and reset buffers, never the shared world."""
    def __init__(self, stage, episode, log=print):
        from pxr import UsdGeom
        self.stage, self.episode, self.log = stage, episode, log
        self.scale = UsdGeom.GetStageMetersPerUnit(stage)
        if not math.isfinite(self.scale) or self.scale <= 0:
            raise ValueError('Invalid stage metersPerUnit')
        self.targets = {path: list(pose) for path, pose in episode['body_poses'].items()}
        expected = {episode['object_prim_path'], episode['container_prim_path'],
                    *episode['parking_positions']}
        if set(self.targets) != expected or len(expected) != 5:
            raise ValueError('Episode must identify four distinct fruits and one container')
        for path, pose in self.targets.items():
            if (len(pose) != 7 or not all(math.isfinite(v) for v in pose)
                    or not math.isclose(math.hypot(*pose[3:]), 1., abs_tol=1e-6)):
                raise ValueError(f'Invalid planned pose for {path}: {pose}')
        if self.targets[episode['object_prim_path']] != [*episode['position'], *episode['orientation_xyzw']]:
            raise ValueError('Selected identity/pose disagrees with episode plan')
        if self.targets[episode['container_prim_path']][:3] != episode['container_position']:
            raise ValueError('Container pose disagrees with episode plan')
        for path, position in episode['parking_positions'].items():
            if self.targets[path][:3] != position:
                raise ValueError(f'Parking pose disagrees with episode plan: {path}')
        self.release()

    def release(self):
        self.simulation_view = None
        self.bodies = {}
        self.buffers = {}
        self.applied = False
        self.verified = False

    def _validate_composition(self, path):
        from pxr import Sdf, Usd, UsdGeom, UsdPhysics
        usd_path = Sdf.Path(path)
        if not usd_path.IsAbsolutePath() or not usd_path.IsPrimPath() or any(c in path for c in '*?[]|'):
            raise ValueError(f'Expected exact physical prim path: {path}')
        prim = self.stage.GetPrimAtPath(path)
        body = UsdPhysics.RigidBodyAPI(prim) if prim else None
        if (not prim or not prim.IsActive() or not body
                or body.GetRigidBodyEnabledAttr().Get() is False
                or body.GetKinematicEnabledAttr().Get() is True):
            raise ValueError(f'Expected enabled dynamic body: {path}')
        if prim.GetAttribute('physxRigidBody:disableGravity').Get() is True:
            raise ValueError(f'Gravity disabled for {path}')
        bodies = [p for p in Usd.PrimRange(prim, Usd.TraverseInstanceProxies())
                  if p.HasAPI(UsdPhysics.RigidBodyAPI)]
        ancestor = prim
        while ancestor and not ancestor.IsPseudoRoot():
            if ancestor.HasAPI(UsdPhysics.ArticulationRootAPI):
                raise ValueError(f'Pool reset may not touch an articulation: {path}')
            if ancestor != prim and ancestor.HasAPI(UsdPhysics.RigidBodyAPI):
                raise ValueError(f'Nested rigid body ancestor for {path}')
            ancestor = ancestor.GetParent()
        if bodies != [prim]:
            raise ValueError(f'Nested rigid bodies under {path}')
        shapes = []
        for child in Usd.PrimRange(prim, Usd.TraverseInstanceProxies()):
            collision = UsdPhysics.CollisionAPI(child)
            if not collision or collision.GetCollisionEnabledAttr().Get() is False:
                continue
            if not UsdGeom.Boundable(child):
                continue
            if child.IsA(UsdGeom.Mesh):
                approximation = UsdPhysics.MeshCollisionAPI(child).GetApproximationAttr().Get()
                if approximation not in ('convexHull', 'convexDecomposition', 'sdf'):
                    raise ValueError(f'Unsupported dynamic collision {approximation}: {child.GetPath()}')
            shapes.append(child)
        if not shapes:
            raise ValueError(f'No enabled collision geometry owned by {path}')

    def bind(self, simulation_view):
        """All five actors pass preflight before any physical state is changed."""
        import numpy as np
        if self.simulation_view is simulation_view:
            return
        self.release()
        bodies, buffers = {}, {}
        for path, target in self.targets.items():
            self._validate_composition(path)
            body = simulation_view.create_rigid_body_view(path)
            if body.count != 1:
                raise ValueError(f'PhysX must resolve exactly one actor: {path}; count={body.count}')
            transforms, velocities = body.get_transforms(), body.get_velocities()
            pose = host_row(transforms, 7, f'pose {path}')
            if not math.isclose(math.hypot(*pose[3:]), 1., abs_tol=1e-3):
                raise ValueError(f'Invalid live quaternion: {path}; pose={pose}')
            host_row(velocities, 6, f'velocity {path}')
            mass = host_row(body.get_masses(), 1, f'mass {path}')[0]
            inertia = np.asarray(host_row(body.get_inertias(), 9, f'inertia {path}')).reshape(3, 3)
            if (mass <= 0 or not np.allclose(inertia, inertia.T, rtol=1e-5, atol=1e-10)
                    or np.linalg.eigvalsh(inertia).min() <= 0):
                raise ValueError(f'Invalid live mass/inertia: {path}; mass={mass}, inertia={inertia.tolist()}')
            physical_pose = [*(v / self.scale for v in target[:3]), *target[3:]]
            buffers[path] = (native_tensor(transforms, [physical_pose]),
                             native_tensor(velocities, [[0.] * 6]),
                             native_tensor(transforms, [0], indices=True))
            bodies[path] = body
        self.simulation_view, self.bodies, self.buffers = simulation_view, bodies, buffers
        self.log('[BFM PHYSX] fruit pool resolved: 4/4; container resolved: 1')

    def apply_episode(self):
        if not self.bodies:
            raise RuntimeError('Bind live physics before applying an episode')
        if self.applied:
            return
        self.log(f"[BFM PHYSX] applying episode {self.episode['episode_id']}")
        for path, body in self.bodies.items():
            pose, velocity, indices = self.buffers[path]
            for label, setter, value in (('pose', body.set_transforms, pose),
                                         ('velocity', body.set_velocities, velocity)):
                try:
                    result = setter(value, indices)
                    if result is not None and not result:
                        raise RuntimeError('Setter returned failure')
                except Exception as error:
                    raise RuntimeError(f'Live reset {label} write failed: {path}: {error}') from error
        self.applied = True

    def verify_episode(self):
        """Read immediately after setters, before another integration step."""
        if not self.applied:
            raise RuntimeError('Apply episode before verifying it')
        for path, target in self.targets.items():
            body = self.bodies[path]
            pose = host_row(body.get_transforms(), 7, f'reset pose {path}')
            pose[:3] = [v * self.scale for v in pose[:3]]
            velocity = host_row(body.get_velocities(), 6, f'reset velocity {path}')
            velocity[:3] = [v * self.scale for v in velocity[:3]]
            norm = math.hypot(*pose[3:])
            orientation_ok = (math.isclose(norm, 1., abs_tol=1e-4)
                              and abs(sum(a*b for a, b in zip(pose[3:], target[3:])) / norm) >= 1.-1e-6)
            if (any(abs(a-b) > 1e-4 for a, b in zip(pose[:3], target[:3]))
                    or not orientation_ok or max(abs(v) for v in velocity) > 1e-5):
                raise ValueError(f'Live reset verification failed: {path}; desired_pose={target}, '
                                 f'measured_pose={pose}, desired_velocity={[0.]*6}, measured_velocity={velocity}')
        self.verified = True
        self.log('[BFM PHYSX] reset verified')

    def parking_settled(self):
        """After reset, allow physical settling in the existing remote cradles."""
        if not self.verified:
            raise RuntimeError('Verify reset before collecting state')
        settled = True
        for path, target in self.episode['parking_positions'].items():
            body = self.bodies[path]
            pose = host_row(body.get_transforms(), 7, f'parking pose {path}')
            velocity = host_row(body.get_velocities(), 6, f'parking velocity {path}')
            if any(abs(p*self.scale-t) > .20 for p, t in zip(pose[:3], target)):
                raise ValueError(f'Parked fruit escaped: {path}; target={target}, measured={pose}')
            settled &= (max(abs(v*self.scale) for v in velocity[:3]) <= .005
                        and max(abs(v) for v in velocity[3:]) <= .005)
        return settled
