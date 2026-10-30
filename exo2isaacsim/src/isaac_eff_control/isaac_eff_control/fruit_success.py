"""Collision-mesh placement with simulation-time confirmation and episode latch."""
import itertools
import math

import numpy as np

TOPIC = '/bfm/success'
METHOD = 'collision_mesh_residence_latched_v2'
DWELL_NS = 300_000_000


def hull(points):
    points = sorted(set(points))
    def cross(o,a,b): return (a[0]-o[0])*(b[1]-o[1])-(a[1]-o[1])*(b[0]-o[0])
    parts = []
    for sequence in (points, list(reversed(points))):
        part = []
        for p in sequence:
            while len(part)>1 and cross(part[-2],part[-1],p)<=0: part.pop()
            part.append(p)
        parts.extend(part[:-1])
    if len(parts)<3: raise ValueError('Missing container floor polygon')
    return parts


class SettledContainment:
    """Confirm sampled residence, independently of update rate or wall time.

    Unobserved intervals between valid contained samples count as residence;
    they are not proof of continuous collision/contact. No callback-rate gate.
    An observed exit/invalid sample breaks qualification. Earned success is an
    episode outcome and survives later exits/invalid samples until reset.
    """

    def __init__(self, polygon, floor, ceiling):
        self.polygon, self.floor, self.ceiling = polygon, floor, ceiling
        edges = np.asarray(polygon, dtype=float)
        if (edges.ndim != 2 or edges.shape[1] != 2 or len(edges) < 3
                or not np.isfinite(edges).all() or not math.isfinite(floor)
                or not math.isfinite(ceiling) or floor >= ceiling):
            raise ValueError('Invalid container interior')
        delta = np.roll(edges, -1, axis=0) - edges
        lengths = np.linalg.norm(delta, axis=1)
        if np.any(lengths == 0):
            raise ValueError('Degenerate container interior edge')
        self.normals = np.column_stack((-delta[:, 1], delta[:, 0])) / lengths[:, None]
        self.offsets = np.sum(edges * self.normals, axis=1)
        self.reset()

    def reset(self):
        """New episode/physics epoch: discard qualification AND earned success."""
        self.since_ns = self.last_ns = self.first_success_ns = None

    def invalid_sample(self, reason):
        """Break pending residence; never invent success from missing data."""
        self.since_ns = None
        return dict(method=METHOD, success=True if self.first_success_ns is not None else None,
                    contained=None, stable=None, measurement_valid=False, reason=str(reason),
                    stable_duration_sim_sec=0., simulation_seconds=None, **self._semantics())

    def _semantics(self):
        return dict(required_dwell_sim_sec=DWELL_NS / 1e9, velocity_gate=False,
                    stability_semantics='sampled_container_residence',
                    success_semantics='episode_success_once',
                    success_latched=self.first_success_ns is not None,
                    first_success_sim_sec=(None if self.first_success_ns is None else
                                           self.first_success_ns / 1e9))

    def update(self, seconds, points, speeds):
        try:
            if not math.isfinite(seconds) or seconds < 0:
                raise ValueError('Invalid success simulation time')
            now_ns = round(seconds * 1_000_000_000)
            # Check rollback even if the new pose later proves invalid.
            if self.last_ns is not None and now_ns < self.last_ns:
                self.reset()
            points = np.asarray(points, dtype=float)
            if (points.ndim != 2 or points.shape[1] != 3 or len(points) < 4
                    or len(speeds) != 4 or not np.isfinite(points).all()
                    or not all(math.isfinite(v) and v >= 0 for v in speeds)):
                raise ValueError('Invalid success geometry/velocity telemetry')
        except (ValueError, TypeError, OverflowError):
            self.invalid_sample('Invalid success sample')
            raise ValueError('Invalid success sample') from None

        # A convex interior contains every mesh triangle (and its convex hull)
        # iff it contains its vertices. No artificial AABB empty-space corners.
        side_clearance = float(np.min(points[:, 0, None] * self.normals[:, 0]
                                      + points[:, 1, None] * self.normals[:, 1] - self.offsets))
        floor_clearance = float(points[:, 2].min() - self.floor)
        rim_clearance = float(self.ceiling - points[:, 2].max())
        inside = floor_clearance >= -.003 and rim_clearance >= .003 and side_clearance >= .003
        interval = None if self.last_ns is None else (now_ns - self.last_ns) / 1e9
        if not inside:
            self.since_ns = None
        elif self.since_ns is None:
            self.since_ns = now_ns
        self.last_ns = now_ns
        dwell_ns = 0 if self.since_ns is None else now_ns - self.since_ns
        qualified = inside and dwell_ns >= DWELL_NS
        if qualified and self.first_success_ns is None:
            self.first_success_ns = now_ns
        return self.first_success_ns is not None, dict(
            method=METHOD, contained=inside, stable=qualified, measurement_valid=True,
            stable_duration_sim_sec=dwell_ns / 1e9, simulation_seconds=seconds,
            sample_interval_sim_sec=interval, collision_vertices_tested=len(points),
            floor_clearance_m=floor_clearance, rim_clearance_m=rim_clearance,
            side_clearance_m=side_clearance, **self._semantics())


class LiveFruitSuccess:
    def __init__(self, stage, config, episode_id):
        from pxr import Usd, UsdGeom, UsdPhysics
        self.episode_id, self.path = episode_id, config['object_prim_path']
        self.container = config['container_prim_path']
        self.scale = UsdGeom.GetStageMetersPerUnit(stage)
        if self.scale != 1.: raise ValueError('Success geometry requires meter stage')
        root = stage.GetPrimAtPath(self.path)
        cache = UsdGeom.XformCache()
        inverse = cache.GetLocalToWorldTransform(root).GetInverse()
        points = []
        for p in Usd.PrimRange(root):
            if p.HasAPI(UsdPhysics.CollisionAPI) and UsdPhysics.CollisionAPI(p).GetCollisionEnabledAttr().Get() is not False:
                if not p.IsA(UsdGeom.Mesh): raise ValueError('Unsupported fruit collision shape')
                transform = cache.GetLocalToWorldTransform(p)*inverse
                points.extend(transform.Transform(v) for v in UsdGeom.Mesh(p).GetPointsAttr().Get())
        if not points: raise ValueError('Fruit collision mesh missing')
        self.vertices = np.unique(np.asarray(points, dtype=float), axis=0)
        self.corners = list(itertools.product(*[(min(p[i] for p in points),max(p[i] for p in points)) for i in range(3)]))
        body = stage.GetPrimAtPath(self.container)
        mesh = stage.GetPrimAtPath(self.container+'/Container_B04_01')
        if not mesh.IsA(UsdGeom.Mesh): raise ValueError('Expected inspected container mesh')
        transform = cache.GetLocalToWorldTransform(mesh)*cache.GetLocalToWorldTransform(body).GetInverse()
        raw = UsdGeom.Mesh(mesh).GetPointsAttr().Get()
        # Inspected top face of the inner floor (32 vertices). Fail closed if
        # this asset changes rather than guessing another cavity.
        floor = [transform.Transform(v) for v in raw if abs(v[2]-.010474557988345623)<1e-6]
        if len(floor)!=32: raise ValueError('Container floor calibration no longer matches mesh')
        polygon = hull([(p[0],p[1]) for p in floor])
        rim = [transform.Transform(v)[2] for v in raw if v[2] > .117]
        if not rim: raise ValueError('Container rim missing')
        self.detector = SettledContainment(polygon, min(p[2] for p in floor), min(rim))
        self.config = config

    def measure(self, sampler, seconds):
        from pxr import Gf
        def row(data):
            if hasattr(data,'detach'): data=data.detach()
            if hasattr(data,'cpu'): data=data.cpu()
            if hasattr(data,'numpy'): data=data.numpy()
            result = list(map(float,data[0]))
            if not all(map(math.isfinite,result)): raise ValueError('Invalid physics success state')
            return result
        def matrix(key):
            view = sampler.views[key]
            pose, velocity = row(view.get_transforms()), row(view.get_velocities())
            if len(pose)!=7 or len(velocity)!=6: raise ValueError('Invalid physics tensor dimensions')
            q = Gf.Quatd(pose[6],Gf.Vec3d(*pose[3:6]))
            if abs(q.GetLength()-1)>1e-5: raise ValueError('Invalid physics rotation')
            m = Gf.Matrix4d(1); m.SetRotate(q); m.SetTranslateOnly(Gf.Vec3d(*pose[:3]))
            return m, [math.hypot(*velocity[:3]),math.hypot(*velocity[3:])]
        fruit, a = matrix('object_prim_path'); container,b = matrix('container_prim_path')
        # Gf matrices transform row vectors; cached vertices include child
        # transforms/scales, while live tensor poses supply the rigid-body pose.
        relative = np.asarray(fruit*container.GetInverse())
        points = self.vertices @ relative[:3, :3] + relative[3, :3]
        success,audit = self.detector.update(seconds,points,a+b)
        return dict(episode_id=self.episode_id, object_prim_path=self.path, success=success,
                    speed_norms_linear_angular_fruit_container=a+b,
                    collision_bound_body_min=[min(p[i] for p in self.corners) for i in range(3)],
                    collision_bound_body_max=[max(p[i] for p in self.corners) for i in range(3)],
                    interior_floor_polygon=self.detector.polygon,
                    interior_floor=self.detector.floor, interior_ceiling=self.detector.ceiling,
                    **audit)
