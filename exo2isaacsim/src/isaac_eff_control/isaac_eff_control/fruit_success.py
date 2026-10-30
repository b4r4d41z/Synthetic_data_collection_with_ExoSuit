"""Conservative collision-bound containment and simulation-time dwell; no writes."""
import itertools
import math

TOPIC = '/bfm/success'


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
    def __init__(self, polygon, floor, ceiling):
        self.polygon, self.floor, self.ceiling = polygon, floor, ceiling
        self.since = self.last = None

    def update(self, seconds, corners, speeds):
        if not math.isfinite(seconds) or seconds < 0 or len(corners)!=8 or len(speeds)!=4:
            raise ValueError('Invalid success sample dimensions/time')
        if not all(math.isfinite(v) for p in corners for v in p) or not all(map(math.isfinite, speeds)):
            self.since = self.last = None
            raise ValueError('Nonfinite success measurement')
        inside = all(self.floor-.003 <= p[2] <= self.ceiling-.003 and all(
            ((b[0]-a[0])*(p[1]-a[1])-(b[1]-a[1])*(p[0]-a[0])) /
            math.hypot(b[0]-a[0], b[1]-a[1]) >= .003
            for a,b in zip(self.polygon,self.polygon[1:]+self.polygon[:1])) for p in corners)
        stable = all(v <= limit for v,limit in zip(speeds,(.02,.2,.02,.2)))
        if self.last is None or seconds < self.last or seconds-self.last > .05 or not inside or not stable:
            self.since = seconds if inside and stable else None
        self.last = seconds
        dwell = seconds-self.since if self.since is not None else 0.
        return bool(inside and stable and dwell >= .3-1e-9), dict(
            method='collision_bound_in_floor_prism_and_settled_v1', contained=inside,
            stable=stable, stable_duration_sim_sec=dwell, simulation_seconds=seconds,
            linear_limit_m_s=.02, angular_limit_rad_s=.2, required_dwell_sim_sec=.3)


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
        relative = fruit*container.GetInverse()
        corners = [relative.Transform(Gf.Vec3d(*p)) for p in self.corners]
        success,audit = self.detector.update(seconds,corners,a+b)
        return dict(episode_id=self.episode_id, object_prim_path=self.path, success=success,
                    speed_norms_linear_angular_fruit_container=a+b,
                    collision_bound_body_min=[min(p[i] for p in self.corners) for i in range(3)],
                    collision_bound_body_max=[max(p[i] for p in self.corners) for i in range(3)],
                    interior_floor_polygon=self.detector.polygon,
                    interior_floor=self.detector.floor, interior_ceiling=self.detector.ceiling,
                    **audit)
