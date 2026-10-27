"""Pose-only stopped resets of a pre-authored stable rigid-body pool."""

import json
import math
from pathlib import Path
import random
from uuid import uuid4


POOL_PATH = "/World/Task/Fruits"
OBJECT_PATH = POOL_PATH + "/avocado"

def fruit_path(asset):
    return POOL_PATH + "/" + Path(asset).stem

CONTAINER_PATH = "/World/container_b04"


def load_configuration(path, repository):
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    if (data.get("schema_version") != 1 or data["parameters"]["object_prim_path"] != OBJECT_PATH
            or data["parameters"]["container_prim_path"] != CONTAINER_PATH):
        raise ValueError("Sorting collection requires schema 1 and the configured fruit pool")
    config = data["randomization"]
    assets = config["object_assets"]
    # The versioned JSON is the whitelist; no directory scanning or other fruits.
    if not isinstance(assets, list) or len(assets) != 4 or len(set(assets)) != 4:
        raise ValueError("Configure exactly four distinct fruit assets")
    root = Path(repository).resolve()
    for asset in assets:
        resolved = (root / asset).resolve()
        if not resolved.is_relative_to(root) or not resolved.is_file():
            raise ValueError(f"Missing or out-of-repository fruit: {asset}")
    for key in ("object_spawn_region", "container_offset_region"):
        region = config[key]
        values = [region[k] for k in ("x_min", "x_max", "y_min", "y_max")]
        if (not all(isinstance(v, (int, float)) and not isinstance(v, bool)
                    and math.isfinite(v) for v in values)
                or values[0] >= values[1] or values[2] >= values[3]):
            raise ValueError(f"{key} must be a finite, nonempty XY rectangle in meters")
    for key in ("yaw_min", "yaw_max", "surface_tolerance_m", "clearance_m",
                "edge_margin_m", "obstacle_margin_m"):
        value = config[key]
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
            raise ValueError(f"Invalid {key}")
    if config["yaw_min"] > config["yaw_max"] or any(config[k] < 0 for k in (
            "surface_tolerance_m", "clearance_m", "edge_margin_m", "obstacle_margin_m")):
        raise ValueError("Invalid yaw interval or negative placement tolerance")
    if type(config["max_attempts"]) is not int or config["max_attempts"] < 1:
        raise ValueError("max_attempts must be a positive integer")
    if config["seed"] is not None and type(config["seed"]) is not int:
        raise ValueError("seed must be an integer or null")
    return data


def sample_candidate(config, rng):
    """XY is the footprint centre, yaw is radians. Seed once, never per episode."""
    region = config["object_spawn_region"]
    return (rng.choice(config["object_assets"]),
            rng.uniform(region["x_min"], region["x_max"]),
            rng.uniform(region["y_min"], region["y_max"]),
            rng.uniform(config["yaw_min"], config["yaw_max"]))


def collision_shapes(prim):
    from pxr import Usd, UsdGeom, UsdPhysics
    for child in Usd.PrimRange(prim, Usd.TraverseInstanceProxies()):
        if (child.HasAPI(UsdPhysics.CollisionAPI) and UsdGeom.Boundable(child)
                and UsdPhysics.CollisionAPI(child).GetCollisionEnabledAttr().Get() is not False):
            yield child


def asset_geometry(path):
    """Reference the actual sole body at its default prim, preserving materials.

    The four inspected sources all have a body at the default prim. Validate
    this rather than silently introducing a visual wrapper or nested bodies.
    Bounds include every visual/collision mesh, including orange2's stem.
    """
    from pxr import Gf, Usd, UsdGeom, UsdPhysics
    stage = Usd.Stage.Open(str(path))
    root = stage.GetDefaultPrim()
    bodies = [p for p in stage.Traverse() if p.HasAPI(UsdPhysics.RigidBodyAPI)]
    if (not root or bodies != [root]
            or UsdPhysics.RigidBodyAPI(root).GetRigidBodyEnabledAttr().Get() is False
            or UsdPhysics.RigidBodyAPI(root).GetKinematicEnabledAttr().Get() is True
            or not list(collision_shapes(root))):
        raise ValueError(f"Asset must have one dynamic default-prim body with collision: {path}")
    if UsdGeom.GetStageMetersPerUnit(stage) != 1 or UsdGeom.GetStageUpAxis(stage) != "Z":
        raise ValueError(f"Expected meter/Z-up fruit: {path}")
    if UsdGeom.Xformable(root).GetLocalTransformation() != Gf.Matrix4d(1):
        raise ValueError(f"Expected the inspected identity fruit root transform: {path}")
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render", "proxy"])
    bounds = cache.ComputeLocalBound(root).ComputeAlignedRange()
    lo, hi = list(bounds.GetMin()), list(bounds.GetMax())
    if not all(math.isfinite(v) for v in lo + hi) or any(a >= b for a, b in zip(lo, hi)):
        raise ValueError(f"Missing/invalid fruit geometry: {path}")
    return str(root.GetPath()), lo, hi


def tabletop_triangles(stage, path):
    """Read actual composed collision mesh surfaces, never guess table height."""
    from pxr import Gf, Usd, UsdGeom
    prim = stage.GetPrimAtPath(path)
    if not prim or not prim.IsActive():
        raise ValueError(f"Table payload is missing: {path}")
    result = []
    for child in collision_shapes(prim):
        matrix = UsdGeom.Xformable(child).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        if child.IsA(UsdGeom.Mesh):
            mesh = UsdGeom.Mesh(child)
            vertices = mesh.GetPointsAttr().Get()
            indices = mesh.GetFaceVertexIndicesAttr().Get()
            counts = mesh.GetFaceVertexCountsAttr().Get()
        elif child.IsA(UsdGeom.Cube):
            half = UsdGeom.Cube(child).GetSizeAttr().Get() / 2
            vertices = [(x, y, z) for z in (-half, half)
                        for y in (-half, half) for x in (-half, half)]
            indices = [0, 1, 3, 2, 4, 5, 7, 6]
            counts = [4, 4]
        else:
            continue
        points = [matrix.Transform(Gf.Vec3d(p)) for p in vertices]
        offset = 0
        for count in counts:
            face = indices[offset:offset + count]
            offset += count
            for i in range(1, count - 1):
                triangle = [points[j] for j in (face[0], face[i], face[i + 1])]
                normal = Gf.Cross(triangle[1] - triangle[0], triangle[2] - triangle[0])
                if normal.GetLength() > 1e-12 and abs(normal[2]) / normal.GetLength() > .99999:
                    result.append(triangle)
    if not result:
        raise ValueError(f"No horizontal collision mesh surface in {path}; load its payload")
    return result


def surface_height(triangles, x, y):
    heights = []
    for a, b, c in triangles:
        denominator = (b[1]-c[1])*(a[0]-c[0]) + (c[0]-b[0])*(a[1]-c[1])
        if abs(denominator) < 1e-12:
            continue
        u = ((b[1]-c[1])*(x-c[0]) + (c[0]-b[0])*(y-c[1])) / denominator
        v = ((c[1]-a[1])*(x-c[0]) + (a[0]-c[0])*(y-c[1])) / denominator
        if min(u, v, 1-u-v) >= -1e-8:
            heights.append(u*a[2] + v*b[2] + (1-u-v)*c[2])
    return max(heights) if heights else None


def overlaps(lo, hi, other_lo, other_hi, margin):
    return all(hi[i] + margin >= other_lo[i] and lo[i] - margin <= other_hi[i]
               for i in range(3))


class SceneRandomizer:
    def __init__(self, stage, data, repository):
        from pxr import Sdf, UsdGeom
        self.stage, self.data, self.repository = stage, data, Path(repository)
        self.config = data["randomization"]
        if UsdGeom.GetStageMetersPerUnit(stage) != 1 or UsdGeom.GetStageUpAxis(stage) != "Z":
            raise ValueError("Sorting runtime requires a meter/Z-up stage")
        # Required so configured world coordinates also mean task-local coordinates.
        from pxr import Gf, Usd
        world = stage.GetPrimAtPath("/World")
        if not world or UsdGeom.Xformable(world).ComputeLocalToWorldTransform(
                Usd.TimeCode.Default()) != Gf.Matrix4d(1):
            raise ValueError("Sorting /World must have an identity world transform")
        self.rng = random.Random(self.config["seed"])
        self.assets = {a: asset_geometry(self.repository / a) for a in self.config["object_assets"]}
        from pxr import UsdPhysics
        container = stage.GetPrimAtPath(CONTAINER_PATH)
        if (not container or not container.HasAPI(UsdPhysics.RigidBodyAPI)
                or UsdPhysics.RigidBodyAPI(container).GetRigidBodyEnabledAttr().Get() is False
                or not list(collision_shapes(container))):
            raise ValueError("Container must be an enabled rigid body with collision")
        # Capture the template once, never accumulate offsets from the last run.
        self.container_transform = UsdGeom.Xformable(container).ComputeLocalToWorldTransform(
            Usd.TimeCode.Default())
        cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render", "proxy"])
        bounds = [cache.ComputeWorldBound(p).ComputeAlignedRange()
                  for p in collision_shapes(container)]
        self.container_bounds = ([min(b.GetMin()[i] for b in bounds) for i in range(3)],
                                 [max(b.GetMax()[i] for b in bounds) for i in range(3)])
        self.layer = Sdf.Layer.CreateAnonymous("bfm_collection_runtime.usda")
        self.session = stage.GetSessionLayer()
        self.session.subLayerPaths.insert(0, self.layer.identifier)
        self.last_episode = None
        self.pool = {a: fruit_path(a) for a in self.assets}
        self.parking = {}
        try:
            for asset, path in self.pool.items():
                prim = stage.GetPrimAtPath(path)
                self._validate_body(path)
                # Require the pre-authored asset; never compose bodies at runtime.
                if not prim.GetAttribute("xformOp:transform:bfmEpisode"):
                    raise ValueError(f"Missing pre-authored pool transform: {path}")
                bounds = cache.ComputeUntransformedBound(prim).ComputeAlignedRange()
                if any(abs(bounds.GetMin()[i]-self.assets[asset][1][i]) > 1e-6
                       or abs(bounds.GetMax()[i]-self.assets[asset][2][i]) > 1e-6
                       for i in range(3)):
                    raise ValueError(f"Pool geometry differs from configured asset: {path}")
                floor = stage.GetPrimAtPath("/World/Task/Parking/" + Path(asset).stem + "/floor")
                if not floor or not list(collision_shapes(floor)):
                    raise ValueError(f"Missing static parking support for {path}")
                if any(p.HasAPI(UsdPhysics.RigidBodyAPI) for p in Usd.PrimRange(floor)):
                    raise ValueError(f"Parking floor must be static: {path}")
                support = cache.ComputeWorldBound(floor).ComputeAlignedRange()
                center = (support.GetMin() + support.GetMax()) / 2
                self.parking[asset] = [center[0], center[1], support.GetMax()[2] - self.assets[asset][1][2]]
                if any(self.assets[asset][2][i] - self.assets[asset][1][i] >= support.GetSize()[i] - .1
                       for i in (0, 1)):
                    raise ValueError(f"Fruit does not fit its parking cradle: {path}")
        except Exception:
            self.close()
            raise

    def close(self):
        if self.layer.identifier in self.session.subLayerPaths:
            self.session.subLayerPaths.remove(self.layer.identifier)

    @property
    def current_object_path(self):
        if self.last_episode is None:
            raise RuntimeError("No episode prepared")
        return self.last_episode["object_prim_path"]

    def _validate_body(self, path):
        from pxr import Usd, UsdPhysics
        prim = self.stage.GetPrimAtPath(path)
        body = UsdPhysics.RigidBodyAPI(prim) if prim else None
        if (not prim or not prim.IsActive() or not body
                or body.GetRigidBodyEnabledAttr().Get() is False
                or body.GetKinematicEnabledAttr().Get() is True
                or not list(collision_shapes(prim))
                or [p for p in Usd.PrimRange(prim) if p.HasAPI(UsdPhysics.RigidBodyAPI)] != [prim]):
            raise ValueError(f"Prepared role must be one active dynamic body with collision: {path}")

    def randomize(self):
        # Planning completes before touching poses. No layer clear/import or
        # topology mutations, even on rejection. Failed preparation cannot PLAY.
        return self._randomize()

    def _randomize(self):
        from pxr import Gf, Usd, UsdGeom, UsdPhysics
        triangles = tabletop_triangles(self.stage, self.config["table_prim_path"])
        cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), ["default", "render", "proxy"])
        obstacles = []
        # Inspect each robot collider instead of rejecting its enormous combined AABB.
        for path in (self.data["provenance"]["robot_root"],):
            root = self.stage.GetPrimAtPath(path)
            if not root:
                raise ValueError(f"Missing obstacle: {path}")
            shapes = list(collision_shapes(root))
            if not shapes:
                raise ValueError(f"No active collision geometry for obstacle {path}")
            for shape in shapes:
                bound = cache.ComputeWorldBound(shape).ComputeAlignedRange()
                obstacles.append((list(bound.GetMin()), list(bound.GetMax())))
        # Validate the container against live support and robot geometry first.
        region = self.config["container_offset_region"]
        for _ in range(self.config["max_attempts"]):
            offset = [self.rng.uniform(region["x_min"], region["x_max"]),
                      self.rng.uniform(region["y_min"], region["y_max"]), 0.]
            low, high = ([v + offset[i] for i, v in enumerate(bound)]
                         for bound in self.container_bounds)
            margin = self.config["edge_margin_m"]
            heights = [surface_height(triangles, x, y)
                       for x in (low[0]-margin, (low[0]+high[0])/2, high[0]+margin)
                       for y in (low[1]-margin, (low[1]+high[1])/2, high[1]+margin)]
            if any(h is None or abs(h-low[2]) > self.config["surface_tolerance_m"]
                   for h in heights):
                continue  # Preserve template Z: only the same support height is valid.
            if any(overlaps(low, high, a, b, self.config["obstacle_margin_m"])
                   for a, b in obstacles):
                continue
            break
        else:
            raise ValueError("No supported, robot-free container pose in configured offset region")
        transform = Gf.Matrix4d(self.container_transform)
        container_position = self.container_transform.ExtractTranslation() + Gf.Vec3d(*offset)
        transform.SetTranslateOnly(container_position)
        # Use the planned translated bounds; do not move the container until
        # a complete valid fruit/container plan exists.
        obstacles.append((low, high))
        for _ in range(self.config["max_attempts"]):
            asset, x, y, yaw = sample_candidate(self.config, self.rng)
            _, source_lo, source_hi = self.assets[asset]
            rotation = Gf.Rotation(Gf.Vec3d(0, 0, 1), math.degrees(yaw))
            corners = [rotation.TransformDir(Gf.Vec3d(a, b, c))
                       for a in (source_lo[0], source_hi[0])
                       for b in (source_lo[1], source_hi[1])
                       for c in (source_lo[2], source_hi[2])]
            lo = [min(p[i] for p in corners) for i in range(3)]
            hi = [max(p[i] for p in corners) for i in range(3)]
            position = [x-(lo[0]+hi[0])/2, y-(lo[1]+hi[1])/2, 0.]
            margin = self.config["edge_margin_m"]
            xs = [position[0]+lo[0]-margin, x, position[0]+hi[0]+margin]
            ys = [position[1]+lo[1]-margin, y, position[1]+hi[1]+margin]
            heights = [surface_height(triangles, a, b) for a in xs for b in ys]
            if any(h is None for h in heights) or max(heights)-min(heights) > self.config["surface_tolerance_m"]:
                continue
            position[2] = max(heights) - lo[2] + self.config["clearance_m"]
            world_lo = [position[i]+lo[i] for i in range(3)]
            world_hi = [position[i]+hi[i] for i in range(3)]
            if any(overlaps(world_lo, world_hi, a, b, self.config["obstacle_margin_m"])
                   for a, b in obstacles):
                continue
            break
        else:
            raise ValueError("No supported, obstacle-free fruit pose in configured spawn region")
        planned = {path: Gf.Matrix4d(1).SetTranslate(Gf.Vec3d(*self.parking[a]))
                   for a, path in self.pool.items()}
        selected = self.pool[asset]
        planned[selected] = Gf.Matrix4d(1).SetRotate(rotation).SetTranslateOnly(Gf.Vec3d(*position))
        planned[CONTAINER_PATH] = transform
        for path in planned:
            self._validate_body(path)
        # Snapshot property values only. Rollback must never remove physical prims.
        snapshots = []
        with Usd.EditContext(self.stage, self.layer):
            try:
                for path, matrix in planned.items():
                    prim = self.stage.GetPrimAtPath(path)
                    xform = UsdGeom.Xformable(prim)
                    attr = prim.GetAttribute("xformOp:transform:bfmEpisode")
                    op = UsdGeom.XformOp(attr) if attr else xform.AddTransformOp(opSuffix="bfmEpisode")
                    body = UsdPhysics.RigidBodyAPI(prim)
                    attrs = [op.GetAttr(), xform.GetXformOpOrderAttr(),
                             body.CreateVelocityAttr(), body.CreateAngularVelocityAttr()]
                    snapshots.extend((attr, attr.Get()) for attr in attrs)
                    xform.SetXformOpOrder([op])
                    op.Set(matrix)
                    attrs[2].Set(Gf.Vec3f(0)); attrs[3].Set(Gf.Vec3f(0))
                # Detect stronger USD/writeback opinions rather than announcing a
                # prepared pose that physics would never receive.
                for path, matrix in planned.items():
                    prim = self.stage.GetPrimAtPath(path)
                    actual = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
                    if not Gf.IsClose(actual, matrix, 1e-8):
                        raise ValueError(f"Composed reset pose is overridden: {path}")
                    body = UsdPhysics.RigidBodyAPI(prim)
                    if body.GetVelocityAttr().Get() != Gf.Vec3f(0) or body.GetAngularVelocityAttr().Get() != Gf.Vec3f(0):
                        raise ValueError(f"Composed reset velocity is overridden: {path}")
            except Exception:
                for attr, value in reversed(snapshots):
                    if value is not None:
                        attr.Set(value)
                    else:
                        attr.Clear()
                raise
        self.last_episode = {"episode_id": uuid4().hex, "asset": Path(asset).name, "asset_path": asset,
                             "object_prim_path": selected, "position": position, "yaw": yaw,
                             "orientation_xyzw": [*rotation.GetQuat().GetImaginary(), rotation.GetQuat().GetReal()],
                             "parking_positions": {self.pool[a]: list(p) for a, p in self.parking.items() if a != asset},
                             "container_prim_path": CONTAINER_PATH,
                             "container_position": list(container_position), "container_offset": offset,
                             "container_transform": [list(row) for row in transform]}
        return self.last_episode



class CollectionLifecycle:
    """Reset poses only while stopped; PLAY activates measured services."""
    def __init__(self, randomizer, start_publisher, is_stopped):
        self.randomizer, self.start_publisher = randomizer, start_publisher
        self.is_stopped = is_stopped
        self.state, self.handle = "stopped", None
        self.pending_prepare = True

    def prepare(self):
        if not self.is_stopped():
            raise RuntimeError("Episode preparation requires a stopped timeline")
        if self.state == "prepared":
            return None
        if self.state != "stopped" or not self.pending_prepare:
            raise RuntimeError("Stop before preparing another episode")
        self.state, self.pending_prepare = "preparing", False
        try:
            episode = self.randomizer.randomize()
        except Exception:
            self.state = "failed"
            raise
        self.state = "prepared"
        return episode

    def play(self):
        if self.state == "playing":
            return
        if self.state == "paused":
            self.state = "playing"
            return
        if self.state != "prepared":
            raise RuntimeError("Wait for an episode to be prepared before Play")
        try:
            self.handle = self.start_publisher()
        except Exception:
            self.state = "failed"
            raise
        self.state = "playing"

    def pause(self):
        if self.state == "playing":
            self.state = "paused"

    def stop(self):
        # A duplicate STOP after preparation must not consume the next episode.
        if self.state in ("prepared", "stopped", "closed"):
            return
        try:
            if self.handle is not None:
                self.handle.stop()
        finally:
            self.handle, self.state = None, "stopped"
            self.pending_prepare = True

    def close(self):
        try:
            self.stop()
        finally:
            self.pending_prepare = False
            self.state = "closed"
            self.randomizer.close()
