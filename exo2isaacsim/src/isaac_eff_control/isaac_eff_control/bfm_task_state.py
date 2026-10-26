"""Measured task state from the active Isaac process, without USD/graph edits.

The project-local bfm_collection extension owns start_task_state_publisher()
automatically. The helper remains available for custom scenes; do not start a
second publisher while the extension is active. See README for setup.
"""

import math

from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from dynamic_biped.msg import BfmState

from .bfm_contract import (
    STATE_ENV, STATE_TOPIC, collection_enabled, scene_config_default,
    state_defaults, validate_state_parameters,
)


STATE_FIELD_SIZES = {
    "right_eef_position": 3, "object_position": 3, "object_velocity": 3,
    "goal_center": 3, "goal_half_extents": 3, "goal_orientation": 4,
    "goal_linear_velocity": 3, "goal_angular_velocity": 3,
}


class IsaacTaskSampler:
    """Use live PhysX tensors, including when USD pose writeback is disabled.

    EEF, object and container must resolve to live rigid bodies. Local offsets
    and half extents are physical meters; rotations use xyzw. Physics tensor
    linear velocities are at the COM, so point velocities account for both
    the COM-to-origin and origin-to-point lever arms. Reference stays static.
    """

    def __init__(self, stage, config, simulation_view=None):
        from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

        if stage is None:
            raise ValueError("Open an Isaac USD stage before configuring task state")
        self.stage = stage
        self.config = validate_state_parameters(config)
        self.scale = UsdGeom.GetStageMetersPerUnit(stage)
        if not math.isfinite(self.scale) or self.scale <= 0:
            raise ValueError("Stage metersPerUnit must be finite and positive")
        self.prims = {}
        for key in ("eef_prim_path", "object_prim_path", "container_prim_path",
                    "reference_prim_path"):
            path = config[key]
            if not path or not Sdf.Path.IsValidPathString(path):
                raise ValueError(f"Configure a valid absolute {key}; got {path!r}")
            usd_path = Sdf.Path(path)
            if not usd_path.IsAbsolutePath() or (path != "/" and not usd_path.IsPrimPath()):
                raise ValueError(f"{key} must be an absolute USD prim path")
            if path == "/" and key != "reference_prim_path":
                raise ValueError(f"{key} must select a real prim, not the stage root")
            prim = stage.GetPseudoRoot() if path == "/" else stage.GetPrimAtPath(path)
            if (not prim or not prim.IsActive()
                    or (path != "/" and not UsdGeom.Xformable(prim))):
                raise ValueError(f"Missing/inactive/non-transformable {key}: {path}")
            self.prims[key] = prim
        for key in ("eef_prim_path", "object_prim_path", "container_prim_path"):
            if not self.prims[key].HasAPI(UsdPhysics.RigidBodyAPI):
                raise ValueError(
                    f"{key} must select a PhysX rigid body in the active physics variant"
                )
            if UsdPhysics.RigidBodyAPI(self.prims[key]).GetRigidBodyEnabledAttr().Get() is False:
                raise ValueError(f"{key} selects a disabled rigid body")

        def require_static(prim):
            while prim and not prim.IsPseudoRoot():
                xform = UsdGeom.Xformable(prim)
                if (prim.HasAPI(UsdPhysics.RigidBodyAPI)
                        or (xform and xform.TransformMightBeTimeVarying())):
                    raise ValueError("Reference must be a static world frame")
                prim = prim.GetParent()

        reference = self.prims["reference_prim_path"]
        require_static(reference)
        self.world_to_reference = (
            Gf.Matrix4d(1) if reference.IsPseudoRoot() else
            UsdGeom.Xformable(reference).ComputeLocalToWorldTransform(
                Usd.TimeCode.Default()).GetInverse()
        )
        matrix = self.world_to_reference
        axes = [matrix.TransformDir(Gf.Vec3d(*(int(i == j) for i in range(3))))
                for j in range(3)]
        if (not math.isclose(matrix.GetDeterminant(), 1.0, abs_tol=1e-6)
                or any(not math.isclose(a * b, float(i == j), abs_tol=1e-6)
                       for i, a in enumerate(axes) for j, b in enumerate(axes))):
            raise ValueError("Reference must be right-handed, with unit scale and no shear")
        self.reference_orientation = matrix.ExtractRotationQuat().GetNormalized()
        self.simulation_view = simulation_view
        self.views = {}
        self.resolved_body_counts = {}

    def read(self):
        from pxr import Gf

        if self.simulation_view is None:
            # Initialize only after Play has initialized physics. These views
            # read current simulator state, not initial USD or command targets.
            import omni.physics.tensors as tensors
            self.simulation_view = tensors.create_simulation_view("numpy")
            self.simulation_view.set_subspace_roots("/")
        if not self.views:
            views = {}
            for key in ("eef_prim_path", "object_prim_path", "container_prim_path"):
                view = self.simulation_view.create_rigid_body_view(self.config[key])
                if view.count != 1:
                    raise ValueError(f"PhysX did not resolve one body for {self.config[key]}")
                views[key] = view
            self.views = views
            self.resolved_body_counts = {self.config[key]: view.count for key, view in views.items()}

        def vector(values, size, label):
            values = [float(v) for v in values]
            if len(values) != size or not all(math.isfinite(v) for v in values):
                raise ValueError(f"PhysX returned invalid {label}")
            return values

        def quaternion(xyzw):
            x, y, z, w = xyzw
            norm = math.hypot(x, y, z, w)
            if not math.isfinite(norm) or norm < 1e-12:
                raise ValueError("PhysX returned an invalid pose quaternion")
            return Gf.Quatd(w / norm, Gf.Vec3d(x / norm, y / norm, z / norm))

        def pose(key):
            raw = vector(self.views[key].get_transforms()[0], 7, "pose")
            return Gf.Vec3d(*raw[:3]) * self.scale, quaternion(raw[3:])

        def body_state(key):
            position, orientation = pose(key)
            rotation = Gf.Rotation(orientation)
            twist = vector(self.views[key].get_velocities()[0], 6, "velocity")
            # Tensor twists are world-space velocities at the center of mass.
            # Never assume that an auto-computed COM equals the USD body origin.
            com = vector(self.views[key].get_coms()[0], 7, "local COM pose")
            omega = Gf.Vec3d(*twist[3:])  # PhysX angular velocities are rad/s.
            com_offset = rotation.TransformDir(Gf.Vec3d(*com[:3]) * self.scale)
            return position, orientation, Gf.Vec3d(*twist[:3]) * self.scale, omega, com_offset

        def point_state(key, offset_m):
            position, orientation, com_velocity, omega, com_offset = body_state(key)
            rotation = Gf.Rotation(orientation)
            offset = rotation.TransformDir(Gf.Vec3d(*offset_m))
            origin_velocity = com_velocity - Gf.Cross(omega, com_offset)
            return position + offset, origin_velocity + Gf.Cross(omega, offset), omega, orientation

        def reference_point(point_m):
            # USD reference translation is in stage units; vector rotations
            # below have no translation and use SI units directly.
            return self.world_to_reference.Transform(point_m / self.scale) * self.scale

        eef, _ = pose("eef_prim_path")
        obj_origin, _, obj_velocity, _, obj_com_offset = body_state("object_prim_path")
        obj = obj_origin + obj_com_offset
        goal, goal_velocity, goal_omega, container_rotation = point_state(
            "container_prim_path", self.config["goal_local_center"])
        goal_rotation = (self.reference_orientation * container_rotation *
                         quaternion(self.config["goal_local_orientation"])).GetNormalized()
        values = {
            "right_eef_position": list(reference_point(eef)),
            "object_position": list(reference_point(obj)),
            "object_velocity": list(self.world_to_reference.TransformDir(obj_velocity)),
            "goal_center": list(reference_point(goal)),
            "goal_half_extents": list(self.config["goal_half_extents"]),
            "goal_orientation": [*goal_rotation.GetImaginary(), goal_rotation.GetReal()],
            "goal_linear_velocity": list(self.world_to_reference.TransformDir(goal_velocity)),
            "goal_angular_velocity": list(self.world_to_reference.TransformDir(goal_omega)),
        }
        if not all(math.isfinite(value) for array in values.values() for value in array):
            raise ValueError("PhysX returned non-finite task measurements")
        return values


class BfmTaskStatePublisher(Node):
    """Publish at native Isaac updates, independently of the 10 Hz policy."""

    def __init__(self, stage, sampler_factory=IsaacTaskSampler, **kwargs):
        super().__init__("bfm_task_state", **kwargs)
        self.enabled = self.declare_parameter(
            "bfm_collection_mode", collection_enabled()).value
        self.scene_config_path = self.declare_parameter(
            "scene_config_path", scene_config_default()).value
        self.config = {key: self.declare_parameter(key, value).value
                       for key, value in state_defaults(self.scene_config_path).items()}
        self.stage, self.factory = stage, sampler_factory
        self.last_time = None
        self.sampler = None
        self._reported_counts = False
        if not self.enabled:
            self.get_logger().info("BFM task state disabled")
            return
        try:
            self.config = validate_state_parameters(self.config)
            if not self.config["state_frame"].strip():
                raise ValueError("state_frame must name the configured reference basis")
            self.sampler = self.factory(stage, self.config)
        except Exception:
            super().destroy_node()
            raise  # Missing/invalid roles are setup errors, never a zero state.
        self.publisher = self.create_publisher(BfmState, STATE_TOPIC, QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        ))
        self.get_logger().info(f"Measured /bfm/state in {self.config['state_frame']}: {self.config}")

    def reset(self):
        self.last_time = None
        self.sampler = None  # Recreate PhysX views after Stop/Play.
        self._reported_counts = False

    def destroy_node(self):
        # Release tensor views before the extension switches fruit references.
        self.reset()
        return super().destroy_node()

    def publish_at(self, simulation_seconds):
        if not self.enabled or simulation_seconds == self.last_time:
            return
        if not math.isfinite(simulation_seconds) or simulation_seconds < 0:
            raise ValueError("Isaac simulation time must be finite and nonnegative")
        if self.last_time is not None and simulation_seconds < self.last_time:
            self.reset()
        if self.sampler is None:
            self.sampler = self.factory(self.stage, self.config)
        values = self.sampler.read()
        if (not isinstance(values, dict) or set(values) != set(STATE_FIELD_SIZES)
                or any(len(values[key]) != size or not all(math.isfinite(x) for x in values[key])
                       for key, size in STATE_FIELD_SIZES.items())
                or any(v <= 0 for v in values["goal_half_extents"])
                or not math.isclose(sum(v*v for v in values["goal_orientation"]), 1., abs_tol=1e-6)):
            raise ValueError("Task sampler must supply the finite moving OBB contract with unit xyzw")
        counts = getattr(self.sampler, "resolved_body_counts", None)
        if not self._reported_counts and isinstance(counts, dict):
            self.get_logger().info(f"Live PhysX resolved body counts: {counts}")
            self._reported_counts = True
        message = BfmState()
        nanoseconds = round(simulation_seconds * 1_000_000_000)
        message.header.stamp.sec, message.header.stamp.nanosec = divmod(nanoseconds, 1_000_000_000)
        message.header.frame_id = self.config["state_frame"]
        for field, value in values.items():
            setattr(message, field, list(value))
        self.publisher.publish(message)
        self.last_time = simulation_seconds


def start_task_state_publisher(**parameters):
    """Install a live Kit callback and return a handle with idempotent stop().

    Keywords are ROS parameters: bfm_collection_mode, eef_prim_path,
    object_prim_path, container_prim_path, reference_prim_path, state_frame,
    scene_config_path and local SI goal geometry. Goal quaternions use xyzw.
    Start per Isaac session; this does not author a persistent USD sensor.
    """
    if "goal_prim_path" in parameters:
        raise ValueError("goal_prim_path was replaced by container_prim_path and local OBB geometry")
    unknown = set(parameters) - {*STATE_ENV, "bfm_collection_mode", "scene_config_path", "use_sim_time"}
    if unknown:
        raise ValueError(f"Unknown task-state parameters: {sorted(unknown)}")
    import omni.kit.app
    import omni.physx
    import omni.timeline
    import omni.usd
    from rclpy.context import Context
    from rclpy.executors import SingleThreadedExecutor
    from rclpy.parameter import Parameter

    context = Context()
    context.init(args=[])
    stage = omni.usd.get_context().get_stage()
    try:
        node = BfmTaskStatePublisher(stage, context=context, parameter_overrides=[
            Parameter(key, value=value) for key, value in parameters.items()
        ])
    except Exception:
        context.shutdown()
        raise
    executor = SingleThreadedExecutor(context=context)
    executor.add_node(node)
    timeline = omni.timeline.get_timeline_interface()

    class Handle:
        subscription = None
        physics_subscription = None
        timeline_subscription = None
        physics_stepped = False
        reported_ready = False
        stopped = False

        def stop(self):
            if self.stopped:
                return
            self.stopped = True
            self.subscription = None
            self.physics_subscription = None
            self.timeline_subscription = None
            self.physics_stepped = False
            executor.remove_node(node)
            node.destroy_node()
            executor.shutdown()
            context.shutdown()

    handle = Handle()
    handle.node = node  # Inspect sampler.resolved_body_counts after Play in Script Editor.

    def physics_step(_dt):
        # This callback is dispatched by PhysX, not Kit's timeline/update clock.
        # Only signal here: access tensors after the step, on Kit post-update.
        if not handle.stopped and timeline.is_playing():
            handle.physics_stepped = True

    def timeline_event(event):
        if event.type == int(omni.timeline.TimelineEventType.STOP):
            handle.physics_stepped = False
            handle.reported_ready = False
            node.reset()
        # PAUSE retains the current episode and valid views.

    def update(_event):
        if handle.stopped:
            return
        executor.spin_once(timeout_sec=0.0)
        if not node.enabled:
            return
        if timeline.is_playing() and handle.physics_stepped:
            handle.physics_stepped = False
            try:
                if omni.usd.get_context().get_stage() != stage:
                    raise ValueError("USD stage changed; restart the task-state publisher")
                node.publish_at(timeline.get_current_time())
                if not handle.reported_ready:
                    node.get_logger().info("[BFM] Physics ready. /bfm/state publisher active.")
                    handle.reported_ready = True
            except Exception as error:
                node.enabled = False
                node.get_logger().error(f"Task-state publication disabled: {error}")
    try:
        handle.physics_subscription = omni.physx.get_physx_interface().subscribe_physics_step_events(
            physics_step)
        handle.timeline_subscription = timeline.get_timeline_event_stream().create_subscription_to_pop(
            timeline_event, name="bfm_state_reset")
        handle.subscription = omni.kit.app.get_app().get_post_update_event_stream().create_subscription_to_pop(
            update, name="bfm_measured_task_state")
    except Exception:
        handle.stop()
        raise
    return handle
