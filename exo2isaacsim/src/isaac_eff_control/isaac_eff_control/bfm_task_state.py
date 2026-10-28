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
    LEFT_ARM_JOINTS, RIGHT_ARM_JOINTS, STATE_ENV, STATE_TOPIC, collection_enabled, scene_config_default,
    state_defaults, validate_state_parameters,
)


STATE_FIELD_SIZES = {
    "left_eef_position": 3, "right_eef_position": 3,
    "left_arm_position": 7, "right_arm_position": 7,
    "left_arm_velocity": 7, "right_arm_velocity": 7,
    "object_position": 3, "object_velocity": 3,
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
        for key in ("left_eef_prim_path", "eef_prim_path", "object_prim_path", "container_prim_path",
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
        for key in ("left_eef_prim_path", "eef_prim_path", "object_prim_path", "container_prim_path"):
            if not self.prims[key].HasAPI(UsdPhysics.RigidBodyAPI):
                raise ValueError(
                    f"{key} must select a PhysX rigid body in the active physics variant"
                )
            if UsdPhysics.RigidBodyAPI(self.prims[key]).GetRigidBodyEnabledAttr().Get() is False:
                raise ValueError(f"{key} selects a disabled rigid body")

        if config["left_eef_prim_path"] == config["eef_prim_path"]:
            raise ValueError("Left and right EEF roles must select distinct physical bodies")
        path = config["articulation_prim_path"]
        if (not path or not Sdf.Path.IsValidPathString(path)
                or not Sdf.Path(path).IsAbsolutePath() or not Sdf.Path(path).IsPrimPath()):
            raise ValueError(f"Configure a valid absolute articulation_prim_path: {path!r}")
        articulation = stage.GetPrimAtPath(path)
        # In this scene ArticulationRootAPI is on a fixed joint, not an Xform.
        if not articulation or not articulation.IsActive() or not articulation.HasAPI(UsdPhysics.ArticulationRootAPI):
            raise ValueError(f"Missing active ArticulationRootAPI: {path}")

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
        self.body_views = {}  # Borrow selected object/container views from the reset component.
        self.resolved_body_counts = {}
        self.articulation_view = None
        self.arm_dof_indices = {}

    def read(self):
        from pxr import Gf

        if self.simulation_view is None:
            raise RuntimeError("Task sampling requires a ready SimulationManager physics view")
        if not self.views:
            views = {}
            for key in ("left_eef_prim_path", "eef_prim_path", "object_prim_path", "container_prim_path"):
                path = self.config[key]
                view = (self.body_views[path] if path in self.body_views else
                        self.simulation_view.create_rigid_body_view(path))
                if view.count != 1:
                    raise ValueError(f"PhysX did not resolve one body for {self.config[key]}")
                views[key] = view
            self.views = views
            self.resolved_body_counts = {self.config[key]: view.count for key, view in views.items()}

        if self.articulation_view is None:
            # Read-only tensor view of the existing articulation. No Articulation
            # wrapper/controller is initialized and no robot state is written.
            view = self.simulation_view.create_articulation_view(self.config["articulation_prim_path"])
            if view.count != 1:
                raise ValueError("PhysX must resolve exactly one robot articulation")
            names = list(view.shared_metatype.dof_names)
            if len(names) != view.max_dofs or len(set(names)) != len(names):
                raise ValueError("PhysX articulation DOF names must be unique and match tensor width")
            for side, joints in (("left", LEFT_ARM_JOINTS), ("right", RIGHT_ARM_JOINTS)):
                missing = [name for name in joints if name not in names]
                if missing:
                    raise ValueError(f"PhysX articulation lacks {side} arm DOFs: {missing}")
                self.arm_dof_indices[side] = [names.index(name) for name in joints]
            self.articulation_view = view

        def vector(values, size, label):
            values = [float(v) for v in values]
            if len(values) != size or not all(math.isfinite(v) for v in values):
                raise ValueError(f"PhysX returned invalid {label}")
            return values

        def tensor_row(values, size, label):
            # Warp arrays deliberately forbid Python item indexing. numpy()
            # performs the supported host transfer (including CUDA arrays).
            # Torch tensors need detach/cpu before numpy; NumPy/list inputs
            # already support indexing. Never index the backend array first.
            if hasattr(values, "detach"):
                values = values.detach()
            if hasattr(values, "cpu"):
                values = values.cpu()
            if hasattr(values, "numpy"):
                values = values.numpy()
            return vector(values[0], size, label)

        def quaternion(xyzw):
            x, y, z, w = xyzw
            norm = math.hypot(x, y, z, w)
            if not math.isfinite(norm) or norm < 1e-12:
                raise ValueError("PhysX returned an invalid pose quaternion")
            return Gf.Quatd(w / norm, Gf.Vec3d(x / norm, y / norm, z / norm))

        def pose(key):
            raw = tensor_row(self.views[key].get_transforms(), 7, "pose")
            return Gf.Vec3d(*raw[:3]) * self.scale, quaternion(raw[3:])

        def body_state(key):
            position, orientation = pose(key)
            rotation = Gf.Rotation(orientation)
            twist = tensor_row(self.views[key].get_velocities(), 6, "velocity")
            # Tensor twists are world-space velocities at the center of mass.
            # Never assume that an auto-computed COM equals the USD body origin.
            com = tensor_row(self.views[key].get_coms(), 7, "local COM pose")
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

        left_eef, _ = pose("left_eef_prim_path")
        eef, _ = pose("eef_prim_path")
        # DOF getters return actual revolute joint angles/rates in rad/rad/s.
        # They are independent of action targets and of stage length units.
        arm_positions = tensor_row(self.articulation_view.get_dof_positions(),
                                   self.articulation_view.max_dofs, "joint positions")
        arm_velocities = tensor_row(self.articulation_view.get_dof_velocities(),
                                    self.articulation_view.max_dofs, "joint velocities")
        obj_origin, _, obj_velocity, _, obj_com_offset = body_state("object_prim_path")
        obj = obj_origin + obj_com_offset
        goal, goal_velocity, goal_omega, container_rotation = point_state(
            "container_prim_path", self.config["goal_local_center"])
        goal_rotation = (self.reference_orientation * container_rotation *
                         quaternion(self.config["goal_local_orientation"])).GetNormalized()
        values = {
            "left_eef_position": list(reference_point(left_eef)),
            **{f"{side}_arm_{field}": [data[i] for i in indices]
               for side, indices in self.arm_dof_indices.items()
               for field, data in (("position", arm_positions), ("velocity", arm_velocities))},
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
        # Release tensor views before the extension prepares the next episode.
        self.reset()
        return super().destroy_node()

    def publish_at(self, simulation_seconds, simulation_view=None, body_views=None):
        if not self.enabled or simulation_seconds == self.last_time:
            return
        if not math.isfinite(simulation_seconds) or simulation_seconds < 0:
            raise ValueError("Isaac simulation time must be finite and nonnegative")
        if self.last_time is not None and simulation_seconds < self.last_time:
            self.reset()
        if self.sampler is None:
            self.sampler = self.factory(self.stage, self.config)
        if simulation_view is not None:
            self.sampler.simulation_view = simulation_view
        if body_views is not None:
            self.sampler.body_views = body_views
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


def start_task_state_publisher(*, episode=None, **parameters):
    """Install a live Kit callback and return a handle with idempotent stop().

    Keywords are ROS parameters: bfm_collection_mode, left_eef_prim_path,
    eef_prim_path (right), articulation_prim_path,
    object_prim_path, container_prim_path, reference_prim_path, state_frame,
    scene_config_path and local SI goal geometry. Goal quaternions use xyzw.
    Optional episode is the read-only plan applied to existing actors via live
    setters before any publication. Published state always comes from PhysX.
    Start per Isaac session; this does not author a persistent USD sensor.
    """
    if "goal_prim_path" in parameters:
        raise ValueError("goal_prim_path was replaced by container_prim_path and local OBB geometry")
    unknown = set(parameters) - {*STATE_ENV, "bfm_collection_mode", "scene_config_path", "use_sim_time"}
    if unknown:
        raise ValueError(f"Unknown task-state parameters: {sorted(unknown)}")
    import omni.kit.app
    from isaacsim.core.simulation_manager import SimulationManager
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
        timeline_subscription = None
        simulation_view = None
        reported_ready = False
        stopped = False
        phase = "waiting_for_physics"
        reset_time = None

        def stop(self):
            if self.stopped:
                return
            self.stopped = True
            self.phase = "stopped"
            self.subscription = None
            self.timeline_subscription = None
            self.simulation_view = None
            if object_pool is not None:
                object_pool.release()
            executor.remove_node(node)
            node.destroy_node()
            executor.shutdown()
            context.shutdown()

    from .physics_object_pool import PhysicsObjectPool
    object_pool = None
    handle = Handle()
    handle.node = node  # Inspect sampler.resolved_body_counts after Play in Script Editor.
    handle.object_pool = object_pool

    def timeline_event(event):
        if not handle.stopped and event.type == int(omni.timeline.TimelineEventType.STOP):
            handle.simulation_view = None
            handle.reported_ready = False
            node.reset()
            handle.phase = "waiting_for_physics"
            handle.reset_time = None
            if object_pool is not None:
                object_pool.release()
        # PAUSE retains the current episode and valid views.

    def update(_event):
        if handle.stopped:
            return
        executor.spin_once(timeout_sec=0.0)
        if not node.enabled:
            return
        if timeline.is_playing():
            try:
                if omni.usd.get_context().get_stage() != stage:
                    raise ValueError("USD stage changed; restart the task-state publisher")
                # A physics-step notification can precede tensor attachment.
                # Borrow the manager's initialized view; never create another
                # backend or change subspace roots on this shared view.
                view = SimulationManager.get_physics_simulation_view()
                valid = getattr(view, "is_valid", True) if view is not None else False
                if callable(valid):
                    valid = valid()
                if not valid:
                    if object_pool is not None and object_pool.applied:
                        raise RuntimeError("PhysX view lost after live reset; STOP before another episode")
                    if handle.simulation_view is not None:
                        node.reset()
                        if object_pool is not None:
                            object_pool.release()
                        handle.simulation_view = None
                        handle.reported_ready = False
                    return
                if handle.simulation_view is not None and handle.simulation_view is not view:
                    if object_pool is not None and object_pool.applied:
                        raise RuntimeError("PhysX view replaced after live reset; STOP before another episode")
                    node.reset()  # Manager rebuilt physics; discard old body views.
                    handle.reported_ready = False
                handle.simulation_view = view
                now = timeline.get_current_time()
                if object_pool is not None:
                    if not object_pool.applied:
                        object_pool.bind(view)
                        handle.phase = "applying_reset"
                        object_pool.apply_episode()
                        handle.phase = "verifying_reset"
                        object_pool.verify_episode()
                        handle.reset_time = now
                        # Observe a real integration step after verification. No
                        # frame countdown, sleep, or arbitrary elapsed-time delay.
                        handle.phase = "settling"
                        return
                    if now <= handle.reset_time or not object_pool.parking_settled():
                        return
                node.publish_at(now, simulation_view=view,
                                body_views=object_pool.bodies if object_pool is not None else None)
                handle.phase = "playing"
                if not handle.reported_ready:
                    node.get_logger().info("[BFM] Physics ready. /bfm/state publisher active.")
                    handle.reported_ready = True
            except Exception as error:
                handle.phase = "failed"
                handle.reported_ready = False
                node.enabled = False
                if object_pool is not None:
                    object_pool.release()
                node.reset()
                node.get_logger().error(f"Task-state publication disabled: {error}")
    try:
        object_pool = PhysicsObjectPool(stage, episode, log=lambda msg: node.get_logger().info(msg)) if episode is not None else None
        handle.object_pool = object_pool
        if object_pool is not None:
            for key in ("object_prim_path", "container_prim_path"):
                if node.config[key] != episode[key]:
                    raise ValueError(f"Measured {key} disagrees with episode identity")
        handle.timeline_subscription = timeline.get_timeline_event_stream().create_subscription_to_pop(
            timeline_event, name="bfm_state_reset")
        handle.subscription = omni.kit.app.get_app().get_post_update_event_stream().create_subscription_to_pop(
            update, name="bfm_measured_task_state")
    except Exception:
        handle.stop()
        raise
    return handle
