"""Measured task state from the active Isaac process, without USD/graph edits.

Run start_task_state_publisher() in Isaac's Script Editor with the built ROS2
packages available to Isaac's Python. Keep its handle alive and call stop()
before rerunning. Configuration and session setup are documented in README.
"""

import math

from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from dynamic_biped.msg import BfmState

from .bfm_contract import STATE_TOPIC, collection_enabled, state_defaults


class IsaacTaskSampler:
    """Use live PhysX tensors, including when USD pose writeback is disabled.

    EEF/object must be actual rigid-body frames in the active physics variant.
    The goal may also be a static Xform. The reference must be stationary and
    rigid; a moving reference would require additional velocity terms.
    """

    def __init__(self, stage, config, simulation_view=None):
        from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics

        if stage is None:
            raise ValueError("Open an Isaac USD stage before configuring task state")
        self.stage = stage
        self.config = config
        self.scale = UsdGeom.GetStageMetersPerUnit(stage)
        if not math.isfinite(self.scale) or self.scale <= 0:
            raise ValueError("Stage metersPerUnit must be finite and positive")
        self.prims = {}
        for key in ("eef_prim_path", "object_prim_path", "goal_prim_path",
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
        for key in ("eef_prim_path", "object_prim_path"):
            if not self.prims[key].HasAPI(UsdPhysics.RigidBodyAPI):
                raise ValueError(
                    f"{key} must select a PhysX rigid body in the active physics variant"
                )

        def require_static(prim):
            while prim and not prim.IsPseudoRoot():
                xform = UsdGeom.Xformable(prim)
                if (prim.HasAPI(UsdPhysics.RigidBodyAPI)
                        or (xform and xform.TransformMightBeTimeVarying())):
                    raise ValueError("Reference/non-rigid goal must be a static world frame")
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
        self.goal_is_rigid = self.prims["goal_prim_path"].HasAPI(UsdPhysics.RigidBodyAPI)
        if not self.goal_is_rigid:
            require_static(self.prims["goal_prim_path"])
        self.simulation_view = simulation_view
        self.views = {}

    def read(self):
        from pxr import Gf, Usd, UsdGeom

        if self.simulation_view is None:
            # Initialize only after Play has initialized physics. These views
            # read current simulator state, not initial USD or command targets.
            import omni.physics.tensors as tensors
            self.simulation_view = tensors.create_simulation_view("numpy")
            self.simulation_view.set_subspace_roots("/")
        if not self.views:
            keys = ["eef_prim_path", "object_prim_path"]
            if self.goal_is_rigid:
                keys.append("goal_prim_path")
            for key in keys:
                view = self.simulation_view.create_rigid_body_view(self.config[key])
                if view.count != 1:
                    raise ValueError(f"PhysX did not resolve one body for {self.config[key]}")
                self.views[key] = view

        def position(key):
            xyz = self.views[key].get_transforms()[0][:3]
            return self.world_to_reference.Transform(Gf.Vec3d(*map(float, xyz))) * self.scale

        eef = position("eef_prim_path")
        obj = position("object_prim_path")
        velocity = self.views["object_prim_path"].get_velocities()[0][:3]
        velocity = self.world_to_reference.TransformDir(
            Gf.Vec3d(*map(float, velocity))) * self.scale
        if self.goal_is_rigid:
            goal = position("goal_prim_path")
        else:
            world = UsdGeom.Xformable(self.prims["goal_prim_path"]).ComputeLocalToWorldTransform(
                Usd.TimeCode.Default()).ExtractTranslation()
            goal = self.world_to_reference.Transform(world) * self.scale
        values = [list(vector) for vector in (eef, obj, velocity, goal)]
        if not all(math.isfinite(value) for vector in values for value in vector):
            raise ValueError("PhysX returned non-finite task measurements")
        return values


class BfmTaskStatePublisher(Node):
    """Publish at native Isaac updates, independently of the 10 Hz policy."""

    def __init__(self, stage, sampler_factory=IsaacTaskSampler, **kwargs):
        super().__init__("bfm_task_state", **kwargs)
        self.enabled = self.declare_parameter(
            "bfm_collection_mode", collection_enabled()).value
        self.config = {key: self.declare_parameter(key, value).value
                       for key, value in state_defaults().items()}
        self.stage, self.factory = stage, sampler_factory
        self.last_time = None
        self.sampler = None
        if not self.enabled:
            self.get_logger().info("BFM task state disabled")
            return
        try:
            if not self.config["state_frame"].strip():
                raise ValueError("state_frame must name the configured reference basis")
            self.sampler = self.factory(stage, self.config)
        except Exception:
            super().destroy_node()
            raise  # A missing object/goal is a setup error, never a zero state.
        self.publisher = self.create_publisher(BfmState, STATE_TOPIC, QoSProfile(
            history=HistoryPolicy.KEEP_LAST, depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        ))
        self.get_logger().info(f"Measured /bfm/state in {self.config['state_frame']}: {self.config}")

    def reset(self):
        self.last_time = None
        self.sampler = None  # Recreate PhysX views after Stop/Play.

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
        if len(values) != 4 or any(len(v) != 3 or not all(math.isfinite(x) for x in v)
                                   for v in values):
            raise ValueError("Task sampler must supply four measured finite 3D vectors")
        message = BfmState()
        nanoseconds = round(simulation_seconds * 1_000_000_000)
        message.header.stamp.sec, message.header.stamp.nanosec = divmod(nanoseconds, 1_000_000_000)
        message.header.frame_id = self.config["state_frame"]
        (message.right_eef_position, message.object_position,
         message.object_velocity, message.goal_position) = values
        self.publisher.publish(message)
        self.last_time = simulation_seconds


def start_task_state_publisher(**parameters):
    """Install a live Kit callback and return a handle with idempotent stop().

    Keywords are ROS parameters: bfm_collection_mode, eef_prim_path,
    object_prim_path, goal_prim_path, reference_prim_path, state_frame.
    Start per Isaac session; this does not author a persistent USD sensor.
    """
    import omni.kit.app
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
        was_playing = False
        stopped = False

        def stop(self):
            if self.stopped:
                return
            self.stopped = True
            self.subscription = None
            executor.remove_node(node)
            node.destroy_node()
            executor.shutdown()
            context.shutdown()

    handle = Handle()

    def update(_event):
        executor.spin_once(timeout_sec=0.0)
        if not node.enabled:
            return
        playing = timeline.is_playing()
        if playing:
            try:
                if omni.usd.get_context().get_stage() != stage:
                    raise ValueError("USD stage changed; restart the task-state publisher")
                node.publish_at(timeline.get_current_time())
            except Exception as error:
                node.enabled = False
                node.get_logger().error(f"Task-state publication disabled: {error}")
        elif handle.was_playing:
            node.reset()
        handle.was_playing = playing

    handle.subscription = omni.kit.app.get_app().get_post_update_event_stream().create_subscription_to_pop(
        update, name="bfm_measured_task_state")
    return handle
