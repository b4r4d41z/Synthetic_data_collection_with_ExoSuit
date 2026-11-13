"""Shared BFM collection semantics, independent of ROS/Isaac imports."""

from functools import lru_cache
import hashlib
from collections.abc import Sequence
import json
import math
import os
from pathlib import Path
import xml.etree.ElementTree as ET


CONTRACT_VERSION = 4
HAND_CONTROL_HZ = 50.0
# PhysX simulation frequency in Hz. Edit only this value, then restart Isaac Sim.
PHYSICS_HZ = 60.0
POLICY_HZ = 15.0
POLICY_DT = 1.0 / POLICY_HZ
LEFT_ARM_JOINTS = (
    "zarm_l1_joint", "zarm_l2_joint", "zarm_l3_joint", "zarm_l4_joint",
    "zarm_l5_joint", "zarm_l6_joint", "zarm_l7_joint",
)
RIGHT_ARM_JOINTS = (
    "zarm_r1_joint", "zarm_r2_joint", "zarm_r3_joint", "zarm_r4_joint",
    "zarm_r5_joint", "zarm_r6_joint", "zarm_r7_joint",
)
ARM_JOINTS = frozenset((*LEFT_ARM_JOINTS, *RIGHT_ARM_JOINTS))
ACTION_GROUP_ORDER = ("left_arm", "left_hand", "right_arm", "right_hand")
ARM_SOURCE_TOPIC = "/bfm/source/arm_joint_target_rad"
HELD_ARM_TOPIC = "/robot_arm_joint_command"
HELD_HAND_TOPIC = "/bfm/command/hand_target"
ACTION_TOPIC = "/bfm/action"
STATE_TOPIC = "/bfm/state"
RAW_HAND_TOPIC = "/robot/command/hand_target"


def validated_physics_hz(value):
    """PhysxSceneAPI.timeStepsPerSecond is a positive signed USD int (Hz)."""
    if (type(value) not in (int, float) or value <= 0 or value > 2**31 - 1
            or not math.isfinite(value) or value != int(value)):
        raise ValueError('PHYSICS_HZ must be a finite positive whole-number Hz value representable as a USD int')
    return int(value)


CLOCK_DOMAINS = {
    "created_at_and_record_directory": "system_wall_time; ISO8601 offset / local folder time",
    "mcap_log_time": "rosbag2 default system receive time; --use-sim-time is not enabled",
    "synchronization": "mcap_log_time (unchanged); headers are not a universal clock",
    "/clock.clock": "IsaacReadSimulationTime simulation time, published on physics steps; existing resetOnStop default",
    "/bfm/action.header.stamp": "selected /clock sample; decision generation, not measured controller application",
    "/bfm/source/input_provenance": "decision_stamp equals Action simulation stamp; embedded input headers retain source-native clocks; join by identity/digest, never MCAP receive order",
    "/bfm/state.header.stamp": "SimulationManager completed physics simulation time at Kit post-update; separate epoch from /clock if reset policies differ",
    "/robot/state/joint_states.header.stamp": "IsaacReadSimulationTime via saved graph; sensorTime is separately the joint sensor simulation time",
    "camera_image.header.stamp": "Isaac ROS2CameraHelper simulation time (default useSystemTime=false); render cadence unchanged",
    "raw_bridge.header.stamp": "source-native ExoSuit/ROS1 time, preserved by bridge",
    "/bfm/source/arm_joint_target_rad.header.stamp": "arm node ROS clock (system unless use_sim_time); conversion time",
    "held_arm.header.stamp": "mapper ROS clock in teleoperation; copied policy decision header in policy mode; not physical attainment",
    "held_hand.header.stamp": "source-native header in teleoperation; policy decision header in policy mode",
    "finger_joint_command.header.stamp": "node ROS clock at wall-scheduled publication, not physical attainment",
    "success_evaluation.evaluated_at": "system_wall_time ISO8601 with offset",
    "success_evaluation.state_age_sec": "recorder local monotonic receive age; not simulation elapsed time",
    "/bfm/episode": "identity/lifecycle events have no time header; use MCAP log time; recording events add system_wall_time event_wall_time",
    "controller_application_time": "not measured by action/source headers; use measured state for response analysis",
}


ARM_MODEL_PATH = "models/robots/s40_sharpa/assembly/s40_sharpa_two_cameras.xml"


def _arm_model_file():
    """Use the source model or its verbatim ROS package installation."""
    for root in Path(__file__).resolve().parents:
        model = root / ARM_MODEL_PATH
        if model.is_file():
            return model
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory("isaac_eff_control")) / "model" / Path(ARM_MODEL_PATH).name


@lru_cache(maxsize=1)
def _arm_model_snapshot():
    """One immutable model revision per process, shared by policy and metadata."""
    content = _arm_model_file().read_bytes()
    model = ET.fromstring(content)
    compiler = model.find("compiler")
    if compiler is None or compiler.get("angle") != "radian":
        raise ValueError("Arm model must explicitly declare radian joint ranges")
    limits = {}
    for joint in model.findall("./worldbody//joint"):
        name = joint.get("name")
        if name not in ARM_JOINTS:
            continue
        bounds = tuple(float(v) for v in joint.get("range", "").split())
        if (name in limits or len(bounds) != 2 or not all(map(math.isfinite, bounds))
                or bounds[0] >= bounds[1] or joint.get("limited") == "false"
                or joint.get("type", "hinge") != "hinge"):
            raise ValueError(f"Invalid or duplicate arm joint limits: {name}")
        limits[name] = bounds
    if set(limits) != ARM_JOINTS:
        raise ValueError("Arm model must define all 14 arm joint ranges")
    ordered = tuple((name, limits[name]) for name in (*LEFT_ARM_JOINTS, *RIGHT_ARM_JOINTS))
    return ordered, hashlib.sha256(content).hexdigest()


def arm_target_processing():
    limits, digest = _arm_model_snapshot()
    return {
        "clamping_enabled": True, "transformation": "per_joint_saturation", "unit": "rad",
        "source_topic": ARM_SOURCE_TOPIC,
        "source_semantics": "requested absolute joint-position target, pre-clamp",
        "controller_topic": HELD_ARM_TOPIC,
        "controller_semantics": "bounded absolute joint-position target",
        "canonical_action_topic": ACTION_TOPIC,
        "canonical_arm_semantics": "sampled bounded operator target; not execution evidence",
        "joint_limits": {name: list(bounds) for name, bounds in limits},
        "bounds_source": ARM_MODEL_PATH, "bounds_source_sha256": digest,
        "model_revision": "sha256:" + digest,
        "boundary": "arm mapper clamps direct controller commands; passive sampler applies the same bounds",
        "controller_cadence": "validated source callback rate; independent of BFM sampling",
        "canonical_action_cadence": "15 Hz simulation-time passive snapshots",
    }


def collection_enabled():
    return os.getenv("BFM_COLLECTION_MODE", "0") == "1"


STATE_ENV = {
    "state_frame": "BFM_STATE_FRAME",
    "reference_prim_path": "BFM_REFERENCE_PRIM_PATH",
    "eef_prim_path": "BFM_EEF_PRIM_PATH",  # Existing right EEF parameter.
    "left_eef_prim_path": "BFM_LEFT_EEF_PRIM_PATH",
    "articulation_prim_path": "BFM_ARTICULATION_PRIM_PATH",
    "object_prim_path": "BFM_OBJECT_PRIM_PATH",
    "container_prim_path": "BFM_CONTAINER_PRIM_PATH",
    "goal_local_center": "BFM_GOAL_LOCAL_CENTER",
    "goal_half_extents": "BFM_GOAL_HALF_EXTENTS",
    "goal_local_orientation": "BFM_GOAL_LOCAL_ORIENTATION",
}
STATE_VECTORS = {
    "goal_local_center": 3,
    "goal_half_extents": 3, "goal_local_orientation": 4,
}


def scene_config_default():
    from .scenarios import scenario
    # Standalone nodes retain legacy defaults until the active episode arrives.
    # SCENARIO belongs to the Isaac launcher, not independently started ROS nodes.
    return os.getenv("BFM_SCENE_CONFIG") or str(scenario("v1_manipulation_one_side_right")["config"])


def validate_state_parameters(state):
    """Validate fixed local SI geometry; normalize the explicit xyzw rotation."""
    if set(state) != set(STATE_ENV):
        raise ValueError(f"State configuration must contain exactly: {', '.join(STATE_ENV)}")
    result = dict(state)
    for key, size in STATE_VECTORS.items():
        values = state[key]
        if (not isinstance(values, Sequence) or len(values) != size
                or any(isinstance(v, bool) or not isinstance(v, (int, float))
                       or not math.isfinite(v) for v in values)):
            raise ValueError(f"{key} must contain {size} finite numbers")
        result[key] = [float(v) for v in values]
    if any(v <= 0 for v in result["goal_half_extents"]):
        raise ValueError("goal_half_extents must be positive lengths in meters")
    norm = math.hypot(*result["goal_local_orientation"])
    if not math.isfinite(norm) or norm < 1e-12:
        raise ValueError("goal_local_orientation must be a nonzero xyzw quaternion")
    result["goal_local_orientation"] = [v / norm for v in result["goal_local_orientation"]]
    for key in set(STATE_ENV) - set(STATE_VECTORS):
        if not isinstance(state[key], str):
            raise ValueError(f"{key} must be a string")
    return result


def state_defaults(scene_config_path=None):
    """Scene JSON < environment < ROS overrides (declared by each node)."""
    if "BFM_GOAL_PRIM_PATH" in os.environ:
        raise ValueError("BFM_GOAL_PRIM_PATH was removed; use BFM_CONTAINER_PRIM_PATH and a local OBB")
    supplied_path = scene_config_default() if scene_config_path is None else scene_config_path
    if not isinstance(supplied_path, (str, os.PathLike)) or not str(supplied_path).strip():
        raise ValueError("scene_config_path must name a scene JSON file")
    path = Path(supplied_path).expanduser()
    data = json.loads(path.read_text(encoding="utf-8"))
    if (not isinstance(data, dict) or data.get("schema_version") != 1
            or not isinstance(data.get("parameters"), dict)):
        raise ValueError("Scene configuration requires schema_version=1 and a parameters object")
    parameters = dict(data["parameters"])
    for key, variable in STATE_ENV.items():
        if variable in os.environ:
            value = os.environ[variable]
            parameters[key] = json.loads(value) if key in STATE_VECTORS else value
    return validate_state_parameters(parameters)


def scene_configuration(path):
    """Keep versioned, configured provenance separate from live verification."""
    content = Path(path).expanduser().read_bytes()
    data = json.loads(content)
    from .scenarios import SCENARIOS, scenario
    name = next((n for n in SCENARIOS if scenario(n)["scene"] == data.get("scene")), None)
    return {"scenario": name, "file": str(path), "sha256": hashlib.sha256(content).hexdigest(),
            "scene": data.get("scene"), "provenance": data.get("provenance", {}),
            "randomization": data.get("randomization"),
            "runtime_verified": False}


def collection_metadata(enabled, state, configuration=None):
    """Describe configured semantics without claiming live runtime verification."""
    state = validate_state_parameters(state)
    from .source_provenance import PROVENANCE_TOPIC, PROVENANCE_VERSION, PROTOCOL
    return {
        "enabled": enabled,
        "policy_hz": POLICY_HZ,
        "policy_dt": POLICY_DT,
        "policy_clock": {
            "domain": "isaac_simulation", "topic": "/clock",
            "period_ns_numerator": 1_000_000_000, "period_ns_denominator": 15,
            "trigger": "physics_step", "scheduler": "rational_integer_deadlines_1ns_quantization_tolerance",
            "quantization": "first physics clock sample at or after deadline",
            "on_backward_jump": "rebase_without_tick", "on_missed_deadlines": "one_current_tick_no_burst",
        },
        "contract_version": CONTRACT_VERSION,
        "control_architecture": "decoupled_teleoperation_v1",
        "action_role": "sampled_operator_target_not_execution_acknowledgement",
        "operator_input_provenance": {
            "protocol": PROTOCOL, "version": PROVENANCE_VERSION,
            "required_for_operator_actions": bool(enabled),
            "topic": PROVENANCE_TOPIC, "message_type": "std_msgs/msg/String",
            "action_fields": ["source_provenance_version", "source_provenance_sha256"],
            "join": ["producer_session_id", "action_id"],
            "digest": "SHA-256 of exact UTF-8 String.data bytes; no reserialization",
            "input_identity": ["producer_session_id", "kind", "input_sequence"],
            "sequence_scope": "each valid input callback; independent arm/hand counters; monotonic across STOP and clock rollback",
            "trace_scope": "one self-contained selected-input bundle per Action, including repeated old inputs; unsampled sequences may be absent",
            "identity_limit": "Gateway-received inputs only; no unique upstream ROS1 identity or physical execution claim",
            "missing_evidence": "fail verification; never approximate or use latest MCAP source",
            "non_operator_actions": "version 0 and empty digest make no operator provenance claim; policy producers leave defaults",
        },
        "control": {
            "arm": "direct mapped bounded targets at source rate; no added trajectory generator",
            "hand_hz": HAND_CONTROL_HZ, "hand_clock": "steady_wall_time",
            "physics_hz": validated_physics_hz(PHYSICS_HZ), "execution_acknowledgements": False,
            "sampling_independent_of_control": True,
            "source_freshness": "no command watchdog; latest valid snapshot may repeat; inspect full-rate source streams",
            "downstream_compatibility": "requires decoupled v4 converter; incompatible with executor-based v4 acceptance joins",
        },
        "action_dim": 26,
        "arm_target_processing": arm_target_processing(),
        "action_group_order": list(ACTION_GROUP_ORDER),
        "action_topic": ACTION_TOPIC,
        "action_semantics": {
            **{group: ({
                "dim": 7, "type": "bounded_absolute_joint_position_target", "unit": "rad",
                "joint_names": list(LEFT_ARM_JOINTS if group == "left_arm" else RIGHT_ARM_JOINTS),
            } if group.endswith("arm") else {
                "dim": 6, "type": "logical_hand_target", "unit": "controller_0_100",
                "range": [0, 100],
                "value_order": [f"{group}_position[{i}]" for i in range(6)],
                "controller": "eff_control_node (existing Sharpa mapping)",
            }) for group in ACTION_GROUP_ORDER},
        },
        "recorded_signal_roles": {
            "raw_inputs": ["/robot/command/arm_joint_target", RAW_HAND_TOPIC],
            "policy_inputs": [ARM_SOURCE_TOPIC, RAW_HAND_TOPIC],
            "held_commands": [HELD_ARM_TOPIC, HELD_HAND_TOPIC],
            "executed_command_replay": [HELD_ARM_TOPIC, "/left_hand_joint_command", "/right_hand_joint_command"],
            "measured_robot_state": "/robot/state/joint_states",
            "canonical_action": ACTION_TOPIC,
            "selected_operator_input_evidence": PROVENANCE_TOPIC,
            "canonical_state": STATE_TOPIC,
        },
        "task_state": {
            "topic": STATE_TOPIC,
            "frame": state["state_frame"],
            "position_unit": "m",
            "velocity_unit": "m/s",
            "linear_velocity_unit": "m/s",
            "angular_velocity_unit": "rad/s",
            "half_extents_unit": "m",
            "quaternion_convention": "xyzw; goal-local axes relative to frame",
            "goal_local_orientation_convention": "xyzw; goal-local axes relative to container body",
            "goal_half_extents_axes": "goal-local X/Y/Z",
            "object_position_reference": "live PhysX center of mass",
            "goal_representation": "oriented_bounding_box",
            "arm_proprioception": {
                "source": "live PhysX articulation DOF positions/velocities",
                "position_unit": "rad", "velocity_unit": "rad/s",
                "left_joint_names": list(LEFT_ARM_JOINTS),
                "right_joint_names": list(RIGHT_ARM_JOINTS),
            },
            "hand_proprioception": {
                "source": "/robot/state/joint_states",
                "semantics": "measured physical finger joint positions/velocities by joint name",
                "logical_hand_targets": "commands only, recorded in /bfm/action",
            },
            **{key: value for key, value in state.items() if key != "state_frame"},
            "configured": all(state[key] for key in (
                "eef_prim_path", "left_eef_prim_path", "articulation_prim_path",
                "object_prim_path", "container_prim_path"
            )),
            "scene_configuration": configuration,
            "runtime_verified": False,
        },
    }
