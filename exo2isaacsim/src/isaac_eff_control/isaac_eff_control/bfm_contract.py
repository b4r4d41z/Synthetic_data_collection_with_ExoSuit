"""Shared BFM collection semantics, independent of ROS/Isaac imports."""

import hashlib
from collections.abc import Sequence
import json
import math
import os
from pathlib import Path


POLICY_HZ = 10.0
POLICY_DT = 1.0 / POLICY_HZ
RIGHT_ARM_JOINTS = (
    "zarm_r1_joint", "zarm_r2_joint", "zarm_r3_joint", "zarm_r4_joint",
    "zarm_r5_joint", "zarm_r6_joint", "zarm_r7_joint",
)
ARM_JOINTS = frozenset((*RIGHT_ARM_JOINTS, *(f"zarm_l{i}_joint" for i in range(1, 8))))
ARM_SOURCE_TOPIC = "/bfm/source/arm_joint_target_rad"
HELD_ARM_TOPIC = "/robot_arm_joint_command"
HELD_HAND_TOPIC = "/bfm/command/hand_target"
ACTION_TOPIC = "/bfm/action"
STATE_TOPIC = "/bfm/state"
RAW_HAND_TOPIC = "/robot/command/hand_target"


def collection_enabled():
    return os.getenv("BFM_COLLECTION_MODE", "0") == "1"


STATE_ENV = {
    "state_frame": "BFM_STATE_FRAME",
    "reference_prim_path": "BFM_REFERENCE_PRIM_PATH",
    "eef_prim_path": "BFM_EEF_PRIM_PATH",
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
    return os.getenv("BFM_SCENE_CONFIG", str(Path(__file__).with_name("sorting_scene.json")))


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
    return {"file": str(path), "sha256": hashlib.sha256(content).hexdigest(),
            "scene": data.get("scene"), "provenance": data.get("provenance", {}),
            "randomization": data.get("randomization"),
            "runtime_verified": False}


def collection_metadata(enabled, state, configuration=None):
    """Describe configured semantics without claiming live runtime verification."""
    state = validate_state_parameters(state)
    return {
        "enabled": enabled,
        "policy_hz": POLICY_HZ,
        "policy_dt": POLICY_DT,
        "action_dim": 13,
        "action_topic": ACTION_TOPIC,
        "action_semantics": {
            "right_arm": {
                "dim": 7,
                "type": "absolute_joint_position_target",
                "unit": "rad",
                "joint_names": list(RIGHT_ARM_JOINTS),
            },
            "right_hand": {
                "dim": 6,
                "type": "logical_hand_target",
                "unit": "controller_0_100",
                "range": [0, 100],
                "value_order": [f"right_hand_position[{i}]" for i in range(6)],
                "controller": "eff_control_node (existing Sharpa mapping)",
            },
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
            **{key: value for key, value in state.items() if key != "state_frame"},
            "configured": all(state[key] for key in (
                "eef_prim_path", "object_prim_path", "container_prim_path"
            )),
            "scene_configuration": configuration,
            "runtime_verified": False,
        },
    }
