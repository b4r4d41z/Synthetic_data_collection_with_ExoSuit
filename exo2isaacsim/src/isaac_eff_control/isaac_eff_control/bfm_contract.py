"""Shared BFM collection semantics, independent of ROS/Isaac imports."""

import os


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


def state_defaults():
    """Same configuration for recorder provenance and the in-process sampler."""
    return {
        "state_frame": os.getenv("BFM_STATE_FRAME", "world"),
        "reference_prim_path": os.getenv("BFM_REFERENCE_PRIM_PATH", "/"),
        "eef_prim_path": os.getenv("BFM_EEF_PRIM_PATH", ""),
        "object_prim_path": os.getenv("BFM_OBJECT_PRIM_PATH", ""),
        "goal_prim_path": os.getenv("BFM_GOAL_PRIM_PATH", ""),
    }


def collection_metadata(enabled, state):
    """Describe configured semantics without claiming live runtime verification."""
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
            **{key: value for key, value in state.items() if key != "state_frame"},
            "configured": all(state[key] for key in (
                "eef_prim_path", "object_prim_path", "goal_prim_path"
            )),
            "runtime_verified": False,
        },
    }
