import math

import pytest

from control.hand_command_mapper import (
    FINGERS,
    FingerCoupling,
    HandCalibration,
    HandCommandMapper,
    HandSide,
    ThumbFlexionCoupling,
    default_calibrations,
)
from control.isaac_hand_controller import IsaacHandController

OPEN_LEFT = [0, 100, 0, 0, 0, 0]
CLOSE_LEFT = [69, 99, 42, 54, 61, 60]
OPEN_RIGHT = [0, 100, 0, 0, 0, 0]
CLOSE_RIGHT = [69, 99, 42, 44, 61, 60]


def assert_close(actual, expected):
    assert actual == pytest.approx(expected, abs=1e-9)


def test_known_open_commands_target_neutral_flexion_and_mirrored_opposition():
    mapper = HandCommandMapper()

    left = mapper.map(HandSide.LEFT, OPEN_LEFT).targets
    right = mapper.map(HandSide.RIGHT, OPEN_RIGHT).targets

    for finger in FINGERS:
        assert left[f"left_{finger}_MCP_FE"] == 0.0
        assert left[f"left_{finger}_PIP"] == 0.0
        assert left[f"left_{finger}_DIP"] == 0.0
        assert right[f"right_{finger}_MCP_FE"] == 0.0
        assert right[f"right_{finger}_PIP"] == 0.0
        assert right[f"right_{finger}_DIP"] == 0.0

    assert left["left_thumb_CMC_FE"] == 0.0
    assert right["right_thumb_CMC_FE"] == 0.0
    assert_close(left["left_thumb_CMC_AA"], 0.3491)
    assert_close(right["right_thumb_CMC_AA"], -0.3491)


def test_known_close_commands_map_to_exact_mjcf_joint_names():
    mapper = HandCommandMapper()

    left = mapper.map("left", CLOSE_LEFT).targets
    right = mapper.map("right", CLOSE_RIGHT).targets

    assert set(left) == {
        "left_thumb_CMC_FE",
        "left_thumb_MCP_FE",
        "left_thumb_IP",
        "left_thumb_CMC_AA",
        "left_index_MCP_FE",
        "left_index_PIP",
        "left_index_DIP",
        "left_middle_MCP_FE",
        "left_middle_PIP",
        "left_middle_DIP",
        "left_ring_MCP_FE",
        "left_ring_PIP",
        "left_ring_DIP",
        "left_pinky_MCP_FE",
        "left_pinky_PIP",
        "left_pinky_DIP",
    }
    assert set(right) == {name.replace("left_", "right_") for name in set(left)}

    assert_close(left["left_thumb_CMC_FE"], 1.9199 * 0.69)
    assert_close(left["left_index_MCP_FE"], 1.5708 * 0.42)
    assert_close(left["left_middle_MCP_FE"], 1.5708 * 0.54)
    assert_close(left["left_ring_MCP_FE"], 1.5708 * 0.61)
    assert_close(left["left_pinky_MCP_FE"], 1.5708 * 0.60)

    assert_close(right["right_thumb_CMC_FE"], -1.9199 * 0.69)
    assert_close(right["right_index_MCP_FE"], -1.5708 * 0.42)
    assert_close(right["right_middle_MCP_FE"], -1.5708 * 0.44)
    assert_close(right["right_ring_MCP_FE"], -1.5708 * 0.61)
    assert_close(right["right_pinky_MCP_FE"], -1.5708 * 0.60)


@pytest.mark.parametrize(
    "command",
    [
        [0, 0, 0, 0, 0],
        [0, 0, 0, 0, 0, 0, 0],
        [-1, 0, 0, 0, 0, 0],
        [0, 101, 0, 0, 0, 0],
        [0, 0, math.inf, 0, 0, 0],
        [0, 0, math.nan, 0, 0, 0],
    ],
)
def test_invalid_inputs_are_rejected(command):
    mapper = HandCommandMapper()

    with pytest.raises(ValueError):
        mapper.map(HandSide.LEFT, command)


def test_targets_are_clamped_to_actual_joint_limits():
    defaults = default_calibrations()
    left = defaults[HandSide.LEFT]
    custom_left = HandCalibration(
        side=HandSide.LEFT,
        prefix=left.prefix,
        joint_limits=left.joint_limits,
        finger_full_flexion={finger: 10.0 for finger in FINGERS},
        thumb_flexion_full=ThumbFlexionCoupling(cmc_fe=10.0, mcp_fe=10.0, ip=10.0),
        thumb_opposition_full=10.0,
    )
    mapper = HandCommandMapper({HandSide.LEFT: custom_left, HandSide.RIGHT: defaults[HandSide.RIGHT]})

    targets = mapper.map(HandSide.LEFT, [100, 100, 100, 100, 100, 100]).targets

    assert targets["left_thumb_CMC_FE"] == 1.9199
    assert targets["left_thumb_MCP_FE"] == 1.3963
    assert targets["left_thumb_IP"] == 1.7453
    assert targets["left_thumb_CMC_AA"] == 0.3491
    for finger in FINGERS:
        assert targets[f"left_{finger}_MCP_FE"] == 1.5708
        assert targets[f"left_{finger}_PIP"] == 1.5708
        assert targets[f"left_{finger}_DIP"] == 1.3963


def test_follower_joint_coupling_is_configurable_per_finger_and_hand():
    defaults = default_calibrations()
    left = defaults[HandSide.LEFT]
    custom_left = HandCalibration(
        side=HandSide.LEFT,
        prefix=left.prefix,
        joint_limits=left.joint_limits,
        finger_full_flexion={finger: 1.0 for finger in FINGERS},
        finger_couplings={"index": FingerCoupling(pip=0.5, dip=0.25)},
    )
    mapper = HandCommandMapper({HandSide.LEFT: custom_left, HandSide.RIGHT: defaults[HandSide.RIGHT]})

    targets = mapper.map(HandSide.LEFT, [0, 0, 50, 50, 50, 50]).targets

    assert_close(targets["left_index_MCP_FE"], 0.5)
    assert_close(targets["left_index_PIP"], 0.25)
    assert_close(targets["left_index_DIP"], 0.125)
    assert_close(targets["left_middle_MCP_FE"], 0.5)
    assert_close(targets["left_middle_PIP"], 0.5)
    assert_close(targets["left_middle_DIP"], 0.5)


def test_left_right_mirroring_for_equivalent_commands():
    mapper = HandCommandMapper()
    command = [75, 25, 20, 40, 60, 80]

    left = mapper.map(HandSide.LEFT, command).targets
    right = mapper.map(HandSide.RIGHT, command).targets

    comparable_suffixes = [
        "thumb_CMC_FE",
        "thumb_MCP_FE",
        "thumb_IP",
        "thumb_CMC_AA",
        "index_MCP_FE",
        "index_PIP",
        "index_DIP",
        "middle_MCP_FE",
        "middle_PIP",
        "middle_DIP",
        "ring_MCP_FE",
        "ring_PIP",
        "ring_DIP",
        "pinky_MCP_FE",
        "pinky_PIP",
        "pinky_DIP",
    ]
    for suffix in comparable_suffixes:
        assert_close(left[f"left_{suffix}"], -right[f"right_{suffix}"])


def test_isaac_controller_keeps_left_and_right_hands_independent_and_preserves_other_targets():
    mapper = HandCommandMapper()
    hand_joint_names = list(mapper.controlled_joint_names)
    all_joint_names = ["base_x", "left_hip", *hand_joint_names, "head_yaw"]
    fake = FakeArticulation(all_joint_names)
    controller = IsaacHandController(fake, mapper=mapper)

    applied = controller.apply_both(CLOSE_LEFT, OPEN_RIGHT)

    assert fake.targets[0] == 10.0
    assert fake.targets[1] == 11.0
    assert fake.targets[-1] == 10.0 + len(all_joint_names) - 1
    assert_close(fake.targets[all_joint_names.index("left_index_MCP_FE")], applied["left_index_MCP_FE"])
    assert fake.targets[all_joint_names.index("right_index_MCP_FE")] == 0.0
    assert "right_index_MCP_FE" in applied
    assert "left_index_MCP_FE" in applied


class FakeArticulation:
    def __init__(self, dof_names):
        self.dof_names = dof_names
        self.targets = [10.0 + index for index, _ in enumerate(dof_names)]

    def get_dof_index(self, name):
        return self.dof_names.index(name)

    def get_joint_position_targets(self):
        return list(self.targets)

    def set_joint_position_targets(self, values):
        self.targets = list(values)
