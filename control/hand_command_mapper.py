"""Pure-Python mapping from Sharpa WAVE hand commands to MJCF joint targets.

The ROS command order is fixed as::

    [thumb_flexion, thumb_opposition, index, middle, ring, pinky]

All command values are continuous robot-control values in the inclusive range
0..100.  This module deliberately has no Isaac Sim or ROS dependencies so that
mapping and validation can be tested in a normal Python environment.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum
from math import isfinite
from typing import Dict, Iterable, Mapping, MutableMapping, Sequence, Tuple

JointTargets = Dict[str, float]
JointLimit = Tuple[float, float]

COMMAND_NAMES: Tuple[str, ...] = (
    "thumb_flexion",
    "thumb_opposition",
    "index",
    "middle",
    "ring",
    "pinky",
)
FINGERS: Tuple[str, ...] = ("index", "middle", "ring", "pinky")


class HandSide(str, Enum):
    """Supported hand sides."""

    LEFT = "left"
    RIGHT = "right"


@dataclass(frozen=True)
class FingerCoupling:
    """Follower-joint coupling coefficients for one finger."""

    pip: float = 1.0
    dip: float = 1.0


@dataclass(frozen=True)
class ThumbFlexionCoupling:
    """Thumb-flexion distribution over flexion/extension joints."""

    cmc_fe: float = 1.0
    mcp_fe: float = 1.0
    ip: float = 1.0


@dataclass(frozen=True)
class HandCalibration:
    """Explicit hand-specific mapping and calibration constants.

    The S40 MJCF files are mirrored: left flexion is positive while right
    flexion is negative.  The ``*_full_flexion`` values therefore carry the
    side-specific sign needed to make equal command values produce equivalent
    physical poses.
    """

    side: HandSide
    prefix: str
    joint_limits: Mapping[str, JointLimit]
    finger_full_flexion: Mapping[str, float]
    finger_zero: Mapping[str, float] = field(default_factory=dict)
    finger_couplings: Mapping[str, FingerCoupling] = field(default_factory=dict)
    thumb_flexion_full: ThumbFlexionCoupling = ThumbFlexionCoupling()
    thumb_flexion_zero: ThumbFlexionCoupling = ThumbFlexionCoupling(0.0, 0.0, 0.0)
    thumb_opposition_zero: float = 0.0
    thumb_opposition_full: float = 0.0

    def __post_init__(self) -> None:
        object.__setattr__(
            self,
            "finger_zero",
            {finger: self.finger_zero.get(finger, 0.0) for finger in FINGERS},
        )
        object.__setattr__(
            self,
            "finger_couplings",
            {finger: self.finger_couplings.get(finger, FingerCoupling()) for finger in FINGERS},
        )


@dataclass(frozen=True)
class HandTargets:
    """Mapped target angles for one hand."""

    side: HandSide
    targets: Mapping[str, float]

    @property
    def joint_names(self) -> Tuple[str, ...]:
        return tuple(self.targets.keys())


class HandCommandMapper:
    """Convert six Sharpa WAVE command values into MJCF joint targets."""

    def __init__(self, calibrations: Mapping[HandSide | str, HandCalibration] | None = None) -> None:
        raw_calibrations = calibrations or default_calibrations()
        self._calibrations = {HandSide(side): calibration for side, calibration in raw_calibrations.items()}

    @property
    def controlled_joint_names(self) -> Tuple[str, ...]:
        """All hand joints that this mapper can target, in deterministic order."""

        names = []
        for side in (HandSide.LEFT, HandSide.RIGHT):
            names.extend(self.map(side, [0, 0, 0, 0, 0, 0]).joint_names)
        return tuple(names)

    def map(self, side: HandSide | str, command: Sequence[float]) -> HandTargets:
        """Map one six-value command array for ``side`` to target angles.

        Raises:
            ValueError: if the side is unknown, the command length is not six,
                any value is non-finite, or any value is outside 0..100.
        """

        hand_side = HandSide(side)
        calibration = self._calibrations[hand_side]
        values = self._validate_command(command)
        targets: MutableMapping[str, float] = {}

        thumb_flexion = values[0] / 100.0
        targets[f"{calibration.prefix}_thumb_CMC_FE"] = self._clamp(
            self._lerp(calibration.thumb_flexion_zero.cmc_fe, calibration.thumb_flexion_full.cmc_fe, thumb_flexion),
            calibration.joint_limits[f"{calibration.prefix}_thumb_CMC_FE"],
        )
        targets[f"{calibration.prefix}_thumb_MCP_FE"] = self._clamp(
            self._lerp(calibration.thumb_flexion_zero.mcp_fe, calibration.thumb_flexion_full.mcp_fe, thumb_flexion),
            calibration.joint_limits[f"{calibration.prefix}_thumb_MCP_FE"],
        )
        targets[f"{calibration.prefix}_thumb_IP"] = self._clamp(
            self._lerp(calibration.thumb_flexion_zero.ip, calibration.thumb_flexion_full.ip, thumb_flexion),
            calibration.joint_limits[f"{calibration.prefix}_thumb_IP"],
        )

        thumb_opposition = values[1] / 100.0
        targets[f"{calibration.prefix}_thumb_CMC_AA"] = self._clamp(
            self._lerp(calibration.thumb_opposition_zero, calibration.thumb_opposition_full, thumb_opposition),
            calibration.joint_limits[f"{calibration.prefix}_thumb_CMC_AA"],
        )

        for offset, finger in enumerate(FINGERS, start=2):
            command_fraction = values[offset] / 100.0
            mcp_name = f"{calibration.prefix}_{finger}_MCP_FE"
            pip_name = f"{calibration.prefix}_{finger}_PIP"
            dip_name = f"{calibration.prefix}_{finger}_DIP"
            mcp_target = self._clamp(
                self._lerp(calibration.finger_zero[finger], calibration.finger_full_flexion[finger], command_fraction),
                calibration.joint_limits[mcp_name],
            )
            coupling = calibration.finger_couplings[finger]
            targets[mcp_name] = mcp_target
            targets[pip_name] = self._clamp(coupling.pip * mcp_target, calibration.joint_limits[pip_name])
            targets[dip_name] = self._clamp(coupling.dip * mcp_target, calibration.joint_limits[dip_name])

        return HandTargets(side=hand_side, targets=dict(targets))

    @staticmethod
    def _validate_command(command: Sequence[float]) -> Tuple[float, ...]:
        if len(command) != 6:
            raise ValueError(f"expected exactly 6 hand command values, got {len(command)}")
        values = tuple(float(value) for value in command)
        for name, value in zip(COMMAND_NAMES, values):
            if not isfinite(value):
                raise ValueError(f"{name} must be finite, got {value!r}")
            if value < 0.0 or value > 100.0:
                raise ValueError(f"{name} must be in the inclusive range 0..100, got {value}")
        return values

    @staticmethod
    def _lerp(start: float, end: float, fraction: float) -> float:
        return start + (end - start) * fraction

    @staticmethod
    def _clamp(value: float, limits: JointLimit) -> float:
        lower, upper = limits
        return min(max(value, lower), upper)


def _limits(prefix: str, sign: float) -> Dict[str, JointLimit]:
    """Actual S40 MJCF joint limits for one side."""

    if sign > 0.0:
        thumb_cmc_fe = (-0.1745, 1.9199)
        thumb_mcp_fe = (-0.5236, 1.3963)
        thumb_ip = (0.0, 1.7453)
        finger_mcp = (-0.174533, 1.5708)
        follower = {"pip": (0.0, 1.7453), "dip": (0.0, 1.3963)}
    else:
        thumb_cmc_fe = (-1.9199, 0.1745)
        thumb_mcp_fe = (-1.3963, 0.5236)
        thumb_ip = (-1.7453, 0.0)
        finger_mcp = (-1.5708, 0.174533)
        follower = {"pip": (-1.7453, 0.0), "dip": (-1.3963, 0.0)}

    limits: Dict[str, JointLimit] = {
        f"{prefix}_thumb_CMC_FE": thumb_cmc_fe,
        f"{prefix}_thumb_CMC_AA": (-0.3491, 0.3491),
        f"{prefix}_thumb_MCP_FE": thumb_mcp_fe,
        f"{prefix}_thumb_IP": thumb_ip,
    }
    for finger in FINGERS:
        limits[f"{prefix}_{finger}_MCP_FE"] = finger_mcp
        limits[f"{prefix}_{finger}_PIP"] = follower["pip"]
        limits[f"{prefix}_{finger}_DIP"] = follower["dip"]
    return limits


def default_calibrations() -> Mapping[HandSide, HandCalibration]:
    """Return explicit default calibrations for both S40 Sharpa WAVE hands."""

    left_prefix = "left"
    right_prefix = "right"
    left_full = {finger: 1.5708 for finger in FINGERS}
    right_full = {finger: -1.5708 for finger in FINGERS}
    return {
        HandSide.LEFT: HandCalibration(
            side=HandSide.LEFT,
            prefix=left_prefix,
            joint_limits=_limits(left_prefix, sign=1.0),
            finger_full_flexion=left_full,
            thumb_flexion_full=ThumbFlexionCoupling(cmc_fe=1.9199, mcp_fe=1.3963, ip=1.7453),
            thumb_opposition_zero=0.0,
            thumb_opposition_full=0.3491,
        ),
        HandSide.RIGHT: HandCalibration(
            side=HandSide.RIGHT,
            prefix=right_prefix,
            joint_limits=_limits(right_prefix, sign=-1.0),
            finger_full_flexion=right_full,
            thumb_flexion_full=ThumbFlexionCoupling(cmc_fe=-1.9199, mcp_fe=-1.3963, ip=-1.7453),
            thumb_opposition_zero=0.0,
            thumb_opposition_full=-0.3491,
        ),
    }
