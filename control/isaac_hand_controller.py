"""Isaac Sim articulation adapter for Sharpa WAVE hand joint targets."""

from __future__ import annotations

from inspect import signature
from typing import Dict, Iterable, Mapping, Sequence

from .hand_command_mapper import HandCommandMapper, HandSide


class IsaacHandController:
    """Apply mapped hand targets to an existing Isaac articulation only by name.

    The controller resolves joint indices once from the provided articulation and
    only writes indices owned by the Sharpa WAVE hands.  If the Isaac API accepts
    sparse ``joint_indices``, sparse updates are used.  Otherwise the controller
    copies the current position target vector and overwrites only hand entries
    before submitting the full vector.
    """

    def __init__(self, articulation, mapper: HandCommandMapper | None = None) -> None:
        self.articulation = articulation
        self.mapper = mapper or HandCommandMapper()
        self._joint_indices = self._resolve_joint_indices(self.mapper.controlled_joint_names)

    @property
    def joint_indices_by_name(self) -> Mapping[str, int]:
        return dict(self._joint_indices)

    def apply(self, side: HandSide | str, command: Sequence[float]) -> Mapping[str, float]:
        """Map and apply one side's command, returning the applied target map."""

        mapped = self.mapper.map(side, command)
        self.apply_targets(mapped.targets)
        return mapped.targets

    def apply_both(self, left_command: Sequence[float], right_command: Sequence[float]) -> Mapping[str, float]:
        """Apply independent left and right commands from one ROS message."""

        targets: Dict[str, float] = {}
        targets.update(self.mapper.map(HandSide.LEFT, left_command).targets)
        targets.update(self.mapper.map(HandSide.RIGHT, right_command).targets)
        self.apply_targets(targets)
        return targets

    def apply_targets(self, targets: Mapping[str, float]) -> None:
        names = list(targets.keys())
        indices = [self._joint_indices[name] for name in names]
        values = [float(targets[name]) for name in names]
        setter = getattr(self.articulation, "set_joint_position_targets")

        if self._setter_accepts_joint_indices(setter):
            setter(values, joint_indices=indices)
            return

        current_targets = list(self._read_current_position_targets())
        for index, value in zip(indices, values):
            current_targets[index] = value
        setter(current_targets)

    def _resolve_joint_indices(self, joint_names: Iterable[str]) -> Dict[str, int]:
        resolved: Dict[str, int] = {}
        for name in joint_names:
            resolved[name] = self._resolve_one_joint_index(name)
        return resolved

    def _resolve_one_joint_index(self, name: str) -> int:
        for method_name in ("get_dof_index", "get_joint_index"):
            method = getattr(self.articulation, method_name, None)
            if method is not None:
                try:
                    index = int(method(name))
                    if index < 0:
                        raise ValueError
                    return index
                except (KeyError, ValueError):
                    pass
        for attr_name in ("dof_names", "joint_names"):
            names = getattr(self.articulation, attr_name, None)
            if names is not None:
                return list(names).index(name)
        raise ValueError(f"could not resolve joint index for {name!r}")

    def _read_current_position_targets(self) -> Sequence[float]:
        for method_name in ("get_joint_position_targets", "get_applied_action"):
            method = getattr(self.articulation, method_name, None)
            if method is None:
                continue
            value = method()
            if method_name == "get_applied_action" and hasattr(value, "joint_positions"):
                value = value.joint_positions
            if value is not None:
                return value
        count = getattr(self.articulation, "num_dof", None) or getattr(self.articulation, "num_joints", None)
        if count is None:
            for attr_name in ("dof_names", "joint_names"):
                names = getattr(self.articulation, attr_name, None)
                if names is not None:
                    count = len(names)
                    break
        if count is None:
            raise RuntimeError("articulation does not expose current targets or joint count")
        return [0.0] * int(count)

    @staticmethod
    def _setter_accepts_joint_indices(setter) -> bool:
        try:
            parameters = signature(setter).parameters
        except (TypeError, ValueError):
            return True
        return "joint_indices" in parameters
