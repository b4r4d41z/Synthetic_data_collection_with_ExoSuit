"""Episode-only final-position labels; no ROS, Isaac, action or reward semantics."""
import json
import math
import os
from pathlib import Path
import tempfile


def _vector(value, size, name):
    try:
        result = tuple(float(v) for v in value)
    except (TypeError, ValueError, OverflowError) as error:
        raise ValueError(f"Invalid {name}") from error
    if len(result) != size or not all(map(math.isfinite, result)):
        raise ValueError(f"{name} must have {size} finite values")
    return result


def final_position_outcome(state):
    """Return a boolean plus compact geometry audit in the BfmState frame.

    Goal orientation is xyzw. Rotate object-minus-center by its inverse, using
    a normalized quaternion. This tests the existing goal OBB, not full fruit
    containment, settling, task termination or a reward.
    """
    position = _vector(state.object_position, 3, "object_position")
    center = _vector(state.goal_center, 3, "goal_center")
    half = _vector(state.goal_half_extents, 3, "goal_half_extents")
    for name in ("object_velocity", "goal_linear_velocity", "goal_angular_velocity"):
        _vector(getattr(state, name), 3, name)
    if any(v <= 0 for v in half):
        raise ValueError("Goal half extents must be positive")
    q = _vector(state.goal_orientation, 4, "goal_orientation")
    norm = math.hypot(*q)
    if norm < 1e-12 or not math.isfinite(norm):
        raise ValueError("Invalid goal quaternion norm")
    x, y, z, w = (v / norm for v in q)
    dx, dy, dz = (a-b for a, b in zip(position, center))
    # R(q)^T * displacement. No stage units are involved: BfmState is SI.
    local = (
        (1-2*(y*y+z*z))*dx + 2*(x*y+z*w)*dy + 2*(x*z-y*w)*dz,
        2*(x*y-z*w)*dx + (1-2*(x*x+z*z))*dy + 2*(y*z+x*w)*dz,
        2*(x*z+y*w)*dx + 2*(y*z-x*w)*dy + (1-2*(x*x+y*y))*dz,
    )
    if not all(map(math.isfinite, local)):
        raise ValueError("Non-finite goal-local displacement")
    return all(abs(v) <= h for v, h in zip(local, half)), {
        "object_position": position, "goal_center": center,
        "goal_half_extents": half, "goal_orientation": [x, y, z, w],
    }


def atomic_outcome_update(path, outcome, identity):
    """Preserve all sidecar fields except success and its evaluation audit."""
    path = Path(path)
    metadata = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(metadata, dict):
        raise ValueError("Episode sidecar must contain a metadata object")
    bfm = metadata.get("bfm", {})
    episode = bfm.get("episode", {}) if isinstance(bfm, dict) else {}
    if not isinstance(episode, dict):
        episode = {}
    if (episode.get("episode_id"), episode.get("object_prim_path")) != identity:
        outcome = {"success": None, "success_evaluation": {
            **outcome["success_evaluation"], "review_required": True,
            "reason": "Sidecar episode/object identity mismatch"}}
    metadata["success"] = outcome["success"]
    metadata["success_evaluation"] = outcome["success_evaluation"]
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(mode="w", encoding="utf-8", dir=path.parent,
                                         prefix=".episode-outcome-", suffix=".tmp", delete=False) as stream:
            temporary = Path(stream.name)
            json.dump(metadata, stream, indent=2, ensure_ascii=False, allow_nan=False)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        if temporary is not None:
            temporary.unlink(missing_ok=True)
    return outcome
