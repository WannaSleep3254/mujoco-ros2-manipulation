"""Strict world-frame TCP goals and explicit tool-reference transforms.

These definitions describe a coordinate reference only. They do not add tool
geometry, a gripper controller, mass, collision shapes or physical grasping.
The digest identifies this reference configuration, not a manufactured tool.
"""
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
import hashlib
import json
import math
import re

from .motion_request import request_id


GOAL_FIELDS = frozenset(("schema_version", "request_id", "arm_id", "tool_id",
                         "tool_sha256", "frame_id", "position_m", "quaternion_xyzw"))
POSE_FIELDS = frozenset(("frame_id", "position_m", "quaternion_xyzw"))
QUATERNION_NORM_TOLERANCE = 1e-6
QUATERNION_NORMALIZATION_ROUNDOFF = 1e-14
TOOL_SCOPE = "origin_reference_only"


def _finite(value):
    try:
        return type(value) in (int, float) and math.isfinite(value)
    except OverflowError:
        return False


def _identifier(value, field):
    if type(value) is not str or not re.fullmatch(r"[a-z][a-z0-9_]{0,63}", value):
        raise ValueError(field + " must be a lowercase identifier of at most 64 characters")
    return value


def validate_frame_name(value):
    """Require a bounded relative TF link name; no frame lookup is performed."""
    if (type(value) is not str or not re.fullmatch(r"[A-Za-z][A-Za-z0-9_/]{0,127}", value)
            or "//" in value or value.endswith("/")):
        raise ValueError("Expected a bounded relative TF frame name")
    return value


def _vector(value, size, name):
    if (not isinstance(value, Sequence) or isinstance(value, (str, bytes))
            or len(value) != size or not all(_finite(v) for v in value)):
        raise ValueError(name + " must contain exactly " + str(size) + " finite numbers")
    return tuple(float(v) if v != 0 else 0.0 for v in value)


def _quaternion(value):
    q = _vector(value, 4, "quaternion_xyzw")
    norm = math.hypot(*q)
    if not math.isfinite(norm) or abs(norm - 1.0) > QUATERNION_NORM_TOLERANCE:
        raise ValueError("quaternion_xyzw must have unit norm within 1e-6")
    # A normalized float quaternion can have a norm one ULP away from one.
    # Dividing it again can change its components on every validation, breaking
    # an otherwise unchanged goal's exact review/digest identity.
    if abs(norm - 1.0) <= QUATERNION_NORMALIZATION_ROUNDOFF:
        return q
    return tuple(v / norm if v != 0 else 0.0 for v in q)


def _canonical_quaternion(value):
    q = _quaternion(value)
    # q and -q describe the same frame. Prefer positive w, then z/y/x for w=0.
    sign = next((1 if v > 0 else -1 for v in reversed(q) if v != 0), 1)
    return tuple(sign * v if v != 0 else 0.0 for v in q)


@dataclass(frozen=True)
class ToolDefinition:
    """Immutable transform from a named robot tip to a TCP reference."""

    id: str
    parent_tip: str
    translation_m: tuple = (0.0, 0.0, 0.0)
    quaternion_xyzw: tuple = (0.0, 0.0, 0.0, 1.0)
    source: str = "configured_tcp_reference"

    def __post_init__(self):
        _identifier(self.id, "tool id")
        validate_frame_name(self.parent_tip)
        if (type(self.source) is not str or not self.source.strip()
                or len(self.source) > 256 or any(ord(c) < 32 for c in self.source)):
            raise ValueError("Tool source must be a nonempty bounded description")
        object.__setattr__(self, "translation_m", _vector(self.translation_m, 3, "translation_m"))
        object.__setattr__(self, "quaternion_xyzw", _canonical_quaternion(self.quaternion_xyzw))

    def record(self):
        return dict(schema_version=1, id=self.id, parent_tip=self.parent_tip,
                    translation_m=list(self.translation_m), quaternion_xyzw=list(self.quaternion_xyzw),
                    source=self.source, scope=TOOL_SCOPE)

    @property
    def sha256(self):
        raw = json.dumps(self.record(), sort_keys=True, separators=(",", ":"), allow_nan=False)
        return hashlib.sha256(raw.encode("utf-8")).hexdigest()


def validate_pose_goal(data, *, arm_ids=None):
    """Validate the eight-field v1 command and return a detached value.

    The caller resolves the arm and tool registry and owns model identity,
    state freshness, IK admission, clearance, planning and execution approval.
    """
    if (not isinstance(data, Mapping) or set(data) != GOAL_FIELDS
            or type(data["schema_version"]) is not int or data["schema_version"] != 1):
        raise ValueError("Expected pose goal schema v1 and its eight declared fields")
    request_id(data["request_id"])
    _identifier(data["arm_id"], "arm_id")
    _identifier(data["tool_id"], "tool_id")
    if arm_ids is not None and data["arm_id"] not in arm_ids:
        raise ValueError("Unknown pose goal arm_id")
    digest = data["tool_sha256"]
    if type(digest) is not str or not re.fullmatch(r"[0-9a-f]{64}", digest):
        raise ValueError("tool_sha256 must be a lowercase SHA-256 digest")
    pose = validate_world_pose({key: data[key] for key in POSE_FIELDS})
    return dict(schema_version=1, request_id=data["request_id"], arm_id=data["arm_id"],
                tool_id=data["tool_id"], tool_sha256=digest, **pose)


def validate_world_pose(data):
    if not isinstance(data, Mapping) or set(data) != POSE_FIELDS or data["frame_id"] != "world":
        raise ValueError("Expected an explicit world-frame position and quaternion pose")
    return dict(frame_id="world", position_m=list(_vector(data["position_m"], 3, "position_m")),
                quaternion_xyzw=list(_quaternion(data["quaternion_xyzw"])))


def _multiply(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (aw*bx + ax*bw + ay*bz - az*by,
            aw*by - ax*bz + ay*bw + az*bx,
            aw*bz + ax*by - ay*bx + az*bw,
            aw*bw - ax*bx - ay*by - az*bz)


def _rotate(q, vector):
    result = _multiply(_multiply(q, (*vector, 0.0)), (-q[0], -q[1], -q[2], q[3]))
    return result[:3]


def _tool_goal(goal, tool):
    if not isinstance(tool, ToolDefinition):
        raise ValueError("Expected an explicit ToolDefinition")
    checked = validate_pose_goal(goal)
    if checked["tool_id"] != tool.id or checked["tool_sha256"] != tool.sha256:
        raise ValueError("Pose goal tool identity differs from the configured TCP reference")
    return checked


def goal_tip_pose(goal, tool):
    """Compute world_T_tip = world_T_tcp * inverse(tip_T_tcp)."""
    goal = _tool_goal(goal, tool)
    q = tool.quaternion_xyzw
    tip_q = _multiply(goal["quaternion_xyzw"], (-q[0], -q[1], -q[2], q[3]))
    offset = _rotate(tip_q, tool.translation_m)
    return validate_world_pose(dict(frame_id="world",
        position_m=[goal["position_m"][i] - offset[i] for i in range(3)], quaternion_xyzw=tip_q))


def tcp_pose_from_tip(tip_pose, tool):
    """Apply the configured tip_T_tcp reference to a world-frame FK result."""
    if not isinstance(tool, ToolDefinition):
        raise ValueError("Expected an explicit ToolDefinition")
    tip_pose = validate_world_pose(tip_pose)
    q = tip_pose["quaternion_xyzw"]
    offset = _rotate(q, tool.translation_m)
    return validate_world_pose(dict(frame_id="world",
        position_m=[tip_pose["position_m"][i] + offset[i] for i in range(3)],
        quaternion_xyzw=_multiply(q, tool.quaternion_xyzw)))


def tcp_pose_error(goal, tip_pose, tool):
    """Compare FK-derived TCP with the goal; quaternion signs are equivalent."""
    goal = _tool_goal(goal, tool)
    actual = tcp_pose_from_tip(tip_pose, tool)
    q = goal["quaternion_xyzw"]
    relative = _multiply((-q[0], -q[1], -q[2], q[3]), actual["quaternion_xyzw"])
    return dict(position_error_m=math.dist(goal["position_m"], actual["position_m"]),
                orientation_error_rad=2*math.atan2(math.hypot(*relative[:3]), abs(relative[3])))
