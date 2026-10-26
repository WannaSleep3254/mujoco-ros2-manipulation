"""Pure identity and transport checks for one read-only scene view.

This helper does not choose models, admit robot commands, assign freshness, or
advance a generation. Those decisions remain with the console owner. A binding
ID identifies the exact declared view; compiled scene/model identities remain in
the pose and must also match the currently selected manifest at the receiver.
"""
from copy import deepcopy
import hashlib
import json
import math
import re


_BINDING_FIELDS = frozenset(("operator_session_id", "generation", "mode", "source_id",
                            "job_id", "run_id", "motion_session_id", "request_id"))
_POSE_FIELDS = frozenset(("schema_version", "scene_sha256", "model_sha256", "geoms"))
_GEOM_FIELDS = frozenset(("id", "position", "rotation"))
_ID32 = re.compile(r"[0-9a-f]{32}")
_SHA256 = re.compile(r"[0-9a-f]{64}")
_SOURCE = re.compile(r"(?:(?:workcell|profile|scene):)?[a-z][a-z0-9_-]*")
_RUN = re.compile(r"[A-Za-z0-9][A-Za-z0-9_.-]*")
_SAFE_INTEGER = 2**53 - 1
_MAX_GEOMS = 10_000
_MAX_COORDINATE_M = 1e9
_MAX_TIME_SEC = 1e9
_ROTATION_TOLERANCE = 1e-4


def _require(condition, message):
    if not condition:
        raise ValueError(message)


def _exact(value, fields, label):
    _require(type(value) is dict and set(value) == fields, f"Invalid {label} fields")


def _identifier(value, pattern, label, maximum):
    _require(isinstance(value, str) and len(value) <= maximum
             and pattern.fullmatch(value) is not None, f"Invalid {label}")
    return value


def _uuid(value, label):
    return _identifier(value, _ID32, label, 32)


def _integer(value, label, minimum=0):
    _require(type(value) is int and minimum <= value <= _SAFE_INTEGER, f"Invalid {label}")
    return value


def _number(value, label, maximum, *, nonnegative=False):
    _require(type(value) in (int, float), f"Invalid {label}")
    _require(abs(value) <= maximum and math.isfinite(value) and (not nonnegative or value >= 0),
             f"Invalid {label}")
    return value


def _checked_binding(value):
    _exact(value, _BINDING_FIELDS, "binding")
    _uuid(value["operator_session_id"], "operator session")
    _integer(value["generation"], "view generation")
    _identifier(value["source_id"], _SOURCE, "scene source identifier", 128)
    _require(value["mode"] in ("live", "model_preview", "plan_preview"), "Invalid view mode")
    for name in ("job_id", "motion_session_id", "request_id"):
        if value[name] is not None:
            _uuid(value[name], name)
    if value["run_id"] is not None:
        _identifier(value["run_id"], _RUN, "run identifier", 128)
    _require((value["job_id"] is None) == (value["run_id"] is None),
             "Job and run identities must be paired")
    if value["mode"] == "live":
        _require(value["request_id"] is None, "Live views do not select a saved request")
    elif value["mode"] == "model_preview":
        _require(all(value[name] is None for name in ("job_id", "run_id", "motion_session_id", "request_id")),
                 "Model preview has no execution identities")
    else:
        _require(value["request_id"] is not None, "Plan preview requires a saved request identity")
    return dict(value)


def make_binding(operator_session_id, generation, source_id, mode, *, job_id=None,
                 run_id=None, motion_session_id=None, request_id=None):
    """Construct the complete eight-field view identity, including null fields.

    ``source_id`` is a declared identifier, for example ``workcell:fr5_ur5e`` or
    ``profile:fr5``. It is never a filename, URL, or arbitrary filesystem path.
    This validates declarations only; it does not prove ownership or admission.
    """
    return _checked_binding(dict(operator_session_id=operator_session_id, generation=generation,
        source_id=source_id, mode=mode, job_id=job_id, run_id=run_id,
        motion_session_id=motion_session_id, request_id=request_id))


def binding_id(binding):
    """SHA256 of sorted compact ASCII JSON after exact-field validation."""
    checked = _checked_binding(binding)
    encoded = json.dumps(checked, sort_keys=True, separators=(",", ":"),
                         ensure_ascii=True, allow_nan=False).encode("ascii")
    return hashlib.sha256(encoded).hexdigest()


def _vector(value, size, label, maximum):
    _require(type(value) is list and len(value) == size, f"Invalid {label}")
    for item in value:
        _number(item, label, maximum)


def _checked_pose(pose):
    _exact(pose, _POSE_FIELDS, "compiled pose")
    _require(type(pose["schema_version"]) is int and pose["schema_version"] == 1,
             "Invalid compiled pose schema")
    for name in ("scene_sha256", "model_sha256"):
        _identifier(pose[name], _SHA256, name, 64)
    geoms = pose["geoms"]
    _require(type(geoms) is list and len(geoms) <= _MAX_GEOMS, "Invalid pose geometry catalog")
    for index, geom in enumerate(geoms):
        _exact(geom, _GEOM_FIELDS, "geometry pose")
        _require(type(geom["id"]) is int and geom["id"] == index,
                 "Geometry poses must retain the complete compiled order")
        _vector(geom["position"], 3, "world geometry position", _MAX_COORDINATE_M)
        rotation = geom["rotation"]
        _vector(rotation, 9, "row-major geometry rotation", 1.000001)
        for row in range(3):
            for other in range(row, 3):
                dot = sum(rotation[row*3+column] * rotation[other*3+column] for column in range(3))
                _require(abs(dot - (1 if row == other else 0)) <= _ROTATION_TOLERANCE,
                         "Geometry rotation must be orthonormal")
        a, b, c, d, e, f, g, h, i = rotation
        determinant = a*(e*i-f*h) - b*(d*i-f*g) + c*(d*h-e*g)
        _require(abs(determinant - 1) <= _ROTATION_TOLERANCE,
                 "Geometry rotation must preserve orientation")
    return deepcopy(pose)


def envelope(binding, pose, sequence, *, source_state_age_sec=None, simulation_sec=None,
             preview_time_sec=None):
    """Copy one validated full pose into the frontend's view envelope.

    Live age is the caller's receiver-relative age, not a client or simulation
    timestamp. A stale finite age remains visible as stale; this function does
    not silently refresh or reject it. Preview age is null and plan playback has
    its own nonnegative ``preview_time_sec``. The receiver must still compare the
    binding, model/scene digest, catalog and increasing sequence to its manifest.
    """
    checked = _checked_binding(binding)
    _integer(sequence, "pose sequence", minimum=1)
    if checked["mode"] == "live":
        _number(source_state_age_sec, "live source age", _MAX_TIME_SEC, nonnegative=True)
    else:
        _require(source_state_age_sec is None, "Preview source age must be null")
    if simulation_sec is not None:
        _number(simulation_sec, "simulation stamp", _MAX_TIME_SEC, nonnegative=True)
    if checked["mode"] == "plan_preview":
        _number(preview_time_sec, "plan preview time", _MAX_TIME_SEC, nonnegative=True)
    else:
        _require(preview_time_sec is None, "Only plan preview carries a playback time")
    result = dict(schema_version=1, binding_id=binding_id(checked), binding=checked,
                  sequence=sequence, source_state_age_sec=source_state_age_sec,
                  simulation_sec=simulation_sec, pose=_checked_pose(pose))
    if checked["mode"] == "plan_preview":
        result["preview_time_sec"] = preview_time_sec
    return result
