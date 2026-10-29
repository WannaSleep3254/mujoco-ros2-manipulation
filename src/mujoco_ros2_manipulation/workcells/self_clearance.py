"""Separate sampled MuJoCo self clearance for simulation execution admission.

The planned 10 mm and observed-start 1 mm thresholds are experimental margins,
not measured tracking bounds, continuous collision certificates or hardware
safety limits. The native contact interlock and MoveIt checks remain required.
Historical cross-arm evaluators and reports do not enable this extra component.
"""
import hashlib
import math

import yaml

SELF_RESULT_SCOPE = "within_each_arm"
MAX_SELF_PAIRS = 4096
_POLICY_KEYS = {"schema_version", "planning_min_self_distance_m", "observed_start_min_self_distance_m", "sample_dt_sec"}
_RESULT_KEYS = {"distance_m", "geom_ids", "bodies", "nearest_points_m", "pair_count", "scope"}


class SelfClearanceRejected(RuntimeError):
    """A native self distance or its required coverage failed admission."""


def _finite(value):
    try:
        return type(value) in (int, float) and math.isfinite(value)
    except OverflowError:
        return False


def validate_self_policy(value):
    if (type(value) is not dict or set(value) != _POLICY_KEYS
            or type(value["schema_version"]) is not int or value["schema_version"] != 1
            or not _finite(value["planning_min_self_distance_m"])
            or not _finite(value["observed_start_min_self_distance_m"])
            or not .001 <= value["observed_start_min_self_distance_m"] <= value["planning_min_self_distance_m"] <= .05
            or type(value["sample_dt_sec"]) not in (int, float) or value["sample_dt_sec"] != .01):
        raise ValueError("Expected explicit bounded self-clearance policy schema 1")
    return value


def self_policy_snapshot(cell):
    path = cell.root / "config/planning/self_clearance.yaml"
    raw = path.read_bytes()
    return path, validate_self_policy(yaml.safe_load(raw)), hashlib.sha256(raw).hexdigest()


def require_self_result(value):
    if (type(value) is not dict or set(value) != _RESULT_KEYS
            or not _finite(value["distance_m"])
            or type(value["pair_count"]) is not int or not 1 <= value["pair_count"] <= MAX_SELF_PAIRS
            or value["scope"] != SELF_RESULT_SCOPE):
        raise ValueError("Expected complete bounded native within-arm self-clearance evidence")
    ids, bodies, points = value["geom_ids"], value["bodies"], value["nearest_points_m"]
    if (type(ids) is not list or len(ids) != 2 or any(type(x) is not int or x < 0 for x in ids) or ids[0] == ids[1]
            or type(bodies) is not list or len(bodies) != 2 or bodies[0] == bodies[1]
            or any(type(x) is not str or not x.strip() or len(x) > 256 for x in bodies)
            or type(points) is not list or len(points) != 2
            or any(type(p) is not list or len(p) != 3 or not all(_finite(x) for x in p) for p in points)):
        raise ValueError("Invalid native self-clearance witness")
    return value


def require_self_clearance(evaluator, state, threshold):
    if not _finite(threshold) or threshold <= 0:
        raise ValueError("Expected positive self-clearance admission threshold")
    if evaluator.self_collision_enabled is not True:
        raise SelfClearanceRejected("Native self-clearance component is not enabled")
    value = require_self_result(evaluator.at(state).get("self_collision"))
    if value["distance_m"] < threshold:
        raise SelfClearanceRejected("Native self-clearance below admission policy")
    return value


def self_evaluator_identity(evaluator, root):
    """Bind a legacy admission to the opened component and current file bytes."""
    from copy import deepcopy
    from mujoco_ros2_manipulation.model_build.integrity import file_sha256
    if evaluator.self_collision_enabled is not True:
        raise SelfClearanceRejected("Native self-clearance component is not enabled")
    metadata = evaluator.metadata()
    component = metadata.get("self_collision", {})
    files = {name: file_sha256(root/name) for name in (
        "native/path_distance.cpp", "native/path_distance_environment.cpp", "native/path_distance_self.cpp", "runtime/native/path_distance.so")}
    if (metadata.get("engine") != "MuJoCo 3.12.0"
            or metadata.get("source_sha256") != files["native/path_distance.cpp"]
            or metadata.get("library_sha256") != files["runtime/native/path_distance.so"]
            or component.get("source_sha256") != files["native/path_distance_self.cpp"]
            or component.get("aggregator_source_sha256") != files["native/path_distance_environment.cpp"]
            or component.get("library_sha256") != files["runtime/native/path_distance.so"]):
        raise SelfClearanceRejected("Native self-clearance files differ from the opened component")
    return dict(files_sha256=files, opened_component=deepcopy(component))
