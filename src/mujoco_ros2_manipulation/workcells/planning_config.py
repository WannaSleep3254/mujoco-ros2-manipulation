"""Strict internal planner/cache options, separate from public motion commands."""
import hashlib
import json
import math


PLANNER_TYPES = {"RRTConnectkConfigDefault": "geometric::RRTConnect",
                 "RRTkConfigDefault": "geometric::RRT"}
DEFAULT_PLANNER_ID = "RRTConnectkConfigDefault"


def canonical_sha256(value):
    return hashlib.sha256(json.dumps(value, sort_keys=True, separators=(",", ":"),
                                    allow_nan=False).encode()).hexdigest()


def planning_configuration(planner_id=DEFAULT_PLANNER_ID, trajectory_cache=False,
                           cache_start_tolerance_rad=0.0):
    if type(planner_id) is not str or planner_id not in PLANNER_TYPES:
        raise ValueError("Expected an explicitly registered RRTConnect or RRT planner ID")
    if type(trajectory_cache) is not bool:
        raise ValueError("Trajectory cache must be an explicit boolean opt-in")
    if (type(cache_start_tolerance_rad) not in (int, float)
            or not math.isfinite(cache_start_tolerance_rad)
            or not 0 <= cache_start_tolerance_rad <= .001
            or (not trajectory_cache and cache_start_tolerance_rad != 0)):
        raise ValueError("Cache start tolerance requires opt-in and must be within 0..0.001rad")
    return dict(planner_id=planner_id, planner_type=PLANNER_TYPES[planner_id],
                trajectory_cache=trajectory_cache,
                cache_start_tolerance_rad=float(cache_start_tolerance_rad))


def trajectory_cache_identity(runtime_identity, *, source_sha256, policy, planner):
    """Hash every checked runtime/build field, with explicit no-tool semantics."""
    if (type(runtime_identity) is not dict or type(runtime_identity.get("build")) is not dict
            or type(runtime_identity.get("planning_scene_sha256")) is not str):
        raise ValueError("Cache requires the complete checked runtime scene identity")
    return dict(source_sha256=source_sha256,
        model_sha256=canonical_sha256(runtime_identity["build"]),
        environment_sha256=canonical_sha256(runtime_identity), tool_sha256=None,
        policy_sha256=canonical_sha256(policy), planner_sha256=canonical_sha256(planner))
