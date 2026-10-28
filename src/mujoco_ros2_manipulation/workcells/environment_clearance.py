"""Experimental sampled robot/environment clearance, separate from inter-arm policy.

The native result covers moving robot collision geometries versus fixed outside-
root environment geometries. It honors collision masks and excludes, omitting
only static world-welded robot mounts. Distances use compiled MuJoCo collision
shapes (including convex mesh approximations). Sampled clearance and a tracking
reserve are not continuous-time certification or a physical safety guarantee;
the native contact interlock remains necessary.
"""
import math
import hashlib
import yaml


ENVIRONMENT_RESULT_SCOPE = "moving_robot_vs_fixed_environment"
ENVIRONMENT_DISTANCE_SEMANTICS = (
    "Sampled signed MuJoCo collision-shape distance between moving robot geometry "
    "and fixed outside-root environment geometry; native masks/excludes apply; "
    "static world-welded robot mounts excluded; no continuous-time certification"
)
MAX_ENVIRONMENT_PAIRS = 4096
_POLICY_KEYS = {"schema_version", "min_environment_distance_m", "tracking_reserve_m"}
_POLICY_BOUNDS = {"min_environment_distance_m": (.001, .1),
                  "tracking_reserve_m": (0., .05)}
_RESULT_KEYS = {"distance_m", "bodies", "nearest_points_m", "pair_count", "scope"}


class EnvironmentClearanceRejected(RuntimeError):
    """Sampled environment clearance failed a planning or observed-motion gate."""


def _finite(value):
    try:
        return type(value) in (int, float) and math.isfinite(value)
    except OverflowError:
        return False


def _require_policy(policy):
    if (type(policy) is not dict or set(policy) != _POLICY_KEYS
            or type(policy["schema_version"]) is not int or policy["schema_version"] != 1):
        raise ValueError("Expected environment-clearance policy schema 1 with exact fields")
    if any(not _finite(policy[name]) or not lower <= policy[name] <= upper
           for name, (lower, upper) in _POLICY_BOUNDS.items()):
        raise ValueError("Environment-clearance policy is outside experimental simulation bounds")
    return policy


def load_environment_policy(cell):
    """Load the global fixed-environment policy; the caller binds model identity."""
    path, policy, _ = environment_policy_snapshot(cell)
    return path, policy


def environment_policy_snapshot(cell):
    """Bind the interpreted policy and digest to the very same file bytes."""
    path = cell.root / "config/planning/environment_clearance.yaml"
    raw = path.read_bytes()
    policy = yaml.safe_load(raw)
    return path, _require_policy(policy), hashlib.sha256(raw).hexdigest()


def required_environment_distance(policy):
    """Return planned sample clearance: minimum plus experimental tracking reserve."""
    policy = _require_policy(policy)
    return float(policy["min_environment_distance_m"] + policy["tracking_reserve_m"])


def require_environment_result(result):
    """Validate the exact native ``at(state)['environment']`` observation.

    Missing geometry or malformed native data must fail closed. A negative signed
    distance is valid evidence of overlap; this function validates the observation,
    while the caller compares it to the policy threshold.
    """
    if type(result) is not dict or set(result) != _RESULT_KEYS:
        raise ValueError("Expected complete native environment-distance fields")
    if not _finite(result["distance_m"]):
        raise ValueError("Environment distance must be finite")
    if (type(result["pair_count"]) is not int
            or not 1 <= result["pair_count"] <= MAX_ENVIRONMENT_PAIRS):
        raise ValueError("Expected positive bounded environment pair count")
    if type(result["scope"]) is not str or result["scope"] != ENVIRONMENT_RESULT_SCOPE:
        raise ValueError("Unexpected environment-distance scope")
    bodies = result["bodies"]
    if (type(bodies) is not list or len(bodies) != 2
            or any(type(name) is not str or not name.strip() or len(name) > 256 for name in bodies)
            or bodies[0] == bodies[1]):
        raise ValueError("Expected two distinct bounded environment body names")
    points = result["nearest_points_m"]
    if (type(points) is not list or len(points) != 2
            or any(type(point) is not list or len(point) != 3
                   or not all(_finite(value) for value in point) for point in points)):
        raise ValueError("Expected two finite three-dimensional environment witness points")
    return result
