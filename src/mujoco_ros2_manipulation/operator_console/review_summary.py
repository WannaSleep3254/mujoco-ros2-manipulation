"""Small read-only plan facts for the operator screen; never an approval input."""
import math


def _finite(value):
    try:
        return type(value) in (int, float) and math.isfinite(value)
    except OverflowError:
        return False


def summarize_review(record):
    """Project a complete sampled path check and the initial-to-target change.

    The immutable review payload/digest remains the execution authority. The
    displayed deltas are signed joint endpoint changes in radians, not path
    length, TCP displacement, or a comparison with the latest live posture.
    Older traces may omit the minimum's time/body names; omit only those facts.
    """
    command = record.get("command")
    quality = record.get("planned_clearance")
    initial = record.get("initial_state_rad")
    if (not isinstance(command, dict)
            or command.get("strategy") not in ("coordinated", "departure_timing")
            or not isinstance(quality, dict) or quality.get("complete") is not True
            or not isinstance(initial, dict) or not initial):
        return None
    target, minimum = command.get("target_rad"), quality.get("minimum")
    duration = quality.get("duration_sec")
    if (not isinstance(target, dict) or set(target) != set(initial)
            or not isinstance(minimum, dict) or not _finite(minimum.get("distance_m"))
            or not _finite(duration) or duration < 0
            or not all(isinstance(name, str) and name and _finite(initial[name])
                       and _finite(target[name]) for name in initial)):
        return None
    delta = {name: target[name] - initial[name] for name in initial}
    if not all(_finite(value) for value in delta.values()):
        return None
    stamp, bodies = minimum.get("time_sec"), minimum.get("bodies")
    stamp = stamp if _finite(stamp) and 0 <= stamp <= duration else None
    pair = (list(bodies) if isinstance(bodies, (tuple, list)) and len(bodies) == 2
            and all(isinstance(name, str) and name for name in bodies) else None)
    return dict(strategy=command["strategy"], duration_sec=duration,
                minimum=dict(distance_m=minimum["distance_m"], time_sec=stamp, pair=pair),
                target_delta_rad=delta,
                max_abs_target_delta_rad=max(abs(value) for value in delta.values()))
