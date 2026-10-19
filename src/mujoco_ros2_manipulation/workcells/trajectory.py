"""Validate and interpolate joint trajectories in a complete shared robot state."""

import math


def sampled_states(names, points, current, *, max_step=0.025, max_dt=0.05):
    if not names or len(names) != len(set(names)) or not set(names).issubset(current):
        raise ValueError("Unknown or duplicate trajectory joints")
    if len(points) < 2:
        raise ValueError("Trajectory needs at least two points")
    previous = [current[name] for name in names]
    last = 0.0
    result = []
    for index, (stamp, positions) in enumerate(points):
        if len(positions) != len(names) or any(not math.isfinite(v) for v in positions):
            raise ValueError("Invalid positions")
        if not math.isfinite(stamp) or stamp < 0 or (index and stamp <= last):
            raise ValueError("Trajectory time must increase")
        if index == 0 and max(abs(a - b) for a, b in zip(previous, positions)) > 0.03:
            raise ValueError("Trajectory start does not match current state")
        count = max(
            1,
            math.ceil((stamp - last) / max_dt),
            math.ceil(max(abs(a - b) for a, b in zip(previous, positions)) / max_step),
        )
        if len(result) + count > 5000:
            raise ValueError("Trajectory exceeds validation sample budget")
        for step in range(1, count + 1):
            state = dict(current)
            state.update(
                {
                    name: a + (b - a) * step / count
                    for name, a, b in zip(names, previous, positions)
                }
            )
            result.append(state)
        previous = list(positions)
        last = stamp
    return result


def partitions(names, instances):
    if len(names) != len(set(names)):
        raise ValueError("Duplicate trajectory joints")
    expected = [name for inst in instances for name in inst.joints]
    if len(expected) != len(set(expected)):
        raise ValueError("Controller joint ownership overlaps")
    if set(names) != set(expected):
        raise ValueError("A coordinated trajectory must command all instance joints")
    return {inst.id: [names.index(name) for name in inst.joints] for inst in instances}
