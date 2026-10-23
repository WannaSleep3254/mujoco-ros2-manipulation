"""Pure composition of two serial arm paths into one full-workcell candidate."""
from collections.abc import Mapping, Sequence
from copy import deepcopy
import math

from mujoco_ros2_manipulation.workcells.interference import Spline


def _ordered_names(value):
    if (not isinstance(value, Sequence) or isinstance(value, (str, bytes))
            or not value or any(not isinstance(name, str) or not name.strip() for name in value)
            or len(value) != len(set(value))):
        raise ValueError("Expected ordered, unique, nonempty joint names")
    return tuple(value)


def _finite_values(values):
    try:
        return all(type(value) in (int, float) and math.isfinite(value) for value in values)
    except (OverflowError, TypeError):
        return False


def _copy_segment(segment):
    if not isinstance(segment, Spline):
        raise ValueError("Serial segments must be Spline instances")
    names = _ordered_names(segment.names)
    if not isinstance(segment.points, Sequence) or isinstance(segment.points, (str, bytes)):
        raise ValueError("Expected spline points")
    for point in segment.points:
        if not isinstance(point, Mapping) or set(point) != {"t", "p", "v", "a"}:
            raise ValueError("Expected spline fields t, p, v, a")
        if not _finite_values([point["t"]]):
            raise ValueError("Expected finite spline times")
        for field in ("p", "v", "a"):
            values = point[field]
            if (not isinstance(values, Sequence) or isinstance(values, (str, bytes))
                    or not _finite_values(values)):
                raise ValueError("Expected finite spline position/derivative arrays")
    # Revalidate the current points rather than trusting a possibly modified instance.
    return Spline(names, deepcopy(segment.points))


def compose_serial_paths(segments, initial, joint_names):
    """Return a detached full-joint Spline for two disjoint arm paths in order.

    Each arm starts within 0.01 rad of its virtual state. Its first position is
    anchored to that state, while its actual final position becomes the other
    leg's hold position. The second t=0 point shares the first leg's terminal
    point, so the result contains one strictly increasing time axis.

    Output points consistently include velocities and accelerations. Supplied
    interior derivatives are preserved, absent derivatives are zero-filled, and
    both endpoints of each leg stop with zero velocity and acceleration. This
    produces a new candidate whose interpolation can differ from its sources.
    The caller must validate the entire result with its normal sampled/native
    and MoveIt gates before dispatch; composition establishes no clearance.
    Existing Spline limits, including the combined 35-second limit, still apply.
    """
    names = _ordered_names(joint_names)
    if (not isinstance(initial, Mapping) or set(initial) != set(names)
            or not _finite_values(initial.values())):
        raise ValueError("Initial state must cover all named joints with finite positions")
    if (not isinstance(segments, Sequence) or isinstance(segments, (str, bytes))
            or len(segments) != 2):
        raise ValueError("Exactly two ordered serial segments are required")
    paths = [_copy_segment(segment) for segment in segments]
    covered = [name for path in paths for name in path.names]
    if len(covered) != len(set(covered)) or set(covered) != set(names):
        raise ValueError("Serial segments must cover each full-workcell joint exactly once")

    indices = {name: index for index, name in enumerate(names)}
    virtual = dict(initial)
    result = []
    offset = 0.0
    for leg, path in enumerate(paths):
        if max(abs(position - virtual[name]) for name, position in zip(path.names, path.points[0]["p"])) > 0.01 + 1e-12:
            raise ValueError("Serial segment start differs from its virtual state by more than 0.01rad")
        for index, point in enumerate(path.points):
            positions = [virtual[name] for name in names]
            velocities = [0.0] * len(names)
            accelerations = [0.0] * len(names)
            stopping = index in (0, len(path.points) - 1)
            for local_index, name in enumerate(path.names):
                full_index = indices[name]
                positions[full_index] = virtual[name] if index == 0 else point["p"][local_index]
                if not stopping:
                    velocities[full_index] = point["v"][local_index] if point["v"] else 0.0
                    accelerations[full_index] = point["a"][local_index] if point["a"] else 0.0
            if leg == 0 or index != 0:
                result.append(dict(t=offset + point["t"], p=positions, v=velocities, a=accelerations))
        virtual.update(zip(path.names, path.points[-1]["p"]))
        offset += path.duration
    return Spline(names, result)
