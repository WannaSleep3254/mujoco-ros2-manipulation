"""Bounded spline evaluation for a matched independent/coordinated planning case."""

from bisect import bisect_right
from copy import deepcopy
import math


def seconds(value):
    return value.sec + value.nanosec * 1e-9


def finite(values):
    return all(type(v) in (int, float) and math.isfinite(v) for v in values)


class Spline:
    """Position / cubic / quintic interpolation matching the configured JTC splines."""

    def __init__(self, names, points):
        self.names = tuple(names)
        self.points = list(points)
        if (
            not self.names
            or len(set(names)) != len(names)
            or not 2 <= len(points) <= 2000
        ):
            raise ValueError("Invalid spline joints or point count")
        self.times = [p["t"] for p in points]
        if (
            not finite(self.times)
            or self.times[0] != 0
            or any(b <= a for a, b in zip(self.times, self.times[1:]))
            or not 0 < self.times[-1] <= 35
        ):
            raise ValueError("Spline must start at zero and end within 35s")
        layouts = set()
        for point in points:
            for key in ("p", "v", "a"):
                values = point[key]
                if (key == "p" or values) and (
                    len(values) != len(names) or not finite(values)
                ):
                    raise ValueError("Missing, malformed or nonfinite spline values")
            if point["a"] and not point["v"]:
                raise ValueError("Accelerations require velocities")
            layouts.add((bool(point["v"]), bool(point["a"])))
        if len(layouts) != 1:
            raise ValueError("Mixed derivative layouts are unsupported")
        self.layout = layouts.pop()

    @classmethod
    def from_ros(cls, trajectory):
        return cls(
            list(trajectory.joint_names),
            [
                dict(
                    t=seconds(p.time_from_start),
                    p=list(p.positions),
                    v=list(p.velocities),
                    a=list(p.accelerations),
                )
                for p in trajectory.points
            ],
        )

    @property
    def duration(self):
        return self.times[-1]

    def position(self, stamp):
        if not math.isfinite(stamp) or not 0 <= stamp <= self.duration + 1e-8:
            raise ValueError("Sample time outside trajectory")
        if stamp >= self.duration:
            return list(self.points[-1]["p"])
        index = max(0, bisect_right(self.times, stamp) - 1)
        left, right = self.points[index : index + 2]
        length = right["t"] - left["t"]
        u = (stamp - left["t"]) / length
        result = []
        for j, (p0, p1) in enumerate(zip(left["p"], right["p"])):
            delta = p1 - p0
            if not self.layout[0]:
                value = p0 + delta * u
            else:
                v0, v1 = left["v"][j] * length, right["v"][j] * length
                if not self.layout[1]:
                    value = (
                        p0
                        + v0 * u
                        + (3 * delta - 2 * v0 - v1) * u * u
                        + (-2 * delta + v0 + v1) * u ** 3
                    )
                else:
                    a0, a1 = left["a"][j] * length ** 2, right["a"][j] * length ** 2
                    d, v, a = delta - v0 - a0 / 2, v1 - v0 - a0, a1 - a0
                    value = (
                        p0
                        + v0 * u
                        + a0 * u * u / 2
                        + (10 * d - 4 * v + a / 2) * u ** 3
                        + (-15 * d + 7 * v - a) * u ** 4
                        + (6 * d - 3 * v + a / 2) * u ** 5
                    )
            result.append(value)
        if not finite(result):
            raise ValueError("Nonfinite interpolated positions")
        return result

    def record(self):
        return dict(names=list(self.names), points=self.points)


def stretch(trajectory, duration):
    """Only slow an independent plan down; scale its derivatives consistently."""
    curve = Spline.from_ros(trajectory)
    if not math.isfinite(duration) or duration < curve.duration or duration > 35:
        raise ValueError("Retiming must stretch within the 35s execution bound")
    scale = duration / curve.duration
    result = deepcopy(trajectory)
    for point in result.points:
        ns = round(seconds(point.time_from_start) * scale * 1e9)
        point.time_from_start.sec, point.time_from_start.nanosec = divmod(
            ns, 1000000000
        )
        point.velocities = [v / scale for v in point.velocities]
        point.accelerations = [a / scale ** 2 for a in point.accelerations]
    Spline.from_ros(result)
    return result


def sampled_parallel(curves, current, *, complete=True, dt=0.02):
    if not curves or not finite(current.values()) or not math.isfinite(dt) or dt <= 0:
        raise ValueError("Invalid sampling inputs")
    names = [n for curve in curves for n in curve.names]
    if len(names) != len(set(names)) or not set(names).issubset(current):
        raise ValueError("Unknown or overlapping joint ownership")
    if complete and set(names) != set(current):
        raise ValueError("Parallel execution must cover all joints")
    duration = max(curve.duration for curve in curves)
    if any(abs(curve.duration - duration) > 1e-8 for curve in curves):
        raise ValueError("Parallel plans must have the same retimed duration")
    for curve in curves:
        if (
            max(
                abs(curve.points[0]["p"][j] - current[n])
                for j, n in enumerate(curve.names)
            )
            > 0.01
        ):
            raise ValueError("Spline start differs from the complete initial state")
    count = math.ceil(duration / dt)
    if count > 5000:
        raise ValueError("Sampling budget exceeded")
    times = sorted(
        {duration * k / count for k in range(count + 1)}
        | {t for curve in curves for t in curve.times}
    )
    if len(times) > 5000:
        raise ValueError("Sampling budget exceeded")
    result = []
    previous = None
    for stamp in times:
        state = dict(current)
        for curve in curves:
            state.update(zip(curve.names, curve.position(stamp)))
        if previous and max(abs(state[n] - previous[n]) for n in current) > 0.025:
            raise ValueError("Interpolated joint step exceeds 0.025rad")
        result.append((stamp, state))
        previous = state
    return result


def load_case(cell, name="dual_arm_crossing"):
    from mujoco_ros2_manipulation.workcells.config import identifier
    import yaml

    identifier(name)
    path = cell.root / "config/scenarios" / (name + ".yaml")
    data = yaml.safe_load(path.read_text())
    if (
        not isinstance(data, dict)
        or set(data)
        != {
            "schema_version",
            "id",
            "workcell",
            "sample_dt_sec",
            "planning",
            "start_rad",
            "goal_rad",
        }
        or data["schema_version"] != 1
        or data["id"] != name
        or data["workcell"] != cell.id
    ):
        raise ValueError("Unsupported or mismatched interference case")
    dt = data["sample_dt_sec"]
    if not finite([dt]) or not 0.005 <= dt <= 0.02:
        raise ValueError("Case sampling interval must be 0.005..0.02s")
    settings = data["planning"]
    expected = {
        "path_tolerance": (0.0001, 0.005),
        "resample_dt": (0.005, 0.02),
        "longest_valid_segment_fraction": (0.0001, 0.001),
    }
    if (
        not isinstance(settings, dict)
        or set(settings) != set(expected)
        or any(
            not finite([settings[k]]) or not lo <= settings[k] <= hi
            for k, (lo, hi) in expected.items()
        )
    ):
        raise ValueError("Invalid strict planning parameters")
    for key in ("start_rad", "goal_rad"):
        group = data[key]
        if not isinstance(group, dict) or set(group) != {i.id for i in cell.instances}:
            raise ValueError("Case must define exactly the workcell instances")
        state = {}
        for inst in cell.instances:
            values = group[inst.id]
            if (
                not isinstance(values, list)
                or len(values) != len(inst.joints)
                or not finite(values)
            ):
                raise ValueError("Case positions must be finite and cover each joint")
            state.update(zip(inst.joints, values))
        data[key] = state
    return path, data
