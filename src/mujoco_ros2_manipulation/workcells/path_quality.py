"""Joint/link-frame path lengths and sampled native cross-arm clearance."""
import math
from mujoco_ros2_manipulation.workcells.interference import finite
from mujoco_ros2_manipulation.workcells.environment_clearance import require_environment_result


def sample_schedule(curves, initial, dt=0.01):
    names = [n for c in curves for n in c.names]
    if (
        not finite(initial.values())
        or not finite([dt])
        or not 0.002 <= dt <= 0.02
        or len(names) != len(set(names))
        or set(names) != set(initial)
    ):
        raise ValueError("Schedule must cover each joint exactly once")
    for c in curves:
        if (
            max(abs(c.points[0]["p"][j] - initial[n]) for j, n in enumerate(c.names))
            > 0.01
        ):
            raise ValueError("Schedule start differs from initial state")
    duration = max(c.duration for c in curves)
    count = math.ceil(duration / dt)
    times = sorted(
        {duration * k / count for k in range(count + 1)}
        | {t for c in curves for t in c.times}
    )
    if len(times) > 5000:
        raise ValueError("Schedule sampling budget exceeded")
    previous = None
    for t in times:
        state = {}
        for c in curves:
            state.update(zip(c.names, c.position(min(t, c.duration))))
        if previous and max(abs(state[n] - previous[n]) for n in initial) > 0.025:
            raise ValueError("Schedule joint sampling step exceeds 0.025rad")
        yield t, state
        previous = state


def evaluate_states(states, cell, evaluator, *, reject_below=None, checkpoint=None,
                    environment_reject_below=None):
    if environment_reject_below is not None and (
            type(environment_reject_below) not in (int, float)
            or not math.isfinite(environment_reject_below) or environment_reject_below <= 0):
        raise ValueError("Expected a positive finite environment threshold")
    previous = None
    minimum = None
    environment_minimum = None
    count = 0
    previous_tips = None
    travel = {n: 0.0 for n in cell.joints}
    tip_length = {i.id: 0.0 for i in cell.instances}
    arm_length = {i.id: 0.0 for i in cell.instances}
    first_time = last_time = None
    max_dt = 0.0
    for t, state in states:
        if checkpoint:
            checkpoint()
        if not finite([t]) or (last_time is not None and t <= last_time):
            raise ValueError("Observation time must increase")
        if first_time is None:
            first_time = t
        if last_time is not None:
            max_dt = max(max_dt, t - last_time)
        last_time = t
        result = evaluator.at(state)
        if checkpoint:
            checkpoint()
        count += 1
        if minimum is None or result["distance_m"] < minimum["distance_m"]:
            minimum = dict(time_sec=t, state_rad=state, **result)
        if "environment" in result or environment_reject_below is not None:
            environment = require_environment_result(result.get("environment"))
            if environment_minimum is None or environment["distance_m"] < environment_minimum["distance_m"]:
                environment_minimum = dict(time_sec=t, state_rad=state, **environment)
        if reject_below is not None and result["distance_m"] < reject_below:
            partial = dict(complete=False, samples=count, minimum=minimum)
            if environment_minimum is not None:
                partial.update(environment_minimum=environment_minimum,
                               rejection_kind="native_clearance")
            return partial
        if (environment_reject_below is not None
                and environment["distance_m"] < environment_reject_below):
            return dict(complete=False, samples=count, minimum=minimum,
                        environment_minimum=environment_minimum,
                        rejection_kind="environment_clearance")
        if previous is not None:
            for n in travel:
                travel[n] += abs(state[n] - previous[n])
            for k, i in enumerate(cell.instances):
                arm_length[i.id] += math.sqrt(
                    sum((state[n] - previous[n]) ** 2 for n in i.joints)
                )
                tip_length[i.id] += math.dist(result["tips_m"][k], previous_tips[k])
        previous, previous_tips = state, result["tips_m"]
    if count < 2:
        raise ValueError("At least two observations are required")
    result = dict(
        complete=True,
        samples=count,
        sample_dt_max_sec=max_dt,
        duration_sec=last_time - first_time,
        joint_total_variation_rad=sum(travel.values()),
        joint_travel_rad=travel,
        arm_joint_path_rad=arm_length,
        tip_frame_path_m=tip_length,
        minimum=minimum,
        distance_semantics="Signed closest collision-geometry distance, sampled; not continuous-time certified",
    )
    if environment_minimum is not None:
        result["environment_minimum"] = environment_minimum
    return result


def evaluate_path(curves, initial, cell, evaluator, *, dt=0.01, reject_below=None, checkpoint=None,
                  environment_reject_below=None):
    return evaluate_states(
        sample_schedule(curves, initial, dt), cell, evaluator, reject_below=reject_below,
        checkpoint=checkpoint, environment_reject_below=environment_reject_below
    )
