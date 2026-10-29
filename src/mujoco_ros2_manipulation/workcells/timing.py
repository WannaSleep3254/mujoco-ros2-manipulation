"""Bounded departure-delay search: preserve every moving spline segment exactly."""
from copy import deepcopy
import math
import re
import time
from pathlib import Path
import yaml
from mujoco_ros2_manipulation.model_build.integrity import file_sha256
from mujoco_ros2_manipulation.workcells.interference import Spline, finite
from mujoco_ros2_manipulation.workcells.interference_comparison import load_bundle
from mujoco_ros2_manipulation.workcells.path_quality import evaluate_path
from mujoco_ros2_manipulation.verification.reports import atomic_json


def load_policy(cell, policy_name="dual_arm_timing"):
    if not isinstance(policy_name, str) or not re.fullmatch(r"[a-z][a-z0-9_]*", policy_name):
        raise ValueError("Invalid timing policy name")
    path = cell.root / "config/planning" / (policy_name + ".yaml")
    data = yaml.safe_load(path.read_text())
    keys = {
        "schema_version",
        "workcell",
        "case",
        "min_inter_arm_distance_m",
        "tracking_reserve_m",
        "sample_dt_sec",
        "delay_step_sec",
        "max_delay_sec",
    }
    if (
        not isinstance(data, dict)
        or set(data) != keys
        or data["schema_version"] != 1
        or data["workcell"] != cell.id
    ):
        raise ValueError("Invalid timing policy")
    bounds = {
        "min_inter_arm_distance_m": (0.001, 0.1),
        "sample_dt_sec": (0.002, 0.01),
        "tracking_reserve_m": (0.0, 0.05),
        "delay_step_sec": (0.02, 0.25),
        "max_delay_sec": (0.1, 10.0),
    }
    if any(
        not finite([data[k]]) or not lo <= data[k] <= hi
        for k, (lo, hi) in bounds.items()
    ):
        raise ValueError("Timing policy is outside validated search bounds")
    if math.ceil(data["max_delay_sec"] / data["delay_step_sec"]) > 500:
        raise ValueError("Too many delay candidates")
    return path, data


def delayed_curve(curve, delay):
    if not finite([delay]) or delay < 0 or curve.duration + delay > 35:
        raise ValueError("Departure delay must fit the 35s execution bound")
    # Hold before departure is exact only when the source starts at rest.
    if any(abs(v) > 1e-10 for key in ("v", "a") for v in curve.points[0][key]):
        raise ValueError("Departure hold requires zero initial velocity/acceleration")
    if any(abs(v) > 1e-8 for v in curve.points[-1]["v"]):
        raise ValueError("Completed arm must finish at zero velocity")
    points = deepcopy(curve.points)
    if delay:
        for point in points:
            point["t"] = round(point["t"] + delay, 9)
        points.insert(0, deepcopy(curve.points[0]))
    return Spline(curve.names, points)


class TimingRejected(ValueError):
    def __init__(self, candidates, profiling=None):
        super().__init__(
            "No departure-only solution within delay/clearance budget; no command is allowed"
        )
        self.candidates = candidates
        self.profiling = profiling


def choose_delay(curves, initial, cell, evaluator, policy, *, diagnostics=None, checkpoint=None, clock=time.monotonic_ns, optimize=True, self_policy=None):
    if len(curves) != 2 or abs(curves[0].duration - curves[1].duration) > 1e-8:
        raise ValueError("Expected two synchronized source curves")
    if type(optimize) is not bool:
        raise ValueError("optimize must be a boolean")
    self_options = {}
    if self_policy is not None:
        from .self_clearance import validate_self_policy
        validate_self_policy(self_policy)
        self_options["self_reject_below"] = self_policy["planning_min_self_distance_m"]
    candidates = []
    decisions = []
    profiling = dict(clock="time.monotonic_ns", elapsed_wall_sec=0.0, candidate_wall_sec=0.0,
                     native_distance_wall_sec=0.0, native_distance_calls=0, candidate_count=0,
                     other_wall_sec=0.0, optimized=optimize,
                     reused_zero_delay_evaluations=0, grid_values_pruned_by_bound=0)
    if diagnostics is not None:
        diagnostics.update(candidates=candidates, profiling=profiling, search_decisions=decisions)
    started = clock()

    class CountedDistance:
        def __init__(self):
            self.calls = 0
            self.wall = 0.0

        def at(self, state):
            before = clock()
            self.calls += 1
            try:
                return evaluator.at(state)
            finally:
                self.wall += (clock() - before) / 1e9

    def assess(scheduled, arm, delay):
        if checkpoint:
            checkpoint()
        entry = dict(delayed_arm=cell.instances[arm].id, delay_sec=delay, status="running",
                     complete=None, samples=0, minimum=None)
        candidates.append(entry)
        before = clock()
        counted = CountedDistance()
        try:
            result = evaluate_path(scheduled, initial, cell, counted,
                dt=min(policy["sample_dt_sec"], self_policy["sample_dt_sec"]) if self_policy is not None else policy["sample_dt_sec"],
                reject_below=policy["min_inter_arm_distance_m"] + policy["tracking_reserve_m"],
                checkpoint=checkpoint, **self_options)
            if "self_minimum" in result:
                entry["self_minimum"] = deepcopy(result["self_minimum"])
                entry["rejection_kind"] = result.get("rejection_kind")
            entry.update(status="completed", complete=result["complete"], samples=result["samples"], minimum=result["minimum"])
            return result
        except BaseException as error:
            entry.update(status="interrupted", error=dict(type=type(error).__name__, message=str(error)))
            raise
        finally:
            elapsed = (clock() - before) / 1e9
            entry.update(elapsed_wall_sec=elapsed, native_distance_calls=counted.calls, native_distance_wall_sec=counted.wall)
            profiling["candidate_wall_sec"] += elapsed
            profiling["native_distance_wall_sec"] += counted.wall
            profiling["native_distance_calls"] += counted.calls

    try:
        return _choose_delay(curves, initial, cell, policy, candidates, profiling, decisions, assess, checkpoint, optimize)
    finally:
        profiling["elapsed_wall_sec"] = (clock() - started) / 1e9
        profiling["candidate_count"] = len(candidates)
        profiling["other_wall_sec"] = max(0.0, profiling["elapsed_wall_sec"] - profiling["candidate_wall_sec"])


def _choose_delay(curves, initial, cell, policy, candidates, profiling, decisions, assess, checkpoint, optimize):
    solutions = []
    zero_result = None
    step = policy["delay_step_sec"]
    limit = min(policy["max_delay_sec"], 35.0 - curves[0].duration)
    count = math.floor(limit / step + 1e-9) + 1
    for arm in (0, 1):
        for index in range(count):
            if checkpoint:
                checkpoint()
            delay = round(index * step, 9)
            # On the right-arm pass, an equal delay loses the existing left-arm tie.
            # Larger delays cannot beat the incumbent, so their grid states need no evaluation.
            if optimize and solutions and delay >= solutions[0][0]:
                profiling["grid_values_pruned_by_bound"] += count - index
                decisions.append(dict(kind="incumbent_bound", delayed_arm=cell.instances[arm].id,
                    first_pruned_delay_sec=delay, incumbent_delay_sec=solutions[0][0],
                    unevaluated_grid_values=count-index, reason="Equal delay retains the left-arm tie; larger delay cannot improve selection"))
                break
            scheduled = [
                delayed_curve(c, delay if n == arm else 0.0)
                for n, c in enumerate(curves)
            ]
            if optimize and arm == 1 and index == 0:
                result = zero_result
                profiling["reused_zero_delay_evaluations"] += 1
                decisions.append(dict(kind="shared_zero_delay", delayed_arm=cell.instances[arm].id,
                    source_candidate_index=0, reason="Both zero-delay schedules are identical, with the same initial state and full-distance evaluation"))
            else:
                result = assess(scheduled, arm, delay)
                if arm == 0 and index == 0:
                    zero_result = result
            if result["complete"]:
                solutions.append((delay, arm, scheduled, result))
                break
    if not solutions:
        raise TimingRejected(candidates, profiling)
    delay, arm, scheduled, result = min(solutions, key=lambda s: (s[0], s[1]))
    return dict(
        delayed_arm=cell.instances[arm].id,
        delay_sec=delay,
        curves=[c.record() for c in scheduled],
        evaluation=result,
        candidates=candidates,
        profiling=profiling,
        search_decisions=decisions,
        selection="Smallest feasible departure delay on the configured grid, both arm orders; no claim of global optimality",
    )


def prepare_timing(bundle_path, output, cell, evaluator):
    bundle, curves = load_bundle(bundle_path, cell)
    policy_path, policy = load_policy(cell)
    if policy["case"] != bundle["case_id"]:
        raise ValueError("Policy/case mismatch")
    reference = [curves["left"], curves["right"]]
    path = Path(output).resolve()
    if (
        path.exists()
        or path.with_suffix(".failure.json").exists()
        or not path.is_relative_to(cell.root / "runtime")
    ):
        raise ValueError("Timing plan output must be a new runtime/ file")
    path.parent.mkdir(parents=True, exist_ok=True)
    try:
        chosen = choose_delay(reference, bundle["initial_rad"], cell, evaluator, policy)
    except TimingRejected as error:
        atomic_json(
            path.with_suffix(".failure.json"),
            dict(
                schema_version=1,
                status="failed",
                error=str(error),
                bundle_sha256=file_sha256(bundle_path),
                policy=policy,
                policy_sha256=file_sha256(policy_path),
                evaluator=evaluator.metadata(),
                candidates=error.candidates,
                profiling=error.profiling,
            ),
        )
        raise
    reference_metrics = evaluate_path(
        reference, bundle["initial_rad"], cell, evaluator, dt=policy["sample_dt_sec"]
    )
    baseline_metrics = evaluate_path(
        [curves["avoidance"]],
        bundle["initial_rad"],
        cell,
        evaluator,
        dt=policy["sample_dt_sec"],
    )
    data = dict(
        schema_version=1,
        bundle_sha256=file_sha256(bundle_path),
        case_sha256=bundle["case_sha256"],
        model_manifest_sha256=bundle["model_manifest_sha256"],
        policy=policy,
        policy_sha256=file_sha256(policy_path),
        evaluator=evaluator.metadata(),
        reference=reference_metrics,
        baseline=baseline_metrics,
        **chosen
    )
    atomic_json(path, data)
    return path


def load_timing(path, bundle_path, cell, evaluator):
    import json

    data = json.loads(Path(path).read_text())
    bundle, curves = load_bundle(bundle_path, cell)
    policy_path, policy = load_policy(cell)
    if (
        data["schema_version"] != 1
        or data["bundle_sha256"] != file_sha256(bundle_path)
        or data["case_sha256"] != bundle["case_sha256"]
        or data["model_manifest_sha256"] != bundle["model_manifest_sha256"]
        or data["policy"] != policy
        or data["policy_sha256"] != file_sha256(policy_path)
        or data["evaluator"] != evaluator.metadata()
    ):
        raise ValueError("Stale or mismatched timing plan")
    if (
        data["delayed_arm"] not in ("left", "right")
        or not finite([data["delay_sec"]])
        or not 0 <= data["delay_sec"] <= policy["max_delay_sec"]
    ):
        raise ValueError("Invalid selected departure delay")
    expected = [
        delayed_curve(
            curves[i.id], data["delay_sec"] if i.id == data["delayed_arm"] else 0
        )
        for i in cell.instances
    ]
    if data["curves"] != [c.record() for c in expected]:
        raise ValueError("Timing plan changed the source geometry or derivatives")
    return data, expected
