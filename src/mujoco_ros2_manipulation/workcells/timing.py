"""Bounded departure-delay search: preserve every moving spline segment exactly."""
from copy import deepcopy
import math
from pathlib import Path
import yaml
from mujoco_ros2_manipulation.model_build.integrity import file_sha256
from mujoco_ros2_manipulation.workcells.interference import Spline, finite
from mujoco_ros2_manipulation.workcells.interference_comparison import load_bundle
from mujoco_ros2_manipulation.workcells.path_quality import evaluate_path
from mujoco_ros2_manipulation.verification.reports import atomic_json


def load_policy(cell):
    path = cell.root / "config/planning/dual_arm_timing.yaml"
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
    def __init__(self, candidates):
        super().__init__(
            "No departure-only solution within delay/clearance budget; no command is allowed"
        )
        self.candidates = candidates


def choose_delay(curves, initial, cell, evaluator, policy):
    if len(curves) != 2 or abs(curves[0].duration - curves[1].duration) > 1e-8:
        raise ValueError("Expected two synchronized source curves")
    candidates = []
    solutions = []
    step = policy["delay_step_sec"]
    limit = min(policy["max_delay_sec"], 35.0 - curves[0].duration)
    for arm in (0, 1):
        for index in range(math.floor(limit / step + 1e-9) + 1):
            delay = round(index * step, 9)
            scheduled = [
                delayed_curve(c, delay if n == arm else 0.0)
                for n, c in enumerate(curves)
            ]
            result = evaluate_path(
                scheduled,
                initial,
                cell,
                evaluator,
                dt=policy["sample_dt_sec"],
                reject_below=policy["min_inter_arm_distance_m"]
                + policy["tracking_reserve_m"],
            )
            candidates.append(
                dict(
                    delayed_arm=cell.instances[arm].id,
                    delay_sec=delay,
                    complete=result["complete"],
                    samples=result["samples"],
                    minimum=result["minimum"],
                )
            )
            if result["complete"]:
                solutions.append((delay, arm, scheduled, result))
                break
    if not solutions:
        raise TimingRejected(candidates)
    delay, arm, scheduled, result = min(solutions, key=lambda s: (s[0], s[1]))
    return dict(
        delayed_arm=cell.instances[arm].id,
        delay_sec=delay,
        curves=[c.record() for c in scheduled],
        evaluation=result,
        candidates=candidates,
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
