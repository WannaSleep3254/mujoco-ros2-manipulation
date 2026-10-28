"""Independent W08 experiment declarations; never edits fixed_motion_v1.

All ordered performance, unscored warm-fill and functional rows are declared
before ROS starts. A reused trajectory remains a candidate requiring complete
validation and a new review, never execution authority.
"""
from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path


EXPERIMENT_ID = "planner-cache-comparison-v1"
CONDITIONS = (
    dict(id="rrtconnect_off", planner_id="RRTConnectkConfigDefault", cache=False, expected_cache="disabled"),
    dict(id="rrt_off", planner_id="RRTkConfigDefault", cache=False, expected_cache="disabled"),
    dict(id="rrtconnect_cold", planner_id="RRTConnectkConfigDefault", cache=True, expected_cache="miss"),
    dict(id="rrtconnect_warm", planner_id="RRTConnectkConfigDefault", cache=True, expected_cache="hit"),
)
FUNCTIONAL_CASES = ("cold_then_warm_new_review", "goal_mismatch", "start_mismatch", "expiry",
    "environment_identity_change", "invalid_cached_path_fresh_fallback", "exact_one_use_approval",
    "cancel_passthrough", "budget_passthrough", "identity_error_passthrough")
INITIAL_TOLERANCE_RAD = .0015
GOAL_TOLERANCE_RAD = .003
CACHE_START_TOLERANCE_RAD = .0001
COMPUTE_BUDGET_SEC = 5.
COMPUTE_SCOPE = "computation_started_through_final_review_identity"
PRIMARY_LATENCY_METRIC = "received_to_plan_ready_wall_sec"


def canonical(value):
    return json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()


def digest(value):
    return hashlib.sha256(canonical(value)).hexdigest()


def make_experiment(cell, *, repeats=3):
    if type(repeats) is not int or not 3 <= repeats <= 30:
        raise ValueError("Planner/cache experiment requires 3..30 declared repetitions")
    names, home = list(cell.joints), list(cell.home)
    if (cell.id != "fr5_ur5e" or len(names) != 12 or len(set(names)) != 12 or len(home) != 12
            or not all(type(q) in (int, float) and math.isfinite(q) for q in home)
            or len(cell.instances) != 2):
        raise ValueError("Experiment requires the checked twelve-joint FR5/UR5e workcell")
    initial = dict(zip(names, home))
    target = dict(initial)
    for instance in cell.instances:
        target[instance.joints[0]] += .12
    return dict(schema_version=1, id=EXPERIMENT_ID, workcell=cell.id, repeats=repeats,
        case=dict(id="local_small", initial_rad=initial, target_rad=target,
            provenance="Defined nominal workcell home; first joint of each arm +0.12rad; copied scalar inputs only, fixed_motion_v1 unchanged"),
        conditions=deepcopy(list(CONDITIONS)), functional_cases=list(FUNCTIONAL_CASES),
        comparison_policy=dict(strategy="coordinated", max_plan_candidates=1,
            initial_tolerance_rad=INITIAL_TOLERANCE_RAD, goal_tolerance_rad=GOAL_TOLERANCE_RAD,
            cache_start_tolerance_rad=CACHE_START_TOLERANCE_RAD,
            compute_budget_wall_sec=COMPUTE_BUDGET_SEC,
            budget_includes_review=True, compute_scope=COMPUTE_SCOPE,
            primary_latency_metric=PRIMARY_LATENCY_METRIC,
            primary_latency_scope="Request received through plan_ready, after the final runtime identity check and new review; same endpoint in every condition",
            budget_enforcement="Cooperative checkpoints and RPC remaining allowance; not a forced 5-second process deadline",
            cache_default="off", planner_default="RRTConnectkConfigDefault",
            selected_planner_response="GetMotionPlan does not return selected planner identity; requested ID and runtime configuration/log evidence remain separate"),
        statistics_policy=dict(individual_samples=True, median=True, maximum=True,
            minimum_valid_per_condition_for_p95=20, p95_requires_every_condition=True,
            failures_retained=True, warm_fill_scored=False, functional_fault_injections_scored=False))


def planned_rows(experiment, *, mode="all"):
    if mode not in ("all", "performance", "functional"):
        raise ValueError("Expected all/performance/functional experiment mode")
    if (experiment.get("schema_version") != 1 or experiment.get("id") != EXPERIMENT_ID
            or experiment.get("conditions") != list(CONDITIONS)
            or experiment.get("functional_cases") != list(FUNCTIONAL_CASES)
            or type(experiment.get("repeats")) is not int or not 3 <= experiment["repeats"] <= 30):
        raise ValueError("Experiment declarations differ from this immutable W08 contract")
    rows = []
    def append(row):
        selected = mode == "all" or mode == ("functional" if row["kind"] == "functional" else "performance")
        rows.append(dict(index=len(rows), selected=selected, status="not_run", error=None,
            not_run_reason=None if selected else "mode_not_selected", **row))
    for repeat in range(1, experiment["repeats"]+1):
        # Rotate comparison order by repetition without looking at any outcomes.
        offset = (repeat-1) % len(CONDITIONS)
        for condition in list(CONDITIONS)[offset:]+list(CONDITIONS)[:offset]:
            prefix = f"r{repeat:02d}-{condition['id']}"
            if condition["id"] == "rrtconnect_warm":
                append(dict(row_id=prefix+"-fill", kind="warm_fill", repeat=repeat,
                    condition=condition["id"], scored=False, test_only=False))
            append(dict(row_id=prefix, kind="performance", repeat=repeat,
                condition=condition["id"], scored=True, test_only=False))
    for name in FUNCTIONAL_CASES:
        append(dict(row_id="functional-"+name, kind="functional", case_id=name,
            repeat=None, condition=None, scored=False,
            test_only=name in ("start_mismatch", "expiry", "invalid_cached_path_fresh_fallback",
                              "cancel_passthrough", "budget_passthrough", "identity_error_passthrough")))
    return rows


def preserve_sources(directory, workspace, model_metadata):
    """Copy bounded public implementation/config/model identity before launch."""
    directory, workspace = Path(directory), Path(workspace).resolve()
    target = directory/"input-snapshot"
    target.mkdir(exist_ok=False)
    paths = sorted((workspace/"src/mujoco_ros2_manipulation/workcells").glob("*.py"))
    paths += sorted((workspace/"src/mujoco_ros2_manipulation/verification").glob("*.py"))
    paths += [workspace/"config/workcells/fr5_ur5e.yaml"]
    generated = workspace/"runtime/generated/workcells/fr5_ur5e"
    paths += [p for p in generated.iterdir() if p.is_file() and p.suffix in (".json", ".yaml", ".xml", ".urdf", ".srdf", ".rviz")]
    paths += [workspace/name for name in model_metadata.get("inputs", {})]
    paths += sorted((workspace/"config/planning").glob("*.yaml"))
    wrapper = workspace/"benchmark_planner_cache.py"
    if wrapper.is_file():
        paths.append(wrapper)
    paths = sorted(set(paths))
    manifest, total = {}, 0
    for path in paths:
        if path.is_symlink() or not path.resolve().is_relative_to(workspace):
            raise ValueError("Experiment input snapshot contains an unsafe path")
    for path in paths:
        raw = path.read_bytes()
        total += len(raw)
        if len(raw) > 8*1024*1024 or total > 32*1024*1024:
            raise ValueError("Experiment source snapshot exceeds its bounded bytes")
        relative = path.relative_to(workspace).as_posix()
        expected = model_metadata.get("inputs", {}).get(relative)
        if expected is not None and hashlib.sha256(raw).hexdigest() != expected:
            raise ValueError("Model input changed during experiment snapshot: "+relative)
        destination = target/relative
        destination.parent.mkdir(parents=True, exist_ok=True)
        destination.write_bytes(raw)
        manifest[relative] = hashlib.sha256(raw).hexdigest()
    value = dict(schema_version=1, files_sha256=manifest, byte_length=total,
        model_metadata=deepcopy(model_metadata), note="Exact pre-launch public inputs; mesh contents remain in checked existing assets with model metadata SHA identities")
    (target/"manifest.json").write_bytes(canonical(value)+b"\n")
    return value


def verify_snapshot(workspace, snapshot):
    """Reject source/model/config drift instead of relabelling old evidence."""
    workspace = Path(workspace).resolve()
    for relative, expected in snapshot["files_sha256"].items():
        path = workspace/relative
        if (Path(relative).is_absolute() or ".." in Path(relative).parts or path.is_symlink()
                or not path.resolve().is_relative_to(workspace)
                or hashlib.sha256(path.read_bytes()).hexdigest() != expected):
            raise ValueError("Predeclared experiment input changed: "+relative)
