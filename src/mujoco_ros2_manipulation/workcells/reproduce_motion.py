"""Replay a historical geometry witness or a saved path, or issue fresh ROS requests."""
import argparse
import json
from pathlib import Path
import uuid

from mujoco_ros2_manipulation.paths import ROOT
from mujoco_ros2_manipulation.model_build.integrity import file_sha256
from mujoco_ros2_manipulation.verification.reports import atomic_json
from .config import load_workcell
from .contracts import describe_workcell
from .distance import DistanceEvaluator
from .integrity import check_workcell
from .interference import Spline
from .motion_diagnostics import measure_endpoints, path_clearance_diagnostic
from .path_quality import evaluate_path
from .planning_budget import PlanningBudget
from .reproduction import load_reproduction, replay_witness
from .timing import load_policy, choose_delay, TimingRejected


def replay_trace(path, cell, manifest, evaluator, policy):
    trace = json.loads(path.read_text())
    session = json.loads((path.parent / "session.json").read_text())
    for name in ("workcell_sha256", "inputs", "outputs"):
        if session["model_manifest"][name] != manifest[name]:
            raise ValueError("Saved trajectory model differs from the current model: " + name)
    if session["policy"] != policy or session["evaluator"]["source_sha256"] != evaluator.metadata()["source_sha256"]:
        raise ValueError("Saved trajectory policy/evaluator source changed")
    initial, target = trace["initial_state_rad"], trace["command"]["target_rad"]
    threshold = policy["min_inter_arm_distance_m"]+policy["tracking_reserve_m"]
    endpoints = measure_endpoints(initial, target, evaluator, threshold)
    result = dict(scope="Offline geometry replay only; no ROS validity or action dispatch", source_trace_sha256=file_sha256(path),
                  request_id=trace["request_id"], endpoint_clearance=endpoints)
    if trace["command"]["strategy"] == "departure_timing" and trace.get("source_trajectories"):
        diagnostics = {}
        try:
            chosen = choose_delay([Spline(**c) for c in trace["source_trajectories"]], initial, cell, evaluator, policy, diagnostics=diagnostics)
            result.update(departure_search=diagnostics, decision="candidate_found", delay_sec=chosen["delay_sec"])
        except TimingRejected:
            result.update(departure_search=diagnostics, decision="no_departure_candidate")
    else:
        if not trace.get("planned_trajectories"):
            raise ValueError("Trace has no saved planned trajectory; historical witness replay is a separate mode")
        curves = [Spline(**c) for c in trace["planned_trajectories"]]
        assessment = evaluate_path(curves, initial, cell, evaluator, dt=policy["sample_dt_sec"], reject_below=threshold)
        result.update(assessment=assessment, diagnostic=path_clearance_diagnostic(endpoints, assessment, max(c.duration for c in curves)),
                      decision="clearance_passed" if assessment["complete"] else "clearance_refused")
    return result


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=("witness", "trace", "live"), default="witness")
    parser.add_argument("--fixture", type=Path, default=ROOT / "config/reproductions/home_return_clearance.json")
    parser.add_argument("--trace", type=Path)
    parser.add_argument("--output", type=Path, help="New result file for offline modes; new trace directory for live")
    parser.add_argument("--domain", type=int, default=154)
    parser.add_argument("--repeats", type=int, default=1, help="Fresh requests per strategy, 1 to 5")
    parser.add_argument("--compute-budget", type=float, default=30.0)
    args = parser.parse_args()
    if not 1 <= args.repeats <= 5 or not 0 <= args.domain <= 232:
        parser.error("Expected repeats 1..5 and ROS domain 0..232")
    if (args.mode == "trace") != (args.trace is not None):
        parser.error("--trace is required only in trace mode")
    PlanningBudget(args.compute_budget)
    if args.mode == "live":
        from .verify_motion_reproduction import run_live
        return run_live(args)
    output = args.output or ROOT / "runtime/validation/motion-replays" / (uuid.uuid4().hex+".json")
    if output.exists() or not output.resolve().is_relative_to((ROOT / "runtime").resolve()):
        parser.error("Output must be a new file under runtime/")
    cell = load_workcell()
    manifest = check_workcell(cell)
    _, policy = load_policy(cell)
    with DistanceEvaluator(cell) as evaluator:
        if args.mode == "witness":
            fixture = load_reproduction(args.fixture, cell, manifest)
            result = replay_witness(fixture, evaluator, policy["min_inter_arm_distance_m"]+policy["tracking_reserve_m"])
        else:
            result = replay_trace(args.trace, cell, manifest, evaluator, policy)
        result.update(schema_version=1, mode=args.mode, workcell_contract=describe_workcell(cell, manifest), evaluator=evaluator.metadata())
    output.parent.mkdir(parents=True, exist_ok=True)
    atomic_json(output, result)
    print(str(output))
    return 0
