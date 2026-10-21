"""Paired seeded goals in one warm, guarded MuJoCo simulation."""
import argparse
from copy import deepcopy
import json
from pathlib import Path
import uuid

import rclpy
from std_msgs.msg import String
from std_srvs.srv import Trigger

from mujoco_ros2_manipulation.paths import ROOT
from mujoco_ros2_manipulation.model_build.integrity import file_sha256
from mujoco_ros2_manipulation.verification.diagnostics import diagnostic_environment, finalize_physics
from mujoco_ros2_manipulation.verification.reports import atomic_json
from mujoco_ros2_manipulation.verification.runner import run_verification
from .config import load_workcell
from .integrity import check_workcell
from .motion_request import COMMAND_TOPIC, command
from .motion_server import MotionCommandServer
from .planning_budget import PlanningBudget
from .random_workload import make_workload, check_workload, workload_hash
from .verify import require


class RandomMotionBenchmark(MotionCommandServer):
    def __init__(self, cell, progress, args):
        directory = args.output or ROOT / "runtime/validation/random-benchmarks" / uuid.uuid4().hex
        super().__init__(cell, directory, progress, compute_budget_sec=args.compute_budget)
        limits = self.metadata["joint_limits"]
        self.workload = check_workload(json.loads(args.workload.read_text()), cell, limits) if args.workload else make_workload(
            cell, limits, seed=args.seed, count=args.count, radius=args.radius)
        atomic_json(self.directory / "workload.json", self.workload)
        self.command_publisher = self.create_publisher(String, COMMAND_TOPIC, 10)
        self.scored_budget = args.compute_budget
        self.benchmark = dict(schema_version=1, status="running", session_id=self.session_id,
            workload_sha256=workload_hash(self.workload), compute_budget_wall_sec=self.scored_budget,
            initial_tolerance_rad=0.0015, reference_state_rad=None,
            expected_budget_exhaustion=args.expect_budget_exhaustion,
            required_all_success=args.require_all_success,
            warmup_and_reset_budget_wall_sec=30.0, reset_goal_tolerance_rad=0.00001,
            source_sha256={name: file_sha256(Path(__file__).parent / name) for name in ("benchmark_motion.py", "random_workload.py")},
            rows=[dict(case_id=c["case_id"], strategy=s, status="not_run", request_id=None)
                  for c in self.workload["cases"] for s in c["strategy_order"]], maintenance=[], error=None)
        self.save_benchmark()

    def save_benchmark(self):
        atomic_json(self.directory / "benchmark.json", self.benchmark)

    def submit(self, target, strategy):
        payload = command(target, strategy)
        self.command_publisher.publish(String(data=json.dumps(payload)))
        self.until(lambda: self.pending is not None or payload["request_id"] in self.replies, 5, "benchmark command receipt")
        if self.pending:
            self.process_next()
        path = self.directory / (payload["request_id"] + ".json")
        return json.loads(path.read_text()), path

    def maintenance(self, target, strategy, role):
        self.compute_budget_sec = 30.0
        self.goal_tolerance_rad = self.benchmark["reset_goal_tolerance_rad"]
        try:
            record, path = self.submit(target, strategy)
            self.benchmark["maintenance"].append(dict(role=role, request_id=record["request_id"], status=record["status"], trace_file=path.name))
            self.save_benchmark()
            require(record["status"] == "succeeded", f"Unscored {role} failed; benchmark aborted: {path}")
        finally:
            self.compute_budget_sec = self.scored_budget
            self.goal_tolerance_rad = 0.003

    def reset(self):
        self.maintenance(self.workload["initial_rad"], "coordinated", "reset")
        self.quiet()
        if self.benchmark["reference_state_rad"] is None:
            self.benchmark["reference_state_rad"] = self.state()
            self.save_benchmark()
        drift = max(abs(self.state()[n] - q) for n, q in self.benchmark["reference_state_rad"].items())
        require(drift <= self.benchmark["initial_tolerance_rad"], "Reset differs from nominal paired initial state")

    def run(self):
        try:
            self.prepare()
            self.until(lambda: self.command_publisher.get_subscription_count() >= 1, 5, "benchmark command subscription")
            self.reset()
            warm = dict(self.workload["initial_rad"])
            for inst in self.cell.instances:
                warm[inst.joints[0]] += 0.05
            for strategy in ("coordinated", "departure_timing"):
                self.maintenance(warm, strategy, "warmup")
                self.reset()
            targets = {c["case_id"]: c["target_rad"] for c in self.workload["cases"]}
            for row in self.benchmark["rows"]:
                self.reset()
                self.progress("random_" + row["case_id"] + "_" + row["strategy"])
                record, path = self.submit(targets[row["case_id"]], row["strategy"])
                row.update(status=record["status"], request_id=record["request_id"], trace_file=path.name)
                self.save_benchmark()
                require(not self.blocked and not self.contact_guard["latched"], "Physical execution/interlock failure; all remaining cases are not_run")
                require(record["status"] in ("succeeded", "failed"), "Unexpected rejection or cancellation of a scored request")
                if record["status"] == "failed":
                    require(record["dispatched_goals"] == 0, "Execution failure must abort rather than continue scoring")
                actual = record["initial_state_rad"]
                row["initial_drift_rad"] = max(abs(actual[n] - q) for n, q in self.benchmark["reference_state_rad"].items())
                require(row["initial_drift_rad"] <= self.benchmark["initial_tolerance_rad"], "Scored request did not begin at the paired baseline")
            if self.benchmark["expected_budget_exhaustion"]:
                for row in self.benchmark["rows"]:
                    trace=json.loads((self.directory / row["trace_file"]).read_text())
                    require(trace["status"] == "failed" and trace["dispatched_goals"] == 0
                            and trace["error"]["type"] == "PlanningBudgetExceeded",
                            "Expected budget exhaustion before dispatch was not observed")
            if self.benchmark["required_all_success"]:
                require(all(row["status"] == "succeeded" for row in self.benchmark["rows"]), "Smoke regression requires every scored motion to succeed")
            self.reset()
            self.benchmark["status"] = "completed"
        except BaseException as error:
            self.benchmark.update(status="aborted", error=dict(type=type(error).__name__, message=str(error)))
            raise
        finally:
            self.save_benchmark()
        return dict(random_benchmark_path=str(self.directory / "benchmark.json"),
            random_benchmark_sha256=file_sha256(self.directory / "benchmark.json"),
            command_trace_directory=str(self.directory), workload_sha256=workload_hash(self.workload),
            scored_requests=len(self.benchmark["rows"]), one_warm_simulation=True,
            contact_guard=deepcopy(self.contact_guard))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--domain", type=int, default=139)
    parser.add_argument("--seed", type=int)
    parser.add_argument("--count", type=int)
    parser.add_argument("--radius", type=float)
    parser.add_argument("--workload", type=Path, help="Replay an unchanged saved workload instead of sampling new cases")
    parser.add_argument("--compute-budget", type=float, default=5.0)
    parser.add_argument("--output", type=Path, help="New directory under runtime/")
    parser.add_argument("--expect-budget-exhaustion", action="store_true", help="Negative regression: require every scored request to exhaust its budget without dispatch")
    parser.add_argument("--require-all-success", action="store_true", help="Positive smoke regression: fail if any scored motion is unsuccessful")
    args = parser.parse_args()
    if args.workload and any(v is not None for v in (args.seed, args.count, args.radius)):
        parser.error("--workload cannot be combined with seed/count/radius")
    args.seed = 20261021 if args.seed is None else args.seed
    args.count = 5 if args.count is None else args.count
    args.radius = 0.25 if args.radius is None else args.radius
    if args.require_all_success and args.expect_budget_exhaustion:
        parser.error("Positive and negative regression expectations cannot be combined")
    PlanningBudget(args.compute_budget)
    args.robot, args.scene, args.gui, args.stop_mode = "fr5_ur5e", "default", False, "interrupt"
    args.require_gpu, args.screenshot_dir = False, None
    return run_verification(args, ROOT, rclpy, lambda cell, progress: RandomMotionBenchmark(cell, progress, args), Trigger,
        profile_loader=lambda *a, **kw: load_workcell(root=kw["root"]), validator=check_workcell,
        command_builder=lambda cell, args: ["bash", str(ROOT / "run_scene.sh"), f"workcell:={cell.id}", "gui:=false", "rviz:=false", "planning_case:=dual_arm_crossing"],
        report_prefix="random_motion_", launch_environment=diagnostic_environment, finalize=finalize_physics)


if __name__ == "__main__":
    raise SystemExit(main())
