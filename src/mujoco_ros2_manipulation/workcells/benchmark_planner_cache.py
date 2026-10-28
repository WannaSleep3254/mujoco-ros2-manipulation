"""Separate opt-in W08 owned-ROS experiment; no publication or automatic retry."""
import argparse
from copy import deepcopy
from datetime import datetime, timezone
import hashlib
import json
from pathlib import Path
import subprocess
import time
import uuid

from ..paths import ROOT
from ..verification.reports import atomic_json
from .planner_cache_experiment import (CONDITIONS, COMPUTE_BUDGET_SEC, CACHE_START_TOLERANCE_RAD,
    INITIAL_TOLERANCE_RAD, GOAL_TOLERANCE_RAD, canonical, digest, make_experiment, planned_rows, preserve_sources, verify_snapshot)
from .planner_cache_results import aggregate_results, summarize_trace, require_validated_cache_hit


def node_class():
    """Delay ROS imports so declarations/results remain usable without ROS."""
    from std_msgs.msg import String
    from .motion_request import COMMAND_TOPIC, PLAN_TOPIC, APPROVE_TOPIC, CANCEL_TOPIC, command
    from .motion_server import MotionCommandServer, CommandCancelled, MotionEnvironmentChanged
    from .planning_budget import PlanningBudgetExceeded
    from .trajectory_cache import TrajectoryCache
    from .verify import require
    from .verify_motion_review import MotionReviewVerification

    class PlannerCacheBenchmark(MotionReviewVerification):
        """Test-only owner; private cache faults never enter performance rows."""
        def __init__(self, cell, progress, directory, experiment, audit, save):
            self.experiment, self.audit, self.save_audit = experiment, audit, save
            self.trace_entries = []
            self.inject_validation_error = None
            self._warm_pair = None
            MotionCommandServer.__init__(self, cell, directory, progress, trace_root=ROOT/"runtime",
                compute_budget_sec=COMPUTE_BUDGET_SEC, max_plan_candidates=1)
            # Experiment only: all four conditions include the final review
            # identity RPC in the same cooperative computation budget.
            self.review_in_compute_budget = True
            self.senders = {name: self.create_publisher(String, topic, 10) for name, topic in (
                ("command", COMMAND_TOPIC), ("plan", PLAN_TOPIC), ("approve", APPROVE_TOPIC), ("cancel", CANCEL_TOPIC))}

        def _healthy(self):
            require(not self.blocked and self.contact_guard is not None and not self.contact_guard["latched"],
                    "Physical execution/interlock failure; abort remaining experiment rows")
            self.check_abort()

        def _trace_request(self, kind, target=None, *, approval=None, role):
            payload = command(target) if approval is None else approval
            entry = dict(role=role, kind=kind, request_id=payload["request_id"], status="pending")
            self.trace_entries.append(entry)
            self.save_audit()
            before = self.dispatched
            try:
                self.send_review_message(kind, payload)
            finally:
                path = self.directory/(payload["request_id"]+".json")
                if path.is_file():
                    raw = path.read_bytes()
                    trace = json.loads(raw)
                    entry.update(status=trace["status"], trace_path=str(path),
                        trace_sha256=hashlib.sha256(raw).hexdigest(),
                        dispatched_delta=self.dispatched-before)
                    self.save_audit()
            require(path.is_file(), "Request trace missing; do not retry the request")
            self._healthy()
            return trace

        def _plan(self, target, role, *, require_success=True):
            before = self.dispatched
            trace = self._trace_request("plan", target, role=role)
            require(trace.get("mode") == "plan_only" and trace["dispatched_goals"] == 0
                    and self.dispatched == before, "Plan-only request dispatched a goal")
            if require_success:
                require(trace["status"] == "succeeded" and trace.get("trajectories")
                    and trace.get("planned_clearance", {}).get("complete") is True
                    and trace.get("moveit_safety_samples", 0) > 0,
                    "Functional/preparation plan failed its complete final validators")
                review = self.review_store.summary()
                require(review and review["plan_request_id"] == trace["request_id"]
                        and review["state"] == "awaiting_approval", "No new one-use review exists")
            return trace

        def _configure(self, condition):
            self.configure_planning(planner_id=condition["planner_id"], trajectory_cache=condition["cache"],
                cache_start_tolerance_rad=CACHE_START_TOLERANCE_RAD if condition["cache"] else 0.)
            self.compute_budget_sec, self.max_plan_candidates, self.goal_tolerance_rad = COMPUTE_BUDGET_SEC, 1, GOAL_TOLERANCE_RAD

        def drive_initial(self, desired, role):
            """Bounded ordinary command corrections; never replace observed state."""
            started = time.perf_counter()
            previous = (self.planner_id, self.trajectory_cache, self.compute_budget_sec,
                        self.max_plan_candidates, self.goal_tolerance_rad, deepcopy(self.planning_configuration))
            entry = dict(role=role, status="running", desired_rad=deepcopy(desired),
                initial_actual_rad=self.state(), requests=[], maximum_corrections=3)
            self.audit["preparation"].append(entry)
            self.save_audit()
            try:
                self.configure_planning(planner_id="RRTConnectkConfigDefault", trajectory_cache=False)
                self.compute_budget_sec, self.max_plan_candidates, self.goal_tolerance_rad = 30., 12, .00001
                for attempt in range(4):
                    self.quiet()
                    actual = self.state()
                    gap = {name: desired[name]-actual[name] for name in self.joint_names}
                    if max(abs(value) for value in gap.values()) <= INITIAL_TOLERANCE_RAD:
                        break
                    require(attempt == 0 or max(abs(value) for value in gap.values()) <= .03,
                            "Ordinary feedback preparation correction exceeds 0.03rad")
                    target = desired if attempt == 0 else {name: self.commanded[name]+gap[name] for name in self.joint_names}
                    record = self._trace_request("command", target, role=role+":"+str(attempt))
                    entry["requests"].append(record["request_id"])
                    require(record["status"] == "succeeded", "Ordinary unscored preparation command failed")
                self.quiet()
                entry["actual_rad"] = self.state()
                entry["actual_max_error_rad"] = max(abs(desired[name]-entry["actual_rad"][name]) for name in self.joint_names)
                require(entry["actual_max_error_rad"] <= INITIAL_TOLERANCE_RAD, "Actual initial-state preparation did not converge")
                entry["status"] = "completed"
            except BaseException as error:
                entry.update(status="failed", error=dict(type=type(error).__name__, message=str(error)))
                raise
            finally:
                (self.planner_id, self.trajectory_cache, self.compute_budget_sec,
                 self.max_plan_candidates, self.goal_tolerance_rad, self.planning_configuration) = previous
                entry["wall_sec"] = time.perf_counter()-started
                self.save_audit()

        def _validate_cached_anchor(self, curves, initial, target):
            injection = self.inject_validation_error
            if injection is not None:
                self.inject_validation_error = None
                self.active.data["experiment_test_only_injection"] = injection
                error_type = {"cancel": CommandCancelled, "budget": PlanningBudgetExceeded,
                              "identity": MotionEnvironmentChanged}[injection]
                raise error_type("Test-only cache-hit "+injection+" passthrough")
            return super()._validate_cached_anchor(curves, initial, target)

        def _seed(self, *, clock=None):
            self._configure(CONDITIONS[2])
            if clock is not None:
                self.trajectory_cache = TrajectoryCache(self.joint_names,
                    goal_tolerance_rad=GOAL_TOLERANCE_RAD,
                    start_tolerance_rad=CACHE_START_TOLERANCE_RAD, clock=clock)
            return self._plan(self.experiment["case"]["target_rad"], "functional:seed")

        @staticmethod
        def _lookup(trace):
            return trace.get("trajectory_cache", {}).get("lookup") or {}

        def functional(self, row):
            name, target = row["case_id"], self.experiment["case"]["target_rad"]
            initial = self.experiment["case"]["initial_rad"]
            details = dict(test_only=row["test_only"], performance_sample=False)
            if name == "expiry":
                shift = [0]
                cold = self._seed(clock=lambda: time.monotonic_ns()+shift[0])
                shift[0] = 301_000_000_000
            else:
                cold = self._seed()
            details["seed_request_id"] = cold["request_id"]
            if name == "cold_then_warm_new_review":
                warm = self._plan(target, name)
                require(self._lookup(cold).get("hit") is False and self._lookup(warm).get("hit") is True,
                        "Expected cold miss followed by warm hit")
                require_validated_cache_hit(warm)
                require(cold["review"]["plan_request_id"] != warm["review"]["plan_request_id"]
                    and cold["review"]["plan_sha256"] != warm["review"]["plan_sha256"],
                    "Warm plan reused a review")
                details["warm_request_id"] = warm["request_id"]
            elif name == "goal_mismatch":
                changed = dict(target)
                changed[self.joint_names[0]] += .04
                trace = self._plan(changed, name)
                require(self._lookup(trace).get("reason") == "goal_mismatch", "Changed goal did not miss cache")
            elif name == "start_mismatch":
                changed = dict(initial)
                changed[self.joint_names[0]] += .03
                try:
                    self.drive_initial(changed, "functional:start_mismatch")
                    trace = self._plan(target, name)
                    require(self._lookup(trace).get("reason") == "start_mismatch", "Changed actual start did not miss")
                    details["actual_start"] = trace["initial_state_rad"]
                finally:
                    self.drive_initial(initial, "functional:start_reset")
            elif name == "expiry":
                trace = self._plan(target, name)
                lookup = self._lookup(trace)
                require(lookup.get("reason") == "expired" and lookup.get("hit") is False
                    and lookup.get("evicted_expired", 0) >= 1 and trace.get("plans")
                    and (trace.get("trajectory_cache", {}).get("store") or {}).get("stored") is True,
                    "Expired candidate must be reported, evicted and replaced by a validated fresh plan")
                details["test_only_clock_offset_sec"] = 301.
            elif name == "environment_identity_change":
                try:
                    self.alter_world()
                    trace = self._plan(target, name)
                    require(self._lookup(trace).get("reason") == "identity_mismatch"
                        and "environment_sha256" in self._lookup(trace).get("mismatch_fields", []),
                        "Actual changed planning-scene identity did not miss cache")
                finally:
                    self.alter_world(remove=True)
            elif name == "invalid_cached_path_fresh_fallback":
                # Test-only private cache fault. The changed intermediate point
                # remains structurally valid, but must fail current joint/path
                # validators. No malformed path enters a performance sample.
                from dataclasses import replace
                cache = self.trajectory_cache
                key, entry = next(iter(cache._entries.items()))
                value = json.loads(entry.raw)
                points = value["trajectories"][0]["points"]
                middle = deepcopy(points[0])
                middle["t"] = (points[0]["t"]+points[-1]["t"])/2
                middle["p"][0] = 100.
                value["trajectories"][0]["points"] = [points[0], middle, points[-1]]
                raw = canonical(value)
                cache._entries[key] = replace(entry, raw=raw, sha256=hashlib.sha256(raw).hexdigest())
                trace = self._plan(target, name)
                receipt = trace.get("trajectory_cache", {})
                require(self._lookup(trace).get("hit") is True and receipt.get("reason") == "validation_rejected_fallback"
                    and receipt.get("fallback") is True
                    and receipt.get("fallback_planning_wall_sec") is not None and trace.get("plans"),
                    "Invalid cached candidate did not fail full validation before fresh fallback")
                details["injection"] = "Private entry with recomputed structural checksum; intermediate joint=100rad"
            elif name == "exact_one_use_approval":
                warm = self._plan(target, name+":warm")
                require_validated_cache_hit(warm)
                review = self.review_store.summary()
                approval = dict(schema_version=1, request_id=uuid.uuid4().hex,
                    plan_request_id=warm["request_id"], session_id=self.session_id, plan_sha256=review["plan_sha256"])
                before = self.dispatched
                executed = self._trace_request("approve", approval=approval, role=name)
                require(executed["status"] == "succeeded" and self.dispatched-before == 2
                    and executed["trajectories"] == warm["trajectories"] and not executed.get("plans"),
                    "Approval did not execute the exact warm-reviewed path as two arm goals")
                self.send_review_message("approve", approval)
                require(self.dispatched-before == 2, "Duplicate approval dispatched again")
                rejected = self._trace_request("approve", approval=dict(approval, request_id=uuid.uuid4().hex), role=name+":consumed")
                require(rejected["status"] in ("failed", "rejected") and rejected["dispatched_goals"] == 0,
                        "Consumed review accepted a second execution")
                details.update(approved_request_id=executed["request_id"], dispatched_goals=2,
                    exact_trajectories=True, duplicate_dispatched_goals=0)
                self.drive_initial(initial, "functional:after_approval_reset")
            else:
                injection = {"cancel_passthrough": "cancel", "budget_passthrough": "budget",
                             "identity_error_passthrough": "identity"}[name]
                self.inject_validation_error = injection
                try:
                    trace = self._plan(target, name, require_success=False)
                finally:
                    self.inject_validation_error = None
                expected = {"cancel": "CommandCancelled", "budget": "PlanningBudgetExceeded",
                            "identity": "MotionEnvironmentChanged"}[injection]
                require(self._lookup(trace).get("hit") is True and (trace.get("error") or {}).get("type") == expected
                    and trace.get("status") == ("cancelled" if injection == "cancel" else "failed")
                    and not trace.get("plans")
                    and not trace.get("trajectory_cache", {}).get("fallback")
                    and trace.get("trajectory_cache", {}).get("fallback_planning_wall_sec") in (None, 0.),
                    "Cancellation/budget/identity error was hidden by cache fallback")
                details["test_only_injection"] = expected
            return details

        def run(self):
            try:
                self.prepare()
                for publisher in self.senders.values():
                    self.until(lambda: publisher.get_subscription_count() >= 1, 5, "experiment topic subscription")
                self.audit["session_id"] = self.session_id
                self.audit["request_traces"] = self.trace_entries
                self.drive_initial(self.experiment["case"]["initial_rad"], "initial_state")
                for row in self.audit["rows"]:
                    if not row["selected"]:
                        continue
                    row.update(status="running", started_utc=datetime.now(timezone.utc).isoformat())
                    row["trace_entry_start"] = len(self.trace_entries)
                    self.audit["current_row_id"] = row["row_id"]
                    self.save_audit()
                    started = time.perf_counter()
                    try:
                        verify_snapshot(ROOT, self.audit["input_snapshot"])
                        self._healthy()
                        if row["kind"] == "functional":
                            row["functional_evidence"] = self.functional(row)
                        else:
                            condition = next(item for item in CONDITIONS if item["id"] == row["condition"])
                            if row["kind"] == "warm_fill":
                                self._configure(condition)
                                self._warm_pair = (row["repeat"], row["condition"])
                            elif condition["expected_cache"] != "hit":
                                self._configure(condition)
                            else:
                                require(self._warm_pair == (row["repeat"], row["condition"]), "Warm row lacks its predeclared seed")
                            self.quiet()
                            record = self._plan(self.experiment["case"]["target_rad"], row["row_id"],
                                                require_success=row["kind"] == "warm_fill")
                            if row["scored"]:
                                row["summary"] = summarize_trace(self.experiment, row, record)
                            else:
                                row["preparation_request_id"] = record["request_id"]
                            if row["kind"] == "performance":
                                self._warm_pair = None
                        row["status"] = "completed"
                    except BaseException as error:
                        row.update(status="aborted", error=dict(type=type(error).__name__, message=str(error)))
                        raise
                    finally:
                        row.update(outer_wall_sec=time.perf_counter()-started, trace_entry_end=len(self.trace_entries),
                                   completed_utc=datetime.now(timezone.utc).isoformat())
                        self.save_audit()
                self.audit["status"] = "completed"
            except BaseException as error:
                self.audit.update(status="aborted", error=dict(type=type(error).__name__, message=str(error)))
                for row in self.audit["rows"]:
                    if row["selected"] and row["status"] == "not_run":
                        row["not_run_reason"] = "experiment_aborted; no retry"
                raise
            finally:
                self.audit["current_row_id"] = None
                self.audit["summary"] = aggregate_results(self.experiment, self.audit["rows"])
                self.save_audit()
            return dict(planner_cache_experiment_path=str(Path(self.audit["output"])/"experiment-run.json"),
                        planner_cache_experiment_sha256=digest(self.audit), contact_guard=deepcopy(self.contact_guard))
    return PlannerCacheBenchmark


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--list", action="store_true", help="Read declarations only; no ROS imports or launch")
    parser.add_argument("--mode", choices=("all", "performance", "functional"), default="all")
    parser.add_argument("--repeats", type=int, default=3)
    parser.add_argument("--domain", type=int, help="Explicit unused owned ROS domain, 0..232")
    parser.add_argument("--output", type=Path, help="Explicit new directory under this workspace runtime/")
    args = parser.parse_args(argv)
    from .config import load_workcell
    from .integrity import check_workcell
    cell = load_workcell("fr5_ur5e")
    experiment = make_experiment(cell, repeats=args.repeats)
    rows = planned_rows(experiment, mode=args.mode)
    if args.list:
        print(json.dumps(dict(experiment=experiment, experiment_sha256=digest(experiment), rows=rows), ensure_ascii=False, indent=2))
        return 0
    if (args.domain is None or not 0 <= args.domain <= 232 or args.output is None
            or args.output.exists() or not args.output.resolve().is_relative_to((ROOT/"runtime").resolve())):
        parser.error("Execution requires --domain 0..232 and an explicit NEW --output under runtime/")
    metadata = check_workcell(cell)
    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    atomic_json(output/"experiment.json", experiment)
    audit = dict(schema_version=1, id="planner-cache-raw-v1", experiment_id=experiment["id"],
        experiment_sha256=digest(experiment), run_id=uuid.uuid4().hex, status="declared", output=str(output),
        ros_domain_id=args.domain, mode=args.mode, rows=rows, preparation=[], request_traces=[],
        current_row_id=None, error=None, started_utc=datetime.now(timezone.utc).isoformat(),
        code_commit=subprocess.check_output(["git", "-C", str(ROOT), "rev-parse", "HEAD"], text=True).strip(),
        note="All rows declared before launch; performance and functional faults separate; no automatic retry/publication")
    def save():
        audit["summary"] = aggregate_results(experiment, rows)
        atomic_json(output/"experiment-run.json", audit)
    save()
    try:
        audit["input_snapshot"] = preserve_sources(output, ROOT, metadata)
        audit["status"] = "running"
        save()
        verify_snapshot(ROOT, audit["input_snapshot"])
        import rclpy
        from std_srvs.srv import Trigger
        from ..verification.diagnostics import diagnostic_environment, finalize_physics
        from ..verification.runner import run_verification
        from .verify_motion_reproduction import require_unused_domain
        require_unused_domain(args.domain)
        args.robot, args.scene, args.gui, args.stop_mode = cell.id, "default", False, "interrupt"
        args.require_gpu, args.screenshot_dir = False, None
        benchmark = node_class()
        captured = {}
        def on_launch(report, profile, child_args, process):
            captured["report"] = report
            audit["verification_result_path"] = str(report.path)
            save()
        code = run_verification(args, ROOT, rclpy,
            lambda profile, progress: benchmark(profile, progress, output/"traces", experiment, audit, save), Trigger,
            profile_loader=lambda *a, **kw: load_workcell("fr5_ur5e", root=kw["root"]), validator=check_workcell,
            command_builder=lambda profile, child: ["bash", str(ROOT/"run_scene.sh"), "workcell:=fr5_ur5e",
                "gui:=false", "rviz:=false", "planning_case:=dual_arm_crossing"],
            report_prefix="planner_cache_"+audit["run_id"]+"_", launch_environment=diagnostic_environment,
            finalize=finalize_physics, on_launch=on_launch)
        audit["verification_exit_code"] = code
        report = captured.get("report")
        if report is not None and report.path.is_file():
            raw = report.path.read_bytes()
            lifecycle = json.loads(raw)
            audit["verification_result_sha256"] = hashlib.sha256(raw).hexdigest()
            audit["verification"] = {name: deepcopy(lifecycle.get(name)) for name in (
                "status", "motion_checks_passed", "physics_checks_passed", "shutdown_clean",
                "launch_return_code", "process_group_gone", "error", "failure_stage")}
        if code != 0:
            audit["status"] = "aborted"
        return code
    except BaseException as error:
        audit.update(status="aborted", error=dict(type=type(error).__name__, message=str(error)))
        return 130 if isinstance(error, KeyboardInterrupt) else 1
    finally:
        if audit["status"] == "aborted":
            for row in rows:
                if row["selected"] and row["status"] == "not_run":
                    row["not_run_reason"] = "experiment_or_owned_lifecycle_aborted; no retry"
        audit["completed_utc"] = datetime.now(timezone.utc).isoformat()
        save()
        detail = audit.get("error") or {}
        print(json.dumps(dict(status=audit["status"], output=str(output),
            result_path=str(output/"experiment-run.json"),
            error_path=str(output/"experiment-run.json") if audit["status"] == "aborted" else None,
            error_type=detail.get("type"), verification_result_path=audit.get("verification_result_path")),
            ensure_ascii=False), flush=True)


if __name__ == "__main__":
    raise SystemExit(main())
