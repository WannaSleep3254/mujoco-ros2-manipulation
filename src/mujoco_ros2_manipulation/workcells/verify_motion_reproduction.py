"""Fresh ROS requests for a fixed home input; all outcomes remain evidence."""
from copy import deepcopy
import json
import time
import uuid

import rclpy
from rclpy.executors import SingleThreadedExecutor
from std_msgs.msg import String
from std_srvs.srv import Trigger

from mujoco_ros2_manipulation.paths import ROOT
from mujoco_ros2_manipulation.model_build.integrity import file_sha256
from mujoco_ros2_manipulation.verification.diagnostics import diagnostic_environment, finalize_physics
from mujoco_ros2_manipulation.verification.reports import atomic_json
from mujoco_ros2_manipulation.verification.runner import run_verification
from .config import load_workcell
from .integrity import check_workcell
from .interference import Spline
from .verify_interference_comparison import to_ros
from .motion_request import COMMAND_TOPIC, command
from .motion_server import MotionCommandServer
from .motion_diagnostics import timing_breakdown
from .reproduction import load_reproduction, replay_witness
from .verify import require


class MotionReproduction(MotionCommandServer):
    def __init__(self, cell, progress, args):
        directory = args.output or ROOT / "runtime/validation/motion-reproductions" / uuid.uuid4().hex
        super().__init__(cell, directory, progress, compute_budget_sec=args.compute_budget)
        self.fixture = load_reproduction(args.fixture, cell, self.metadata)
        self.command_publisher = self.create_publisher(String, COMMAND_TOPIC, 10)
        self.repeats = args.repeats
        self.preparation = None
        self.result_record = dict(schema_version=1, status="running", fixture_sha256=file_sha256(args.fixture),
            fixture_id=self.fixture["id"], compute_budget_wall_sec=args.compute_budget,
            source_sha256={name: file_sha256(ROOT / "src/mujoco_ros2_manipulation/workcells" / name)
                for name in ("verify_motion_reproduction.py", "reproduction.py", "reproduce_motion.py")},
            initial_tolerance_rad=0.0015, maintenance=[], requests=[],
            note="Fixed inputs, fresh nondeterministic MoveIt paths. A passing path is retained, not forced to reproduce a historical failure.")
        self.save_result()

    def save_result(self):
        atomic_json(self.directory / "reproduction.json", self.result_record)

    def submit(self, target, strategy, role):
        payload = command(target, strategy)
        self.command_publisher.publish(String(data=json.dumps(payload)))
        self.until(lambda: self.pending is not None or payload["request_id"] in self.replies, 5, "reproduction command receipt")
        if self.pending:
            self.process_next()
        path = self.directory / (payload["request_id"]+".json")
        record = json.loads(path.read_text())
        row = dict(role=role, request_id=payload["request_id"], strategy=strategy,
            status=record["status"], trace_file=path.name, trace_sha256=file_sha256(path),
            dispatched_goals=record["dispatched_goals"], error=record["error"],
            failure_stage=record.get("failure_stage"), timing=timing_breakdown(record),
            clearance_diagnostic=record.get("clearance_diagnostic"))
        self.result_record["maintenance" if role == "setup" else "requests"].append(row)
        self.save_result()
        require(not self.blocked and not self.contact_guard["latched"], "Execution/interlock failure; no further requests are permitted")
        require(record["status"] in ("succeeded", "failed"), "Unexpected rejection/cancellation in fixed reproduction")
        if record["status"] == "failed":
            require(record["dispatched_goals"] == 0 and record["metrics"]["received_to_first_motion_wall_sec"] is None,
                    "A failed reproduction must not have dispatched or moved")
        return record, row

    def plan(self, group, target, start_state=None):
        if self.preparation is None:
            return super().plan(group, target, start_state)
        self.compute_checkpoint()
        self.event("planning_"+group)
        with self.active.span("preparation_trajectory"):
            curve = Spline(**self.preparation["trajectory"])
            require(group == "both_arms" and set(curve.names) == set(start_state), "Saved preparation must cover both arms")
            start_delta = max(abs(curve.points[0]["p"][j]-start_state[n]) for j, n in enumerate(curve.names))
            require(start_delta <= .01,
                    "Saved preparation start differs by more than 0.01rad")
            require(max(abs(curve.points[-1]["p"][j]-target[n]) for j, n in enumerate(curve.names)) <= .003+1e-9,
                    "Saved preparation endpoint differs from its declared target")
            self.active.data["preparation_source"] = dict(self.preparation["source"],
                initial_anchor_delta_rad=start_delta, planning_source="saved_preparation_anchored",
                original_goal_tolerance_rad=.003)
            trajectory = self.anchored(to_ros(curve))
        self.active.data.setdefault("plans", []).append(dict(group=group, source="saved_preparation_anchored",
            wall_sec=None, server_planning_sec=None, requested_planner_id=None,
            note="No MoveIt planning RPC; the current full final checks still apply"))
        self.compute_checkpoint()
        return trajectory

    def reset_initial(self):
        self.goal_tolerance_rad = 0.00001
        try:
            # An unchanged failed home request leaves both arms at the fixed start.
            drift = max(abs(self.state()[n]-q) for n, q in self.fixture["initial_rad"].items())
            waypoints = [] if drift <= self.result_record["initial_tolerance_rad"] else [step["target_rad"] for step in self.fixture.get("setup", {}).get("sequence", [])]
            for index, target in enumerate(waypoints):
                self.preparation = self.fixture["_setup_trajectories"][index] if index == len(waypoints)-1 else None
                try:
                    record, _ = self.submit(target, "coordinated", "setup")
                    require(record["status"] == "succeeded", "Saved setup failed current checks; run preserved")
                finally:
                    self.preparation = None
            record, _ = self.submit(self.fixture["initial_rad"], "coordinated", "setup")
            require(record["status"] == "succeeded", "Setup path was refused; preserve this run rather than bypassing checks")
        finally:
            self.goal_tolerance_rad = 0.003
        self.quiet()
        drift = max(abs(self.state()[n]-q) for n, q in self.fixture["initial_rad"].items())
        require(drift <= self.result_record["initial_tolerance_rad"], "Reproduction start drift exceeds tolerance")

    def run(self):
        try:
            self.prepare()
            self.until(lambda: self.command_publisher.get_subscription_count() >= 1, 5, "reproduction command subscription")
            self.result_record["historical_witness"] = replay_witness(self.fixture, self.evaluator,
                self.policy["min_inter_arm_distance_m"]+self.policy["tracking_reserve_m"])
            self.result_record["endpoint_moveit_validity"] = {
                role: self.validity_of(self.fixture[key]).valid for role, key in (("start", "initial_rad"), ("goal", "target_rad"))}
            self.save_result()
            require(all(self.result_record["endpoint_moveit_validity"].values()), "Fixed endpoint is invalid in the current MoveIt scene")
            for strategy in ("coordinated", "departure_timing"):
                for repeat in range(self.repeats):
                    self.progress(f"home_reproduction_{strategy}_{repeat+1}")
                    self.reset_initial()
                    record, row = self.submit(self.fixture["target_rad"], strategy, "home_return")
                    row["repeat"] = repeat+1
                    row["initial_drift_rad"] = max(abs(record["initial_state_rad"][n]-q) for n, q in self.fixture["initial_rad"].items())
                    require(row["initial_drift_rad"] <= self.result_record["initial_tolerance_rad"], "Home request did not start from the fixed state")
                    if strategy == "coordinated":
                        row["historical_failure_reproduced"] = record["status"] == "failed" and record.get("clearance_diagnostic", {}).get("violation_location") == "intermediate"
                    else:
                        row["historical_failure_reproduced"] = record["status"] == "failed" and record["error"]["type"] == "TimingRejected"
                    self.save_result()
            self.result_record["status"] = "completed"
        except BaseException as error:
            self.result_record.update(status="aborted", error=dict(type=type(error).__name__, message=str(error)))
            raise
        finally:
            self.save_result()
        return dict(reproduction_path=str(self.directory / "reproduction.json"),
            reproduction_sha256=file_sha256(self.directory / "reproduction.json"),
            command_trace_directory=str(self.directory), fixture_sha256=self.result_record["fixture_sha256"],
            historical_witness_reproduced=True, fixed_input_requests=len(self.result_record["requests"]),
            contact_guard=deepcopy(self.contact_guard))


def require_unused_domain(domain):
    """Refuse a domain with other nodes before creating an owned simulation."""
    context = rclpy.context.Context()
    probe = executor = None
    try:
        rclpy.init(context=context, domain_id=domain)
        name = "motion_reproduction_probe_"+uuid.uuid4().hex
        probe = rclpy.create_node(name, context=context, enable_rosout=False)
        executor = SingleThreadedExecutor(context=context)
        executor.add_node(probe)
        deadline = time.monotonic()+1.0
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.1)
        others = [(n, ns) for n, ns in probe.get_node_names_and_namespaces() if n != name]
        require(not others, "ROS domain already contains nodes; choose an unused --domain: "+str(others))
    finally:
        if executor is not None:
            executor.shutdown()
        if probe is not None:
            probe.destroy_node()
        if context.ok():
            context.shutdown()


def run_live(args):
    # Validate before starting any owned ROS processes.
    cell = load_workcell()
    load_reproduction(args.fixture, cell, check_workcell(cell))
    require_unused_domain(args.domain)
    args.robot, args.scene, args.gui, args.stop_mode = cell.id, "default", False, "interrupt"
    args.require_gpu, args.screenshot_dir = False, None
    return run_verification(args, ROOT, rclpy, lambda cell, progress: MotionReproduction(cell, progress, args), Trigger,
        profile_loader=lambda *a, **kw: load_workcell(root=kw["root"]), validator=check_workcell,
        command_builder=lambda cell, args: ["bash", str(ROOT / "run_scene.sh"), f"workcell:={cell.id}", "gui:=false", "rviz:=false", "planning_case:=dual_arm_crossing"],
        report_prefix="home_reproduction_", launch_environment=diagnostic_environment, finalize=finalize_physics)
