"""Simulation-only persistent command executor with request-correlated wall timings."""
import argparse
from copy import deepcopy
import json
from pathlib import Path
import time
import subprocess
import signal
import platform
from rclpy.utilities import get_rmw_implementation_identifier
import uuid

import rclpy
from rcl_interfaces.srv import GetParameters
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String
from moveit_msgs.msg import MoveItErrorCodes

from mujoco_ros2_manipulation.paths import ROOT
from mujoco_ros2_manipulation.model_build.integrity import file_sha256
from mujoco_ros2_manipulation.verification.diagnostics import host_info
from mujoco_ros2_manipulation.verification.reports import atomic_json
from mujoco_ros2_manipulation.workcells.config import load_workcell
from mujoco_ros2_manipulation.workcells.integrity import check_workcell
from mujoco_ros2_manipulation.workcells.distance import DistanceEvaluator
from mujoco_ros2_manipulation.workcells.interference import Spline, stretch
from mujoco_ros2_manipulation.workcells.path_quality import evaluate_path, evaluate_states, sample_schedule
from mujoco_ros2_manipulation.workcells.timing import choose_delay, load_policy
from mujoco_ros2_manipulation.workcells.verify import WorkcellVerification, require, seconds
from mujoco_ros2_manipulation.workcells.verify_interference import InterferenceVerification
from mujoco_ros2_manipulation.workcells.verify_interference_comparison import to_ros
from mujoco_ros2_manipulation.workcells.motion_request import (
    COMMAND_TOPIC, CANCEL_TOPIC, STATUS_TOPIC, decode_message, request_id, validate_command, validate_cancel,
)
from mujoco_ros2_manipulation.workcells.request_trace import RequestTrace
from mujoco_ros2_manipulation.workcells.planning_budget import PlanningBudget


def status_qos():
    return QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL)


class CommandCancelled(RuntimeError):
    pass


class MotionCommandServer(WorkcellVerification):
    split = InterferenceVerification.split

    def __init__(self, cell, output, progress=None, *, compute_budget_sec=30.0):
        PlanningBudget(compute_budget_sec)  # Validate before creating a ROS node/output directory.
        self.compute_budget_sec = float(compute_budget_sec)
        self.compute_budget = None
        self.goal_tolerance_rad = 0.003
        self.metadata = check_workcell(cell)
        _, self.policy = load_policy(cell)
        self.directory = Path(output).resolve()
        require(self.directory.is_relative_to((cell.root / "runtime").resolve()), "Command traces must be under runtime/")
        self.directory.mkdir(parents=True, exist_ok=False)
        self.session_id = uuid.uuid4().hex
        self.pending = self.active = None
        self.initialized = self.cancel_requested = self.blocked = self.stop_requested = False
        self.state_received_ns = self.clock_received_ns = None
        self.state_stamp = None
        self.replies = {}
        self.observations = []
        self.first_by_arm = set()
        self.motion_start = None
        super().__init__(cell, progress)
        self.evaluator = DistanceEvaluator(cell)
        self.publisher = self.create_publisher(String, STATUS_TOPIC, status_qos())
        self.command_subscription = self.create_subscription(String, COMMAND_TOPIC, self.receive, 10)
        self.create_subscription(String, CANCEL_TOPIC, self.cancel, 10)
        self.create_timer(1.0, self.heartbeat)
        self.dispatch_observer = self.action_event
        self.dispatch_abort = self.check_abort
        atomic_json(self.directory / "session.json", dict(schema_version=1, session_id=self.session_id,
            workcell=cell.id, model_manifest=self.metadata, policy=self.policy,
            evaluator=self.evaluator.metadata(), motion_threshold_rad=0.001, host=host_info(),
            runtime=dict(python=platform.python_version(), os=platform.freedesktop_os_release().get("PRETTY_NAME", platform.platform()),
                         middleware=get_rmw_implementation_identifier()),
            code_commit=subprocess.check_output(["git", "-C", str(cell.root), "rev-parse", "HEAD"], text=True).strip(),
            compute_budget_sec=self.compute_budget_sec,
            implementation_sha256={name: file_sha256(cell.root / "src/mujoco_ros2_manipulation/workcells" / name)
                for name in ("motion_server.py", "motion_client.py", "motion_request.py", "request_trace.py", "verify.py", "timing.py", "path_quality.py", "planning_budget.py")},
            note="Warm persistent server; simulator startup excluded from request latency. Simulation only."))

    def destroy_node(self):
        try:
            if hasattr(self, "evaluator"):
                self.evaluator.close()
        finally:
            result = super().destroy_node()
        return result

    def publish(self, value):
        self.publisher.publish(String(data=json.dumps(value, allow_nan=False)))

    def heartbeat(self):
        if self.initialized and self.active is None and self.pending is None:
            self.publish(dict(schema_version=1, session_id=self.session_id, request_id=None,
                              status="blocked" if self.blocked else "ready", trace_directory=str(self.directory)))

    def path(self, trace):
        return self.directory / (trace.data["request_id"] + ".json")

    def persist(self, trace):
        trace.save(self.path(trace))
        reply = trace.summary(self.path(trace), self.session_id)
        self.replies[trace.data["request_id"]] = reply
        self.publish(reply)

    def event(self, name, **details):
        if self.active:
            self.active.event(name, simulation_sec=self.clock_last, **details)
            self.persist(self.active)

    def check_abort(self):
        if self.cancel_requested or self.stop_requested:
            raise CommandCancelled("Cancellation requested; no later command stage is allowed")
        require(not self.contact_guard or not self.contact_guard["latched"], "Contact/clock interlock latched")

    def receive(self, message):
        received = time.monotonic_ns()
        identifier = uuid.uuid4().hex
        try:
            data = decode_message(message.data)
            identifier = request_id(data.get("request_id"))
        except (ValueError, TypeError) as error:
            trace = RequestTrace(identifier, received_ns=received)
            trace.finish("rejected", dict(type=type(error).__name__, message=str(error)))
            self.persist(trace)
            return
        # Repeated IDs replay the original reply and never dispatch a second trajectory.
        if identifier in self.replies:
            self.publish(self.replies[identifier])
            return
        trace = RequestTrace(identifier, received_ns=received)
        trace.data["command"] = data
        try:
            validate_command(data, self.metadata["joint_limits"])
            require(self.initialized, "Command server is not ready")
            require(not self.blocked, "Previous execution failure requires a server restart")
            require(self.pending is None and self.active is None, "Busy: another command owns both arms")
        except (ValueError, RuntimeError) as error:
            trace.finish("rejected", dict(type=type(error).__name__, message=str(error)))
        else:
            trace.data["status"] = "queued"
            self.pending = trace
        self.persist(trace)

    def cancel(self, message):
        try:
            identifier = validate_cancel(decode_message(message.data))
        except ValueError as error:
            self.get_logger().warning("Ignored invalid cancellation: " + str(error))
            return
        target = self.active or self.pending
        if target and target.data["request_id"] == identifier:
            self.cancel_requested = True
            target.event("cancel_received", simulation_sec=self.clock_last)
            self.persist(target)
        elif identifier in self.replies:
            self.publish(self.replies[identifier])
        else:
            self.publish(dict(schema_version=1, session_id=self.session_id, request_id=identifier,
                status="rejected", stage="cancel_not_found", trace_path=None, metrics={},
                error=dict(type="UnknownRequest", message="No request with this ID in this server session")))

    def clock(self, message):
        super().clock(message)
        self.clock_received_ns = time.monotonic_ns()

    def joints(self, message):
        previous = getattr(self, "joint_count", 0)
        super().joints(message)
        if self.joint_count == previous:
            return
        self.state_received_ns = time.monotonic_ns()
        self.state_stamp = seconds(message.header.stamp)
        if not self.active or self.motion_start is None or self.state_stamp < self.motion_start:
            return
        trace = self.active
        if not self.observations or self.state_stamp > self.observations[-1]["simulation_sec"]:
            self.observations.append(dict(simulation_sec=self.state_stamp, wall_sec=trace.elapsed(), state_rad=self.state()))
        initial = trace.data["initial_state_rad"]
        for inst in self.cell.instances:
            if inst.id not in self.first_by_arm and max(abs(self.state()[n] - initial[n]) for n in inst.joints) > 0.001:
                if not self.first_by_arm:
                    self.event("first_motion", joint_state_stamp_simulation_sec=self.state_stamp)
                self.first_by_arm.add(inst.id)
                self.event("first_motion_" + inst.id, joint_state_stamp_simulation_sec=self.state_stamp)

    def fresh(self):
        now = time.monotonic_ns()
        require(self.state_received_ns is not None and (now - self.state_received_ns) / 1e9 <= 0.3, "Joint state is stale")
        require(self.clock_received_ns is not None and (now - self.clock_received_ns) / 1e9 <= 0.3, "Simulation clock is stale")
        require(not self.clock_backwards, "Simulation clock moved backwards")

    def quiet(self, *, cancellable=True):
        seen, consecutive = [self.joint_count], [0]
        def reached():
            if cancellable:
                self.check_abort()
            if seen[0] == self.joint_count:
                return False
            seen[0] = self.joint_count
            consecutive[0] = consecutive[0] + 1 if max(abs(self.latest_velocities.get(n, float("inf"))) for n in self.joint_names) < 0.0001 else 0
            return consecutive[0] >= 10
        self.until(reached, 5, "stationary complete workcell state")
        self.fresh()

    def prepare(self):
        self.until(lambda: self.positions is not None and self.clock_count > 10, 30, "shared state and clock")
        self.ready_guard()
        for client in list(self.arm_clients.values()) + [self.planner, self.validity]:
            ready = client.server_is_ready if hasattr(client, "server_is_ready") else client.service_is_ready
            self.until(ready, 20, "command execution service")
        parameters = self.create_client(GetParameters, "/mujoco_ros2_control_node/get_parameters")
        self.until(parameters.service_is_ready, 5, "simulation hardware parameters")
        answer = self.result(parameters.call_async(GetParameters.Request(names=["robot_description", "use_sim_time"])), 5, "simulation identity")
        require("<plugin>mujoco_ros2_control/MujocoSystemInterface</plugin>" in answer.values[0].string_value
                and answer.values[1].bool_value, "Persistent commands require MuJoCo simulation hardware")
        for inst in self.cell.instances:
            parameters = self.create_client(GetParameters, f"/{inst.controller}/get_parameters")
            self.until(parameters.service_is_ready, 5, "controller parameters")
            interpolation = self.result(parameters.call_async(GetParameters.Request(names=["interpolation_method"])), 5, "controller interpolation")
            require(interpolation.values[0].string_value == "splines", "Only validated spline interpolation is supported")
        require(len(self.get_publishers_info_by_topic("/clock")) == 1 and len(self.get_publishers_info_by_topic("/joint_states")) == 1, "Expected one clock and one complete joint-state publisher")
        require(self.count_publishers(STATUS_TOPIC) == 1, "Only one command server may own this ROS domain")
        require(not any(name.endswith(("/inject_test_contact", "/inject_test_clock_stall")) for name, _ in self.get_service_names_and_types()), "Fault injection must not be exposed")
        self.quiet()
        self.commanded = self.state()
        self.initialized = True
        self.heartbeat()

    def compute_checkpoint(self):
        self.check_abort()
        if self.compute_budget:
            self.compute_budget.check()

    def finish_computation(self, trace):
        self.compute_checkpoint()
        trace.data["compute_budget"] = self.compute_budget.finish()
        self.event("computation_done")

    def plan(self, group, target, start_state=None):
        self.compute_checkpoint()
        self.event("planning_" + group)
        with self.active.span("planning:" + group):
            answer = super().plan(group, target, start_state, planner_id="RRTConnectkConfigDefault",
                                  allowed_planning_sec=min(5.0, self.compute_budget.remaining()), goal_tolerance_rad=self.goal_tolerance_rad)
        self.active.data.setdefault("plans", []).append(deepcopy(self.planning_timings[-1]))
        self.compute_checkpoint()
        require(answer.error_code.val == MoveItErrorCodes.SUCCESS, "MoveIt plan failed: " + str(answer.error_code.val))
        return self.anchored(answer.trajectory.joint_trajectory)

    def action_event(self, name, arm, **details):
        self.event(name, arm=arm, **details)

    def process_next(self):
        if self.pending is None:
            return
        trace, self.pending = self.pending, None
        self.active = trace
        self.handles = []
        self.motion_start = None
        self.observations, self.first_by_arm = [], set()
        trace.data["status"] = "running"
        before_dispatch = self.dispatched
        try:
            self.event("state_snapshot")
            with trace.span("state_snapshot"):
                self.quiet()
                initial = self.state()
                self.commanded = dict(initial)
                trace.data.update(initial_state_rad=initial, joint_state_stamp_simulation_sec=self.state_stamp,
                    state_receive_age_wall_sec=(time.monotonic_ns() - self.state_received_ns) / 1e9,
                    clock_receive_age_wall_sec=(time.monotonic_ns() - self.clock_received_ns) / 1e9)
            self.compute_budget = PlanningBudget(self.compute_budget_sec)
            self.event("computation_started")
            target = trace.data["command"]["target_rad"]
            strategy = trace.data["command"]["strategy"]
            if max(abs(target[n] - initial[n]) for n in self.joint_names) <= 0.001:
                trace.data["no_motion_required"] = True
                trace.skip("planning", "Target already within the 0.001rad movement threshold")
                trace.skip("departure_search", "No motion required")
                with trace.span("safety_checks"):
                    require(self.validity_of(initial).valid, "Current state is colliding")
                    require(self.evaluator.at(initial)["distance_m"] >= self.policy["min_inter_arm_distance_m"] + self.policy["tracking_reserve_m"], "Current clearance below planning policy")
                self.finish_computation(trace)
                self.event("settled")
                trace.finish("succeeded")
                return
            if strategy == "coordinated":
                curves = [Spline.from_ros(self.plan("both_arms", target, initial))]
                trace.skip("departure_search", "Coordinated joint-space plan; no departure-only search")
            else:
                independent = [(i, self.plan(i.group, {n: target[n] for n in i.joints}, initial)) for i in self.cell.instances]
                duration = max(Spline.from_ros(t).duration for _, t in independent)
                source = [Spline.from_ros(stretch(t, duration)) for _, t in independent]
                self.event("departure_search")
                trace.data["departure_search"] = {}
                with trace.span("departure_search"):
                    chosen = choose_delay(source, initial, self.cell, self.evaluator, self.policy,
                        diagnostics=trace.data["departure_search"], checkpoint=self.compute_checkpoint)
                trace.data["departure_search"] = {k: v for k, v in chosen.items() if k != "curves"}
                curves = [Spline(**record) for record in chosen["curves"]]
            self.check_abort()
            self.event("safety_checks")
            with trace.span("safety_checks"):
                threshold = self.policy["min_inter_arm_distance_m"] + self.policy["tracking_reserve_m"]
                planned = evaluate_path(curves, initial, self.cell, self.evaluator, dt=self.policy["sample_dt_sec"], reject_below=threshold, checkpoint=self.compute_checkpoint)
                trace.data["planned_clearance"] = planned
                require(planned["complete"], "Planned clearance below 30mm policy")
                samples = 0
                for _, state in sample_schedule(curves, initial, self.policy["sample_dt_sec"]):
                    self.compute_checkpoint()
                    require(self.validity_of(state).valid, "MoveIt rejected interpolated schedule")
                    samples += 1
                trace.data["moveit_safety_samples"] = samples
            self.quiet()
            require(max(abs(self.state()[n] - initial[n]) for n in self.joint_names) < 0.002, "Robot moved while planning; stale plan refused")
            self.finish_computation(trace)
            self.motion_start = self.clock_last + 1.0
            trace.data["scheduled_start_simulation_sec"] = self.motion_start
            trace.data["trajectories"] = [c.record() for c in curves]
            commands = self.split(to_ros(curves[0]), self.motion_start) if len(curves) == 1 else [(inst, to_ros(curve, self.motion_start)) for inst, curve in zip(self.cell.instances, curves)]
            self.event("dispatch")
            with trace.span("dispatch_to_action_results"):
                self.dispatch(commands, scheduled_start=self.motion_start)
            self.event("actions_done")
            with trace.span("settling"):
                endpoint = dict(self.commanded)
                self.settle([endpoint[n] for n in self.joint_names])
            self.event("settled")
            self.check_abort()
            if len(self.observations) >= 2:
                with trace.span("observed_clearance"):
                    observed = evaluate_states(((s["simulation_sec"], s["state_rad"]) for s in self.observations), self.cell, self.evaluator)
                    trace.data["observed_clearance"] = observed
                    require(observed["minimum"]["distance_m"] >= self.policy["min_inter_arm_distance_m"], "Observed clearance below 20mm policy")
            trace.finish("succeeded")
        except BaseException as error:
            if self.dispatched > before_dispatch:
                try:
                    self.quiet(cancellable=False)
                except Exception as settling_error:
                    trace.data["post_failure_settle_error"] = str(settling_error)
                    self.blocked = True
                self.blocked = self.blocked or not isinstance(error, (CommandCancelled, KeyboardInterrupt))
            trace.finish("cancelled" if isinstance(error, (CommandCancelled, KeyboardInterrupt)) and not self.blocked else "failed",
                         dict(type=type(error).__name__, message=str(error)))
            if isinstance(error, KeyboardInterrupt):
                raise
        finally:
            if self.compute_budget:
                trace.data["compute_budget"] = self.compute_budget.finish()
                self.compute_budget = None
            trace.data["shutdown_requested"] = self.stop_requested
            trace.data["dispatched_goals"] = self.dispatched - before_dispatch
            trace.data["action_results"] = [dict(status=f.result().status, error_code=f.result().result.error_code) for _, f in self.handles if f.done() and f.result() is not None]
            trace.data["observations"] = self.observations
            trace.data["contact_guard"] = deepcopy(self.contact_guard)
            self.persist(trace)
            self.active = None
            self.motion_start = None
            self.cancel_requested = False

    def run(self):
        self.prepare()
        print("READY: " + str(self.directory), flush=True)
        try:
            while rclpy.ok() and not self.stop_requested:
                rclpy.spin_once(self, timeout_sec=0.1)
                if not self.stop_requested:
                    self.process_next()
        finally:
            if self.pending:
                self.pending.finish("cancelled", dict(type="Shutdown", message="Server stopped before execution"))
                self.persist(self.pending)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--workcell", default="fr5_ur5e")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--compute-budget", type=float, default=30.0, help="Cooperative planning/validation wall seconds, excludes execution")
    args = parser.parse_args()
    cell = load_workcell(args.workcell)
    output = args.output or ROOT / "runtime/commands" / uuid.uuid4().hex
    # Keep ROS alive while cancelling outstanding goals on SIGINT/SIGTERM.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = None
    previous_handlers = {}
    try:
        node = MotionCommandServer(cell, output, compute_budget_sec=args.compute_budget)
        def stop(signum, frame):
            node.stop_requested = True
        for signum in (signal.SIGINT, signal.SIGTERM):
            previous_handlers[signum] = signal.signal(signum, stop)
        node.run()
    except CommandCancelled:
        if node is None or not node.stop_requested:
            raise
    except KeyboardInterrupt:
        pass
    finally:
        for signum, previous in previous_handlers.items():
            signal.signal(signum, previous)
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
