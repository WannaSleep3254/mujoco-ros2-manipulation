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
import hashlib
import xml.etree.ElementTree as ET

import rclpy
from rcl_interfaces.srv import GetParameters
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import String
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.msg import PlanningSceneComponents
from moveit_msgs.srv import GetPlanningScene
from rosidl_runtime_py.convert import message_to_ordereddict

from mujoco_ros2_manipulation.paths import ROOT
from mujoco_ros2_manipulation.model_build.integrity import file_sha256
from mujoco_ros2_manipulation.verification.diagnostics import host_info
from mujoco_ros2_manipulation.verification.reports import atomic_json
from mujoco_ros2_manipulation.workcells.config import load_workcell
from mujoco_ros2_manipulation.workcells.integrity import check_workcell
from mujoco_ros2_manipulation.workcells.execution_target import ExecutionTarget, load_execution_target
from mujoco_ros2_manipulation.workcells.distance import DistanceEvaluator
from mujoco_ros2_manipulation.workcells.environment_clearance import (
    EnvironmentClearanceRejected, environment_policy_snapshot, required_environment_distance,
)
from mujoco_ros2_manipulation.workcells.self_clearance import (
    SelfClearanceRejected, self_policy_snapshot, require_self_clearance, require_self_result)
from mujoco_ros2_manipulation.workcells.interference import Spline, stretch
from mujoco_ros2_manipulation.workcells.path_quality import evaluate_path, evaluate_states, sample_schedule
from mujoco_ros2_manipulation.workcells.timing import choose_delay, load_policy, TimingRejected
from mujoco_ros2_manipulation.workcells.verify import WorkcellVerification, require, seconds
from mujoco_ros2_manipulation.workcells.verify_interference import InterferenceVerification
from mujoco_ros2_manipulation.workcells.verify_interference_comparison import to_ros
from mujoco_ros2_manipulation.workcells.motion_request import (
    COMMAND_TOPIC, CANCEL_TOPIC, STATUS_TOPIC, PLAN_TOPIC, APPROVE_TOPIC,
    decode_message, request_id, validate_command, validate_cancel, validate_approval,
)
from mujoco_ros2_manipulation.workcells.motion_review import PlanReviewStore, ReviewRejected
from mujoco_ros2_manipulation.workcells.pose_planning import PosePlanningMixin
from mujoco_ros2_manipulation.workcells.pose_adapter import PoseAdapterError
from mujoco_ros2_manipulation.workcells.pose_goal import validate_pose_goal
from mujoco_ros2_manipulation.workcells.pose_tools import describe_pose_tools, validate_goal_context
from mujoco_ros2_manipulation.workcells.request_trace import RequestTrace
from mujoco_ros2_manipulation.workcells.operation_contract import describe_operation
from mujoco_ros2_manipulation.workcells.planning_budget import PlanningBudget, PlanningBudgetExceeded
from mujoco_ros2_manipulation.workcells.plan_selection import MAX_PLAN_CANDIDATES, PlanCandidateRejected, PlanCandidatesExhausted, select_clearance_plan
from mujoco_ros2_manipulation.workcells.contracts import describe_workcell
from mujoco_ros2_manipulation.workcells.spatial_candidates import compose_serial_paths
from mujoco_ros2_manipulation.workcells.motion_diagnostics import (
    measure_endpoints, path_clearance_diagnostic, update_candidate_path_diagnostic)
from mujoco_ros2_manipulation.workcells.planning_config import (
    PLANNER_TYPES, planning_configuration, canonical_sha256, trajectory_cache_identity)
from mujoco_ros2_manipulation.workcells.trajectory_cache import TrajectoryCache
from mujoco_ros2_manipulation.workcells.cache_dynamics import validate_reanchored_dynamics, CacheDynamicsRejected


PLANNING_SOURCE_FILES = ("motion_server.py", "motion_client.py", "motion_request.py", "motion_review.py",
    "request_trace.py", "verify.py", "planning_adapter.py", "execution_adapter.py", "execution_target.py",
    "operation_contract.py", "environment_clearance.py", "self_clearance.py", "timing.py", "path_quality.py", "planning_budget.py",
    "contracts.py", "motion_diagnostics.py", "plan_selection.py", "spatial_candidates.py", "pose_goal.py",
    "pose_adapter.py", "pose_tools.py", "pose_planning.py", "planning_config.py", "trajectory_cache.py",
    "cache_dynamics.py", "prepare.py", "interference.py", "distance.py", "verify_interference.py",
    "verify_interference_comparison.py", "config.py", "integrity.py")


def status_qos():
    return QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                      durability=DurabilityPolicy.TRANSIENT_LOCAL)


class CommandCancelled(RuntimeError):
    pass


class MotionPlanningFailed(RuntimeError):
    """A completed MoveIt response with no plan, rather than a transport failure."""
    def __init__(self, error_code):
        self.error_code = int(error_code)
        super().__init__("MoveIt plan failed: " + str(self.error_code))


class InvalidMotionEndpoint(RuntimeError):
    def __init__(self, role, reason):
        self.role = role
        super().__init__(role + " endpoint is invalid: " + reason)


class MotionStateChanged(RuntimeError):
    pass


class MotionEnvironmentChanged(RuntimeError):
    pass


class MotionPathRejected(RuntimeError):
    """A completed path check rejected a candidate; transport errors stay distinct."""


class MotionCommandServer(PosePlanningMixin, WorkcellVerification):
    split = InterferenceVerification.split

    def __init__(self, cell, output, progress=None, *, compute_budget_sec=30.0, max_plan_candidates=MAX_PLAN_CANDIDATES, policy_name="dual_arm_timing", trace_root=None, execution_target=None, planner_id="RRTConnectkConfigDefault", trajectory_cache=False, cache_start_tolerance_rad=0.0, review_in_compute_budget=False):
        PlanningBudget(compute_budget_sec)  # Validate before creating a ROS node/output directory.
        self.review_in_compute_budget = review_in_compute_budget
        planning_configuration(planner_id, trajectory_cache, cache_start_tolerance_rad)
        if type(max_plan_candidates) is not int or not 1 <= max_plan_candidates <= MAX_PLAN_CANDIDATES:
            raise ValueError(f"Expected 1..{MAX_PLAN_CANDIDATES} plan candidates")
        self.max_plan_candidates = max_plan_candidates
        self._last_compute_spin_ns = 0
        self._defer_trace_persist = False
        self._trace_journal_cursor = 0
        self.compute_budget_sec = float(compute_budget_sec)
        self.compute_budget = None
        self.goal_tolerance_rad = 0.003
        self.metadata = check_workcell(cell)
        if execution_target is not None:
            if not isinstance(execution_target, ExecutionTarget) or execution_target.base is not cell:
                raise ValueError("Execution target must belong to this exact checked base workcell")
            execution_target.validate()
        self.execution_target = execution_target
        self.execution_runtime_identity = None
        policy_path, self.policy = load_policy(cell, policy_name)
        (self.environment_policy_path, self.environment_policy,
         self.environment_policy_sha256) = environment_policy_snapshot(cell)
        self.environment_threshold_m = required_environment_distance(self.environment_policy)
        self.self_policy_path, self.self_policy, self.self_policy_sha256 = self_policy_snapshot(cell)
        self.directory = Path(output).resolve()
        # The optional internal root is used by isolated verification workspaces;
        # it is never accepted from command messages, HTTP or the public CLI.
        allowed_runtime = (cell.root / "runtime").resolve() if trace_root is None else Path(trace_root).resolve()
        require(self.directory.is_relative_to(allowed_runtime), "Command traces must be under runtime/")
        self.directory.mkdir(parents=True, exist_ok=False)
        self.session_id = uuid.uuid4().hex
        self.review_store = PlanReviewStore(self.session_id)
        self.review_identity = None
        self.policy_path = policy_path
        self.pending = self.active = None
        self.initialized = self.cancel_requested = self.blocked = self.stop_requested = False
        self.state_received_ns = self.clock_received_ns = None
        self.state_stamp = None
        self.replies = {}
        self.observations = []
        self.first_by_arm = set()
        self.motion_start = None
        super().__init__(cell, progress)
        self.configure_planning(planner_id=planner_id, trajectory_cache=trajectory_cache,
                                cache_start_tolerance_rad=cache_start_tolerance_rad)
        self.planning_source_sha256 = self._planning_source_sha256()
        self.evaluator = DistanceEvaluator(execution_target or cell, environment=True, self_collision=True)
        self.opened_self_identity = self._self_admission_identity()
        self.publisher = self.create_publisher(String, STATUS_TOPIC, status_qos())
        self.command_subscription = self.create_subscription(String, COMMAND_TOPIC, self.receive, 10)
        self.create_subscription(String, PLAN_TOPIC, self.receive_plan, 10)
        self.create_subscription(String, APPROVE_TOPIC, self.receive_approval, 10)
        self.create_subscription(String, CANCEL_TOPIC, self.cancel, 10)
        self.review_scene_client = self.create_client(GetPlanningScene, "/get_planning_scene")
        self.review_hardware_client = self.create_client(GetParameters, "/mujoco_ros2_control_node/get_parameters")
        self.review_moveit_client = self.create_client(GetParameters, "/move_group/get_parameters")
        self.configure_pose_planning(passthrough_exceptions=(CommandCancelled,
            PlanningBudgetExceeded, MotionStateChanged, MotionEnvironmentChanged))
        self.create_timer(1.0, self.heartbeat)
        self.dispatch_observer = self.action_event
        self.dispatch_abort = self._dispatch_self_abort
        implementation_root = Path(__file__).resolve().parents[3]
        code_root = implementation_root if (implementation_root / ".git").exists() else cell.root
        session = dict(schema_version=1, session_id=self.session_id,
            workcell=cell.id, model_manifest=self.metadata, workcell_contract=describe_workcell(cell, self.metadata), policy=self.policy,
            policy_name=policy_name, policy_sha256=file_sha256(policy_path),
            self_policy=self.self_policy, self_policy_sha256=self.self_policy_sha256,
            environment_policy=self.environment_policy,
            environment_policy_sha256=self.environment_policy_sha256,
            evaluator=self.evaluator.metadata(), motion_threshold_rad=0.001, host=host_info(),
            runtime=dict(python=platform.python_version(), os=platform.freedesktop_os_release().get("PRETTY_NAME", platform.platform()),
                         middleware=get_rmw_implementation_identifier()),
            code_commit=subprocess.check_output(["git", "-C", str(code_root), "rev-parse", "HEAD"], text=True).strip(),
            code_worktree_dirty=bool(subprocess.check_output(["git", "-C", str(code_root), "status", "--porcelain"], text=True).strip()),
            compute_budget_sec=self.compute_budget_sec, max_plan_candidates=self.max_plan_candidates,
            planning_configuration=deepcopy(self.planning_configuration),
            selection_policy="Bounded first valid RRT or two arm-order waypoint candidates; not shortest-path optimization",
            implementation_sha256={name: file_sha256(Path(__file__).with_name(name))
                for name in PLANNING_SOURCE_FILES},
            note="Warm persistent server; simulator startup excluded from request latency. Simulation only.")
        if execution_target is not None:
            session["execution_target"] = execution_target.description()
        atomic_json(self.directory / "session.json", session)

    def configure_planning(self, *, planner_id="RRTConnectkConfigDefault", trajectory_cache=False,
                           cache_start_tolerance_rad=0.0):
        """Internal experiment setup only; every change starts a cold cache."""
        if getattr(self, "active", None) is not None or getattr(self, "pending", None) is not None:
            raise RuntimeError("Planning configuration can change only while the owner is idle")
        configuration = planning_configuration(planner_id, trajectory_cache, cache_start_tolerance_rad)
        self.planner_id = planner_id
        self.planning_configuration = configuration
        self.trajectory_cache = (TrajectoryCache(self.joint_names,
            goal_tolerance_rad=self.goal_tolerance_rad,
            start_tolerance_rad=configuration["cache_start_tolerance_rad"])
            if trajectory_cache else None)

    @property
    def review_in_compute_budget(self):
        return getattr(self, "_review_in_compute_budget", False)

    @review_in_compute_budget.setter
    def review_in_compute_budget(self, enabled):
        if type(enabled) is not bool:
            raise ValueError("Review computation scope requires an explicit boolean")
        if getattr(self, "active", None) is not None or getattr(self, "pending", None) is not None:
            raise RuntimeError("Review computation scope can change only while the owner is idle")
        self._review_in_compute_budget = enabled

    def _planning_source_sha256(self):
        return canonical_sha256({name: file_sha256(Path(__file__).with_name(name))
                                 for name in PLANNING_SOURCE_FILES})

    def _planner_configuration_evidence(self):
        """Observe registered OMPL config; GetMotionPlan cannot report selected ID."""
        configuration = getattr(self, "planning_configuration", planning_configuration())
        planner = configuration["planner_id"]
        require(getattr(self, "planner_id", planner) == planner,
                "Planner configuration changed without the idle configuration gate")
        self.check_abort()
        self.until(self.review_moveit_client.service_is_ready, 5, "planner configuration service")
        names = ["ompl.planner_configs." + planner + suffix for suffix in (".type", ".range")]
        values = self.result(self.review_moveit_client.call_async(GetParameters.Request(names=names)),
                             5, "registered planner configuration").values
        require(len(values) == 2 and values[0].string_value == PLANNER_TYPES[planner]
                and values[1].type == 3 and values[1].double_value == 0.0,
                "Live MoveIt planner configuration differs from the explicit requested OMPL config")
        self.check_abort()
        return dict(requested_planner_id=planner, expected_type=PLANNER_TYPES[planner],
                    observed_type=values[0].string_value, observed_range=values[1].double_value,
                    evidence="live MoveIt registered parameters", selected_planner_id=None,
                    selection_evidence="GetMotionPlan has no selected planner field; inspect MoveIt logs separately")

    def _native_cache_provenance(self):
        opened = self.evaluator.metadata()
        files = {name: file_sha256(self.cell.root / name) for name in (
            "native/path_distance.cpp", "native/path_distance_environment.cpp", "native/path_distance_self.cpp", "runtime/native/path_distance.so")}
        if (opened.get("source_sha256") != files["native/path_distance.cpp"]
                or opened.get("library_sha256") != files["runtime/native/path_distance.so"]
                or opened.get("environment", {}).get("source_sha256") != files["native/path_distance_environment.cpp"]
                or opened.get("self_collision", {}).get("source_sha256") != files["native/path_distance_self.cpp"]
                or opened.get("engine") != "MuJoCo 3.12.0"):
            raise ReviewRejected("Native evaluator files/engine differ from the opened implementation; restart")
        return dict(opened=opened, files_sha256=files,
            scope="Opened evaluator metadata plus current helper binary/source SHA; transitive runtime libraries are not attested")

    def _self_admission_identity(self):
        require(file_sha256(self.self_policy_path) == self.self_policy_sha256,
                "Self-clearance policy changed; restart and replan")
        native = self._native_cache_provenance()
        value = dict(policy_sha256=self.self_policy_sha256, native=native)
        if getattr(self, "opened_self_identity", value) != value:
            raise SelfClearanceRejected("Native self-clearance implementation changed; restart and replan")
        return value

    def _cache_runtime_context(self, trace):
        started = time.monotonic()
        try:
            self.compute_checkpoint()
            current_source = self._planning_source_sha256()
            if current_source != self.planning_source_sha256:
                raise ReviewRejected("Planning implementation changed; restart before cache reuse")
            native_provenance = self._native_cache_provenance()
            planner_evidence = self._planner_configuration_evidence()
            runtime = self._review_runtime_identity()
            self.compute_checkpoint()
            configuration = self.planning_configuration
            identity = trajectory_cache_identity(runtime,
                source_sha256=canonical_sha256(dict(implementation=current_source, native=native_provenance)),
                policy=dict(timing=self.policy, environment=self.environment_policy, self_clearance=self.self_policy,
                    goal_tolerance_rad=self.goal_tolerance_rad, joint_limits=self.metadata["joint_limits"],
                    max_plan_candidates=self.max_plan_candidates,
                    cache_start_tolerance_rad=configuration["cache_start_tolerance_rad"]),
                planner=dict(configuration=configuration, runtime=planner_evidence))
            trace.data["planner_configuration_evidence"] = deepcopy(planner_evidence)
            trace.data["trajectory_cache"]["native_evaluator_provenance"] = deepcopy(native_provenance)
            return dict(identity=identity, runtime_identity=runtime)
        finally:
            record = trace.data["trajectory_cache"]
            record["source_validation_wall_sec"] += time.monotonic() - started

    def _recheck_cache_context(self, context, trace):
        current = self._cache_runtime_context(trace)
        if any(current[key] != context[key] for key in ("identity", "runtime_identity")):
            raise ReviewRejected("Cached planning source/model/environment/policy/planner identity changed; replan")
        self.confirm_start_unchanged(trace.data["initial_state_rad"])

    def _lookup_cached_path(self, initial, target, trace):
        context = self._cache_runtime_context(trace)
        record = trace.data["trajectory_cache"]
        record.update(identity=deepcopy(context["identity"]),
                      source_runtime_identity=deepcopy(context["runtime_identity"]))
        self.compute_checkpoint()
        lookup = self.trajectory_cache.lookup(context["identity"], initial, target)
        self.compute_checkpoint()
        record["lookup"] = {key: deepcopy(value) for key, value in lookup.items() if key != "trajectories"}
        record.update(hit=lookup["hit"], reason=lookup["reason"])
        context["lookup"] = lookup
        return ([Spline(**item) for item in lookup["trajectories"]] if lookup["hit"] else None), context

    def _validate_cached_anchor(self, curves, initial, target):
        if (len(curves) != 1 or curves[0].names != tuple(self.joint_names)
                or curves[0].points[0]["p"] != [initial[name] for name in self.joint_names]
                or max(abs(curves[0].points[-1]["p"][index] - target[name])
                       for index, name in enumerate(self.joint_names)) > self.goal_tolerance_rad + 1e-9):
            raise MotionPathRejected("Cached whole-workcell anchor/order/endpoint differs from the current request")
        samples = iter(sample_schedule(curves, initial, self.policy["sample_dt_sec"]))
        while True:
            self.compute_checkpoint()
            try:
                _, state = next(samples)
            except StopIteration:
                break
            except ValueError as error:
                # Only the pure generator is in this catch. ROS callback and
                # checkpoint errors above are never candidate cache misses.
                raise MotionPathRejected("Cached schedule cannot be sampled: " + str(error)) from error
            if any(not self.metadata["joint_limits"][name]["lower"] <= state[name]
                   <= self.metadata["joint_limits"][name]["upper"] for name in self.joint_names):
                raise MotionPathRejected("Cached schedule exceeds a configured joint position limit")

    def validate_motion_path(self, curves, initial, trace):
        """The same complete native and MoveIt gates for fresh and cached paths."""
        threshold = self.policy["min_inter_arm_distance_m"] + self.policy["tracking_reserve_m"]
        sampling_dt = min(self.policy["sample_dt_sec"], self.self_policy["sample_dt_sec"])
        planned = evaluate_path(curves, initial, self.cell, self.evaluator, dt=sampling_dt,
            reject_below=threshold, checkpoint=self.compute_checkpoint,
            environment_reject_below=self.environment_threshold_m,
            self_reject_below=self.self_policy["planning_min_self_distance_m"])
        trace.data["planned_clearance"] = planned
        trace.data["clearance_diagnostic"] = path_clearance_diagnostic(trace.data["clearance_endpoints"],
            planned, max(c.duration for c in curves))
        if not planned["complete"]:
            if planned.get("rejection_kind") == "self_clearance":
                raise SelfClearanceRejected("Planned native self-clearance below policy")
            if planned.get("rejection_kind") == "environment_clearance":
                raise EnvironmentClearanceRejected("Planned fixed-environment clearance below policy")
            raise MotionPathRejected("Planned inter-arm clearance below policy")
        samples = 0
        for _, state in sample_schedule(curves, initial, sampling_dt):
            self.compute_checkpoint()
            if not self.validity_of(state).valid:
                raise MotionPathRejected("MoveIt rejected interpolated schedule")
            samples += 1
        trace.data["moveit_safety_samples"] = samples

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
            record = dict(schema_version=1, session_id=self.session_id, request_id=None,
                              status="blocked" if self.blocked else "ready", trace_directory=str(self.directory),
                              review=self.review_store.summary())
            record["pose_tools"] = describe_pose_tools(self.cell)
            if getattr(self, "execution_target", None) is not None:
                record["execution_target"] = self.execution_target.description()
            self.publish(record)

    def path(self, trace):
        return self.directory / (trace.data["request_id"] + ".json")

    def persist(self, trace):
        if trace is self.active and self._defer_trace_persist:
            # Rewriting large candidate curves at every action event can consume
            # the shared start reservation. Keep small append-only event records
            # here; the complete in-memory trace is saved once at termination.
            journal = self.path(trace).with_suffix(".events.jsonl")
            self._trace_journal_cursor = trace.append_events(journal, self._trace_journal_cursor)
        else:
            trace.save(self.path(trace))
        reply = trace.summary(self.path(trace), self.session_id)
        if "execution_target" in trace.data:
            reply["execution_target"] = deepcopy(trace.data["execution_target"])
        if "mode" in trace.data:
            reply["mode"] = trace.data["mode"]
        if "pose_goal" in trace.data:
            try:
                reply["pose_goal"] = validate_pose_goal(trace.data["pose_goal"])
            except (ValueError, TypeError):
                # Keep the rejected raw request in its bounded trace, never
                # propagate arbitrary fields into every live status snapshot.
                pass
        if "pose_kinematics" in trace.data:
            reply["pose_kinematics"] = deepcopy(trace.data["pose_kinematics"])
        if "pose_completion" in trace.data:
            reply["pose_completion"] = deepcopy(trace.data["pose_completion"])
        if trace.data.get("mode") == "approved_plan":
            approval = trace.data.get("approval")
            try:
                validate_approval(approval)
            except (ValueError, TypeError):
                pass
            else:
                if approval["request_id"] == trace.data["request_id"] and approval["session_id"] == self.session_id:
                    reply.update(approval=deepcopy(approval), plan_request_id=approval["plan_request_id"],
                                 execution_request_id=trace.data["request_id"])
        if trace.data.get("mode") == "plan_only" and trace.data["status"] == "succeeded":
            reply["stage"] = "plan_not_required" if trace.data.get("no_motion_required") else "plan_ready"
        if "review" in trace.data:
            current = self.review_store.summary()
            reply["review"] = (current if current and current["plan_request_id"] == trace.data["review"]["plan_request_id"]
                               else deepcopy(trace.data["review"]))
        reply["operation"] = describe_operation(dict(trace.data, **reply))
        # Status polling carries the latest event. Full bounded timelines are
        # available from the requested trace; do not repeat them in forty rows.
        observed_events = reply["operation"]["events"]
        observed_events["items"] = observed_events["items"][-1:]
        observed_events["retained_count"] = len(observed_events["items"])
        observed_events["omitted_prefix_count"] = observed_events["source_count"] - observed_events["retained_count"]
        self.replies[trace.data["request_id"]] = reply
        self.publish(reply)

    def event(self, name, **details):
        if self.active:
            self.active.event(name, simulation_sec=self.clock_last, **details)
            self.persist(self.active)

    def _dispatch_self_abort(self):
        # ExecutionAdapter calls this after goal observers, immediately before
        # each send, and again while waiting. Only new submissions need a fresh
        # start/identity gate; moving trajectories must not be compared to start.
        self.check_abort()
        ledger = self.execution_ledger
        if ledger is not None and any(not item.submission_started for item in ledger.attempts):
            self.confirm_start_unchanged(self.active.data["initial_state_rad"])
            # Hashing/native geometry above can block without pumping ROS.
            # Keep the original clock receipt age; it must consume reservation
            # headroom rather than make a cached simulation stamp look current.
            now = self.fresh()
            clock_age_sec = (now-self.clock_received_ns)/1e9
            require(self.motion_start is not None and self.motion_start-self.clock_last-clock_age_sec > .1,
                    "Self admission exhausted the future execution reservation; no later goal is allowed")
        self.check_abort()

    def check_abort(self):
        if self.cancel_requested or self.stop_requested:
            raise CommandCancelled("Cancellation requested; no later command stage is allowed")
        require(not self.contact_guard or not self.contact_guard["latched"], "Contact/clock interlock latched")

    def replay_reply(self, identifier):
        reply = deepcopy(self.replies[identifier])
        current = self.review_store.summary()
        if (current is not None and isinstance(reply.get("review"), dict)
                and reply["review"]["plan_request_id"] == current["plan_request_id"]):
            reply["review"] = current
            self.replies[identifier] = reply
        self.publish(reply)

    def receive(self, message):
        self._receive_command(message, plan_only=False)

    def receive_plan(self, message):
        self._receive_command(message, plan_only=True)

    def _receive_command(self, message, *, plan_only):
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
            self.replay_reply(identifier)
            return
        trace = RequestTrace(identifier, received_ns=received)
        trace.data["command"] = data
        if plan_only:
            trace.data["mode"] = "plan_only"
        try:
            validate_command(data, self.metadata["joint_limits"])
            require(self.initialized, "Command server is not ready")
            require(not self.blocked, "Previous execution failure requires a server restart")
            require(self.pending is None and self.active is None, "Busy: another command owns both arms")
        except (ValueError, RuntimeError) as error:
            trace.finish("rejected", dict(type=type(error).__name__, message=str(error)))
        else:
            self.review_store.invalidate("A newer plan or motion request was accepted; replan")
            trace.data["status"] = "queued"
            self.pending = trace
        self.persist(trace)

    def receive_approval(self, message):
        """Claim the immutable plan before publishing an execution reservation.

        This callback does not spin; the single-threaded ROS executor owns both
        the busy gate and pending assignment. The store also locks consumption.
        Runtime scene/state checks are repeated in process_next before dispatch.
        """
        received, identifier = time.monotonic_ns(), uuid.uuid4().hex
        try:
            data = decode_message(message.data)
            identifier = request_id(data.get("request_id"))
        except (ValueError, TypeError) as error:
            trace = RequestTrace(identifier, received_ns=received)
            trace.finish("rejected", dict(type=type(error).__name__, message=str(error)))
            self.persist(trace)
            return
        if identifier in self.replies:
            self.replay_reply(identifier)
            return
        trace = RequestTrace(identifier, received_ns=received)
        trace.data.update(mode="approved_plan", approval=data)
        try:
            validate_approval(data)
            require(self.initialized and not self.blocked, "Command server is not ready for approval")
            require(self.pending is None and self.active is None, "Busy: another command owns both arms")
            self.check_abort()
            self.fresh()
            # Disk changes are refused here; dynamic planning-scene changes are
            # checked after this callback, under the newly reserved ownership.
            identity = self._review_build_identity()
            require(self.review_identity is not None and identity == self.review_identity["build"],
                    "Model, environment or policy changed; create a new plan")
            payload, review = self.review_store.claim(data, identity=self.review_identity,
                state_rad=self.state(), simulation_sec=self.clock_last)
            trace.approved_plan = payload
            if "goal_context" in payload:
                validate_goal_context(self.cell, payload["goal_context"])
                trace.data["goal_context"] = deepcopy(payload["goal_context"])
                trace.data["pose_goal"] = deepcopy(payload["goal_context"]["goal"])
            trace.data.update(status="queued", review=review,
                command=dict(payload["command"], request_id=identifier))
            self.pending = trace
        except (ValueError, RuntimeError) as error:
            trace.finish("rejected", dict(type=type(error).__name__, message=str(error)))
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
        elif (self.review_store.summary() or {}).get("plan_request_id") == identifier and self.review_store.summary()["state"] == "awaiting_approval":
            review = self.review_store.invalidate("Plan review was cancelled; create a new plan")
            if identifier in self.replies:
                reply = dict(self.replies[identifier], review=review, stage="plan_revoked")
                # The revoked review is a new status observation. Its old
                # projection described plan_ready and must not be replayed.
                reply["operation"] = describe_operation(reply)
                self.replies[identifier] = reply
                self.publish(reply)
        elif identifier in self.replies:
            self.replay_reply(identifier)
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
        require(self.state_received_ns is not None and 0 <= (now - self.state_received_ns) / 1e9 <= 0.3, "Joint state is stale")
        require(self.clock_received_ns is not None and 0 <= (now - self.clock_received_ns) / 1e9 <= 0.3, "Simulation clock is stale")
        require(not self.clock_backwards, "Simulation clock moved backwards")
        return now

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
        if self.execution_target is not None:
            self.execution_runtime_identity = deepcopy(self._recheck_execution_target())
        if self.planner_id != "RRTConnectkConfigDefault" or self.trajectory_cache is not None:
            self._planner_configuration_evidence()
        self.initialized = True
        self.heartbeat()

    def compute_checkpoint(self):
        self.check_abort()
        if self.compute_budget:
            self.compute_budget.check()
        # Native sampling otherwise starves this single-threaded node's cancel topic.
        now = time.monotonic_ns()
        if now - self._last_compute_spin_ns >= 20_000_000:
            self._last_compute_spin_ns = now
            rclpy.spin_once(self, timeout_sec=0.0)
            self.check_abort()
            if self.compute_budget:
                self.compute_budget.check()

    def finish_computation(self, trace, *, pump=True):
        if pump:
            self.compute_checkpoint()
        else:
            self.check_abort()
            self.compute_budget.check()
        trace.data["compute_budget"] = self.compute_budget.finish()
        self.event("computation_done")

    def plan(self, group, target, start_state=None, *, goal_tolerance_rad=None):
        self.compute_checkpoint()
        planner_id = getattr(self, "planner_id", "RRTConnectkConfigDefault")
        if planner_id != "RRTConnectkConfigDefault" and "planner_configuration_evidence" not in self.active.data:
            self.active.data["planner_configuration_evidence"] = self._planner_configuration_evidence()
            self.compute_checkpoint()
        self.event("planning_" + group)
        with self.active.span("planning:" + group):
            answer = super().plan(group, target, start_state, planner_id=planner_id,
                                  allowed_planning_sec=min(5.0, self.compute_budget.remaining()),
                                  goal_tolerance_rad=(self.goal_tolerance_rad if goal_tolerance_rad is None
                                                      else goal_tolerance_rad))
        self.active.data.setdefault("plans", []).append(deepcopy(self.planning_timings[-1]))
        self.compute_checkpoint()
        if answer.error_code.val != MoveItErrorCodes.SUCCESS:
            raise MotionPlanningFailed(answer.error_code.val)
        return self.anchored(answer.trajectory.joint_trajectory, initial=start_state)

    def serial_spatial_candidate(self, initial, target, order, record):
        virtual = dict(initial)
        segments = []
        record.update(proposal_kind="arm_order_waypoint", arm_order=[i.id for i in order],
                      segment_trajectories=[])
        for instance in order:
            self.compute_checkpoint()
            trajectory = self.plan(instance.group, {n: target[n] for n in instance.joints}, dict(virtual))
            segment = Spline.from_ros(trajectory)
            require(set(segment.names) == set(instance.joints), "Arm waypoint plan has unexpected joints")
            segments.append(segment)
            record["segment_trajectories"].append(segment.record())
            virtual.update(zip(segment.names, segment.points[-1]["p"]))
            if len(segments) == 1:
                record["waypoint_state_rad"] = dict(virtual)
            self.compute_checkpoint()
        try:
            combined = compose_serial_paths(segments, initial, self.joint_names)
        except ValueError as error:
            raise PlanCandidateRejected(str(error)) from error
        return to_ros(combined)

    def assess_spatial_candidate(self, curve, initial, *, diagnostics=None):
        result = {} if diagnostics is None else diagnostics
        result.update(complete=False, samples=0, invalid_samples=0, witness=None)
        for stamp, state in sample_schedule([curve], initial, self.policy["sample_dt_sec"]):
            self.compute_checkpoint()
            answer = self.validity_of(state)
            self.compute_checkpoint()
            result["samples"] += 1
            if not answer.valid:
                result["invalid_samples"] += 1
                result["witness"] = dict(time_from_start_sec=stamp, state_rad=state,
                    contacts=[dict(body1=c.contact_body_1, body2=c.contact_body_2) for c in answer.contacts])
                return result
        result["complete"] = True
        return result

    def coordinated_path(self, initial, target, trace):
        selection = trace.data["plan_selection"] = dict(
            group="both_arms", start_rad=dict(initial), target_rad=dict(target),
            strategy="bounded_first_valid", goal_tolerance_rad=self.goal_tolerance_rad,
            threshold_m=self.policy["min_inter_arm_distance_m"] + self.policy["tracking_reserve_m"])
        self.event("spatial_candidate_search")

        def propose():
            try:
                candidate = selection["candidates"][-1]
                if candidate["attempt"] in (5, 6):
                    order = list(self.cell.instances)
                    if candidate["attempt"] == 6:
                        order.reverse()
                    trajectory = self.serial_spatial_candidate(initial, target, order, candidate)
                else:
                    candidate["proposal_kind"] = "coordinated_rrt"
                    trajectory = self.plan("both_arms", target, initial)
            except MotionPlanningFailed as error:
                return error.error_code, None
            # Replay adapters declare their original tolerance in the per-plan record.
            # A fresh MoveIt request always records the current requested tolerance.
            plans = trace.data.get("plans", [])
            effective_tolerance = plans[-1].get("goal_tolerance_rad", self.goal_tolerance_rad) if plans else self.goal_tolerance_rad
            selection.setdefault("proposal_goal_tolerances_rad", []).append(effective_tolerance)
            return MoveItErrorCodes.SUCCESS, trajectory

        def validate_curve(curve):
            require(set(curve.names) == set(initial), "Planner response must cover the whole workcell")
            endpoint = dict(zip(curve.names, curve.points[-1]["p"]))
            effective_tolerance = selection["proposal_goal_tolerances_rad"][-1]
            require(max(abs(endpoint[n] - target[n]) for n in target) <= effective_tolerance + 1e-9,
                    "Planner endpoint differs from its declared goal tolerance")

        def save(candidates):
            selection["candidates"] = candidates
            update_candidate_path_diagnostic(trace.data, candidates)
            self.persist(trace)

        def moveit_check(curve):
            partial = selection["candidates"][-1]["moveit"] = {}
            return self.assess_spatial_candidate(curve, initial, diagnostics=partial)

        with trace.span("spatial_candidate_search"):
            try:
                trajectory = select_clearance_plan(propose, initial, self.cell, self.evaluator, self.policy,
                    moveit_check, save, max_candidates=self.max_plan_candidates,
                    checkpoint=self.compute_checkpoint, diagnostics=selection,
                    validate_curve=validate_curve, environment_policy=self.environment_policy, self_policy=self.self_policy)
            except (CommandCancelled, PlanningBudgetExceeded):
                # A production checkpoint may also interrupt inside a ROS callback
                # supplied to the engine-independent selector.
                selection["status"] = "interrupted"
                if selection.get("candidates"):
                    selection["candidates"][-1]["status"] = "interrupted"
                raise
        return Spline.from_ros(trajectory)

    def confirm_start_unchanged(self, initial):
        self.fresh()
        self._self_admission_identity()
        require_self_clearance(self.evaluator, self.state(), self.self_policy["observed_start_min_self_distance_m"])
        if max(abs(self.state()[n] - initial[n]) for n in self.joint_names) >= 0.002:
            raise MotionStateChanged("Robot moved while planning; stale plan refused")
        self.check_abort()
        self.fresh()

    def _review_build_identity(self):
        """Rehash the current generated model, environment, limits and policy."""
        try:
            metadata = check_workcell(self.cell)
            require(metadata == self.metadata, "Validated workcell changed; restart and replan")
            require(file_sha256(self.environment_policy_path) == self.environment_policy_sha256,
                    "Environment clearance policy changed; restart and replan")
            identity = dict(self_clearance=self._self_admission_identity(), workcell_sha256=self.cell.fingerprint,
                        outputs_sha256=metadata["outputs"],
                        policy_sha256=file_sha256(self.policy_path),
                        environment_policy_sha256=self.environment_policy_sha256)
            target = getattr(self, "execution_target", None)
            if target is not None:
                try:
                    identity["execution_target"] = target.validate()
                except Exception as error:
                    self.blocked = True
                    self.review_store.invalidate("Execution environment changed; restart and replan")
                    raise MotionEnvironmentChanged("Execution environment changed; restart and replan") from error
            return identity
        except OSError as error:
            raise ReviewRejected("Model or policy files changed; replan") from error

    @staticmethod
    def _review_xml_sha(text):
        # Ignore XML formatting, while retaining every element and attribute.
        return hashlib.sha256(ET.canonicalize(text, strip_text=True).encode()).hexdigest()

    def _review_robot_sha(self, text):
        robot = ET.fromstring(text)
        # Launch changes this rendering-only option for native GUI/headless.
        # Keep every model, control and collision field in the identity.
        headless = robot.find("ros2_control/hardware/param[@name='headless']")
        require(headless is not None and headless.text in ("true", "false"),
                "Invalid simulation headless option")
        headless.text = "false"
        return self._review_xml_sha(ET.tostring(robot, encoding="unicode"))

    def _review_runtime_identity(self):
        """Bind review to actual hardware/MoveIt geometry, including scene edits.

        Reviews and explicit environment targets need these additive queries.
        Base-model automatic commands retain their existing execution path and
        strict four-field command contract.
        Header stamps are excluded from scene geometry identity; transforms and
        object poses themselves remain part of the hash.
        """
        self.check_abort()
        require(len(self.get_publishers_info_by_topic("/clock")) == 1
                and len(self.get_publishers_info_by_topic("/joint_states")) == 1
                and self.count_publishers(STATUS_TOPIC) == 1,
                "Review requires one simulation state, clock and command owner")
        build = self._review_build_identity()
        for client in (self.review_hardware_client, self.review_moveit_client, self.review_scene_client):
            self.until(client.service_is_ready, 5, "plan review identity service")
        hardware = self.result(self.review_hardware_client.call_async(GetParameters.Request(
            names=["robot_description", "use_sim_time"])), 5, "review simulation identity").values
        require(len(hardware) == 2 and hardware[1].bool_value, "Review requires simulation time")
        output = self.cell.output if getattr(self, "execution_target", None) is None else self.execution_target.output
        root = ET.fromstring(hardware[0].string_value)
        model = root.find("ros2_control/hardware/param[@name='mujoco_model']")
        require(model is not None and model.text is not None
                and Path(model.text).resolve() == (output / "cell.xml").resolve(),
                "Live simulator uses a different model/environment; replan")
        expected_robot = (output / "cell.urdf").read_text()
        expected_semantics = (output / "cell.srdf").read_text()
        require(self._review_robot_sha(hardware[0].string_value) == self._review_robot_sha(expected_robot),
                "Live simulator robot description differs from the checked model")
        moveit = self.result(self.review_moveit_client.call_async(GetParameters.Request(
            names=["robot_description", "robot_description_semantic"])), 5, "review MoveIt identity").values
        require(len(moveit) == 2
                and self._review_robot_sha(moveit[0].string_value) == self._review_robot_sha(expected_robot)
                and self._review_xml_sha(moveit[1].string_value) == self._review_xml_sha(expected_semantics),
                "MoveIt model/environment differs from the checked model")
        components = (PlanningSceneComponents.WORLD_OBJECT_GEOMETRY
            | PlanningSceneComponents.ROBOT_STATE_ATTACHED_OBJECTS
            | PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
            | PlanningSceneComponents.LINK_PADDING_AND_SCALING
            | PlanningSceneComponents.TRANSFORMS)
        request = GetPlanningScene.Request()
        request.components.components = components
        scene = self.result(self.review_scene_client.call_async(request), 5, "review planning scene").scene
        # Deliberately omit current joint positions: they have their own strict
        # initial-state gate and naturally change while ROS messages arrive.
        geometry = dict(world=message_to_ordereddict(scene.world),
            attached=[message_to_ordereddict(v) for v in scene.robot_state.attached_collision_objects],
            allowed_collision_matrix=message_to_ordereddict(scene.allowed_collision_matrix),
            link_padding=[message_to_ordereddict(v) for v in scene.link_padding],
            link_scale=[message_to_ordereddict(v) for v in scene.link_scale],
            fixed_frame_transforms=[message_to_ordereddict(v) for v in scene.fixed_frame_transforms])
        def without_stamps(value):
            if isinstance(value, dict):
                return {k: (dict(v, stamp={"sec": 0, "nanosec": 0}) if k == "header" and isinstance(v, dict)
                            else without_stamps(v)) for k, v in value.items()}
            return [without_stamps(v) for v in value] if isinstance(value, list) else value
        encoded = json.dumps(without_stamps(geometry), sort_keys=True, allow_nan=False,
                             separators=(",", ":")).encode()
        require(len(encoded) <= 8*1024*1024, "Planning-scene review identity exceeds its 8MiB bound")
        self.check_abort()
        self.fresh()
        return dict(build=build, planning_scene_sha256=hashlib.sha256(encoded).hexdigest(),
                    simulator_description_sha256=self._review_robot_sha(hardware[0].string_value))

    def _recheck_execution_target(self, expected=None):
        """Opt-in target identity applies to automatic and reviewed execution."""
        try:
            current = self._review_runtime_identity()
            if expected is None:
                expected = getattr(self, "execution_runtime_identity", None)
            if expected is not None:
                require(current == expected, "Execution environment/runtime scene changed while planning")
            # Runtime queries spin callbacks. Rehash after the final query too.
            self.execution_target.validate()
            self.fresh()
            return current
        except (CommandCancelled, KeyboardInterrupt):
            raise
        except Exception as error:
            self.blocked = True
            self.review_store.invalidate("Execution environment changed; restart and replan")
            raise MotionEnvironmentChanged("Execution environment identity failed; restart and replan") from error

    def _issue_review(self, trace, initial, identity, *, finish_budget=False):
        current = self._review_runtime_identity()
        require(current == identity, "Model/environment changed while planning; create a new plan")
        self.confirm_start_unchanged(initial)
        if finish_budget:
            self.finish_computation(trace, pump=False)
        self.review_identity = current
        trace.data["review"] = self.review_store.issue(trace.data["request_id"], identity=current,
            initial_state_rad=initial, simulation_sec=self.clock_last,
            command=trace.data["command"], trajectories=trace.data.get("trajectories", []),
            no_motion_required=trace.data.get("no_motion_required", False),
            goal_context=trace.data.get("goal_context"))
        trace.data["review_payload"] = self.review_store.payload_for(
            trace.data["request_id"], trace.data["review"]["plan_sha256"])
        trace.event("plan_ready", simulation_sec=self.clock_last)
        trace.finish("succeeded")

    def _recheck_approved(self, payload, *, live):
        if "goal_context" in payload:
            validate_goal_context(self.cell, payload["goal_context"])
        identity = self._review_runtime_identity() if live else self.review_identity
        require(identity is not None, "Plan identity is no longer available; replan")
        self.review_store.recheck_claim(payload, identity=identity,
            state_rad=self.state(), simulation_sec=self.clock_last)

    def action_event(self, name, arm, **details):
        self.event(name, arm=arm, **details)

    def process_next(self):
        if self.pending is None:
            return
        trace, self.pending = self.pending, None
        self.active = trace
        self._defer_trace_persist = False
        self._trace_journal_cursor = 0
        self.handles = []
        self.execution_ledger = self.execution_adapter = self.execution_transport = None
        self.motion_start = None
        self.observations, self.first_by_arm = [], set()
        trace.data["status"] = "running"
        before_dispatch = self.dispatched
        approved_plan = getattr(trace, "approved_plan", None)
        plan_only = trace.data.get("mode") == "plan_only"
        try:
            cache_context = None
            cache_hit = False
            trace.data.setdefault("plans", [])
            configuration = getattr(self, "planning_configuration", planning_configuration())
            budget_includes_review = self.review_in_compute_budget or configuration["trajectory_cache"]
            trace.data["planning_configuration"] = dict(configuration,
                requested_planner_id=configuration["planner_id"],
                trajectory_cache_enabled=configuration["trajectory_cache"],
                max_plan_candidates=getattr(self, "max_plan_candidates", MAX_PLAN_CANDIDATES),
                compute_budget_wall_sec=self.compute_budget_sec,
                goal_tolerance_rad=getattr(self, "goal_tolerance_rad", .003),
                budget_includes_review=budget_includes_review,
                compute_scope=("computation_started_through_final_review_identity" if budget_includes_review
                               else "computation_started_before_final_review_identity"))
            cache_record = trace.data["trajectory_cache"] = dict(
                enabled=configuration["trajectory_cache"], eligible=False, hit=False,
                reason="disabled" if not configuration["trajectory_cache"] else "ineligible",
                lookup=None, source_validation_wall_sec=0., full_revalidation_wall_sec=0.,
                fallback_planning_wall_sec=0., store=None)
            execution_identity = None
            if getattr(self, "execution_target", None) is not None:
                trace.data["execution_target"] = self.execution_target.description()
            self.event("state_snapshot")
            with trace.span("state_snapshot"):
                self.quiet()
                initial = self.state()
                self.commanded = dict(initial)
                trace.data.update(initial_state_rad=initial, joint_state_stamp_simulation_sec=self.state_stamp,
                    state_receive_age_wall_sec=(time.monotonic_ns() - self.state_received_ns) / 1e9,
                    clock_receive_age_wall_sec=(time.monotonic_ns() - self.clock_received_ns) / 1e9)
            if getattr(self, "execution_target", None) is not None:
                execution_identity = self._recheck_execution_target()
                trace.data["execution_target_identity"] = deepcopy(execution_identity)
            review_identity = execution_identity if plan_only and execution_identity is not None else (
                self._review_runtime_identity() if plan_only else None)
            if approved_plan is not None:
                self.event("approved_plan_recheck")
                self._recheck_approved(approved_plan, live=True)
                # Preserve the exact reviewed anchor. The live state has passed
                # the same <0.002rad gate and is still checked before dispatch.
                trace.data["approval_observed_state_rad"] = dict(initial)
                initial = deepcopy(approved_plan["initial_state_rad"])
                trace.data["initial_state_rad"] = initial
            self.compute_budget = PlanningBudget(self.compute_budget_sec)
            trace.data["self_policy"] = deepcopy(self.self_policy)
            trace.data["self_clearance_identity"] = self._self_admission_identity()
            trace.data["environment_policy"] = deepcopy(self.environment_policy)
            trace.data["environment_policy_sha256"] = self.environment_policy_sha256
            require(file_sha256(self.environment_policy_path) == self.environment_policy_sha256,
                    "Environment clearance policy changed; restart and replan")
            self.event("computation_started")
            if "pose_goal" in trace.data and approved_plan is None:
                self.resolve_pending_pose(trace, initial)
            target = trace.data["command"]["target_rad"]
            strategy = trace.data["command"]["strategy"]
            if cache_record["enabled"]:
                if approved_plan is not None:
                    cache_record["reason"] = "approved_execution_excluded"
                elif "goal_context" in trace.data:
                    cache_record["reason"] = "pose_excluded"
                elif strategy != "coordinated":
                    cache_record["reason"] = "departure_excluded"
            threshold = self.policy["min_inter_arm_distance_m"] + self.policy["tracking_reserve_m"]
            self.event("endpoint_clearance")
            with trace.span("endpoint_clearance"):
                trace.data["clearance_endpoints"] = measure_endpoints(initial, target, self.evaluator, threshold,
                    checkpoint=self.compute_checkpoint, environment_threshold_m=self.environment_threshold_m,
                    self_threshold_m=self.self_policy["planning_min_self_distance_m"])
            at_goal = (trace.data["pose_kinematics"]["already_at_goal"] if "goal_context" in trace.data
                       and approved_plan is None else max(abs(target[n] - initial[n]) for n in self.joint_names) <= 0.001)
            if ((approved_plan is None and at_goal)
                    or (approved_plan is not None and approved_plan["no_motion_required"])):
                trace.data["no_motion_required"] = True
                if cache_record["enabled"]:
                    cache_record["reason"] = "no_motion_excluded"
                trace.skip("planning", "Observed TCP already within the Cartesian no-op condition"
                           if "goal_context" in trace.data else "Target already within the 0.001rad movement threshold")
                trace.skip("departure_search", "No motion required")
                with trace.span("safety_checks"):
                    require(self.validity_of(initial).valid, "Current state is colliding")
                    require(self.evaluator.at(initial)["distance_m"] >= self.policy["min_inter_arm_distance_m"] + self.policy["tracking_reserve_m"], "Current clearance below planning policy")
                    require(not trace.data["clearance_endpoints"]["endpoints"]["start"]["below_policy"],
                            "Current state below inter-arm or environment planning policy")
                self.finish_computation(trace)
                with trace.span("final_state_recheck"):
                    self.confirm_start_unchanged(initial)
                    if execution_identity is not None:
                        self._recheck_execution_target(execution_identity)
                        self.confirm_start_unchanged(initial)
                if "goal_context" in trace.data:
                    self.confirm_pose_completion(trace)
                if plan_only:
                    # A no-op has no path to inspect or approve. Preserve its
                    # successful zero-dispatch result without issuing a token.
                    trace.event("plan_not_required", simulation_sec=self.clock_last)
                    trace.finish("succeeded")
                    return
                if approved_plan is not None:
                    self._recheck_approved(approved_plan, live=True)
                    self.confirm_start_unchanged(initial)
                self.event("settled")
                trace.finish("succeeded")
                return
            self.event("endpoint_validity")
            trace.data["endpoint_moveit_validity"] = {}
            with trace.span("endpoint_validity"):
                for role, state in (("start", initial), ("goal", target)):
                    self.compute_checkpoint()
                    if trace.data["clearance_endpoints"]["endpoints"][role]["below_policy"]:
                        raise InvalidMotionEndpoint(role, "clearance below planning policy")
                    valid = self.validity_of(state).valid
                    trace.data["endpoint_moveit_validity"][role] = valid
                    self.compute_checkpoint()
                    if not valid:
                        raise InvalidMotionEndpoint(role, "MoveIt state validity rejected")
            if approved_plan is not None:
                curves = [Spline(**record) for record in approved_plan["trajectories"]]
                require(curves, "Reviewed motion has no trajectories")
                trace.skip("planning", "Execute the exact reviewed trajectories; do not replan after approval")
                trace.skip("departure_search", "Reviewed departure schedule is immutable")
            elif "goal_context" in trace.data:
                curves = [self.pose_path(initial, target, trace)]
                trace.skip("departure_search", "Selected arm plan with all other joints held")
            elif strategy == "coordinated":
                if getattr(self, "trajectory_cache", None) is not None:
                    cache_record["eligible"] = True
                    curves, cache_context = self._lookup_cached_path(initial, target, trace)
                    cache_hit = curves is not None
                else:
                    curves = None
                if curves is None:
                    curves = [self.coordinated_path(initial, target, trace)]
                trace.skip("departure_search", "Coordinated joint-space plan; no departure-only search")
            else:
                independent = [(i, self.plan(i.group, {n: target[n] for n in i.joints}, initial)) for i in self.cell.instances]
                duration = max(Spline.from_ros(t).duration for _, t in independent)
                source = [Spline.from_ros(stretch(t, duration)) for _, t in independent]
                trace.data["source_trajectories"] = [c.record() for c in source]
                self.event("departure_search")
                trace.data["departure_search"] = {}
                with trace.span("departure_search"):
                    chosen = choose_delay(source, initial, self.cell, self.evaluator, self.policy,
                        diagnostics=trace.data["departure_search"], checkpoint=self.compute_checkpoint, self_policy=self.self_policy)
                trace.data["departure_search"] = {k: v for k, v in chosen.items() if k != "curves"}
                curves = [Spline(**record) for record in chosen["curves"]]
            trace.data["planned_trajectories"] = [c.record() for c in curves]
            self.check_abort()
            self.event("safety_checks")
            with trace.span("safety_checks"):
                validation_started = time.monotonic()
                try:
                    if cache_hit:
                        self._validate_cached_anchor(curves, initial, target)
                        cache_record["dynamics"] = validate_reanchored_dynamics(
                            [curve.record() for curve in curves], self.metadata["joint_limits"],
                            max_acceleration=1.0)
                    self.validate_motion_path(curves, initial, trace)
                except (MotionPathRejected, EnvironmentClearanceRejected, SelfClearanceRejected, CacheDynamicsRejected) as error:
                    if not cache_hit:
                        raise
                    cache_record["full_revalidation_wall_sec"] = time.monotonic() - validation_started
                    cache_record["rejected_candidate"] = dict(type=type(error).__name__, message=str(error),
                        planned_clearance=deepcopy(trace.data.get("planned_clearance")),
                        dynamics=deepcopy(getattr(error, "diagnostic", None)))
                    # Identity, cancellation, budget and transport failures here
                    # escape. They cannot evict a path or authorize a retry.
                    self._recheck_cache_context(cache_context, trace)
                    cache_record["eviction"] = self.trajectory_cache.evict(
                        cache_context["lookup"]["key_sha256"], "validation_rejected")
                    cache_record.update(reason="validation_rejected_fallback", fallback=True)
                    fallback_started = time.monotonic()
                    try:
                        self.compute_checkpoint()
                        curves = [self.coordinated_path(initial, target, trace)]
                    finally:
                        cache_record["fallback_planning_wall_sec"] = time.monotonic() - fallback_started
                    trace.data["planned_trajectories"] = [curve.record() for curve in curves]
                    self.validate_motion_path(curves, initial, trace)
                    cache_hit = False
                else:
                    if cache_hit:
                        cache_record["full_revalidation_wall_sec"] = time.monotonic() - validation_started
                finally:
                    if (cache_context is not None and cache_context["lookup"]["hit"]
                            and cache_record["full_revalidation_wall_sec"] == 0):
                        cache_record["full_revalidation_wall_sec"] = time.monotonic() - validation_started
            trace.data["trajectories"] = [c.record() for c in curves]
            # Persist the full validated path before refreshing state and choosing
            # an execution start time. Later execution updates remain compact.
            self.event("state_recheck")
            with trace.span("state_recheck"):
                self.quiet()
                self.confirm_start_unchanged(initial)
            if cache_context is not None:
                self._recheck_cache_context(cache_context, trace)
                self.compute_checkpoint()
                if not cache_hit:
                    cache_record["store"] = self.trajectory_cache.store(cache_context["identity"],
                        initial, target, [curve.record() for curve in curves])
            if plan_only:
                if not budget_includes_review:
                    self.finish_computation(trace)
                self._issue_review(trace, initial, review_identity, finish_budget=budget_includes_review)
                return
            if approved_plan is not None:
                # The scene query may spin callbacks. All state, cancellation
                # and budget gates below still run before the first goal send.
                self._recheck_approved(approved_plan, live=True)
            self._defer_trace_persist = True
            if cache_context is None:
                self.finish_computation(trace)
            # finish_computation may pump a newer state/cancel callback. Check that
            # last received state again, with no intervening spin before dispatch.
            with trace.span("final_state_recheck"):
                if cache_context is not None:
                    self._recheck_cache_context(cache_context, trace)
                if execution_identity is not None:
                    self._recheck_execution_target(execution_identity)
                self.confirm_start_unchanged(initial)
                require(file_sha256(self.environment_policy_path) == self.environment_policy_sha256,
                        "Environment clearance policy changed; restart and replan")
                if approved_plan is not None:
                    self._recheck_approved(approved_plan, live=False)
            if cache_context is not None:
                self.finish_computation(trace, pump=False)
                self.confirm_start_unchanged(initial)
            self.motion_start = self.clock_last + 1.0
            trace.data["scheduled_start_simulation_sec"] = self.motion_start
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
            require(len(self.observations) >= 2, "Executed motion lacks observed joint-state coverage")
            if len(self.observations) >= 2:
                with trace.span("observed_clearance"):
                    observed = evaluate_states(((s["simulation_sec"], s["state_rad"]) for s in self.observations), self.cell, self.evaluator)
                    trace.data["observed_clearance"] = observed
                    require(observed["minimum"]["distance_m"] >= self.policy["min_inter_arm_distance_m"], "Observed clearance below 20mm policy")
                    if observed["self_minimum"]["distance_m"] < self.self_policy["observed_start_min_self_distance_m"]:
                        raise SelfClearanceRejected("Observed native self-clearance below policy")
                    if observed["environment_minimum"]["distance_m"] < self.environment_policy["min_environment_distance_m"]:
                        raise EnvironmentClearanceRejected("Observed fixed-environment clearance below policy")
            if "goal_context" in trace.data:
                self.confirm_pose_completion(trace)
            trace.finish("succeeded")
        except BaseException as error:
            ledger = self.execution_ledger
            execution_uncertain = ledger is not None and ledger.cleanup_uncertain
            if isinstance(error, (CommandCancelled, KeyboardInterrupt)):
                trace.data["failure_kind"] = "cancelled"
            elif isinstance(error, PlanningBudgetExceeded):
                trace.data["failure_kind"] = "budget_exhausted"
            elif isinstance(error, PlanCandidatesExhausted):
                trace.data["failure_kind"] = "candidate_exhausted"
            elif isinstance(error, TimingRejected):
                trace.data["failure_kind"] = "departure_exhausted"
            elif isinstance(error, InvalidMotionEndpoint):
                trace.data["failure_kind"] = "invalid_" + error.role
            elif isinstance(error, SelfClearanceRejected):
                trace.data["failure_kind"] = "self_clearance"
            elif isinstance(error, EnvironmentClearanceRejected):
                trace.data["failure_kind"] = "environment_clearance"
            elif isinstance(error, MotionPlanningFailed):
                trace.data["failure_kind"] = "planner_failed"
            elif isinstance(error, PoseAdapterError):
                trace.data["failure_kind"] = error.kind
            elif isinstance(error, MotionStateChanged):
                trace.data["failure_kind"] = "state_changed"
            elif isinstance(error, MotionEnvironmentChanged):
                trace.data["failure_kind"] = "execution_environment_changed"
            elif isinstance(error, ReviewRejected):
                trace.data["failure_kind"] = "review_invalidated"
            else:
                trace.data["failure_kind"] = "execution_failed" if self.dispatched > before_dispatch or execution_uncertain else "planning_error"
            if self.dispatched > before_dispatch or execution_uncertain:
                # A transport exception can leave submission unknown even when
                # no acceptance future was returned and the send counter is 0.
                self.blocked = self.blocked or execution_uncertain
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
            ledger = self.execution_ledger
            if ledger is not None:
                transport = ledger.snapshot()
                trace.data["execution_transport"] = transport
                # Read the bounded terminal observations already validated by
                # the adapter. Re-reading an exceptional ROS future here could
                # erase the primary failure and prevent trace persistence.
                trace.data["action_results"] = [dict(status=item["terminal_status"],
                    error_code=item["terminal_error_code"])
                    for item in transport["attempts"] if item["terminal_seen"]]
            else:
                trace.data["action_results"] = []
            trace.data["observations"] = self.observations
            trace.data["contact_guard"] = deepcopy(self.contact_guard)
            self._defer_trace_persist = False
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
    parser.add_argument("--execution-environment", type=Path)
    parser.add_argument("--execution-manifest-sha256")
    parser.add_argument("--policy", default="dual_arm_timing", help="Named policy under config/planning for the selected workcell")
    parser.add_argument("--output", type=Path)
    parser.add_argument("--compute-budget", type=float, default=30.0, help="Cooperative planning/validation wall seconds, excludes execution")
    parser.add_argument("--max-plan-candidates", type=int, default=MAX_PLAN_CANDIDATES,
        help="Bounded first-valid spatial plans per coordinated request, 1..12; one shared computation budget")
    parser.add_argument("--planner-id", choices=tuple(PLANNER_TYPES), default="RRTConnectkConfigDefault",
        help="Explicit registered OMPL planner; selected planner still requires separate MoveIt log evidence")
    parser.add_argument("--trajectory-cache", action="store_true",
        help="Opt-in bounded coordinated joint-path candidate reuse; all current validators remain mandatory")
    parser.add_argument("--cache-start-tolerance", type=float, default=0.0,
        help="Internal cache experiment only: exact start by default, at most 0.001rad with full revalidation")
    args = parser.parse_args()
    cell = load_workcell(args.workcell)
    execution_target = load_execution_target(cell, args.execution_environment, args.execution_manifest_sha256)
    output = args.output or ROOT / "runtime/commands" / uuid.uuid4().hex
    # Keep ROS alive while cancelling outstanding goals on SIGINT/SIGTERM.
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    node = None
    previous_handlers = {}
    try:
        node = MotionCommandServer(cell, output, compute_budget_sec=args.compute_budget, max_plan_candidates=args.max_plan_candidates, policy_name=args.policy, execution_target=execution_target,
            planner_id=args.planner_id, trajectory_cache=args.trajectory_cache,
            cache_start_tolerance_rad=args.cache_start_tolerance)
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
