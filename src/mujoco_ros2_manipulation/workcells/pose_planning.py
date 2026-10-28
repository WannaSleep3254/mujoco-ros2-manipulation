"""Additive plan-only TCP goals; all motion uses the existing reviewed path.

The initial registry contains flange origins only. This module does not attach
a gripper or relax collision, environment, clock, ownership or approval gates.
"""
from copy import deepcopy
import time
import uuid

from .interference import Spline
from .motion_request import POSE_PLAN_TOPIC, decode_message, request_id
from .pose_adapter import MoveItPoseAdapter, PoseAdapterError
from .pose_goal import goal_tip_pose, tcp_pose_error
from .pose_tools import configured_pose_tools, resolve_pose_tool, validate_goal_context
from .request_trace import RequestTrace


IK_POSITION_TOLERANCE_M = 0.0001
IK_ORIENTATION_TOLERANCE_RAD = 0.001
POSE_PLANNING_JOINT_TOLERANCE_RAD = 0.0003
PLANNED_POSITION_TOLERANCE_M = 0.001
PLANNED_ORIENTATION_TOLERANCE_RAD = 0.003
EXECUTION_POSITION_TOLERANCE_M = 0.005
EXECUTION_ORIENTATION_TOLERANCE_RAD = 0.03


def hold_other_joints(trajectory, selected_names, initial, joint_names):
    """Copy the selected-arm spline and hold every other joint throughout it."""
    curve = Spline.from_ros(trajectory)
    names, selected = tuple(joint_names), tuple(selected_names)
    if set(curve.names) != set(selected) or set(initial) != set(names):
        raise ValueError("Pose path does not cover exactly the selected arm")
    local = {name: index for index, name in enumerate(curve.names)}
    points = []
    for point in curve.points:
        row = dict(t=point["t"], p=[], v=[], a=[])
        for name in names:
            if name in local:
                index = local[name]
                row["p"].append(point["p"][index])
                row["v"].append(point["v"][index] if point["v"] else 0.)
                row["a"].append(point["a"][index] if point["a"] else 0.)
            else:
                row["p"].append(initial[name])
                row["v"].append(0.)
                row["a"].append(0.)
        points.append(row)
    return Spline(names, points)


class PosePlanningMixin:
    def configure_pose_planning(self, *, passthrough_exceptions=()):
        from moveit_msgs.srv import GetPositionIK, GetPositionFK
        from std_msgs.msg import String
        self.pose_tools = configured_pose_tools(self.cell)
        self.pose_ik_client = self.create_client(GetPositionIK, "/compute_ik")
        self.pose_fk_client = self.create_client(GetPositionFK, "/compute_fk")
        self.pose_adapter = MoveItPoseAdapter(self.pose_ik_client, self.pose_fk_client,
            self.pose_wait_result, GetPositionIK.Request, GetPositionFK.Request,
            self.joint_names, self.metadata["joint_limits"],
            passthrough_exceptions=passthrough_exceptions)
        self.create_subscription(String, POSE_PLAN_TOPIC, self.receive_pose_plan, 10)

    def pose_wait_result(self, future, timeout_sec, description):
        def ready():
            self.check_abort()
            if self.compute_budget is not None:
                self.compute_budget.check()
            return future.done()
        self.until(ready, timeout_sec, description)
        result = future.result()
        if result is None:
            raise RuntimeError("No result: " + description)
        return result

    def receive_pose_plan(self, message):
        received, identifier = time.monotonic_ns(), uuid.uuid4().hex
        try:
            raw = decode_message(message.data)
            identifier = request_id(raw.get("request_id"))
        except (ValueError, TypeError) as error:
            trace = RequestTrace(identifier, received_ns=received)
            trace.finish("rejected", dict(type=type(error).__name__, message=str(error)))
            self.persist(trace)
            return
        if identifier in self.replies:
            self.replay_reply(identifier)
            return
        trace = RequestTrace(identifier, received_ns=received)
        trace.data.update(mode="plan_only", pose_goal=deepcopy(raw))
        try:
            goal, tool = resolve_pose_tool(self.cell, raw)
            if not self.initialized or self.blocked:
                raise ValueError("Command server is not ready for Pose planning")
            if self.pending is not None or self.active is not None:
                raise ValueError("Busy: another command owns both arms")
            trace.data["pose_goal"] = goal
            trace.data["goal_context"] = dict(kind="tcp_pose", goal=goal,
                tool=dict(tool.record(), sha256=tool.sha256))
        except (ValueError, RuntimeError) as error:
            trace.finish("rejected", dict(type=type(error).__name__, message=str(error)))
        else:
            self.review_store.invalidate("A newer Pose plan was accepted; replan")
            trace.data["status"] = "queued"
            self.pending = trace
        self.persist(trace)

    def resolve_pending_pose(self, trace, initial):
        goal, tool = validate_goal_context(self.cell, trace.data["goal_context"])
        instance = next(row for row in self.cell.instances if row.id == goal["arm_id"])
        self.compute_checkpoint()
        for client in (self.pose_ik_client, self.pose_fk_client):
            def available():
                self.check_abort()
                self.compute_budget.check()
                return client.service_is_ready()
            self.until(available, min(2., self.compute_budget.remaining()),
                       "Pose kinematics service")
        self.event("pose_ik")
        with trace.span("pose_ik"):
            target, ik = self.pose_adapter.solve_ik(instance.group, tool.parent_tip,
                goal_tip_pose(goal, tool), initial, instance.joints, timeout_sec=1.,
                rpc_timeout_sec=min(2., self.compute_budget.remaining()))
        self.compute_checkpoint()
        with trace.span("pose_fk_check"):
            actual, fk = self.pose_adapter.forward_kinematics(tool.parent_tip, target,
                timeout_sec=min(1., self.compute_budget.remaining()))
            errors = tcp_pose_error(goal, actual, tool)
            if (errors["position_error_m"] > IK_POSITION_TOLERANCE_M
                    or errors["orientation_error_rad"] > IK_ORIENTATION_TOLERANCE_RAD):
                raise PoseAdapterError("fk_goal_error", "IK solution does not reach the requested TCP pose")
            observed, initial_fk = self.pose_adapter.forward_kinematics(tool.parent_tip, initial,
                timeout_sec=min(1., self.compute_budget.remaining()))
            initial_errors = tcp_pose_error(goal, observed, tool)
        trace.data["pose_kinematics"] = dict(ik=ik, fk=fk, ik_errors=errors,
            initial_tcp_pose=observed, initial_fk=initial_fk, initial_tcp_errors=initial_errors,
            already_at_goal=(initial_errors["position_error_m"] <= PLANNED_POSITION_TOLERANCE_M
                and initial_errors["orientation_error_rad"] <= PLANNED_ORIENTATION_TOLERANCE_RAD),
            reference_scope="flange_reference_only", selected_arm=instance.id,
            other_arm_policy="hold full reviewed spline; ownership remains both arms")
        trace.data["command"] = dict(schema_version=1, request_id=trace.data["request_id"],
            strategy="coordinated", target_rad=target)
        self.compute_checkpoint()

    def pose_path(self, initial, target, trace):
        goal, tool = validate_goal_context(self.cell, trace.data["goal_context"])
        instance = next(row for row in self.cell.instances if row.id == goal["arm_id"])
        trajectory = self.plan(instance.group, {name: target[name] for name in instance.joints}, initial,
                               goal_tolerance_rad=POSE_PLANNING_JOINT_TOLERANCE_RAD)
        curve = hold_other_joints(trajectory, instance.joints, initial, self.joint_names)
        endpoint = dict(zip(curve.names, curve.position(curve.duration)))
        with trace.span("pose_planned_endpoint_fk"):
            actual, timing = self.pose_adapter.forward_kinematics(tool.parent_tip, endpoint,
                timeout_sec=min(1., self.compute_budget.remaining()))
            errors = tcp_pose_error(goal, actual, tool)
            if (errors["position_error_m"] > PLANNED_POSITION_TOLERANCE_M
                    or errors["orientation_error_rad"] > PLANNED_ORIENTATION_TOLERANCE_RAD):
                raise PoseAdapterError("fk_goal_error", "Planned endpoint misses the requested TCP tolerance")
        trace.data["pose_kinematics"].update(planned_endpoint_fk=timing, planned_endpoint_errors=errors,
            planned_position_tolerance_m=PLANNED_POSITION_TOLERANCE_M,
            planned_orientation_tolerance_rad=PLANNED_ORIENTATION_TOLERANCE_RAD)
        self.compute_checkpoint()
        return curve

    def confirm_pose_completion(self, trace):
        goal, tool = validate_goal_context(self.cell, trace.data["goal_context"])
        with trace.span("pose_execution_fk"):
            self.quiet()
            self.fresh()
            observed = self.state()
            actual, timing = self.pose_adapter.forward_kinematics(tool.parent_tip, observed,
                                                                 timeout_sec=1.)
            self.check_abort()
            self.fresh()
            errors = tcp_pose_error(goal, actual, tool)
            trace.data["pose_completion"] = dict(observed_state_rad=observed,
                actual_tcp_pose=actual, fk=timing, errors=errors,
                position_tolerance_m=EXECUTION_POSITION_TOLERANCE_M,
                orientation_tolerance_rad=EXECUTION_ORIENTATION_TOLERANCE_RAD)
            if (errors["position_error_m"] > EXECUTION_POSITION_TOLERANCE_M
                    or errors["orientation_error_rad"] > EXECUTION_ORIENTATION_TOLERANCE_RAD):
                raise PoseAdapterError("pose_execution_tolerance",
                    "Joint action completed but the observed TCP did not reach its declared tolerance")
