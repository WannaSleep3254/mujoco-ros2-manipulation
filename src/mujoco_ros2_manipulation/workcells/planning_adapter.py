"""MoveIt request conversion and one RPC, without admission or planning policy.

ROS message factories, the client and its bounded future waiter are injected.
Joint positions and tolerances use radians for the current rotational workcells;
the full start-state order is the configured joint order. The caller supplies a
detached observed or virtual start state and owns its freshness, clock domain,
anchor, limits, candidate search, clearance and execution approval checks.
"""
from collections.abc import Mapping, Sequence
import math
import time


def _finite(value):
    try:
        return type(value) in (int, float) and math.isfinite(value)
    except OverflowError:
        return False


class MoveItPlanningAdapter:
    """Build one joint-target request and return its raw response plus timing.

    This adapter never reads live state, reorders an input goal, anchors a
    trajectory, retries a request, or turns a transport error into a planner
    failure code. The 25-second RPC wait is wall time, supplied by the caller's
    waiter; it is separate from MoveIt's allowed planning time.
    """

    def __init__(self, client, wait_result, request_factory, constraints_factory,
                 joint_constraint_factory, joint_names, monotonic=time.monotonic):
        if not isinstance(joint_names, Sequence) or isinstance(joint_names, (str, bytes)):
            raise ValueError("Expected unique ordered planning joint names")
        names = tuple(joint_names)
        if (not names or any(type(name) is not str or not name.strip() for name in names)
                or len(names) != len(set(names))):
            raise ValueError("Expected unique ordered planning joint names")
        self.client = client
        self.wait_result = wait_result
        self.request_factory = request_factory
        self.constraints_factory = constraints_factory
        self.joint_constraint_factory = joint_constraint_factory
        self.joint_names = names
        self.monotonic = monotonic

    def plan_joint_target(self, group, target_rad, start_rad, planner_id="",
                          allowed_planning_sec=5.0, goal_tolerance_rad=.003):
        started = self.monotonic()
        if type(group) is not str or not group.strip() or type(planner_id) is not str:
            raise ValueError("Expected a planning group and planner ID strings")
        if (not isinstance(start_rad, Mapping) or set(start_rad) != set(self.joint_names)
                or not all(_finite(value) for value in start_rad.values())):
            raise ValueError("Planning start state must cover every configured joint with finite radians")
        if (not isinstance(target_rad, Mapping) or not target_rad
                or not set(target_rad).issubset(self.joint_names)
                or not all(_finite(value) for value in target_rad.values())):
            raise ValueError("Planning target must contain known joints with finite radians")
        if (not _finite(allowed_planning_sec) or allowed_planning_sec <= 0
                or not _finite(goal_tolerance_rad) or goal_tolerance_rad < 0):
            raise ValueError("Expected positive finite planning time and nonnegative finite tolerance")
        # Copy only scalar values into new messages. A caller may supply a
        # virtual waypoint, so there is deliberately no live-state callback.
        start = {name: start_rad[name] for name in self.joint_names}
        target = list(target_rad.items())
        request = self.request_factory()
        motion = request.motion_plan_request
        motion.group_name = group
        motion.pipeline_id = "ompl"
        motion.planner_id = planner_id
        motion.num_planning_attempts = 3
        motion.allowed_planning_time = float(allowed_planning_sec)
        motion.max_velocity_scaling_factor = .1
        motion.max_acceleration_scaling_factor = .1
        motion.start_state.is_diff = True
        motion.start_state.joint_state.name = list(self.joint_names)
        motion.start_state.joint_state.position = [start[name] for name in self.joint_names]
        constraint = self.constraints_factory()
        for name, value in target:
            joint = self.joint_constraint_factory()
            joint.joint_name = name
            joint.position = float(value)
            joint.tolerance_above = joint.tolerance_below = float(goal_tolerance_rad)
            joint.weight = 1.
            constraint.joint_constraints.append(joint)
        motion.goal_constraints = [constraint]
        answer = self.wait_result(
            self.client.call_async(request), 25, "shared motion plan"
        ).motion_plan_response
        timing = dict(
            group=group, wall_sec=self.monotonic() - started,
            server_planning_sec=answer.planning_time,
            requested_pipeline_id=motion.pipeline_id,
            requested_planner_id=motion.planner_id,
            selected_planner_id=None,  # GetMotionPlan does not expose this field.
            attempts=motion.num_planning_attempts,
            allowed_planning_sec=motion.allowed_planning_time,
            goal_tolerance_rad=goal_tolerance_rad,
            error_code=answer.error_code.val,
        )
        return answer, timing
