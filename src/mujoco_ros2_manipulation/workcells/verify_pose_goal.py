"""Owned-domain ROS checks for explicit TCP-reference Pose plans.

The first version tests a flange-origin reference on the existing FR5 geometry.
It does not add a physical gripper or demonstrate grasping. FK-derived reachable
goals, IK, final path checks and single-use approval are exercised against a new
owned simulation. Run outcomes and failed assertions remain in the result.
"""
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
from .interference import Spline
from .motion_request import POSE_PLAN_TOPIC
from .pose_goal import tcp_pose_error, tcp_pose_from_tip, validate_pose_goal
from .verify import require
from .verify_motion_review import MotionReviewVerification
from .verify_motion_reproduction import require_unused_domain


EXECUTION_POSITION_TOLERANCE_M = .005
EXECUTION_ORIENTATION_TOLERANCE_RAD = .03
OBSERVED_HOLD_TOLERANCE_RAD = .01


class PoseGoalVerification(MotionReviewVerification):
    """Use the real server admission and approval paths, never direct dispatch."""

    def __init__(self, cell, progress, output):
        super().__init__(cell, progress, output)
        self.senders["pose_plan"] = self.create_publisher(String, POSE_PLAN_TOPIC, 10)
        source = Path(__file__).parent
        names = {"verify_pose_goal.py", "motion_server.py", "motion_review.py", "motion_request.py"}
        names.update(path.name for path in source.glob("pose_*.py"))
        self.review_check.update(
            implementation_sha256={name: file_sha256(source/name) for name in sorted(names)},
            execution_tcp_tolerance=dict(position_m=EXECUTION_POSITION_TOLERANCE_M,
                orientation_rad=EXECUTION_ORIENTATION_TOLERANCE_RAD),
            observed_other_arm_hold_tolerance_rad=OBSERVED_HOLD_TOLERANCE_RAD,
            note="Functional checks on an owned MuJoCo/MoveIt simulation. The selected FR5 flange-origin TCP has no added gripper geometry. Planned spline hold and observed servo deviation are separate checks. No physical grasp, performance, or hardware safety claim.")
        self.save_review_checks()

    def save_review_checks(self):
        atomic_json(self.directory/"pose-verification.json", self.review_check)

    def make_goal(self, instance, state):
        tool = self.pose_tools[instance.id]
        tip, timing = self.pose_adapter.forward_kinematics(tool.parent_tip, state)
        pose = tcp_pose_from_tip(tip, tool)
        goal = dict(schema_version=1, request_id=uuid.uuid4().hex, arm_id=instance.id,
            tool_id=tool.id, tool_sha256=tool.sha256, **pose)
        return validate_pose_goal(goal, arm_ids=self.pose_tools), timing

    def plan_pose_for_review(self, goal):
        before = self.dispatched
        self.send_review_message("pose_plan", goal)
        record = self.read_review_trace(goal["request_id"])
        require(record["status"] == "succeeded" and record.get("mode") == "plan_only"
                and record["dispatched_goals"] == 0 and self.dispatched == before,
                "Pose plan-only request failed or dispatched an action")
        require(record.get("pose_goal") == validate_pose_goal(goal, arm_ids=self.pose_tools)
                and record.get("trajectories")
                and record.get("planned_clearance", {}).get("complete")
                and record.get("moveit_safety_samples", 0) > 0,
                "Pose plan did not retain its goal or pass every full-path gate")
        context = record.get("review_payload", {}).get("goal_context")
        tool = self.pose_tools[goal["arm_id"]]
        require(context == dict(kind="tcp_pose", goal=record["pose_goal"],
                tool=dict(tool.record(), sha256=tool.sha256)),
                "Review digest did not bind the explicit Pose and TCP reference")
        review = self.review_store.summary()
        require(review and review["state"] == "awaiting_approval"
                and review["plan_request_id"] == goal["request_id"], "Pose plan issued no current review")
        return record, dict(schema_version=1, request_id=uuid.uuid4().hex,
            plan_request_id=goal["request_id"], session_id=self.session_id,
            plan_sha256=review["plan_sha256"])

    def reject_pose(self, goal, *, name, failure_kinds=None):
        before = self.dispatched
        self.send_review_message("pose_plan", goal)
        record = self.read_review_trace(goal["request_id"])
        require(record["status"] in ("rejected", "failed")
                and self.dispatched == before and record["dispatched_goals"] == 0,
                "Invalid Pose goal reached action dispatch")
        review = self.review_store.summary()
        require(not review or review["plan_request_id"] != goal["request_id"],
                "Failed Pose request issued an approval token")
        if failure_kinds is not None:
            require(record.get("failure_kind") in failure_kinds,
                    "Pose rejection did not preserve the expected failure class")
        self.record_check(name, request_id=goal["request_id"], status=record["status"],
            error=record["error"], failure_kind=record.get("failure_kind"), dispatched_goals=0)
        return record

    def check_other_arm_hold(self, plan, selected):
        initial = plan["initial_state_rad"]
        other = set(self.joint_names)-set(selected.joints)
        seen, maximum = set(), 0.
        for item in plan["trajectories"]:
            curve = Spline(**item)
            for index, name in enumerate(curve.names):
                if name not in other:
                    continue
                require(name not in seen, "Other-arm hold appears in overlapping curves")
                seen.add(name)
                for point in curve.points:
                    maximum = max(maximum, abs(point["p"][index]-initial[name]))
                    require((not point["v"] or point["v"][index] == 0.)
                            and (not point["a"] or point["a"][index] == 0.),
                            "Pose path commands a derivative on the other arm")
        require(seen == other and maximum <= 1e-12,
                "Pose path does not hold every non-selected joint for its whole spline")
        return maximum

    def run(self):
        try:
            self.prepare()
            for publisher in self.senders.values():
                self.until(lambda: publisher.get_subscription_count() >= 1, 5,
                           "Pose verification topic subscription")
            for client in (self.pose_adapter.ik_client, self.pose_adapter.fk_client):
                self.until(client.service_is_ready, 5, "Pose verification kinematics service")
            selected = self.cell.instances[0]
            require(selected.profile.id == "fr5", "This representative Pose fixture requires FR5 first")
            initial = self.state()
            target_state = dict(initial)
            target_state[selected.joints[0]] += .04
            goal, goal_fk = self.make_goal(selected, target_state)
            plan, approval = self.plan_pose_for_review(goal)
            hold_delta = self.check_other_arm_hold(plan, selected)
            self.record_check("pose_plan_only_zero_dispatch_and_full_spline_hold",
                plan_request_id=goal["request_id"], goal=goal, source_fk=goal_fk,
                pose_kinematics=deepcopy(plan.get("pose_kinematics")),
                planned_other_arm_delta_rad=hold_delta, dispatched_goals=0)

            before = self.dispatched
            self.send_review_message("approve", approval)
            executed = self.read_review_trace(approval["request_id"])
            require(executed["status"] == "succeeded"
                    and self.dispatched-before == len(self.cell.instances)
                    and executed["trajectories"] == plan["trajectories"]
                    and not executed.get("plans"),
                    "Pose approval did not execute the exact checked path without replanning")
            self.quiet()
            tool = self.pose_tools[selected.id]
            observed_tip, final_fk = self.pose_adapter.forward_kinematics(tool.parent_tip, self.state())
            errors = tcp_pose_error(goal, observed_tip, tool)
            other = set(self.joint_names)-set(selected.joints)
            observations = executed.get("observations", [])
            require(observations and all(other.issubset(item.get("state_rad", {})) for item in observations),
                    "Execution did not preserve complete other-arm observations")
            observed_hold = max(abs(item["state_rad"][name]-plan["initial_state_rad"][name])
                for item in observations for name in other)
            self.record_check("approved_exact_pose_path_execution", request_id=approval["request_id"],
                dispatched_goals=executed["dispatched_goals"], tcp_errors=errors, final_fk=final_fk,
                observed_other_arm_max_delta_rad=observed_hold,
                action_results=executed["action_results"])
            require(errors["position_error_m"] <= EXECUTION_POSITION_TOLERANCE_M
                    and errors["orientation_error_rad"] <= EXECUTION_ORIENTATION_TOLERANCE_RAD,
                    "Observed TCP after settling misses the declared execution tolerance")
            require(observed_hold <= OBSERVED_HOLD_TOLERANCE_RAD,
                    "Non-selected arm observation exceeds the declared hold tolerance")

            self.reject_pose(dict(goal, request_id=uuid.uuid4().hex, tool_sha256="0"*64),
                             name="wrong_tool_digest_zero_dispatch")
            self.reject_pose(dict(goal, request_id=uuid.uuid4().hex, position_m=[100., 100., 100.]),
                name="unreachable_pose_zero_dispatch", failure_kinds={"ik_no_solution", "ik_timeout"})

            return_goal, _ = self.make_goal(selected, initial)
            _, changed = self.plan_pose_for_review(return_goal)
            try:
                self.alter_world()
                self.reject_approval(changed, name="changed_world_invalidates_pose_review")
            finally:
                self.alter_world(remove=True)

            cancelled_goal, _ = self.make_goal(selected, initial)
            self.send_review_message("pose_plan", cancelled_goal, process=False)
            # A previous invalidated review is not evidence that this queued
            # request received cancellation. Wait for this server's latch.
            self.senders["cancel"].publish(String(data=json.dumps(
                dict(request_id=cancelled_goal["request_id"]), allow_nan=False)))
            self.until(lambda: self.cancel_requested, 5, "queued Pose cancellation receipt")
            before = self.dispatched
            self.process_next()
            cancelled = self.read_review_trace(cancelled_goal["request_id"])
            require(cancelled["status"] == "cancelled" and self.dispatched == before
                    and cancelled["dispatched_goals"] == 0,
                    "Cancellation of a pending Pose request still dispatched an action")
            self.record_check("cancel_queued_pose_zero_dispatch", request_id=cancelled_goal["request_id"],
                              status=cancelled["status"], dispatched_goals=0)
            self.review_check["status"] = "completed"
        except BaseException as error:
            self.review_check.update(status="aborted", error=dict(type=type(error).__name__, message=str(error)))
            raise
        finally:
            self.save_review_checks()
        return dict(pose_verification_path=str(self.directory/"pose-verification.json"),
            checks=len(self.review_check["checks"]), contact_guard=deepcopy(self.contact_guard))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--domain", type=int, required=True)
    parser.add_argument("--output", type=Path, help="New directory under this workspace's runtime/")
    parser.add_argument("--model-workspace", type=Path, default=ROOT,
        help="Read existing checked models/runtime here; write results in this workspace")
    args = parser.parse_args()
    require_unused_domain(args.domain)
    args.robot, args.scene, args.gui, args.stop_mode = "fr5_ur5e", "default", False, "interrupt"
    args.require_gpu, args.screenshot_dir = False, None
    output = args.output or ROOT/"runtime/validation/pose-sessions"/uuid.uuid4().hex
    return run_verification(args, ROOT, rclpy,
        lambda cell, progress: PoseGoalVerification(cell, progress, output), Trigger,
        profile_loader=lambda *a, **kw: load_workcell(root=args.model_workspace.resolve()),
        validator=check_workcell,
        command_builder=lambda cell, args: ["bash", str(cell.root/"run_scene.sh"), f"workcell:={cell.id}",
            "gui:=false", "rviz:=false", "planning_case:=dual_arm_crossing"],
        report_prefix="pose_goal_", launch_environment=diagnostic_environment, finalize=finalize_physics)


if __name__ == "__main__":
    raise SystemExit(main())
