"""Owned-domain ROS checks for plan-only review and single-use execution.

No existing simulation is attached. A newly launched simulation is required, and
only its launch process group is stopped by the common verification lifecycle.
These are functional checks, not timing or simulator safety certification.
"""
import argparse
from copy import deepcopy
import json
from pathlib import Path
import uuid

import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.msg import CollisionObject
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import String
from std_srvs.srv import Trigger

from mujoco_ros2_manipulation.paths import ROOT
from mujoco_ros2_manipulation.model_build.integrity import file_sha256
from mujoco_ros2_manipulation.verification.diagnostics import diagnostic_environment, finalize_physics
from mujoco_ros2_manipulation.verification.reports import atomic_json
from mujoco_ros2_manipulation.verification.runner import run_verification
from .config import load_workcell
from .integrity import check_workcell
from .motion_request import COMMAND_TOPIC, PLAN_TOPIC, APPROVE_TOPIC, CANCEL_TOPIC, command
from .motion_server import MotionCommandServer
from .verify import require
from .verify_motion_reproduction import require_unused_domain


class MotionReviewVerification(MotionCommandServer):
    def __init__(self, cell, progress, output):
        super().__init__(cell, output, progress,trace_root=ROOT/"runtime")
        self.senders = {name:self.create_publisher(String, topic, 10) for name,topic in (
            ("command",COMMAND_TOPIC),("plan",PLAN_TOPIC),("approve",APPROVE_TOPIC),("cancel",CANCEL_TOPIC))}
        self.review_check = dict(schema_version=1,status="running",session_id=self.session_id,checks=[],
            implementation_sha256={name:file_sha256(Path(__file__).with_name(name))
                for name in ("verify_motion_review.py","motion_review.py","motion_request.py","motion_server.py")},
            note="Functional checks on an owned simulation: a small actual motion, rejected review identities, cancellation and a temporary far-away MoveIt world object. No performance or real-robot safety claim.")
        self.save_review_checks()

    def save_review_checks(self):
        atomic_json(self.directory/"review-verification.json",self.review_check)

    def send_review_message(self, kind, payload, *, process=True):
        identifier = payload["request_id"]
        self.senders[kind].publish(String(data=json.dumps(payload,allow_nan=False)))
        if kind == "cancel":
            self.until(lambda:self.cancel_requested or (self.review_store.summary() or {}).get("state") == "invalidated",
                       5,"review cancellation")
        else:
            self.until(lambda:(self.pending is not None and self.pending.data["request_id"] == identifier)
                       or identifier in self.replies,5,"review request receipt")
            if process and self.pending:
                self.process_next()
        return self.replies.get(identifier)

    def read_review_trace(self, identifier):
        return json.loads((self.directory/(identifier+".json")).read_text())

    def record_check(self, name, **details):
        self.review_check["checks"].append(dict(name=name,**details))
        self.save_review_checks()

    def plan_for_review(self, target):
        payload = command(target)
        before = self.dispatched
        self.send_review_message("plan",payload)
        record = self.read_review_trace(payload["request_id"])
        require(record["status"] == "succeeded" and record["mode"] == "plan_only"
                and record["dispatched_goals"] == 0 and self.dispatched == before,
                "Plan-only request failed or dispatched goals")
        require(record.get("trajectories") and record.get("planned_clearance",{}).get("complete")
                and record.get("moveit_safety_samples",0) > 0,"Plan did not pass its full final checks")
        review = self.review_store.summary()
        require(review and review["state"] == "awaiting_approval","No review was issued")
        return record,dict(schema_version=1,request_id=uuid.uuid4().hex,
            plan_request_id=payload["request_id"],session_id=self.session_id,plan_sha256=review["plan_sha256"])

    def reject_approval(self, approval, *, name):
        before = self.dispatched
        result = self.send_review_message("approve",approval)
        require(result["status"] in ("rejected","failed") and self.dispatched == before,
                "Invalid approval was dispatched")
        record = self.read_review_trace(approval["request_id"])
        require(record["dispatched_goals"] == 0,"Invalid approval sent an action goal")
        self.record_check(name,request_id=approval["request_id"],status=record["status"],
                          error=record["error"],dispatched_goals=0)

    def alter_world(self, *, remove=False):
        client = self.create_client(ApplyPlanningScene,"/apply_planning_scene")
        self.until(client.service_is_ready,5,"owned planning-scene test service")
        request = ApplyPlanningScene.Request()
        request.scene.is_diff = True
        obj = CollisionObject()
        obj.header.frame_id = "world"
        obj.id = "motion_review_verification_"+self.session_id
        obj.operation = CollisionObject.REMOVE if remove else CollisionObject.ADD
        if not remove:
            box = SolidPrimitive()
            box.type,box.dimensions = SolidPrimitive.BOX,[.1,.1,.1]
            pose = Pose()
            pose.position.x,pose.position.y,pose.position.z = 10.0,10.0,10.0
            pose.orientation.w = 1.0
            obj.primitives,obj.primitive_poses = [box],[pose]
        request.scene.world.collision_objects = [obj]
        require(self.result(client.call_async(request),5,"owned planning-scene test change").success,
                "Could not change the owned test planning scene")

    def run(self):
        try:
            self.prepare()
            for publisher in self.senders.values():
                self.until(lambda:publisher.get_subscription_count() >= 1,5,"review topic subscription")
            initial = self.state()
            target = dict(initial)
            target[self.joint_names[0]] += .04
            plan,approval = self.plan_for_review(target)
            self.record_check("plan_only_zero_dispatch",plan_request_id=plan["request_id"],
                dispatched_goals=0,trajectory_count=len(plan["trajectories"]))
            self.reject_approval(dict(approval,request_id=uuid.uuid4().hex,plan_sha256="0"*64),name="wrong_digest")
            self.reject_approval(dict(approval,request_id=uuid.uuid4().hex,session_id="0"*32),name="wrong_session")
            before = self.dispatched
            self.send_review_message("approve",approval)
            executed = self.read_review_trace(approval["request_id"])
            require(executed["status"] == "succeeded" and self.dispatched-before == len(self.cell.instances)
                    and executed["trajectories"] == plan["trajectories"],
                    "Approved execution did not run the exact reviewed curves")
            require(not executed.get("plans") and executed.get("moveit_safety_samples",0) > 0,
                    "Approval replanned or skipped safety checks")
            self.record_check("approved_exact_path_execution",request_id=approval["request_id"],
                dispatched_goals=executed["dispatched_goals"],planned_clearance=executed["planned_clearance"],
                observed_clearance=executed.get("observed_clearance"),action_results=executed["action_results"])
            before = self.dispatched
            self.send_review_message("approve",approval)
            require(self.dispatched == before,"Duplicate approval executed twice")
            self.record_check("duplicate_execution_id_replays_without_dispatch",dispatched_goals=0)
            self.reject_approval(dict(approval,request_id=uuid.uuid4().hex),name="consumed_review")

            _,cancelled = self.plan_for_review(initial)
            self.send_review_message("cancel",dict(request_id=cancelled["plan_request_id"]))
            self.reject_approval(cancelled,name="cancelled_review")

            _,changed = self.plan_for_review(initial)
            try:
                self.alter_world()
                self.reject_approval(changed,name="changed_world_scene")
            finally:
                self.alter_world(remove=True)

            _,cancel_pending = self.plan_for_review(initial)
            self.send_review_message("approve",cancel_pending,process=False)
            self.send_review_message("cancel",dict(request_id=cancel_pending["request_id"]))
            before = self.dispatched
            self.process_next()
            result = self.read_review_trace(cancel_pending["request_id"])
            require(result["status"] == "cancelled" and self.dispatched == before,
                    "Cancellation before dispatch still executed")
            self.record_check("cancel_approved_pending",request_id=cancel_pending["request_id"],dispatched_goals=0)
            self.reject_approval(dict(cancel_pending,request_id=uuid.uuid4().hex),name="cancelled_approval_not_reusable")
            self.review_check["status"] = "completed"
        except BaseException as error:
            self.review_check.update(status="aborted",error=dict(type=type(error).__name__,message=str(error)))
            raise
        finally:
            self.save_review_checks()
        return dict(review_verification_path=str(self.directory/"review-verification.json"),
            checks=len(self.review_check["checks"]),contact_guard=deepcopy(self.contact_guard))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--domain",type=int,required=True)
    parser.add_argument("--output",type=Path,help="New directory under this workspace's runtime/")
    parser.add_argument("--model-workspace",type=Path,default=ROOT,
        help="Read existing checked models/runtime here; write all test results in this workspace")
    args = parser.parse_args()
    require_unused_domain(args.domain)
    args.robot,args.scene,args.gui,args.stop_mode = "fr5_ur5e","default",False,"interrupt"
    args.require_gpu,args.screenshot_dir = False,None
    output = args.output or ROOT/"runtime/validation/review-sessions"/uuid.uuid4().hex
    return run_verification(args,ROOT,rclpy,lambda cell,progress:MotionReviewVerification(cell,progress,output),Trigger,
        profile_loader=lambda *a,**kw:load_workcell(root=args.model_workspace.resolve()),validator=check_workcell,
        command_builder=lambda cell,args:["bash",str(cell.root/"run_scene.sh"),f"workcell:={cell.id}",
            "gui:=false","rviz:=false","planning_case:=dual_arm_crossing"],
        report_prefix="motion_review_",launch_environment=diagnostic_environment,finalize=finalize_physics)


if __name__ == "__main__":
    raise SystemExit(main())
