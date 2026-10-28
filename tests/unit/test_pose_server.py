"""Run production admission/review paths with the ROS transport replaced."""
from copy import deepcopy
import json
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock
import uuid

from tests.unit import test_motion_review_server as server_tests
from mujoco_ros2_manipulation.workcells.interference import Spline
from mujoco_ros2_manipulation.workcells.pose_tools import configured_pose_tools
from mujoco_ros2_manipulation.workcells.pose_adapter import PoseAdapterError
from mujoco_ros2_manipulation.workcells.request_trace import RequestTrace


class PoseServerTests(unittest.TestCase):
    def setUp(self):
        owner = server_tests.ReviewServerTests(methodName="runTest")
        owner.setUp()
        self.addCleanup(owner.doCleanups)
        self.node = n = owner.node
        for instance in n.cell.instances:
            instance.prefix = instance.id + "_"
            instance.group = instance.id + "_arm"
            instance.profile = NS(label=instance.id, ros={"tip_frame":"wrist3_link"})
        n.pose_tools = configured_pose_tools(n.cell)
        n.pose_ik_client = n.pose_fk_client = NS(service_is_ready=lambda:True)
        n.until = lambda predicate, *_: self.assertTrue(predicate())
        self.pose = dict(frame_id="world", position_m=[.3, .2, .5],
                         quaternion_xyzw=[0., 0., 0., 1.])
        n.pose_adapter = NS(solve_ik=Mock(return_value=({"left":.1, "right":0.}, {"wall_sec":.01})),
                            forward_kinematics=Mock(side_effect=lambda _, state, **kw: (
                                dict(self.pose, position_m=([.3,.2,.5] if state["left"] else [.2,.2,.5])),
                                {"wall_sec":.01})))
        n.pose_path = Mock(return_value=Spline(["left", "right"], [
            dict(t=0., p=[0., 0.], v=[], a=[]), dict(t=.1, p=[.1, 0.], v=[], a=[])]))

    def goal(self):
        tool = self.node.pose_tools["left"]
        return dict(schema_version=1, request_id=uuid.uuid4().hex, arm_id="left",
                    tool_id=tool.id, tool_sha256=tool.sha256, **deepcopy(self.pose))

    def send(self, goal):
        self.node.receive_pose_plan(NS(data=json.dumps(goal)))

    def test_pose_is_queued_once_and_has_no_implicit_dispatch(self):
        goal = self.goal(); self.send(goal)
        pending = self.node.pending
        self.send(goal)
        self.assertIs(self.node.pending, pending)
        self.node.process_next()
        record = json.loads(self.node.path(pending).read_text())
        self.assertEqual(record["status"], "succeeded", record.get("error"))
        self.assertEqual(record["dispatched_goals"], 0)
        self.node.dispatch.assert_not_called()
        self.assertEqual(record["review_payload"]["goal_context"]["goal"], goal)
        self.assertEqual(record["review_payload"]["trajectories"], record["trajectories"])
        self.assertEqual(record["command"]["target_rad"]["right"], 0.)

    def test_joint_success_cannot_hide_observed_tcp_error(self):
        goal = self.goal(); tool = self.node.pose_tools["left"]
        trace = RequestTrace(goal["request_id"])
        trace.data["goal_context"] = dict(kind="tcp_pose", goal=goal,
                                         tool=dict(tool.record(), sha256=tool.sha256))
        with self.assertRaises(PoseAdapterError) as caught:
            self.node.confirm_pose_completion(trace)
        self.assertEqual(caught.exception.kind, "pose_execution_tolerance")
        self.assertAlmostEqual(trace.data["pose_completion"]["errors"]["position_error_m"], .1)
        self.node.dispatch.assert_not_called()

    def test_cartesian_noop_is_checked_and_issues_no_approval(self):
        goal = self.goal(); goal["position_m"] = [.2, .2, .5]
        self.node.pose_adapter.solve_ik.return_value = ({"left":0., "right":0.}, {"wall_sec":.01})
        self.send(goal); self.node.process_next()
        reply = self.node.replies[goal["request_id"]]
        self.assertEqual(reply["status"], "succeeded", reply.get("error"))
        self.assertEqual(reply["stage"], "plan_not_required")
        self.assertEqual(reply["pose_completion"]["errors"]["position_error_m"], 0.)
        self.assertIsNone(self.node.review_store.summary())
        self.node.pose_path.assert_not_called()
        self.node.dispatch.assert_not_called()

    def test_bad_tool_and_busy_admission_never_run_ik(self):
        goal = self.goal(); goal["tool_sha256"] = "0"*64
        self.send(goal)
        self.assertIsNone(self.node.pending)
        self.assertEqual(self.node.replies[goal["request_id"]]["status"], "rejected")
        self.node.pose_adapter.solve_ik.assert_not_called()
        self.node.active = NS(data={"request_id":"e"*32})
        goal = self.goal(); self.send(goal)
        self.assertIsNone(self.node.pending)
        self.assertEqual(self.node.replies[goal["request_id"]]["status"], "rejected")
        self.node.dispatch.assert_not_called()

    def test_malformed_pose_stays_in_trace_without_polluting_status(self):
        goal = self.goal(); goal["position_m"] = {"unexpected":"not a vector"}
        self.send(goal)
        reply = self.node.replies[goal["request_id"]]
        self.assertEqual(reply["status"], "rejected")
        self.assertNotIn("pose_goal", reply)
        trace = json.loads((self.node.directory/(goal["request_id"]+".json")).read_text())
        self.assertEqual(trace["pose_goal"], goal)
        self.node.pose_adapter.solve_ik.assert_not_called()
        self.node.dispatch.assert_not_called()

    def test_inaccurate_ik_has_no_review_or_action_goal(self):
        goal = self.goal(); self.send(goal)
        self.node.pose_adapter.forward_kinematics.side_effect = None
        self.node.pose_adapter.forward_kinematics.return_value = (
            dict(self.pose, position_m=[.4, .2, .5]), {"wall_sec":.01})
        self.node.process_next()
        reply = self.node.replies[goal["request_id"]]
        self.assertEqual(reply["status"], "failed")
        self.assertEqual(reply["failure_kind"], "fk_goal_error")
        self.assertIsNone(self.node.review_store.summary())
        self.node.dispatch.assert_not_called()

    def test_changed_tool_after_review_is_rejected_before_dispatch(self):
        goal = self.goal(); self.send(goal); self.node.process_next()
        review = self.node.review_store.summary()
        self.assertEqual(review["state"], "awaiting_approval")
        self.node.cell.instances[0].profile.ros["tip_frame"] = "different_tip"
        approval = dict(schema_version=1, request_id=uuid.uuid4().hex,
            plan_request_id=goal["request_id"], plan_sha256=review["plan_sha256"],
            session_id=self.node.session_id)
        self.node.receive_approval(NS(data=json.dumps(approval)))
        self.assertEqual(self.node.replies[approval["request_id"]]["status"], "rejected")
        self.assertIsNone(self.node.pending)
        self.node.dispatch.assert_not_called()


if __name__ == "__main__":
    unittest.main()
