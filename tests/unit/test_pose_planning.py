from copy import deepcopy
import json
from types import SimpleNamespace as NS
import unittest

from mujoco_ros2_manipulation.workcells.pose_goal import ToolDefinition
from mujoco_ros2_manipulation.workcells.pose_planning import hold_other_joints
from mujoco_ros2_manipulation.workcells.pose_tools import (
    describe_pose_tools, resolve_pose_tool, validate_context_identity, validate_goal_context)
from mujoco_ros2_manipulation.workcells.motion_review import PlanReviewStore
from mujoco_ros2_manipulation.operator_console.view_plan import ReviewPlan
from tests.unit.test_view_plan import fixture


def context():
    tool = ToolDefinition("left_flange_reference", "left_wrist3_link",
        source="Checked robot URDF tip origin; no added gripper geometry")
    goal = dict(schema_version=1, request_id="a"*32, arm_id="left",
        tool_id=tool.id, tool_sha256=tool.sha256, frame_id="world",
        position_m=[.3, .2, .5], quaternion_xyzw=[0., 0., 0., 1.])
    return dict(kind="tcp_pose", goal=goal, tool=dict(tool.record(), sha256=tool.sha256))


def cell():
    profile = NS(label="FAIRINO FR5 V6", ros={"tip_frame":"wrist3_link"})
    return NS(instances=[NS(id="left", prefix="left_", profile=profile)])


class PosePlanningTests(unittest.TestCase):
    def test_registered_reference_is_explicit_and_not_gripper_geometry(self):
        rows = describe_pose_tools(cell())
        self.assertEqual(rows[0]["scope"], "flange_reference_only")
        self.assertEqual(rows[0]["tool"], context()["tool"])
        goal, tool = resolve_pose_tool(cell(), context()["goal"])
        self.assertEqual(tool.sha256, goal["tool_sha256"])
        self.assertEqual(validate_goal_context(cell(), context())[0], goal)

    def test_unknown_arm_tool_and_changed_reference_are_refused(self):
        for mutate in (
            lambda c: c["goal"].update(arm_id="right"),
            lambda c: c["goal"].update(tool_sha256="0"*64),
            lambda c: c["tool"].update(schema_version=True),
            lambda c: c["tool"]["translation_m"].__setitem__(2, .1),
            lambda c: c["tool"].update(scope="physical_gripper")):
            row = context(); mutate(row)
            with self.subTest(row=row), self.assertRaises(ValueError):
                validate_goal_context(cell(), row)

    def test_other_arm_is_held_at_every_spline_point_and_between_points(self):
        selected = ["left_"+str(i) for i in range(6)]
        others = ["right_"+str(i) for i in range(6)]
        names = others + selected
        initial = {name:(.4 if name in others else 0.) for name in names}
        points = [NS(time_from_start=NS(sec=t, nanosec=0), positions=[.1*t]*6,
                     velocities=[0. if t in (0, 2) else .1]*6,
                     accelerations=[0.]*6) for t in (0, 1, 2)]
        path = hold_other_joints(NS(joint_names=selected, points=points), selected, initial, names)
        for point in path.points:
            self.assertEqual(point["p"][:6], [.4]*6)
            self.assertEqual(point["v"][:6], [0.]*6)
            self.assertEqual(point["a"][:6], [0.]*6)
        for step in range(101):
            self.assertEqual(list(path.position(step*.02))[:6], [.4]*6)
        self.assertAlmostEqual(path.position(2.)[-1], .2)

    def test_other_joint_cannot_leak_into_selected_path(self):
        trajectory = NS(joint_names=["left", "right"], points=[
            NS(time_from_start=NS(sec=t, nanosec=0), positions=[0., 0.],
               velocities=[], accelerations=[]) for t in (0, 1)])
        with self.assertRaises(ValueError):
            hold_other_joints(trajectory, ["left"], {"left":0., "right":0.}, ["left", "right"])

    def test_review_digest_binds_original_pose_and_detaches_it(self):
        trace, _, _ = fixture()
        issue = dict(identity={"model":"test"}, initial_state_rad={"joint":0.},
            simulation_sec=1., command=trace["command"], trajectories=trace["trajectories"])
        original = context()
        store = PlanReviewStore("b"*32, clock=lambda:0.)
        review = store.issue("a"*32, **issue, goal_context=original)
        original["goal"]["position_m"][0] += .1
        payload = store.payload_for("a"*32, review["plan_sha256"])
        self.assertEqual(payload["goal_context"], context())
        other = PlanReviewStore("b"*32, clock=lambda:0.)
        altered = other.issue("a"*32, **issue, goal_context=original)
        self.assertNotEqual(altered["plan_sha256"], review["plan_sha256"])
        with self.assertRaises(ValueError):
            validate_context_identity(context(), plan_request_id="c"*32)

    def test_pending_preview_rejects_pose_different_from_exact_review(self):
        trace, session, options = fixture()
        row = context()
        store = PlanReviewStore("b"*32, clock=lambda:0.)
        review = store.issue("a"*32, identity={"model":"test"},
            initial_state_rad=trace["initial_state_rad"], simulation_sec=1.,
            command=trace["command"], trajectories=trace["trajectories"], goal_context=row)
        trace.update(mode="plan_only", dispatched_goals=0, review=review,
            goal_context=deepcopy(row), pose_goal=deepcopy(row["goal"]),
            review_payload=store.payload_for("a"*32, review["plan_sha256"]))
        def build():
            return ReviewPlan(json.dumps(trace).encode(), json.dumps(session).encode(),
                              review=review, **options)
        self.assertEqual(build().positions(1.)["joint"], 1.)
        trace["pose_goal"]["position_m"][0] += .01
        with self.assertRaisesRegex(ValueError, "TCP goal"):
            build()


if __name__ == "__main__":
    unittest.main()
