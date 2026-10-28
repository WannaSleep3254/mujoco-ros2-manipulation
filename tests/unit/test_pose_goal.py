"""TCP goal validation and noncommuting reference-frame transformations."""
from copy import deepcopy
from dataclasses import FrozenInstanceError
import math
import unittest

from mujoco_ros2_manipulation.workcells.pose_goal import (
    TOOL_SCOPE, ToolDefinition, goal_tip_pose, tcp_pose_error,
    tcp_pose_from_tip, validate_pose_goal, validate_world_pose,
)


def goal(tool, *, position=None, quaternion=None):
    return dict(schema_version=1, request_id="a"*32, arm_id="left", tool_id=tool.id,
                tool_sha256=tool.sha256, frame_id="world",
                position_m=position or [.5, -.2, 1.], quaternion_xyzw=quaternion or [0., 0., 0., 1.])


class PoseGoalTests(unittest.TestCase):
    def setUp(self):
        self.tool = ToolDefinition("fr5_flange_reference", "left_wrist3_link")

    def test_goal_has_exact_schema_known_arm_and_detached_vectors(self):
        source = goal(self.tool)
        checked = validate_pose_goal(source, arm_ids=("left", "right"))
        self.assertEqual(source, checked)
        source["position_m"][0] = 99
        source["quaternion_xyzw"][3] = 99
        self.assertEqual(checked["position_m"], [.5, -.2, 1.])
        self.assertEqual(checked["quaternion_xyzw"], [0., 0., 0., 1.])
        with self.assertRaisesRegex(ValueError, "Unknown"):
            validate_pose_goal(checked, arm_ids=("right",))

    def test_unknown_missing_version_and_identifier_fields_are_rejected(self):
        changes = [dict(extra=1), dict(schema_version=True), dict(schema_version=2),
                   dict(request_id="A"*32), dict(request_id="a"*31), dict(arm_id="../left"),
                   dict(arm_id=""), dict(tool_id="/tcp"), dict(tool_id="x"*65),
                   dict(tool_sha256="A"*64), dict(tool_sha256="a"*63), dict(frame_id="base")]
        for change in changes:
            with self.subTest(change=change), self.assertRaises(ValueError):
                validate_pose_goal(dict(goal(self.tool), **change))
        for key in goal(self.tool):
            missing = goal(self.tool)
            del missing[key]
            with self.subTest(missing=key), self.assertRaises(ValueError):
                validate_pose_goal(missing)
        for value in (None, [], "pose"):
            with self.subTest(value=value), self.assertRaises(ValueError):
                validate_pose_goal(value)

    def test_invalid_numeric_vectors_and_nonunit_quaternion_are_rejected(self):
        for field, values in (("position_m", ([1, 2], [1, 2, 3, 4], [True, 0, 0],
                                             [float("nan"), 0, 0], [10**1000, 0, 0], "123")),
                              ("quaternion_xyzw", ([0, 0, 0], [0, 0, 0, 0], [0, 0, 0, 2],
                                                   [0, 0, 0, 1.000002], [0, 0, 0, True],
                                                   [0, 0, float("inf"), 1]))):
            for value in values:
                with self.subTest(field=field, value=value), self.assertRaises(ValueError):
                    validate_pose_goal(dict(goal(self.tool), **{field: value}))
        normalized = validate_pose_goal(dict(goal(self.tool), quaternion_xyzw=[0, 0, 0, 1.0000005]))
        self.assertEqual(normalized["quaternion_xyzw"], [0., 0., 0., 1.])

    def test_tool_is_immutable_detached_and_explicitly_reference_only(self):
        translation, quaternion = [0., .1, .2], [0., 0., 0., -1.]
        tool = ToolDefinition("tcp", "left_wrist3_link", translation, quaternion)
        before = tool.sha256
        translation[0] = 99
        quaternion[0] = 99
        self.assertEqual(tool.translation_m, (0., .1, .2))
        self.assertEqual(tool.quaternion_xyzw, (0., 0., 0., 1.))
        with self.assertRaises(FrozenInstanceError):
            tool.parent_tip = "right_tool0"
        record = tool.record()
        self.assertEqual(record["scope"], TOOL_SCOPE)
        self.assertNotIn("collision_geometry", record)
        record["translation_m"][0] = 99
        self.assertEqual(before, tool.sha256)
        self.assertEqual(before, ToolDefinition("tcp", "left_wrist3_link", [0., .1, .2]).sha256)
        for changed in (ToolDefinition("tcp2", tool.parent_tip, tool.translation_m),
                        ToolDefinition("tcp", "right_tool0", tool.translation_m),
                        ToolDefinition("tcp", tool.parent_tip, [0., .1, .3]),
                        ToolDefinition("tcp", tool.parent_tip, tool.translation_m, source="another_reference")):
            self.assertNotEqual(before, changed.sha256)

    def test_revalidation_keeps_normalized_quaternion_and_tool_digest_stable(self):
        # Repeated unconditional normalization changed these unit components
        # by an ULP, so receive/resolve/review validations could disagree.
        q = [-.21942876015509213, .32403225411536274,
             -.917299254917545, -.07359479898357303]
        checked = validate_pose_goal(goal(self.tool, quaternion=q))
        for _ in range(8):
            repeated = validate_pose_goal(checked)
            self.assertEqual(repeated, checked)
            checked = repeated
        tool = ToolDefinition("stable", "left_wrist3_link", quaternion_xyzw=q)
        recreated = ToolDefinition(tool.id, tool.parent_tip,
            tool.translation_m, tool.record()["quaternion_xyzw"], tool.source)
        self.assertEqual(tool.record(), recreated.record())
        self.assertEqual(tool.sha256, recreated.sha256)

    def test_invalid_tool_definition_is_rejected_before_use(self):
        for kwargs in (dict(id="bad tool"), dict(parent_tip="/world"), dict(parent_tip="a//b"),
                       dict(parent_tip="a/"), dict(translation_m=[0, True, 0]),
                       dict(quaternion_xyzw=[0, 0, 0, .5]), dict(source=""), dict(source="line\nbreak")):
            args = dict(id="reference", parent_tip="left_wrist3_link")
            args.update(kwargs)
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                ToolDefinition(**args)

    def test_zero_offset_reference_preserves_pose_and_sign_equivalent_fk(self):
        source = goal(self.tool, quaternion=[0., 0., 0., -1.])
        tip = goal_tip_pose(source, self.tool)
        self.assertEqual(tip, {key: source[key] for key in ("frame_id", "position_m", "quaternion_xyzw")})
        positive_fk = dict(tip, quaternion_xyzw=[0., 0., 0., 1.])
        self.assertEqual(tcp_pose_error(source, positive_fk, self.tool),
                         dict(position_error_m=0., orientation_error_rad=0.))

    def test_inverse_tool_translation_is_rotated_by_world_tip_orientation(self):
        half = math.sqrt(.5)
        tool = ToolDefinition("offset", "left_wrist3_link", [.2, 0., 0.])
        source = goal(tool, position=[1., 2., 3.], quaternion=[0., 0., half, half])
        tip = goal_tip_pose(source, tool)
        for actual, expected in zip(tip["position_m"], [1., 1.8, 3.]):
            self.assertAlmostEqual(actual, expected)
        actual = tcp_pose_from_tip(tip, tool)
        self.assertLess(math.dist(actual["position_m"], source["position_m"]), 1e-14)
        self.assertLess(tcp_pose_error(source, tip, tool)["orientation_error_rad"], 1e-14)

    def test_noncommuting_tool_rotation_inverse_is_applied_on_correct_side(self):
        half = math.sqrt(.5)
        tool = ToolDefinition("offset", "left_wrist3_link", [.1, .2, .3], [half, 0., 0., half])
        source = goal(tool, position=[1., 2., 3.], quaternion=[0., half, 0., half])
        tip = goal_tip_pose(source, tool)
        expected_q = [-.5, .5, .5, .5]
        for actual, expected in zip(tip["quaternion_xyzw"], expected_q):
            self.assertAlmostEqual(actual, expected)
        # world_T_tip has axes: x -> -z, y -> -x, z -> y.
        for actual, expected in zip(tip["position_m"], [1.2, 1.7, 3.1]):
            self.assertAlmostEqual(actual, expected)
        error = tcp_pose_error(source, tip, tool)
        self.assertLess(error["position_error_m"], 1e-14)
        self.assertLess(error["orientation_error_rad"], 1e-14)

    def test_tcp_error_has_metric_translation_and_shortest_orientation_angle(self):
        source = goal(self.tool, position=[0., 0., 0.])
        fk = dict(frame_id="world", position_m=[.003, .004, 0.],
                  quaternion_xyzw=[0., 0., math.sin(.2/2), math.cos(.2/2)])
        error = tcp_pose_error(source, fk, self.tool)
        self.assertAlmostEqual(error["position_error_m"], .005)
        self.assertAlmostEqual(error["orientation_error_rad"], .2)
        negated = dict(fk, quaternion_xyzw=[-v for v in fk["quaternion_xyzw"]])
        self.assertEqual(error, tcp_pose_error(source, negated, self.tool))

    def test_tool_identity_change_and_wrong_world_pose_are_rejected(self):
        for change in (dict(tool_id="other"), dict(tool_sha256="0"*64)):
            with self.subTest(change=change), self.assertRaisesRegex(ValueError, "identity"):
                goal_tip_pose(dict(goal(self.tool), **change), self.tool)
        pose = dict(frame_id="world", position_m=[0, 0, 0], quaternion_xyzw=[0, 0, 0, 1])
        for changed in (dict(pose, frame_id="left_base_link"), dict(pose, extra=1),
                        dict(pose, quaternion_xyzw=[0, 0, 0, 0])):
            with self.subTest(changed=changed), self.assertRaises(ValueError):
                validate_world_pose(changed)
