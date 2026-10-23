"""Serial composition preserves ownership, actual endpoints and stop joins."""
from copy import deepcopy
import unittest

from mujoco_ros2_manipulation.workcells.interference import Spline
from mujoco_ros2_manipulation.workcells.spatial_candidates import compose_serial_paths


def path(names, positions, times, velocities=None, accelerations=None):
    return Spline(names, [dict(t=stamp, p=list(position),
        v=[] if velocities is None else list(velocities[index]),
        a=[] if accelerations is None else list(accelerations[index]))
        for index, (stamp, position) in enumerate(zip(times, positions))])


class SerialPathTests(unittest.TestCase):
    def setUp(self):
        self.names = ["right_b", "left_a", "right_a", "left_b"]
        self.initial = dict(left_a=0.2, left_b=-0.4, right_a=0.7, right_b=-0.1)
        self.left = path(["left_b", "left_a"], [[-0.4, 0.2], [-0.1, 0.3], [0.197, 0.499]],
            [0.0, 0.75, 2.0], [[9, 8], [0.02, 0.03], [7, 6]], [[5, 4], [0.04, 0.05], [3, 2]])
        self.right = path(["right_a", "right_b"], [[0.7, -0.1], [0.8, -0.2], [0.9, -0.3]],
            [0.0, 1.0, 3.5], [[8, 7], [0.06, 0.07], [6, 5]], [[4, 3], [0.08, 0.09], [2, 1]])

    def compose(self, segments=None, initial=None, names=None):
        return compose_serial_paths([self.left, self.right] if segments is None else segments,
            self.initial if initial is None else initial, self.names if names is None else names)

    def state(self, curve, stamp):
        return dict(zip(curve.names, curve.position(stamp)))

    def test_unequal_durations_and_reordered_names_have_one_strict_time_axis(self):
        result = self.compose()
        self.assertEqual(result.names, tuple(self.names))
        self.assertEqual(result.times, [0.0, 0.75, 2.0, 3.0, 5.5])
        self.assertEqual(result.duration, 5.5)
        self.assertEqual(self.state(result, 0), self.initial)
        self.assertEqual(self.state(result, result.duration), dict(left_a=0.499, left_b=0.197, right_a=0.9, right_b=-0.3))

    def test_other_arm_holds_initial_then_actual_first_leg_endpoint(self):
        result = self.compose()
        for stamp in (0, 0.25, 0.75, 1.75, 2.0):
            state = self.state(result, stamp)
            self.assertEqual(state["right_a"], self.initial["right_a"])
            self.assertEqual(state["right_b"], self.initial["right_b"])
        for stamp in (2.0, 2.25, 3.0, 5.25, 5.5):
            state = self.state(result, stamp)
            self.assertEqual(state["left_a"], 0.499)
            self.assertEqual(state["left_b"], 0.197)

    def test_endpoint_derivatives_stop_and_interior_derivatives_are_preserved(self):
        result = self.compose()
        for index in (0, 2, 4):
            self.assertEqual(result.points[index]["v"], [0.0] * 4)
            self.assertEqual(result.points[index]["a"], [0.0] * 4)
        self.assertEqual(result.points[1]["v"], [0.0, 0.03, 0.0, 0.02])
        self.assertEqual(result.points[1]["a"], [0.0, 0.05, 0.0, 0.04])
        self.assertEqual(result.points[3]["v"], [0.07, 0.0, 0.06, 0.0])
        self.assertEqual(result.points[3]["a"], [0.09, 0.0, 0.08, 0.0])
        join = result.position(2.0)
        for stamp in (2.0 - 1e-4, 2.0 + 1e-4):
            self.assertLess(max(abs(a-b) for a, b in zip(result.position(stamp), join)), 1e-8)

    def test_start_tolerance_is_anchored_without_a_duplicate_join_or_jump(self):
        self.left.points[0]["p"] = [-0.395, 0.21]
        self.right.points[0]["p"] = [0.691, -0.095]
        result = self.compose()
        self.assertEqual(self.state(result, 0), self.initial)
        self.assertEqual(result.times.count(2.0), 1)
        self.assertEqual(self.state(result, 2.0), dict(self.initial, left_a=0.499, left_b=0.197))

    def test_each_leg_start_outside_tolerance_is_rejected(self):
        for index, key in ((0, "left_b"), (1, "right_a")):
            segments = deepcopy([self.left, self.right])
            segments[index].points[0]["p"][0] = self.initial[key] + 0.0101
            with self.subTest(leg=index), self.assertRaisesRegex(ValueError, "virtual state"):
                self.compose(segments)

    def test_reversing_legs_changes_which_arm_holds_without_changing_joint_order(self):
        result = self.compose([self.right, self.left])
        self.assertEqual(result.names, tuple(self.names))
        state = self.state(result, 1.0)
        self.assertEqual(state["left_a"], self.initial["left_a"])
        self.assertEqual(state["left_b"], self.initial["left_b"])
        state = self.state(result, 4.0)
        self.assertEqual(state["right_a"], 0.9)
        self.assertEqual(state["right_b"], -0.3)

    def test_duplicate_missing_unknown_and_unordered_joint_contracts_fail(self):
        overlap = path(["left_a", "right_a"], [[0.2, 0.7], [0.3, 0.8]], [0, 1])
        unknown = path(["other", "right_b"], [[0.7, -0.1], [0.9, -0.3]], [0, 1])
        for segments, initial, names in (
                ([self.left, overlap], self.initial, self.names),
                ([self.left, unknown], self.initial, self.names),
                ([self.left], self.initial, self.names),
                ([self.left, self.right, self.left], self.initial, self.names),
                ([self.left, self.right], dict(self.initial, other=0), self.names),
                ([self.left, self.right], {k:v for k,v in self.initial.items() if k != "left_a"}, self.names),
                ([self.left, self.right], self.initial, self.names + ["left_a"]),
                ([self.left, self.right], self.initial, set(self.names)),
                ([self.left, self.right], self.initial, "left_a")):
            with self.subTest(names=names, segments=len(segments)), self.assertRaises(ValueError):
                self.compose(segments, initial, names)

    def test_modified_spline_fields_and_nonfinite_initial_are_revalidated(self):
        variants = []
        for field, value in (("t", 0), ("p", [float("nan"), 0]), ("p", [0]),
                             ("v", [float("inf"), 0]), ("a", [0]), ("p", "bad")):
            broken = deepcopy(self.left)
            broken.points[-1][field] = value
            variants.append(broken)
        broken = deepcopy(self.left)
        broken.names = ("left_a", "left_a")
        variants.append(broken)
        broken = deepcopy(self.left)
        broken.points[0]["unknown"] = 1
        variants.append(broken)
        variants.append(self.left.record())
        for broken in variants:
            with self.subTest(segment=broken), self.assertRaises(ValueError):
                self.compose([broken, self.right])
        for value in (True, float("nan"), float("inf"), 10**1000):
            with self.subTest(initial=value), self.assertRaises(ValueError):
                self.compose(initial=dict(self.initial, left_a=value))

    def test_missing_derivatives_are_zero_filled_to_consistent_output_layout(self):
        left = path(["left_b", "left_a"], [[-0.4, 0.2], [0.197, 0.499]], [0, 2])
        right = path(["right_a", "right_b"], [[0.7, -0.1], [0.8, -0.2], [0.9, -0.3]],
            [0, 1, 3.5], [[7, 6], [0.06, 0.07], [5, 4]])
        result = self.compose([left, right])
        self.assertEqual(result.layout, (True, True))
        self.assertEqual(result.points[2]["v"], [0.07, 0.0, 0.06, 0.0])
        self.assertTrue(all(point["a"] == [0.0]*4 for point in result.points))

    def test_combined_duration_and_knot_limits_remain_enforced(self):
        long_left = path(["left_b", "left_a"], [[-0.4, 0.2], [0.197, 0.499]], [0, 20])
        long_right = path(["right_a", "right_b"], [[0.7, -0.1], [0.9, -0.3]], [0, 16])
        with self.assertRaisesRegex(ValueError, "35s"):
            self.compose([long_left, long_right])
        dense_left = path(["left_b", "left_a"], [[-0.4, 0.2]]*1001, [i/1000 for i in range(1001)])
        dense_right = path(["right_a", "right_b"], [[0.7, -0.1]]*1001, [i/1000 for i in range(1001)])
        with self.assertRaisesRegex(ValueError, "point count"):
            self.compose([dense_left, dense_right])

    def test_sources_and_initial_are_immutable_and_output_is_detached(self):
        before = deepcopy((self.left.record(), self.right.record(), self.initial, self.names))
        result = self.compose()
        self.assertEqual((self.left.record(), self.right.record(), self.initial, self.names), before)
        result.points[1]["p"][0] = 999
        result.points[1]["v"][1] = 999
        result.points[3]["a"][0] = 999
        self.assertEqual((self.left.record(), self.right.record(), self.initial, self.names), before)


if __name__ == "__main__":
    unittest.main()
