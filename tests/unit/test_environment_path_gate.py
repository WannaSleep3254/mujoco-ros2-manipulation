"""A clear inter-arm path must not hide a grazing fixed-environment path."""
from copy import deepcopy
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock

from mujoco_ros2_manipulation.workcells.interference import Spline
from mujoco_ros2_manipulation.workcells.motion_diagnostics import measure_endpoints, path_clearance_diagnostic
from mujoco_ros2_manipulation.workcells.path_quality import evaluate_path
from mujoco_ros2_manipulation.workcells.plan_selection import select_clearance_plan, PlanCandidatesExhausted
from tests.unit.test_spatial_proposal_contract import trajectory


ENV_POLICY = dict(schema_version=1, min_environment_distance_m=.005, tracking_reserve_m=.01)
ARM_POLICY = dict(min_inter_arm_distance_m=.02, tracking_reserve_m=.01, sample_dt_sec=.01)


def environment(distance):
    return dict(distance_m=distance, bodies=["robot_wrist", "fixed_post"],
                nearest_points_m=[[0., 0., 0.], [distance, 0., 0.]], pair_count=4,
                scope="moving_robot_vs_fixed_environment")


class EnvironmentPathGateTests(unittest.TestCase):
    def setUp(self):
        self.initial = dict(left=0., right=0.)
        self.cell = NS(joints=list(self.initial), instances=[NS(id=name, joints=[name]) for name in self.initial])
        self.curve = Spline(["left", "right"], [dict(t=0., p=[0., 0.], v=[], a=[]),
                                                        dict(t=1., p=[.1, .1], v=[], a=[])])

    def evaluator(self, distance):
        def at(state):
            gap = distance(state) if callable(distance) else distance
            return dict(distance_m=1., bodies=["left", "right"],
                        nearest_points_m=[[0., 0., 0.], [1., 0., 0.]],
                        tips_m=[[state["left"], 0., 0.], [state["right"], 1., 0.]],
                        environment=environment(gap))
        return NS(at=Mock(side_effect=at))

    def test_bad_intermediate_environment_sample_rejects_despite_clear_endpoints_and_other_arm(self):
        evaluator = self.evaluator(lambda q: .002 if .03 < q["left"] < .07 else .1)
        endpoints = measure_endpoints(self.initial, dict(left=.1, right=.1), evaluator, .03,
                                      environment_threshold_m=.015)
        self.assertEqual(endpoints["endpoint_violations"], [])
        result = evaluate_path([self.curve], self.initial, self.cell, evaluator,
                               reject_below=.03, environment_reject_below=.015)
        self.assertFalse(result["complete"])
        self.assertEqual(result["rejection_kind"], "environment_clearance")
        self.assertEqual(result["minimum"]["distance_m"], 1.)
        self.assertEqual(result["environment_minimum"]["distance_m"], .002)
        self.assertLess(result["samples"], 101)
        diagnostic = path_clearance_diagnostic(endpoints, result, 1.)
        self.assertEqual(diagnostic["violation_location"], "intermediate")
        self.assertEqual(diagnostic["witness"]["bodies"], ["robot_wrist", "fixed_post"])

    def test_threshold_boundary_penetration_and_missing_native_coverage_fail_closed(self):
        allowed = evaluate_path([self.curve], self.initial, self.cell, self.evaluator(.015),
                                environment_reject_below=.015)
        self.assertTrue(allowed["complete"])
        for distance in (.015 - 1e-9, -.001):
            with self.subTest(distance=distance):
                result = evaluate_path([self.curve], self.initial, self.cell, self.evaluator(distance),
                                       environment_reject_below=.015)
                self.assertFalse(result["complete"])
        for bad in (None, dict(environment(.1), pair_count=0)):
            evaluator = self.evaluator(.1)
            result = evaluator.at(self.initial)
            result["environment"] = bad
            evaluator.at = Mock(return_value=result)
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                evaluate_path([self.curve], self.initial, self.cell, evaluator,
                              environment_reject_below=.015)

    def test_environment_endpoint_is_invalid_independently_from_inter_arm_clearance(self):
        endpoints = measure_endpoints(self.initial, dict(left=.1, right=.1), self.evaluator(.002), .03,
                                      environment_threshold_m=.015)
        self.assertEqual(endpoints["endpoint_violations"], ["start", "goal"])
        self.assertTrue(endpoints["endpoints"]["goal"]["environment_below_policy"])
        self.assertEqual(endpoints["endpoints"]["goal"]["distance_m"], 1.)

    def test_bounded_selector_retries_environment_rejection_and_preserves_its_own_witness(self):
        proposals = [trajectory(), trajectory()]
        phase = {"candidate": 0}
        def propose():
            phase["candidate"] += 1
            return 1, proposals[phase["candidate"] - 1]
        evaluator = self.evaluator(lambda state: .002 if phase["candidate"] == 1 else .1)
        saved = []
        moveit = Mock(return_value=dict(invalid_samples=0))
        chosen = select_clearance_plan(propose, self.initial, self.cell, evaluator, ARM_POLICY,
            moveit, lambda candidates: saved.append(deepcopy(candidates)), max_candidates=2,
            environment_policy=ENV_POLICY)
        self.assertIs(chosen, proposals[1])
        first, second = saved[-1]
        self.assertEqual(first["rejection_kind"], "environment_clearance")
        self.assertEqual(first["native"]["environment_minimum"]["distance_m"], .002)
        self.assertNotIn("moveit", first)
        self.assertTrue(second["accepted"])
        moveit.assert_called_once()

    def test_all_grazing_candidates_exhaust_original_count_without_moveit_or_dispatch(self):
        propose = Mock(return_value=(1, trajectory()))
        moveit = Mock()
        with self.assertRaises(PlanCandidatesExhausted):
            select_clearance_plan(propose, self.initial, self.cell, self.evaluator(.002), ARM_POLICY,
                                  moveit, lambda candidates: None, max_candidates=2,
                                  environment_policy=ENV_POLICY)
        self.assertEqual(propose.call_count, 2)
        moveit.assert_not_called()

    def test_legacy_cross_arm_evaluator_still_works_when_environment_was_not_requested(self):
        evaluator = self.evaluator(.1)
        result = evaluator.at(self.initial)
        result.pop("environment")
        evaluator.at = Mock(return_value=result)
        assessment = evaluate_path([self.curve], self.initial, self.cell, evaluator, reject_below=.03)
        self.assertTrue(assessment["complete"])
        self.assertNotIn("environment_minimum", assessment)
