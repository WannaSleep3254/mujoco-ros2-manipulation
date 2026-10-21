"""Exact search equivalence and failed comparison evidence without ROS."""
from copy import deepcopy
import unittest

from mujoco_ros2_manipulation.workcells.search_comparison import compare_case
from tests.unit.test_timing import curve,cell,AnalyticDistance


class SearchComparisonTests(unittest.TestCase):
    def setUp(self):
        self.policy=dict(sample_dt_sec=0.01,min_inter_arm_distance_m=0.05,tracking_reserve_m=0.0,delay_step_sec=0.2,max_delay_sec=1.2)

    def test_paired_input_unchanged_order_alternates_and_warmup_excluded(self):
        curves=[curve("l"),curve("r")]
        before=deepcopy([c.record() for c in curves])
        result=compare_case(curves,{"l":0.0,"r":0.0},cell(),AnalyticDistance(),self.policy,2)
        self.assertEqual(result["status"],"passed")
        self.assertEqual(len(result["warmup"]),2)
        self.assertEqual([p["order"] for p in result["samples"]],[["reference","optimized"],["optimized","reference"]])
        self.assertTrue(all(p["selection_and_clearance_identical"] for p in result["samples"]))
        self.assertEqual([c.record() for c in curves],before)

    def test_failure_is_saved_with_partial_candidate_diagnostics(self):
        data={};saved=[]
        class Broken(AnalyticDistance):
            def at(self,state): raise RuntimeError("native evaluator failure")
        with self.assertRaisesRegex(RuntimeError,"native evaluator"):
            compare_case([curve("l"),curve("r")],{"l":0.0,"r":0.0},cell(),Broken(),self.policy,1,
                record=data,progress=lambda:saved.append(deepcopy(data)))
        self.assertEqual(data["status"],"failed")
        self.assertEqual(data["warmup"][0]["status"],"failed")
        self.assertEqual(data["warmup"][0]["diagnostics"]["candidates"][0]["status"],"interrupted")
        self.assertEqual(data["samples"],[])
        self.assertEqual(saved[-1]["status"],"failed")
