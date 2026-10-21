"""Publication rejects lost input cases and changed selection/timing evidence."""
from copy import deepcopy
import unittest

from mujoco_ros2_manipulation.reporting.search_optimization import check_comparison
from mujoco_ros2_manipulation.workcells.search_comparison import compare_case
from tests.unit.test_timing import curve,cell,AnalyticDistance


def fixture():
    policy=dict(sample_dt_sec=0.01,min_inter_arm_distance_m=0.05,tracking_reserve_m=0.0,delay_step_sec=0.2,max_delay_sec=1.2)
    case=dict(case_id="crossing")
    compare_case([curve("l"),curve("r")],{"l":0.0,"r":0.0},cell(),AnalyticDistance(),policy,2,record=case)
    return dict(status="passed",error=None,repeats=2,planned_case_ids=["crossing"],cases=[case],policy=policy)


class SearchReportTests(unittest.TestCase):
    def test_warmups_and_measured_pairs_keep_exact_geometry_and_accounting(self):
        data=fixture()
        self.assertIs(check_comparison(data),data)
        self.assertEqual(len(data["cases"][0]["samples"]),2)

    def test_dropped_input_filtered_repeat_changed_geometry_or_time_is_rejected(self):
        for field in ("case","repeat","order","shape","time","calls"):
            data=fixture();case=data["cases"][0]
            if field=="case":data["planned_case_ids"].append("missing-case")
            if field=="repeat":case["samples"].pop()
            if field=="order":case["samples"][0]["order"].reverse()
            if field=="shape":case["samples"][0]["variants"][0]["selection"]["curves"][0]["points"][-1]["p"][0]+=0.01
            profile=case["samples"][0]["variants"][0]["diagnostics"]["profiling"]
            if field=="time":profile["elapsed_wall_sec"]+=1
            if field=="calls":profile["native_distance_calls"]+=1
            with self.subTest(field=field),self.assertRaises(ValueError):check_comparison(data)
