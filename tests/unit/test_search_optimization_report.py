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


class PublishedSearchReportTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        import gzip
        import hashlib
        import json
        from pathlib import Path
        cls.folder=Path(__file__).resolve().parents[2]/"docs/reports/2026-10-21-search-optimization"
        cls.report=json.loads((cls.folder/"report.json").read_text())
        cls.evidence=json.loads(gzip.decompress((cls.folder/"evidence.json.gz").read_bytes()))
        cls.hashes={a["path"]:hashlib.sha256((cls.folder/a["path"]).read_bytes()).hexdigest() for a in cls.report["assets"]}

    def test_publication_reconstructs_every_sample_and_uses_actual_gui_images(self):
        from mujoco_ros2_manipulation.reporting.search_optimization import search_report
        from mujoco_ros2_manipulation.reporting.data import validate
        report=search_report(self.evidence,self.report["report_id"],self.report["published_date"],self.hashes)
        self.assertEqual(self.report,report)
        validate(report,self.folder)
        comparison=self.evidence["comparison"]
        self.assertEqual(len(comparison["cases"]),6)
        self.assertEqual(sum(len(c["samples"]) for c in comparison["cases"]),30)
        self.assertEqual(sum(len(c["warmup"]) for c in comparison["cases"]),12)
        self.assertEqual(len(report["sample_groups"]),13)
        self.assertEqual(sum(len(m["values"]) for m in report["metrics"]),85)

    def test_failed_gui_or_mismatched_owned_screenshot_is_rejected(self):
        from mujoco_ros2_manipulation.reporting.search_optimization import search_report
        # Alter GUI evidence only; avoid copying the large, immutable candidate snapshot.
        for field in ("status","shutdown","contacts","gpu","clock","cancellation","model","image"):
            evidence=dict(comparison=self.evidence["comparison"],gui=deepcopy(self.evidence["gui"]))
            run=evidence["gui"]["verification"];hashes=dict(self.hashes)
            if field=="status":run["status"]="failed"
            if field=="shutdown":run["shutdown_clean"]=False
            if field=="contacts":run["physics"]["contact_steps"]=1
            if field=="gpu":run["rviz_gpu_process_seen"]=False
            if field=="clock":run["clock_publishers"]=2
            if field=="cancellation":run["active_cancellation_stops_both_arms"]=False
            if field=="model":run["model_manifest_sha256"]="0"*64
            if field=="image":hashes["mujoco-gui.png"]="0"*64
            with self.subTest(field=field),self.assertRaises(ValueError):
                search_report(evidence,self.report["report_id"],self.report["published_date"],hashes)
