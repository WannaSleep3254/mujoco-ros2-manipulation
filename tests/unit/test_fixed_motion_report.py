"""Fixed public report formatting does not hide failures or change provenance."""
from contextlib import redirect_stderr, redirect_stdout
from copy import deepcopy
import gzip
import hashlib
import io
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from tests.unit import test_fixed_motion_export as export_tests
from mujoco_ros2_manipulation.reporting.data import validate
from mujoco_ros2_manipulation.reporting.fixed_motion import fixed_report
from mujoco_ros2_manipulation.workcells.fixed_results import aggregate_results, summarize_row
from mujoco_ros2_manipulation.workcells.fixed_suite import planned_rows, suite_hash
from mujoco_ros2_manipulation.workcells.export_fixed_motion import export_fixed_motion, SOURCE_KEYS


def full_public_evidence():
    fixture = export_tests.FixedMotionExportTests("runTest")
    fixture.setUp()
    suite = fixture.suite
    fixture.catalog = planned_rows(suite, repeats=3)
    for row in fixture.catalog:
        case = next(case for case in suite["cases"] if case["case_id"] == row["case_id"])
        row.update(row_id=f'r{row["repeat"]:02d}-{row["case_id"]}-{row["variant"]}',
            strategy="coordinated", expectation=case["expectation"], domain=171+row["index"])
    fixture.sources = {row["row_id"]: dict.fromkeys(SOURCE_KEYS) for row in fixture.catalog}
    fixture.rows = [dict(row,status="not_run",request_id=None,actual_request_status=None,outcome=None,
        fixed_case_result_sha256=None,trace_sha256=None,verification_result_sha256=None,
        unindexed_trace_present=False,unindexed_trace_sha256=None) for row in fixture.catalog]
    for index,row in enumerate(fixture.catalog):
        outcome = ("expected_invalid_goal" if row["expectation"] == "invalid_goal" else
                   "planning_failed" if row["case_id"] == "home_return" or
                       (row["case_id"] == "local_small" and row["variant"] == "single") else "motion_succeeded")
        fixture.install(index,outcome)
    compact = [summarize_row(suite,row,**{key:fixture.sources[row["row_id"]][key]
        for key in ("fixed_case_raw","trace_raw","verification_raw")}) for row in fixture.catalog]
    counts = dict(planned=60,selected=60,completed=60,aborted=0,not_run=0,
        motion_succeeded=sum(row["outcome"]=="motion_succeeded" for row in fixture.rows),
        planning_failed=sum(row["outcome"]=="planning_failed" for row in fixture.rows),expected_invalid_goal=6)
    fixture.raw = dict(schema_version=1,id="fixed-motion-raw-v1",status="completed",suite_sha256=suite_hash(suite),
        compute_budget_wall_sec=5.,initial_tolerance_rad=.0015,repeats=3,requested_case_ids=None,
        domain_start=171,domain_range=dict(start=171,end=230,count=60),rows=fixture.rows,results=compact,
        summary=aggregate_results(suite,fixture.catalog,compact),counts=counts,source_sha256=fixture.code)
    return suite,export_fixed_motion(export_tests.encoded(suite),export_tests.encoded(fixture.raw),
        fixture.sources,expected_suite=suite)


class FixedMotionReportTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.suite,cls.evidence = full_public_evidence()

    def report(self, evidence=None, evidence_name="evidence.json.gz"):
        value = self.evidence if evidence is None else evidence
        raw = gzip.compress(export_tests.encoded(value),mtime=0) if evidence_name.endswith("gz") else export_tests.encoded(value)
        return fixed_report(value,"2026-10-23",hashlib.sha256(raw).hexdigest(),
            expected_suite=self.suite,evidence_name=evidence_name),raw

    def test_twenty_case_variant_groups_keep_n_three_and_negative_separate(self):
        report,_ = self.report()
        self.assertEqual(report["status"],"preliminary")
        self.assertEqual(len(report["sample_groups"]),20)
        self.assertTrue(all(group["observations"]==3 and group["unique_cases"]==1 for group in report["sample_groups"]))
        groups={group["id"]:group for group in report["sample_groups"]}
        self.assertEqual((groups["local-small-single"]["successes"],groups["local-small-single"]["failures"]),(0,3))
        for variant in ("single","bounded"):
            group=groups["invalid-goal-"+variant]
            self.assertIsNone(group["successes"])
            self.assertIsNone(group["failures"])
            self.assertIn("invalid_goal 거부 3/3",group["description"])
            self.assertIn("관측 이동 0회, Action 전송 0개",group["description"])
        text=json.dumps(report,ensure_ascii=False)
        self.assertIn("인과 효과",text)
        self.assertIn("n=3",text)
        self.assertFalse(any(value["statistic"] in ("p95","p99") for metric in report["metrics"] for value in metric["values"]))

    def test_failed_compute_is_included_but_prefix_clearance_is_not_full_path(self):
        report,_=self.report()
        values={metric["id"]:{value["group_id"]:value for value in metric["values"]} for metric in report["metrics"]}
        compute=values["compute"]["local-small-single"]
        self.assertEqual((compute["sample_count"],compute["value"],compute["statistic"]),(3,5.01,"median"))
        for metric in ("first-motion","planned-clearance","planned-joint-travel","observed-clearance"):
            value=values[metric]["local-small-single"]
            self.assertEqual((value["value"],value["sample_count"],value["statistic"]),(None,0,"not_measured"))
        self.assertEqual(values["planned-clearance"]["local-small-bounded"]["value"],40.)

    def test_measured_commit_and_dirty_status_are_never_replaced_by_current_git(self):
        report,_=self.report()
        self.assertEqual(report["provenance"]["commit"],"1"*40)
        self.assertEqual(report["environment"]["measurement_worktree"],"dirty")
        self.assertTrue(any("implementation SHA" in text for text in report["limitations"]))
        self.assertEqual(report["references"][0]["url"].split("/")[-1],"1"*40)

    def test_actual_selected_attempts_and_all_failed_cases_drive_the_findings(self):
        report,_=self.report()
        self.assertTrue(any("후속 후보에 의한 회복 성공은 관측되지" in item for item in report["findings"]))
        findings=[item for item in report["findings"] if "계획 실패 6/6회" in item]
        self.assertEqual(len(findings),1)
        self.assertIn("Action 전송 0개",findings[0])
        self.assertIn("전역적으로 해가 없음을 뜻하지",findings[0])
        evidence=deepcopy(self.evidence)
        row=next(row for row in evidence["rows"] if row["variant"]=="bounded" and row["outcome"]=="motion_succeeded")
        row["metrics"].update(selected_attempt=2,candidate_count=2,planned_candidate_attempt=2)
        evidence["summary"]=aggregate_results(self.suite,evidence["planned_catalog"],evidence["rows"])
        report,_=self.report(evidence)
        self.assertTrue(any("후속 후보 선택 성공 1/" in item for item in report["findings"]))
        self.assertFalse(any("회복 성공은 관측되지" in item for item in report["findings"]))

    def test_report_schema_asset_hash_and_compact_privacy(self):
        report,raw=self.report()
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)
            (root/"evidence.json.gz").write_bytes(raw)
            self.assertEqual(validate(report,root),report)
        text=json.dumps(report)
        for forbidden in ("private-user-77","private-box-77","/home/","process_command","password="):
            self.assertNotIn(forbidden,text)

    def test_partial_run_remains_visible_and_cannot_claim_sixty_completed(self):
        fixture=export_tests.FixedMotionExportTests("runTest");fixture.setUp()
        evidence=fixture.export()
        report,_=self.report(evidence)
        self.assertEqual(len(report["sample_groups"]),20)
        self.assertEqual(sum(group["observations"] for group in report["sample_groups"]),3)
        self.assertTrue(any("미실행 17" in finding for finding in report["findings"]))
        self.assertIn("진단 실행",report["limitations"][0])
        validate(report,Path("/unused"),check_assets=False)

    def test_mutated_summary_dropped_row_or_scope_change_is_rejected(self):
        for mutation in (lambda value:value["rows"].pop(),
            lambda value:value["summary"]["valid_goal"].__setitem__("motion_succeeded",999),
            lambda value:value["scope"].__setitem__("repeats",30)):
            evidence=deepcopy(self.evidence);mutation(evidence)
            with self.assertRaises(ValueError):
                self.report(evidence)

    def test_read_only_check_detects_report_edit_and_create_refuses_existing_assets(self):
        from mujoco_ros2_manipulation.reporting import fixed_motion
        report,raw=self.report()
        with tempfile.TemporaryDirectory() as directory:
            root=Path(directory)/report["report_id"];root.mkdir()
            (root/"evidence.json.gz").write_bytes(raw)
            (root/"report.json").write_text(json.dumps(report))
            with patch("sys.argv",["fixed-report","--check",str(root)]),redirect_stdout(io.StringIO()):
                self.assertEqual(fixed_motion.main(),0)
            changed=deepcopy(report);changed["findings"][0]="Edited finding"
            (root/"report.json").write_text(json.dumps(changed))
            with patch("sys.argv",["fixed-report","--check",str(root)]),self.assertRaisesRegex(ValueError,"differs"):
                fixed_motion.main()
            with patch("sys.argv",["fixed-report","--evidence",str(root/"evidence.json.gz"),"--output",str(root)]),redirect_stderr(io.StringIO()),self.assertRaises(SystemExit):
                fixed_motion.main()
            self.assertEqual((root/"evidence.json.gz").read_bytes(),raw)


if __name__=="__main__":
    unittest.main()
