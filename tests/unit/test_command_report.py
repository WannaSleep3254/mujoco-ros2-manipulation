"""The published snapshot must retain negative controls and the evaluated source."""
import copy
import hashlib
import json
from pathlib import Path
import unittest

from mujoco_ros2_manipulation.reporting.data import load_report
from mujoco_ros2_manipulation.reporting.motion_requests import command_report


class CommandReportTests(unittest.TestCase):
    def setUp(self):
        self.directory=Path(__file__).resolve().parents[2]/"docs/reports/2026-10-21-request-timing"
        self.report=load_report(self.directory/"report.json")
        raw=(self.directory/"evidence.json").read_bytes()
        self.evidence=json.loads(raw)
        self.sha=hashlib.sha256(raw).hexdigest()

    def make(self,evidence):
        return command_report(evidence,self.report["report_id"],self.report["published_date"],self.sha)

    def test_published_values_match_preserved_request_timeline(self):
        self.assertEqual(self.report,self.make(self.evidence))
        self.assertEqual(len(self.report["sample_groups"]),11)
        self.assertEqual(sum(g["failures"] for g in self.report["sample_groups"]),6)
        self.assertEqual(self.report["provenance"]["commit"],self.evidence["session"]["code_commit"])

    def test_failed_request_cannot_be_silently_removed(self):
        changed=copy.deepcopy(self.evidence)
        changed["requests"]=[r for r in changed["requests"] if r["status"]!="failed"]
        with self.assertRaisesRegex(ValueError,"Incomplete"):
            self.make(changed)

    def test_mixed_implementation_sessions_are_rejected(self):
        changed=copy.deepcopy(self.evidence)
        changed["shutdown_session"]["code_commit"]="0"*40
        with self.assertRaisesRegex(ValueError,"Mixed"):
            self.make(changed)
