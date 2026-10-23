"""ROS-free tests of the actual fixed-suite runner's admission and lifecycle.

Only imports/functions that do not initialize ROS are executed. Stubbed lifecycle
artifacts are synthetic and every write stays inside TemporaryDirectory.
"""
import argparse
import ast
from contextlib import redirect_stdout
from copy import deepcopy
from datetime import datetime, timezone
import hashlib
import io
import json
import os
from pathlib import Path
import re
import tempfile
import time
from types import SimpleNamespace
import unittest
from unittest.mock import Mock
import uuid

from mujoco_ros2_manipulation.paths import ROOT
from mujoco_ros2_manipulation.model_build.integrity import file_sha256
from mujoco_ros2_manipulation.verification.reports import atomic_json
from mujoco_ros2_manipulation.workcells.config import load_workcell
from mujoco_ros2_manipulation.workcells.fixed_results import summarize_row
from mujoco_ros2_manipulation.workcells.fixed_suite import (
    BASE_ID, POST_ID, make_suite, planned_rows, suite_hash,
)


RUNNER_SOURCE = ROOT / "src/mujoco_ros2_manipulation/workcells/benchmark_fixed_motion.py"
CANONICAL_SUITE = ROOT / "config/evaluations/fixed_motion_v1.json"


def require(value, message):
    if not value:
        raise RuntimeError(message)


def load_runner_functions(**overrides):
    """Compile current production functions while skipping ROS/GUI imports."""
    source = ast.parse(RUNNER_SOURCE.read_text())
    nodes = [deepcopy(node) for node in source.body if isinstance(node, (ast.FunctionDef, ast.Assign))]
    parsed = ast.fix_missing_locations(ast.Module(body=nodes, type_ignores=[]))
    namespace = dict(__file__=str(RUNNER_SOURCE), ROOT=ROOT, argparse=argparse,
        deepcopy=deepcopy, datetime=datetime, timezone=timezone, hashlib=hashlib,
        json=json, os=os, Path=Path, re=re, time=time, uuid=uuid, require=require,
        file_sha256=file_sha256, atomic_json=atomic_json, make_suite=make_suite,
        planned_rows=planned_rows, suite_hash=suite_hash, BASE_ID=BASE_ID, POST_ID=POST_ID)
    namespace.update(overrides)
    exec(compile(parsed, str(RUNNER_SOURCE), "exec"), namespace)
    return namespace


class FixedRunnerTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.suite = json.loads(CANONICAL_SUITE.read_text())
        cls.cells = {name: load_workcell(name) for name in (BASE_ID, POST_ID)}

    def test_canonical_file_and_identical_copy_bind_actual_source_bytes(self):
        functions = load_runner_functions()
        suite, digest = functions["_load_declared_suite"](CANONICAL_SUITE, self.cells)
        self.assertEqual(suite, self.suite)
        self.assertEqual(digest, file_sha256(CANONICAL_SUITE))
        with tempfile.TemporaryDirectory() as directory:
            copied = Path(directory) / "same-suite.json"
            copied.write_text(json.dumps(self.suite, separators=(",", ":")))
            value, copied_digest = functions["_load_declared_suite"](copied, self.cells)
            self.assertEqual(value, self.suite)
            self.assertEqual(copied_digest, file_sha256(copied))
            self.assertNotEqual(copied_digest, digest)

    def test_immutable_suite_change_or_duplicate_json_field_is_rejected(self):
        functions = load_runner_functions()
        with tempfile.TemporaryDirectory() as directory:
            changed = Path(directory) / "changed.json"
            value = deepcopy(self.suite)
            name = next(iter(value["cases"][0]["target_rad"]))
            value["cases"][0]["target_rad"][name] += .001
            changed.write_text(json.dumps(value))
            with self.assertRaisesRegex(RuntimeError, "immutable factory"):
                functions["_load_declared_suite"](changed, self.cells)
            duplicate = Path(directory) / "duplicate.json"
            duplicate.write_text('{"schema_version":1,' + json.dumps(self.suite)[1:])
            with self.assertRaisesRegex(ValueError, "Duplicate suite field"):
                functions["_load_declared_suite"](duplicate, self.cells)

    def test_complete_catalog_retains_order_and_uses_sixty_unique_domains(self):
        functions = load_runner_functions()
        rows = functions["_full_rows"](self.suite, 3, None, domain_start=171)
        original = planned_rows(self.suite, repeats=3)
        keys = ("case_id", "variant", "max_plan_candidates", "repeat", "index", "selected")
        self.assertEqual([[row[key] for key in keys] for row in rows],
                         [[row[key] for key in keys] for row in original])
        self.assertEqual(len(rows), 60)
        self.assertEqual([row["domain"] for row in rows], list(range(171, 231)))
        self.assertEqual(len({row["row_id"] for row in rows}), 60)
        self.assertTrue(all(row["status"] == "not_run" and row["request_id"] is None for row in rows))

    def test_unselected_rows_have_no_domain_and_do_not_consume_selected_domains(self):
        functions = load_runner_functions()
        rows = functions["_full_rows"](self.suite, 1, ["home_return"], domain_start=171)
        self.assertEqual(len(rows), 20)
        self.assertEqual([row["domain"] for row in rows if row["selected"]], [171, 172])
        self.assertTrue(all(row["domain"] is None and row["not_run_reason"] == "case_not_selected"
                            for row in rows if not row["selected"]))
        with self.assertRaises(ValueError):
            functions["_full_rows"](self.suite, 1, ["unknown"], domain_start=171)

    def test_domain_overflow_is_refused_before_output_creation_or_launch(self):
        launch = Mock()
        functions = load_runner_functions(run_verification=launch)
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "must-not-exist"
            args = SimpleNamespace(repeats=10, cases=None, domain=171)
            with self.assertRaisesRegex(RuntimeError, "unique ROS domains"):
                functions["run_suite"](self.suite, self.cells, args, output)
            self.assertFalse(output.exists())
            launch.assert_not_called()
        for domain in (True, -1, 233):
            with self.subTest(domain=domain), self.assertRaises(RuntimeError):
                functions["_full_rows"](self.suite, 1, ["local_small"], domain_start=domain)

    def orphan_fixed_result(self, row, declared):
        case = next(case for case in self.suite["cases"] if case["case_id"] == row["case_id"])
        return dict(schema_version=1, status="aborted", case=deepcopy(case), case_id=case["case_id"],
            workcell=case["workcell"], variant=row["variant"], repeat=row["repeat"], row_index=row["index"],
            suite_sha256=suite_hash(self.suite), policy_name=case["policy_name"], source_sha256="a" * 64,
            policy=dict(schema_version=1, workcell=case["workcell"], case="dual_arm_crossing", **self.suite["policy"]),
            scored_settings=dict(compute_budget_wall_sec=5., max_plan_candidates=row["max_plan_candidates"], goal_tolerance_rad=.003),
            maintenance=[], scored_attempted=True, scored_request_id=declared, scored=None,
            error=dict(type="SyntheticPersistenceError", message="Scored index unavailable"))

    def test_orphan_trace_is_hash_retained_without_becoming_validated_scored_evidence(self):
        functions = load_runner_functions()
        row = functions["_full_rows"](self.suite, 1, ["local_small"], domain_start=171)[0]
        case = self.suite["cases"][0]
        declared = "a" * 32
        fixed = self.orphan_fixed_result(row, declared)
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory)
            atomic_json(output / "fixed-case.json", fixed)
            orphan = output / (declared + ".json")
            atomic_json(orphan, dict(request_id=declared, status="succeeded", note="Synthetic unindexed bytes"))
            verification, collected = functions["_collect_row"](row, case, output, None, suite_hash(self.suite))
            self.assertIsNone(verification)
            self.assertEqual(collected, fixed)
            self.assertTrue(row["unindexed_trace_present"])
            self.assertEqual(Path(row["unindexed_trace_path"]), orphan)
            self.assertEqual(row["unindexed_trace_sha256"], file_sha256(orphan))
            self.assertIsNone(row["trace_path"])
            self.assertIsNone(row["trace_sha256"])
            self.assertIsNone(row["request_id"])
            compact = summarize_row(self.suite, row, fixed_case_raw=(output / "fixed-case.json").read_bytes())
            self.assertEqual(compact["status"], "failed")
            self.assertTrue(compact["attempted"] and compact["delivery_unknown"])
            self.assertFalse(compact["scored"])
            self.assertIsNone(compact["request_id"])
            self.assertEqual(compact["declared_request_id"], declared)

    def test_orphan_trace_symlink_cannot_escape_its_row_directory(self):
        functions = load_runner_functions()
        row = functions["_full_rows"](self.suite, 1, ["local_small"], domain_start=171)[0]
        declared = "b" * 32
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            output = root / "row"
            output.mkdir()
            atomic_json(output / "fixed-case.json", self.orphan_fixed_result(row, declared))
            outside = root / "outside-row.json"
            atomic_json(outside, dict(synthetic=True))
            (output / (declared + ".json")).symlink_to(outside)
            with self.assertRaisesRegex(RuntimeError, "Unindexed trace escapes"):
                functions["_collect_row"](row, self.suite["cases"][0], output, None, suite_hash(self.suite))

    def lifecycle(self, directory, *, preparation_failure=False, malformed_summary=False):
        root = Path(directory) / "workspace"
        (root / "runtime/validation").mkdir(parents=True)
        launches, probes = [], []

        def summarize(suite, row, *, fixed_case_raw=None, trace_raw=None, verification_raw=None):
            if malformed_summary and trace_raw is not None:
                raise ValueError("Synthetic independent summary rejection")
            return dict(index=row["index"], status="not_run" if trace_raw is None else "completed")

        def aggregate(suite, planned_catalog, compact_rows):
            self.assertEqual(len(planned_catalog), len(compact_rows))
            return dict(planned=len(planned_catalog), completed=sum(row["status"] == "completed" for row in compact_rows))

        class Child:
            def __init__(child, profile, progress, case, row, *, output, suite_sha256):
                child.case, child.row, child.output, child.suite_sha256 = case, row, output, suite_sha256
                output.mkdir(parents=True, exist_ok=False)
                progress("fixed_case_scored_request")

        def verification(args, workspace, ros, factory, trigger_type, **kwargs):
            launches.append(args.domain)
            profile = SimpleNamespace(id=args.robot)
            report_path = root / "runtime/validation" / ("synthetic-report-" + str(len(launches)) + ".json")
            kwargs["on_launch"](SimpleNamespace(path=report_path), profile, args, None)
            child = factory(profile, lambda stage: None)
            failed = preparation_failure
            result = dict(schema_version=1, case_id=child.row["case_id"], variant=child.row["variant"],
                repeat=child.row["repeat"], suite_sha256=child.suite_sha256,
                status="aborted" if failed else "completed", maintenance=[], scored=None,
                error=dict(type="SyntheticSetupError", message="Preparation stopped") if failed else None)
            if not failed:
                planning_failed = len(launches) == 1
                request = f'{child.row["index"]+1:032x}'
                trace = dict(schema_version=1, request_id=request,
                    status="failed" if planning_failed else "succeeded",
                    command=dict(strategy="coordinated", target_rad=child.case["target_rad"]))
                trace_path = child.output / (request + ".json")
                atomic_json(trace_path, trace)
                result["scored"] = dict(request_id=request, status=trace["status"], trace_file=trace_path.name,
                    trace_sha256=file_sha256(trace_path), outcome="planning_failed" if planning_failed else "motion_succeeded",
                    failure_kind="budget_exhausted" if planning_failed else None, failure_stage=None)
            atomic_json(child.output / "fixed-case.json", result)
            report = dict(run_id="synthetic-" + str(len(launches)), ros_domain_id=args.domain,
                status="failed" if failed else "passed", motion_checks_passed=not failed,
                physics_checks_passed=True, shutdown_clean=True, launch_return_code=0,
                process_group_gone=True, error=result["error"])
            atomic_json(report_path, report)
            return 1 if failed else 0

        functions = load_runner_functions(ROOT=root, run_verification=verification,
            require_unused_domain=probes.append, check_workcell=lambda cell: {},
            load_workcell=lambda name, **kwargs: SimpleNamespace(id=name),
            summarize_row=summarize, aggregate_results=aggregate, FixedCaseExecution=Child,
            rclpy=SimpleNamespace(), Trigger=object(), diagnostic_environment=lambda *args: None,
            finalize_physics=lambda *args: None)
        args = SimpleNamespace(repeats=1, cases=["local_small"], domain=171,
                               suite=CANONICAL_SUITE, suite_file_sha256=file_sha256(CANONICAL_SUITE))
        output = root / "runtime/validation/synthetic-suite"
        with redirect_stdout(io.StringIO()):
            code = functions["run_suite"](self.suite, self.cells, args, output)
        raw = json.loads((output / "fixed-benchmark.json").read_text())
        return code, raw, launches, probes

    def assert_catalog_preserved(self, raw):
        self.assertEqual(len(raw["rows"]), 20)
        self.assertEqual(len(raw["results"]), 20)
        self.assertEqual(sum(row["selected"] for row in raw["rows"]), 2)
        self.assertTrue(all(row["status"] == "not_run" and row["domain"] is None
                            for row in raw["rows"] if not row["selected"]))
        self.assertIsNone(raw["current_row_id"])
        self.assertIsNotNone(raw["completed_utc"])
        self.assertIn("cpu_count", raw["numeric_host_load_end"])

    def test_failed_preparation_stops_and_retains_every_later_not_run_row(self):
        with tempfile.TemporaryDirectory() as directory:
            code, raw, launches, probes = self.lifecycle(directory, preparation_failure=True)
            self.assert_catalog_preserved(raw)
            self.assertEqual((code, launches, probes), (1, [171], [171]))
            self.assertEqual(raw["status"], "aborted")
            selected = [row for row in raw["rows"] if row["selected"]]
            self.assertEqual([row["status"] for row in selected], ["aborted", "not_run"])
            self.assertTrue(selected[1]["not_run_reason"].startswith("suite_aborted"))
            self.assertIsNone(selected[0]["request_id"])
            self.assertEqual(file_sha256(Path(selected[0]["fixed_case_result_path"])), selected[0]["fixed_case_result_sha256"])
            self.assertEqual(file_sha256(Path(selected[0]["verification_result_path"])), selected[0]["verification_result_sha256"])

    def test_expected_planning_failure_continues_into_fresh_next_row(self):
        with tempfile.TemporaryDirectory() as directory:
            code, raw, launches, probes = self.lifecycle(directory)
            self.assert_catalog_preserved(raw)
            self.assertEqual((code, launches, probes), (0, [171, 172], [171, 172]))
            self.assertEqual(raw["status"], "completed")
            selected = [row for row in raw["rows"] if row["selected"]]
            self.assertEqual([row["status"] for row in selected], ["completed", "completed"])
            self.assertEqual([row["outcome"] for row in selected], ["planning_failed", "motion_succeeded"])
            self.assertEqual(raw["counts"]["planning_failed"], 1)
            self.assertEqual(raw["counts"]["motion_succeeded"], 1)
            for row in selected:
                for path_key, sha_key in (("trace_path", "trace_sha256"),
                                         ("fixed_case_result_path", "fixed_case_result_sha256"),
                                         ("verification_result_path", "verification_result_sha256")):
                    self.assertEqual(file_sha256(Path(row[path_key])), row[sha_key])
                self.assertGreaterEqual(row["outer_wall_sec"], 0)
                self.assertGreaterEqual(row["launch_created_wall_offset_sec"], 0)
                self.assertGreaterEqual(row["node_created_wall_offset_sec"], 0)
                self.assertGreaterEqual(row["unscored_scoring_ready_wall_offset_sec"], 0)
                self.assertIn("cpu_count", row["numeric_host_load_start"])

    def test_malformed_independent_summary_aborts_but_preserves_scored_raw_evidence(self):
        with tempfile.TemporaryDirectory() as directory:
            code, raw, launches, probes = self.lifecycle(directory, malformed_summary=True)
            self.assert_catalog_preserved(raw)
            self.assertEqual((code, launches, probes), (1, [171], [171]))
            selected = [row for row in raw["rows"] if row["selected"]]
            self.assertEqual([row["status"] for row in selected], ["aborted", "not_run"])
            first = selected[0]
            self.assertIn("summary_error", first)
            self.assertIsNotNone(first["request_id"])
            self.assertEqual(first["actual_request_status"], "failed")
            self.assertEqual(file_sha256(Path(first["trace_path"])), first["trace_sha256"])
            self.assertEqual(file_sha256(Path(first["fixed_case_result_path"])), first["fixed_case_result_sha256"])


if __name__ == "__main__":
    unittest.main()
