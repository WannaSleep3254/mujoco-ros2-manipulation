"""Separate W08 declarations, input provenance and honest result aggregation."""
from copy import deepcopy
from contextlib import redirect_stdout
import hashlib
import io
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import tempfile
from types import SimpleNamespace, ModuleType
import unittest
from unittest.mock import patch

from mujoco_ros2_manipulation.workcells.planner_cache_experiment import (
    CONDITIONS, FUNCTIONAL_CASES, COMPUTE_SCOPE, PRIMARY_LATENCY_METRIC,
    canonical, digest, make_experiment, planned_rows, preserve_sources, verify_snapshot)
from mujoco_ros2_manipulation.workcells.planner_cache_results import aggregate_results, summarize_trace, require_validated_cache_hit
from mujoco_ros2_manipulation.paths import ROOT


def cell():
    names = ["left_j"+str(i) for i in range(1, 7)]+["right_j"+str(i) for i in range(1, 7)]
    return SimpleNamespace(id="fr5_ur5e", joints=names, home=[0.]*12,
        instances=[SimpleNamespace(joints=names[:6]), SimpleNamespace(joints=names[6:])])


class ExperimentContractTests(unittest.TestCase):
    def setUp(self):
        self.experiment = make_experiment(cell())

    def test_all_rows_order_and_warm_fill_are_declared_before_execution(self):
        rows = planned_rows(self.experiment)
        self.assertEqual(len(rows), 25)
        self.assertEqual(sum(row["scored"] for row in rows), 12)
        self.assertEqual(sum(row["kind"] == "warm_fill" for row in rows), 3)
        self.assertEqual([row["case_id"] for row in rows if row["kind"] == "functional"], list(FUNCTIONAL_CASES))
        self.assertEqual([row["index"] for row in rows], list(range(25)))
        self.assertEqual(len({row["row_id"] for row in rows}), 25)
        for index, row in enumerate(rows):
            self.assertEqual(row["status"], "not_run")
            if row["kind"] == "performance" and row["condition"] == "rrtconnect_warm":
                self.assertEqual(rows[index-1]["kind"], "warm_fill")
                self.assertEqual(rows[index-1]["repeat"], row["repeat"])

    def test_inputs_are_same_and_conditions_only_change_planner_or_opt_in_cache(self):
        before = deepcopy(cell().__dict__)
        self.assertEqual(self.experiment["case"]["initial_rad"], dict(zip(cell().joints, cell().home)))
        target = self.experiment["case"]["target_rad"]
        self.assertEqual({name for name, value in target.items() if value}, {"left_j1", "right_j1"})
        self.assertEqual(target["left_j1"], .12)
        self.assertEqual(self.experiment["comparison_policy"]["max_plan_candidates"], 1)
        self.assertEqual(self.experiment["comparison_policy"]["compute_budget_wall_sec"], 5.)
        self.assertEqual(cell().__dict__, before)

    def test_paired_order_is_rotated_deterministically_without_outcome_selection(self):
        rows = planned_rows(self.experiment)
        groups = [[row["condition"] for row in rows if row["kind"] == "performance" and row["repeat"] == repeat]
                  for repeat in (1, 2, 3)]
        self.assertEqual(groups[1], groups[0][1:]+groups[0][:1])
        self.assertEqual(groups[2], groups[0][2:]+groups[0][:2])
        self.assertEqual(rows, planned_rows(deepcopy(self.experiment)))

    def test_selecting_mode_preserves_all_unselected_rows_and_excludes_faults_from_scoring(self):
        for mode in ("functional", "performance"):
            rows = planned_rows(self.experiment, mode=mode)
            self.assertEqual(len(rows), 25)
            for row in rows:
                expected = (row["kind"] == "functional") == (mode == "functional")
                self.assertEqual(row["selected"], expected)
                if row["test_only"]:
                    self.assertFalse(row["scored"])
                if not expected:
                    self.assertEqual(row["not_run_reason"], "mode_not_selected")

    def test_invalid_repeats_cell_or_modified_condition_are_refused(self):
        for repeats in (True, 1, 2, 31, 3.1):
            with self.subTest(repeats=repeats), self.assertRaises(ValueError):
                make_experiment(cell(), repeats=repeats)
        invalid = cell()
        invalid.id = "other"
        with self.assertRaises(ValueError):
            make_experiment(invalid)
        changed = deepcopy(self.experiment)
        changed["conditions"][0]["planner_id"] = "other"
        with self.assertRaises(ValueError):
            planned_rows(changed)

    def test_digest_binds_order_settings_and_values(self):
        changed = deepcopy(self.experiment)
        changed["case"]["target_rad"]["left_j1"] += .01
        self.assertNotEqual(digest(changed), digest(self.experiment))
        self.assertEqual(digest(self.experiment), hashlib.sha256(canonical(self.experiment)).hexdigest())

    def test_exact_public_sources_and_generated_model_snapshot_is_new_and_never_overwrites(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)/"workspace"
            paths = ["src/mujoco_ros2_manipulation/workcells/motion_server.py",
                     "src/mujoco_ros2_manipulation/verification/runner.py", "config/workcells/fr5_ur5e.yaml",
                     "runtime/generated/workcells/fr5_ur5e/cell.xml"]
            for name in paths:
                path = root/name
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_bytes(("fixed:"+name).encode())
            output = Path(temporary)/"output"
            output.mkdir()
            result = preserve_sources(output, root, {"model": "pin"})
            self.assertEqual(set(result["files_sha256"]), set(paths))
            verify_snapshot(root, result)
            (root/paths[0]).write_text("later changed source")
            with self.assertRaisesRegex(ValueError, "input changed"):
                verify_snapshot(root, result)
            for name in paths:
                raw = (output/"input-snapshot"/name).read_bytes()
                self.assertEqual(hashlib.sha256(raw).hexdigest(), result["files_sha256"][name])
            with self.assertRaises(FileExistsError):
                preserve_sources(output, root, {})

    def test_source_snapshot_refuses_symlinked_source_file(self):
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            source = root/"src/mujoco_ros2_manipulation/workcells/unsafe.py"
            source.parent.mkdir(parents=True)
            source.symlink_to("/etc/hosts")
            (root/"runtime/generated/workcells/fr5_ur5e").mkdir(parents=True)
            with self.assertRaisesRegex(ValueError, "unsafe"):
                preserve_sources(root, root, {})


class ResultEvidenceTests(unittest.TestCase):
    def setUp(self):
        self.experiment = make_experiment(cell())
        self.rows = planned_rows(self.experiment)

    def trace(self, row):
        condition = next(item for item in CONDITIONS if item["id"] == row["condition"])
        cache = dict(enabled=condition["cache"], eligible=condition["cache"],
            reason="hit" if condition["expected_cache"] == "hit" else "cold",
            lookup=dict(hit=condition["expected_cache"] == "hit", full_revalidation_required=True),
            full_revalidation_wall_sec=.2, store=None)
        return dict(request_id="1"*32, mode="plan_only", status="succeeded", dispatched_goals=0,
            command=dict(target_rad=deepcopy(self.experiment["case"]["target_rad"]), strategy="coordinated"),
            initial_state_rad=deepcopy(self.experiment["case"]["initial_rad"]),
            planning_configuration=dict(requested_planner_id=condition["planner_id"],
                trajectory_cache_enabled=condition["cache"], max_plan_candidates=1,
                compute_budget_wall_sec=5., goal_tolerance_rad=.003,
                budget_includes_review=True, compute_scope=COMPUTE_SCOPE), trajectory_cache=cache,
            events=[dict(name="computation_started", wall_sec=1.), dict(name="computation_done", wall_sec=2.),
                    dict(name="plan_ready", wall_sec=2.1)],
            metrics=dict(received_to_terminal_wall_sec=2.2),
            plans=[] if condition["expected_cache"] == "hit" else [dict(requested_planner_id=condition["planner_id"],
                selected_planner_id=None, wall_sec=.7)],
            planned_clearance=dict(complete=True, joint_total_variation_rad=.24, minimum=dict(distance_m=.5)))

    def test_full_wall_includes_lookup_validation_and_fallback_not_just_planning_rpc(self):
        row = self.rows[0]
        summary = summarize_trace(self.experiment, row, self.trace(row))
        self.assertEqual(summary["computation_wall_sec"], 1.)
        self.assertEqual(summary["received_to_plan_ready_wall_sec"], 2.1)
        self.assertEqual(summary["planning_rpc_wall_sec"], .7)
        self.assertEqual(summary["selected_planner_id"], None)
        self.assertEqual(summary["path_quality"]["joint_total_variation_rad"], .24)

    def test_planning_failure_keeps_raw_error_and_elapsed_cost(self):
        row = self.rows[0]
        trace = self.trace(row)
        trace.update(status="failed", events=[], compute_budget=dict(elapsed_wall_sec=5.1),
            error=dict(type="PlanningBudgetExceeded"), failure_kind="budget_exhausted", planned_clearance=None)
        summary = summarize_trace(self.experiment, row, trace)
        self.assertEqual(summary["computation_wall_sec"], 5.1)
        self.assertEqual(summary["status"], "failed")
        self.assertEqual(summary["error"]["type"], "PlanningBudgetExceeded")
        self.assertIsNone(summary["path_quality"])
        self.assertIsNone(summary["received_to_plan_ready_wall_sec"])

    def test_success_requires_the_same_plan_ready_endpoint_after_computation(self):
        row = self.rows[0]
        for ready in (None, .5):
            trace = self.trace(row)
            trace["events"] = [item for item in trace["events"] if item["name"] != "plan_ready"]
            if ready is not None:
                trace["events"].append(dict(name="plan_ready", wall_sec=ready))
            with self.subTest(ready=ready), self.assertRaisesRegex(ValueError, "common plan_ready endpoint"):
                summarize_trace(self.experiment, row, trace)

    def test_wrong_goal_dispatch_config_or_claimed_selected_planner_are_not_scored(self):
        row = self.rows[0]
        changes = (lambda t: t.update(dispatched_goals=2),
            lambda t: t["command"]["target_rad"].update(left_j1=.9),
            lambda t: t["planning_configuration"].update(max_plan_candidates=12),
            lambda t: t["planning_configuration"].update(compute_budget_wall_sec=30.),
            lambda t: t["planning_configuration"].update(trajectory_cache_enabled=True),
            lambda t: t["planning_configuration"].update(budget_includes_review=False),
            lambda t: t["planning_configuration"].update(compute_scope="legacy_scope"),
            lambda t: t["plans"][0].update(selected_planner_id="RRTConnect"),
            lambda t: t.update(status="cancelled"),
            lambda t: t["initial_state_rad"].update(left_j1=.01))
        for change in changes:
            trace = self.trace(row)
            change(trace)
            with self.subTest(trace=trace), self.assertRaises(ValueError):
                summarize_trace(self.experiment, row, trace)

    def test_warm_hit_requires_full_validation_without_store_or_fallback(self):
        row = next(item for item in self.rows if item["kind"] == "performance" and item["condition"] == "rrtconnect_warm")
        self.assertTrue(summarize_trace(self.experiment, row, self.trace(row))["trajectory_cache"]["lookup"]["hit"])
        for change in (dict(full_revalidation_wall_sec=None), dict(store={"stored": True}),
                       dict(fallback=True), dict(reason="validation_rejected_fallback")):
            trace = self.trace(row)
            trace["trajectory_cache"].update(change)
            with self.subTest(change=change), self.assertRaises(ValueError):
                summarize_trace(self.experiment, row, trace)

    def test_warm_validation_rejection_with_fresh_fallback_is_measured_separately_from_reuse(self):
        row = next(item for item in self.rows if item["kind"] == "performance" and item["condition"] == "rrtconnect_warm")
        trace = self.trace(row)
        trace["plans"] = [dict(requested_planner_id="RRTConnectkConfigDefault", selected_planner_id=None, wall_sec=.8)]
        trace["trajectory_cache"].update(fallback=True, reason="validation_rejected_fallback",
            rejected_candidate=dict(type="CacheDynamicsRejected"), eviction=dict(evicted=True), fallback_planning_wall_sec=.8,
            store=dict(stored=True))
        result = summarize_trace(self.experiment, row, trace)
        self.assertTrue(result["cache_lookup_hit"])
        self.assertFalse(result["cached_path_reused"])
        self.assertTrue(result["cache_validation_fallback"])
        self.assertEqual(result["cache_fallback_planning_wall_sec"], .8)

    def test_functional_reuse_rejects_fresh_fallback_even_when_the_lookup_hit(self):
        row = next(item for item in self.rows if item["kind"] == "performance" and item["condition"] == "rrtconnect_warm")
        trace = self.trace(row)
        trace["moveit_safety_samples"] = 10
        require_validated_cache_hit(trace)
        for change in (dict(fallback=True, reason="validation_rejected_fallback"),
                       dict(full_revalidation_wall_sec=None), dict(store={"stored": True}),
                       dict(reason="cold")):
            changed = deepcopy(trace)
            changed["trajectory_cache"].update(change)
            with self.subTest(change=change), self.assertRaisesRegex(ValueError, "Functional reuse"):
                require_validated_cache_hit(changed)
        trace["moveit_safety_samples"] = 0
        with self.assertRaises(ValueError):
            require_validated_cache_hit(trace)

    def completed(self, repeats=3):
        experiment = make_experiment(cell(), repeats=repeats)
        rows = planned_rows(experiment)
        for row in rows:
            if row["kind"] == "performance":
                row["status"] = "completed"
                row["summary"] = dict(measured=True, status="succeeded", computation_wall_sec=float(row["repeat"]),
                    received_to_plan_ready_wall_sec=float(row["repeat"])+.3,
                    planning_rpc_wall_sec=.5, received_to_terminal_wall_sec=float(row["repeat"])+1.)
        return experiment, rows

    def test_three_repeats_keep_individuals_median_maximum_without_p95(self):
        experiment, rows = self.completed()
        result = aggregate_results(experiment, rows)
        self.assertFalse(result["p95_published"])
        self.assertEqual(result["primary_latency_metric"], PRIMARY_LATENCY_METRIC)
        self.assertEqual(len(result["conditions"]), 4)
        for item in result["conditions"]:
            self.assertEqual(len(item["samples"]), 3)
            self.assertEqual(item["statistics"]["computation_wall_sec"], dict(count=3, median=2., maximum=3.))

    def test_p95_is_withheld_for_every_condition_until_all_have_twenty_valid_samples(self):
        experiment, rows = self.completed(20)
        result = aggregate_results(experiment, rows)
        self.assertTrue(result["p95_published"])
        self.assertTrue(all(item["statistics"]["computation_wall_sec"]["p95"] == 19. for item in result["conditions"]))
        next(row for row in rows if row["kind"] == "performance")["status"] = "aborted"
        result = aggregate_results(experiment, rows)
        self.assertFalse(result["p95_published"])
        self.assertTrue(all("p95" not in item["statistics"]["computation_wall_sec"] for item in result["conditions"]))

    def test_twenty_computation_samples_without_twenty_plan_ready_samples_do_not_publish_p95(self):
        experiment, rows = self.completed(20)
        row = next(item for item in rows if item["kind"] == "performance")
        row["summary"].update(status="failed", received_to_plan_ready_wall_sec=None)
        result = aggregate_results(experiment, rows)
        self.assertFalse(result["p95_published"])
        self.assertEqual(result["conditions"][0]["failed"], 1)
        self.assertEqual(result["conditions"][0]["statistics"][PRIMARY_LATENCY_METRIC]["count"], 19)

    def test_failures_count_and_fault_or_warm_fill_rows_never_enter_performance_statistics(self):
        experiment, rows = self.completed()
        scored = next(row for row in rows if row["kind"] == "performance")
        scored["summary"].update(status="failed", computation_wall_sec=7., error={"type": "PlanningBudgetExceeded"})
        for row in rows:
            if row["kind"] != "performance":
                row.update(status="completed", summary=dict(measured=True, status="succeeded", computation_wall_sec=999.))
        result = aggregate_results(experiment, rows)
        condition = result["conditions"][0]
        self.assertEqual(condition["measured"], 3)
        self.assertEqual(condition["failed"], 1)
        self.assertEqual(condition["statistics"]["computation_wall_sec"]["maximum"], 7.)


class RunnerBoundaryTests(unittest.TestCase):
    def setUp(self):
        # Stub only ROS-facing base classes, then exercise the real runner's
        # preparation/injection code. No ROS node, domain or process is created.
        self.base_calls = []
        owner = self
        class Base:
            def configure_planning(self, **options):
                owner.base_calls.append(options)
                self.planner_id = options["planner_id"]
                self.planning_configuration = deepcopy(options)
                self.trajectory_cache = None
            def _validate_cached_anchor(self, curves, initial, target):
                owner.base_calls.append("base_validator")
                return "base-result"
        class Review(Base):
            pass
        server = ModuleType("mujoco_ros2_manipulation.workcells.motion_server")
        server.MotionCommandServer = Base
        server.CommandCancelled = type("CommandCancelled", (RuntimeError,), {})
        server.MotionEnvironmentChanged = type("MotionEnvironmentChanged", (RuntimeError,), {})
        review = ModuleType("mujoco_ros2_manipulation.workcells.verify_motion_review")
        review.MotionReviewVerification = Review
        verify = ModuleType("mujoco_ros2_manipulation.workcells.verify")
        def require(condition, message):
            if not condition:
                raise RuntimeError(message)
        verify.require = require
        messages = ModuleType("std_msgs.msg")
        messages.String = lambda **value: value
        self.modules = {server.__name__: server, review.__name__: review,
                        verify.__name__: verify, "std_msgs.msg": messages}
        from mujoco_ros2_manipulation.workcells.benchmark_planner_cache import node_class
        with patch.dict(sys.modules, self.modules):
            cls = node_class()
        self.node = object.__new__(cls)
        self.node.save_audit = lambda: None
        self.node.quiet = lambda: None
        self.node.audit = {"preparation": []}
        self.node.joint_names = ["a", "b"]
        self.node.planner_id = "RRTkConfigDefault"
        self.node.planning_configuration = {"planner_id": "RRTkConfigDefault", "preserve": True}
        self.cache = object()
        self.node.trajectory_cache = self.cache
        self.node.compute_budget_sec, self.node.max_plan_candidates, self.node.goal_tolerance_rad = 5., 1, .003
        self.node.commanded = {"a": 0., "b": 0.}
        self.actual = {"a": -.004, "b": -.004}
        self.node.state = lambda: dict(self.actual)

    def test_preparation_uses_normal_commands_and_feedback_without_overwriting_observations(self):
        requests = []
        def submit(kind, target, *, role):
            requests.append((kind, dict(target), role))
            self.node.commanded = dict(target)
            self.actual = {name: value-.004 for name, value in target.items()}
            return dict(request_id=str(len(requests)), status="succeeded")
        self.node._trace_request = submit
        self.node.drive_initial({"a": 0., "b": 0.}, "test")
        self.assertEqual([item[0] for item in requests], ["command", "command"])
        self.assertEqual(requests[0][1], {"a": 0., "b": 0.})
        self.assertEqual(requests[1][1], {"a": .004, "b": .004})
        stage = self.node.audit["preparation"][0]
        self.assertEqual(stage["initial_actual_rad"], {"a": -.004, "b": -.004})
        self.assertLessEqual(stage["actual_max_error_rad"], .0015)
        self.assertEqual(self.node.planner_id, "RRTkConfigDefault")
        self.assertIs(self.node.trajectory_cache, self.cache)
        self.assertEqual(self.node.planning_configuration, {"planner_id": "RRTkConfigDefault", "preserve": True})

    def test_unsafe_preparation_aborts_without_scored_retry_and_restores_settings(self):
        self.actual = {"a": -.1, "b": -.1}
        requests = []
        def failed_response(kind, target, *, role):
            requests.append(kind)
            return dict(request_id="one", status="succeeded")
        self.node._trace_request = failed_response
        with self.assertRaisesRegex(RuntimeError, "exceeds 0.03rad"):
            self.node.drive_initial({"a": 0., "b": 0.}, "negative")
        self.assertEqual(requests, ["command"])
        self.assertEqual(self.node.audit["preparation"][0]["status"], "failed")
        self.assertIs(self.node.trajectory_cache, self.cache)
        self.assertEqual(self.node.compute_budget_sec, 5.)

    def test_cache_hit_faults_propagate_without_calling_normal_validator_or_fallback(self):
        for injection, expected in (("cancel", "CommandCancelled"), ("budget", "PlanningBudgetExceeded"),
                                    ("identity", "MotionEnvironmentChanged")):
            self.node.inject_validation_error = injection
            trace = self.node.active = SimpleNamespace(data={"trajectory_cache": {"lookup": {"hit": True}}})
            with self.subTest(injection=injection), self.assertRaises(RuntimeError) as raised:
                self.node._validate_cached_anchor([], {}, {})
            self.assertEqual(type(raised.exception).__name__, expected)
            self.assertIsNone(self.node.inject_validation_error)
            self.assertEqual(trace.data["experiment_test_only_injection"], injection)
        self.assertEqual(self.base_calls, [])

    def test_normal_cached_anchor_delegates_to_complete_production_validator(self):
        self.node.inject_validation_error = None
        self.assertEqual(self.node._validate_cached_anchor([], {}, {}), "base-result")
        self.assertEqual(self.base_calls, ["base_validator"])

    def test_prelaunch_import_failure_preserves_rows_and_prints_only_compact_result_paths(self):
        from mujoco_ros2_manipulation.workcells import benchmark_planner_cache as runner
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary)
            (root/"runtime").mkdir()
            output = root/"runtime/new-experiment"
            stdout = io.StringIO()
            with patch.object(runner, "ROOT", root), \
                    patch("mujoco_ros2_manipulation.workcells.config.load_workcell", return_value=cell()), \
                    patch("mujoco_ros2_manipulation.workcells.integrity.check_workcell", return_value={}), \
                    patch.object(runner, "preserve_sources", return_value={"files_sha256": {}}), \
                    patch.object(runner.subprocess, "check_output", return_value="local-commit\n"), \
                    patch.dict(sys.modules, {"rclpy": None}), redirect_stdout(stdout):
                code = runner.main(["--domain", "228", "--output", str(output)])
            self.assertEqual(code, 1)
            summary = json.loads(stdout.getvalue())
            self.assertEqual(summary["status"], "aborted")
            self.assertEqual(summary["error_type"], "ModuleNotFoundError")
            self.assertEqual(summary["error_path"], str(output/"experiment-run.json"))
            self.assertNotIn("rows", summary)
            self.assertNotIn("trace", summary)
            raw = json.loads((output/"experiment-run.json").read_text())
            self.assertEqual(len(raw["rows"]), 25)
            self.assertTrue(all(row["status"] == "not_run" for row in raw["rows"]))
            self.assertTrue(all(row["not_run_reason"] for row in raw["rows"] if row["selected"]))


if __name__ == "__main__":
    unittest.main()
