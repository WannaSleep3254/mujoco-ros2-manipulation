"""Fixed result evidence stays hash-bound and includes failures and unrun rows."""
from copy import deepcopy
import hashlib
import json
import unittest

from mujoco_ros2_manipulation.workcells.config import load_workcell
from mujoco_ros2_manipulation.workcells.fixed_suite import make_suite, planned_rows, suite_hash
from mujoco_ros2_manipulation.workcells.fixed_results import POLICY_NUMBERS, summarize_row, aggregate_results


def encoded(value):
    return (json.dumps(value, allow_nan=True, sort_keys=True) + "\n").encode()


def digest(raw):
    return hashlib.sha256(raw).hexdigest()


class FixedResultsTests(unittest.TestCase):
    def setUp(self):
        self.suite = make_suite(load_workcell(), load_workcell("fr5_ur5e_eval_post"))
        self.suite["policy"] = deepcopy(POLICY_NUMBERS)
        self.catalog = planned_rows(self.suite, case_ids=["local_small", "invalid_goal"])

    def row(self, case="local_small", variant="single"):
        return next(row for row in self.catalog if row["case_id"] == case and row["variant"] == variant)

    def artifacts(self, row=None, outcome="motion_succeeded", *, verification_failed=False, request_id=None):
        row = self.row() if row is None else row
        case = next(case for case in self.suite["cases"] if case["case_id"] == row["case_id"])
        request = request_id or f"{row['index']+1:032x}"
        initial = deepcopy(case["initial_rad"])
        curve = dict(names=list(initial), points=[dict(t=0, p=list(initial.values()), v=[], a=[]),
            dict(t=2, p=list(case["target_rad"].values()), v=[], a=[])])
        native = dict(complete=True, samples=201, minimum=dict(time_sec=1., distance_m=.04, state_rad=initial),
            duration_sec=2., sample_dt_max_sec=.01, joint_total_variation_rad=1.5,
            tip_frame_path_m=dict(fr5=.2, ur5e=.3))
        observed = dict(complete=True, samples=101, minimum=dict(time_sec=10., distance_m=.035, state_rad=initial),
            duration_sec=2.1, sample_dt_max_sec=.02, joint_total_variation_rad=1.51,
            tip_frame_path_m=dict(fr5=.21, ur5e=.31))
        command = dict(schema_version=1, request_id=request, strategy="coordinated", target_rad=deepcopy(case["target_rad"]))
        trace = dict(schema_version=1, request_id=request, command=command, initial_state_rad=initial,
            status="succeeded", failure_kind=None, failure_stage=None, error=None, dispatched_goals=2,
            action_results=[dict(status=4, error_code=0), dict(status=4, error_code=0)],
            contact_guard=dict(latched=False, phase="ready"), planned_trajectories=[curve],
            planned_clearance=native, observed_clearance=observed, planned_candidate_attempt=1,
            compute_budget=dict(limit_wall_sec=5., elapsed_wall_sec=.8, exceeded=False),
            plans=[dict(group="both_arms", wall_sec=.2, server_planning_sec=.1)],
            plan_selection=dict(max_candidates=row["max_plan_candidates"], attempted=1, selected_attempt=1, status="selected",
                candidates=[dict(attempt=1, accepted=True, proposal_kind="coordinated_rrt", curve=curve, native=native)]),
            events=[dict(name="received", wall_sec=0), dict(name="first_motion", wall_sec=1.),
                    dict(name="settled", wall_sec=3.), dict(name="terminal", wall_sec=3.1)],
            spans=[dict(name="settling", duration_wall_sec=.2)],
            metrics=dict(received_to_first_motion_wall_sec=1., received_to_actions_done_wall_sec=2.8,
                received_to_settled_wall_sec=3., received_to_terminal_wall_sec=3.1))
        if outcome != "motion_succeeded":
            trace.update(status="failed", dispatched_goals=0, action_results=[], observed_clearance=None,
                failure_kind="invalid_goal" if outcome == "expected_invalid_goal" else "budget_exhausted",
                failure_stage="endpoint_validity" if outcome == "expected_invalid_goal" else "spatial_candidate_search",
                error=dict(type="InvalidMotionEndpoint" if outcome == "expected_invalid_goal" else "PlanningBudgetExceeded", message="No dispatch"))
            trace["metrics"].update(received_to_first_motion_wall_sec=None, received_to_actions_done_wall_sec=None,
                received_to_settled_wall_sec=None, received_to_terminal_wall_sec=5.01)
            trace["events"] = [dict(name="received", wall_sec=0), dict(name="terminal", wall_sec=5.01)]
            trace["spans"] = []
            if outcome == "expected_invalid_goal":
                trace.pop("plan_selection")
                trace.pop("planned_clearance")
                trace.pop("planned_trajectories")
                trace["plans"] = []
            else:
                trace["compute_budget"].update(elapsed_wall_sec=5.01, exceeded=True)
                trace.pop("planned_clearance")
                trace["plan_selection"].update(selected_attempt=None, status="interrupted")
                candidate = trace["plan_selection"]["candidates"][0]
                candidate.update(accepted=False, status="interrupted",
                    native_progress=dict(samples=20, minimum=dict(distance_m=.029, state_rad=initial)))
                candidate.pop("native")
        trace_raw = encoded(trace)
        scored = dict(role="scored", request_id=request, status=trace["status"], trace_file=request+".json",
            trace_sha256=digest(trace_raw), command=deepcopy(command), initial_state_rad=deepcopy(initial), outcome=outcome)
        fixed = dict(schema_version=1, status="completed", case=deepcopy(case), case_id=case["case_id"],
            workcell=case["workcell"], variant=row["variant"], repeat=row["repeat"], row_index=row["index"],
            suite_sha256=suite_hash(self.suite), policy_name=case["policy_name"], source_sha256="d"*64,
            policy=dict(schema_version=1, workcell=case["workcell"],
                case="eval_post" if case["workcell"].endswith("eval_post") else "dual_arm_crossing", **POLICY_NUMBERS),
            scored_settings=dict(compute_budget_wall_sec=5., max_plan_candidates=row["max_plan_candidates"], goal_tolerance_rad=.003),
            scored_attempted=True, scored_request_id=request, scored=scored, error=None)
        fixed_raw = encoded(fixed)
        run_id = "test-run-" + str(row["index"])
        verification = dict(schema_version=2, run_id=run_id, status="failed" if verification_failed else "passed",
            robot_profile_sha256=self.suite["workcell_fingerprints"][case["workcell"]],
            model_manifest_sha256="b"*64, physics_report_sha256="c"*64,
            motion_checks_passed=True, physics_checks_passed=True, shutdown_clean=not verification_failed,
            process_group_gone=True, launch_return_code=0, fixed_case_result_sha256=digest(fixed_raw),
            error=dict(type="ShutdownError", message="Cleanup failed") if verification_failed else None,
            physics=dict(complete=True, run_id=run_id, contact_steps=0, invalid_values=0, steps=1000))
        return fixed, trace, verification

    def summarize(self, row=None, **options):
        row = self.row() if row is None else row
        fixed, trace, verification = self.artifacts(row, **options)
        return summarize_row(self.suite, row, fixed_case_raw=encoded(fixed), trace_raw=encoded(trace), verification_raw=encoded(verification))

    def test_completed_motion_metrics_and_original_hashes_are_preserved(self):
        row = self.row()
        fixed, trace, verification = self.artifacts(row)
        raw = dict(fixed_case_raw=encoded(fixed), trace_raw=encoded(trace), verification_raw=encoded(verification))
        result = summarize_row(self.suite, row, **raw)
        self.assertEqual(result["status"], "completed")
        self.assertTrue(result["attempted"] and result["scored"])
        self.assertEqual(result["source_sha256"], dict(fixed_case=digest(raw["fixed_case_raw"]),
            trace=digest(raw["trace_raw"]), verification=digest(raw["verification_raw"])))
        self.assertEqual(result["metrics"]["compute_wall_sec"], .8)
        self.assertEqual(result["metrics"]["planning_rpc_recorded_calls"], 1)
        self.assertEqual(result["metrics"]["planning_rpc_wall_sec"], .2)
        self.assertEqual(result["metrics"]["candidate_count"], 1)
        self.assertEqual(result["metrics"]["planned"]["joint_travel_rad"], 1.5)
        self.assertEqual(result["metrics"]["observed"]["tip_frame_path_m"], dict(fr5=.21, ur5e=.31))
        self.assertEqual(result["metrics"]["settling_duration_wall_sec"], .2)
        self.assertEqual(result["metrics"]["candidate_duration_sec"], 2)
        result["metrics"]["observed"]["tip_frame_path_m"]["fr5"] = 99
        self.assertEqual(trace["observed_clearance"]["tip_frame_path_m"]["fr5"], .21)

    def test_planning_failure_keeps_partial_minimum_separate_from_full_path_metric(self):
        result = self.summarize(outcome="planning_failed")
        self.assertEqual((result["status"], result["outcome"]), ("completed", "planning_failed"))
        self.assertTrue(result["metrics"]["budget_exceeded"])
        self.assertEqual(result["metrics"]["planned"]["status"], "interrupted")
        self.assertEqual(result["metrics"]["planned"]["prefix_minimum_distance_m"], .029)
        self.assertIsNone(result["metrics"]["planned"]["minimum_distance_m"])
        self.assertIsNone(result["metrics"]["planned"]["joint_travel_rad"])
        self.assertEqual(result["metrics"]["candidate_duration_sec"], 2)
        self.assertEqual(result["failure_kind"], "budget_exhausted")

    def test_expected_invalid_goal_is_a_completed_rejection_with_zero_rpcs(self):
        result = self.summarize(self.row("invalid_goal"), outcome="expected_invalid_goal")
        self.assertEqual((result["status"], result["outcome"]), ("completed", "expected_invalid_goal"))
        self.assertEqual(result["metrics"]["candidate_count"], 0)
        self.assertEqual(result["metrics"]["planning_rpc_recorded_calls"], 0)
        self.assertIsNone(result["metrics"]["first_motion_wall_sec"])

    def test_cleanup_failure_retains_actual_scored_success_metrics_but_is_not_completed(self):
        result = self.summarize(verification_failed=True)
        self.assertEqual(result["status"], "failed")
        self.assertEqual(result["request_status"], "succeeded")
        self.assertEqual(result["outcome"], "motion_succeeded")
        self.assertEqual(result["metrics"]["planned"]["joint_travel_rad"], 1.5)
        self.assertEqual(result["errors"][-1]["source"], "verification")

    def test_preparation_failure_and_attempt_without_trace_are_distinct(self):
        for attempted in (False, True):
            with self.subTest(attempted=attempted):
                fixed, _, verification = self.artifacts()
                fixed.update(status="aborted", scored=None, scored_attempted=attempted,
                    scored_request_id="a"*32 if attempted else None,
                    error=dict(type="RuntimeError", message="Transport failure" if attempted else "Preparation failed"))
                verification.update(status="failed", motion_checks_passed=False, fixed_case_result_sha256=digest(encoded(fixed)))
                result = summarize_row(self.suite, self.row(), fixed_case_raw=encoded(fixed), verification_raw=encoded(verification))
                self.assertEqual(result["status"], "failed" if attempted else "not_run")
                self.assertEqual(result["attempted"], attempted)
                self.assertFalse(result["scored"])
                self.assertEqual(result["delivery_unknown"], attempted)
                self.assertIsNone(result["source_sha256"]["trace"])
                self.assertIsNone(result["request_id"])
                self.assertEqual(result["declared_request_id"], "a"*32 if attempted else None)

    def test_source_hash_goal_suite_policy_and_nonfinite_tampering_are_rejected(self):
        mutations = [
            lambda f,t,v: f["scored"].update(trace_sha256="0"*64),
            lambda f,t,v: f.update(suite_sha256="0"*64),
            lambda f,t,v: f["policy"].update(sample_dt_sec=.02),
            lambda f,t,v: v.update(robot_profile_sha256="0"*64),
            lambda f,t,v: t["command"]["target_rad"].update({next(iter(t["initial_state_rad"])):99}),
            lambda f,t,v: t["metrics"].update(received_to_first_motion_wall_sec=float("nan")),
        ]
        for index, mutate in enumerate(mutations):
            fixed, trace, verification = self.artifacts()
            mutate(fixed, trace, verification)
            with self.subTest(index=index), self.assertRaises(ValueError):
                summarize_row(self.suite, self.row(), fixed_case_raw=encoded(fixed), trace_raw=encoded(trace), verification_raw=encoded(verification))
        with self.assertRaisesRegex(ValueError, "Duplicate"):
            summarize_row(self.suite, self.row(), fixed_case_raw=b'{"schema_version":1,"schema_version":1}')

    def test_domain_mismatch_and_injected_scored_source_are_rejected(self):
        row = dict(self.row(), domain=171)
        fixed, trace, verification = self.artifacts(row)
        verification["ros_domain_id"] = 172
        with self.assertRaisesRegex(ValueError, "domain mismatch"):
            summarize_row(self.suite, row, fixed_case_raw=encoded(fixed), trace_raw=encoded(trace), verification_raw=encoded(verification))
        fixed, trace, verification = self.artifacts()
        trace["verification_injection"] = dict(mode="withhold_one_success")
        fixed["scored"]["trace_sha256"] = digest(encoded(trace))
        verification["fixed_case_result_sha256"] = digest(encoded(fixed))
        with self.assertRaisesRegex(ValueError, "substitution/injection"):
            summarize_row(self.suite, self.row(), fixed_case_raw=encoded(fixed), trace_raw=encoded(trace), verification_raw=encoded(verification))

    def test_full_catalog_aggregation_separates_expected_invalid_and_keeps_unrun_cases(self):
        rows = [summarize_row(self.suite, row) for row in self.catalog]
        success = self.row(variant="bounded")
        failed_plan = self.row()
        invalid = self.row("invalid_goal")
        rows[success["index"]] = self.summarize(success)
        rows[failed_plan["index"]] = self.summarize(failed_plan, outcome="planning_failed")
        rows[invalid["index"]] = self.summarize(invalid, outcome="expected_invalid_goal")
        result = aggregate_results(self.suite, self.catalog, rows)
        self.assertEqual(result["all_rows"]["planned"], 20)
        self.assertEqual(result["all_rows"]["completed"], 3)
        self.assertEqual(result["all_rows"]["not_run"], 17)
        self.assertEqual(result["valid_goal"]["motion_denominators"], dict(all_planned=18, selected=2, scored=2))
        self.assertEqual(result["all_rows"]["motion_denominators"], dict(all_planned=18, selected=2, scored=2))
        self.assertEqual(result["valid_goal"]["motion_success_rate_selected"], .5)
        self.assertEqual(result["expected_invalid_goal"]["expected_invalid_rejection_rate_selected"], .5)
        self.assertEqual(result["valid_goal"]["compute_all_scored"]["count"], 2)
        self.assertEqual(result["by_case"]["crossing"]["not_run"], 2)

    def test_zero_selected_or_scored_denominators_are_null_and_no_rows_are_dropped(self):
        catalog = planned_rows(self.suite, case_ids=[])
        rows = [summarize_row(self.suite, row) for row in catalog]
        result = aggregate_results(self.suite, catalog, rows)
        self.assertEqual(result["all_rows"]["not_run"], 20)
        self.assertIsNone(result["valid_goal"]["motion_success_rate_selected"])
        self.assertIsNone(result["valid_goal"]["motion_success_rate_scored"])
        self.assertIsNone(result["expected_invalid_goal"]["expected_invalid_rejection_rate_selected"])

    def test_catalog_drop_reorder_duplicate_and_suite_rewrite_are_rejected(self):
        rows = [summarize_row(self.suite, row) for row in self.catalog]
        for changed in (rows[:-1], [rows[1], rows[0], *rows[2:]], [rows[0], rows[0], *rows[2:]]):
            with self.subTest(kind=len(changed)), self.assertRaises(ValueError):
                aggregate_results(self.suite, self.catalog, changed)
        suite = deepcopy(self.suite)
        suite["cases"][0]["label"] += " changed"
        with self.assertRaisesRegex(ValueError, "identity mismatch"):
            aggregate_results(suite, self.catalog, rows)

    def test_same_request_cannot_be_reused_for_two_fixed_rows(self):
        rows = [summarize_row(self.suite, row) for row in self.catalog]
        for variant in ("single", "bounded"):
            row = self.row(variant=variant)
            rows[row["index"]] = self.summarize(row, request_id="a"*32)
        with self.assertRaisesRegex(ValueError, "reused across fixed rows"):
            aggregate_results(self.suite, self.catalog, rows)


    def test_rebound_hashes_do_not_hide_semantically_malformed_success_or_cancellation(self):
        for problem in ("boolean_cap", "missing_actions", "cancelled_as_invalid", "missing_report_source"):
            with self.subTest(problem=problem):
                row = self.row("invalid_goal") if problem == "cancelled_as_invalid" else self.row()
                fixed, trace, verification = self.artifacts(row, outcome="expected_invalid_goal" if problem == "cancelled_as_invalid" else "motion_succeeded")
                if problem == "boolean_cap":
                    trace["plan_selection"]["max_candidates"] = True
                elif problem == "missing_actions":
                    trace["action_results"] = []
                elif problem == "cancelled_as_invalid":
                    trace["status"] = fixed["scored"]["status"] = "cancelled"
                else:
                    verification.pop("model_manifest_sha256")
                fixed["scored"]["trace_sha256"] = digest(encoded(trace))
                verification["fixed_case_result_sha256"] = digest(encoded(fixed))
                with self.assertRaises(ValueError):
                    summarize_row(self.suite, row, fixed_case_raw=encoded(fixed), trace_raw=encoded(trace), verification_raw=encoded(verification))

    def test_valid_goal_endpoint_failure_is_not_an_ordinary_planning_failure(self):
        fixed, trace, verification = self.artifacts(outcome="planning_failed")
        trace["failure_kind"] = "invalid_goal"
        fixed["scored"]["trace_sha256"] = digest(encoded(trace))
        verification["fixed_case_result_sha256"] = digest(encoded(fixed))
        with self.assertRaisesRegex(ValueError, "outcome differs"):
            summarize_row(self.suite, self.row(), fixed_case_raw=encoded(fixed), trace_raw=encoded(trace), verification_raw=encoded(verification))


    def test_completed_verification_requires_actual_zero_launch_exit(self):
        for code in (None, True, 1, -2):
            with self.subTest(code=code):
                fixed, trace, verification = self.artifacts()
                verification["launch_return_code"] = code
                with self.assertRaisesRegex(ValueError, "launch return code"):
                    summarize_row(self.suite, self.row(), fixed_case_raw=encoded(fixed), trace_raw=encoded(trace), verification_raw=encoded(verification))
