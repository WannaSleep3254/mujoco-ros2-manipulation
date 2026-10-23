"""Public fixed-evaluation projections preserve failures and reject private data."""
from copy import deepcopy
import hashlib
import json
import unittest

from mujoco_ros2_manipulation.workcells.config import load_workcell
from mujoco_ros2_manipulation.workcells.fixed_results import summarize_row, aggregate_results
from mujoco_ros2_manipulation.workcells.fixed_suite import make_suite, planned_rows, suite_hash
from mujoco_ros2_manipulation.workcells.export_fixed_motion import export_fixed_motion, SOURCE_KEYS, CODE_FILES, IMPLEMENTATION_FILES


def encoded(value):
    return (json.dumps(value, sort_keys=True, allow_nan=False) + "\n").encode()


def digest(value):
    return hashlib.sha256(value).hexdigest()


class FixedMotionExportTests(unittest.TestCase):
    def setUp(self):
        self.suite = make_suite(load_workcell(), load_workcell("fr5_ur5e_eval_post"))
        self.catalog = planned_rows(self.suite, case_ids=["local_small", "invalid_goal"])
        ordinal = 0
        for row in self.catalog:
            case = next(case for case in self.suite["cases"] if case["case_id"] == row["case_id"])
            row.update(row_id=f'r{row["repeat"]:02d}-{row["case_id"]}-{row["variant"]}', strategy="coordinated",
                expectation=case["expectation"], domain=171+ordinal if row["selected"] else None)
            ordinal += bool(row["selected"])
        self.sources = {row["row_id"]: dict.fromkeys(SOURCE_KEYS) for row in self.catalog}
        self.rows = [dict(row, status="not_run", request_id=None, actual_request_status=None, outcome=None,
            fixed_case_result_sha256=None, trace_sha256=None, verification_result_sha256=None,
            unindexed_trace_present=False, unindexed_trace_sha256=None) for row in self.catalog]
        self.code = {name:"d"*64 for name in CODE_FILES}
        for index, outcome in ((0,"planning_failed"), (1,"motion_succeeded"), (18,"expected_invalid_goal"), (19,None)):
            self.install(index, outcome)
        self.refresh()

    def artifacts(self, row, outcome):
        case = next(case for case in self.suite["cases"] if case["case_id"] == row["case_id"])
        request, session_id = f'{row["index"]+1:032x}', f'{row["index"]+101:032x}'
        initial = deepcopy(case["initial_rad"])
        curve = dict(names=list(initial), points=[dict(t=0., p=list(initial.values()), v=[], a=[]),
            dict(t=2., p=list(case["target_rad"].values()), v=[], a=[])])
        native = dict(complete=True, samples=201, minimum=dict(time_sec=1., distance_m=.04), duration_sec=2.,
            sample_dt_max_sec=.01, joint_total_variation_rad=1.5, tip_frame_path_m=dict(left=.2,right=.3))
        trace = dict(schema_version=1, request_id=request,
            command=dict(schema_version=1, request_id=request, strategy="coordinated", target_rad=deepcopy(case["target_rad"])),
            initial_state_rad=initial, status="succeeded", failure_kind=None, failure_stage=None, error=None,
            dispatched_goals=2, action_results=[dict(status=4,error_code=0),dict(status=4,error_code=0)],
            contact_guard=dict(latched=False), planned_trajectories=[curve], planned_clearance=deepcopy(native),
            observed_clearance=deepcopy(native), planned_candidate_attempt=1,
            compute_budget=dict(limit_wall_sec=5.,elapsed_wall_sec=.8,exceeded=False),
            plans=[dict(group="both_arms",wall_sec=.2,server_planning_sec=.1)],
            plan_selection=dict(max_candidates=row["max_plan_candidates"],attempted=1,selected_attempt=1,status="selected",
                candidates=[dict(attempt=1,accepted=True,proposal_kind="coordinated_rrt",curve=curve,native=deepcopy(native))]),
            events=[dict(name="received",wall_sec=0.),dict(name="first_motion",wall_sec=1.),
                    dict(name="settled",wall_sec=3.),dict(name="terminal",wall_sec=3.1)],
            spans=[dict(name="settling",duration_wall_sec=.2)],
            metrics=dict(received_to_first_motion_wall_sec=1.,received_to_actions_done_wall_sec=2.8,
                         received_to_settled_wall_sec=3.,received_to_terminal_wall_sec=3.1))
        if outcome in ("planning_failed", "expected_invalid_goal"):
            trace.update(status="failed",dispatched_goals=0,action_results=[],observed_clearance=None,
                failure_kind="invalid_goal" if outcome=="expected_invalid_goal" else "budget_exhausted",
                failure_stage="endpoint_validity" if outcome=="expected_invalid_goal" else "spatial_candidate_search",
                error=dict(type="RuntimeError",message="private-user-77 /home/private-user-77/model.step password=opaque"))
            trace["metrics"].update(received_to_first_motion_wall_sec=None,received_to_actions_done_wall_sec=None,
                received_to_settled_wall_sec=None,received_to_terminal_wall_sec=5.01)
            trace["events"]=[dict(name="received",wall_sec=0.),dict(name="terminal",wall_sec=5.01)]
            trace["spans"]=[]
            trace.pop("planned_clearance")
            if outcome=="expected_invalid_goal":
                trace.pop("plan_selection");trace.pop("planned_trajectories");trace["plans"]=[]
            else:
                trace["compute_budget"].update(elapsed_wall_sec=5.01,exceeded=True)
                trace["plan_selection"].update(selected_attempt=None,status="interrupted")
                candidate=trace["plan_selection"]["candidates"][0]
                candidate.update(accepted=False,status="interrupted",native_progress=dict(samples=20,minimum=dict(distance_m=.029)))
                candidate.pop("native")
        trace_raw = None if outcome is None else encoded(trace)
        policy=dict(schema_version=1,workcell=case["workcell"],
            case="eval_post" if case["policy_name"]=="eval_post_timing" else "dual_arm_crossing",**self.suite["policy"])
        fixed=dict(schema_version=1,status="aborted" if outcome is None else "completed",case=deepcopy(case),
            case_id=case["case_id"],workcell=case["workcell"],variant=row["variant"],repeat=row["repeat"],row_index=row["index"],
            suite_sha256=suite_hash(self.suite),policy_name=case["policy_name"],source_sha256="d"*64,
            policy=policy,session_id=session_id,scored_attempted=outcome is not None,
            scored_request_id=request if outcome is not None else None,
            scored_settings=dict(compute_budget_wall_sec=5.,max_plan_candidates=row["max_plan_candidates"],goal_tolerance_rad=.003),
            error=dict(type="PreparationError",message="private-box-77 /home/private-user-77/failed.log") if outcome is None else None,
            scored=None if outcome is None else dict(role="scored",request_id=request,status=trace["status"],
                trace_file=request+".json",trace_sha256=digest(trace_raw),command=deepcopy(trace["command"]),
                initial_state_rad=deepcopy(initial),outcome=outcome))
        manifest=dict(workcell_sha256=self.suite["workcell_fingerprints"][case["workcell"]],joints=list(initial),
                      private_unused_asset="/home/private-user-77/private-cad.step")
        manifest_raw=encoded(manifest)
        session=dict(schema_version=1,session_id=session_id,workcell=case["workcell"],model_manifest=manifest,
            policy=policy,compute_budget_sec=5.,max_plan_candidates=row["max_plan_candidates"],
            code_commit="1"*40,code_worktree_dirty=True,implementation_sha256={name:"a"*64 for name in IMPLEMENTATION_FILES},
            host=dict(cpu="Public Test CPU",hostname="private-box-77",username="private-user-77",memory_total_bytes=999),
            runtime=dict(os="Ubuntu 22.04",python="3.10.12",middleware="rmw_cyclonedds_cpp",cwd="/home/private-user-77"),
            evaluator=dict(engine="MuJoCo 3.3.0",source_sha256="b"*64,library_sha256="c"*64))
        fixed_raw=encoded(fixed)
        run_id="synthetic-run-"+str(row["index"])
        verification=dict(schema_version=2,run_id=run_id,ros_domain_id=row["domain"],
            status="failed" if outcome is None else "passed",robot_profile_sha256=manifest["workcell_sha256"],
            model_manifest_sha256=digest(manifest_raw),physics_report_sha256="e"*64,
            motion_checks_passed=outcome is not None,physics_checks_passed=True,shutdown_clean=True,process_group_gone=True,launch_return_code=0,
            fixed_case_result_sha256=digest(fixed_raw),error=fixed["error"],
            log_path="/home/private-user-77/worker.log",process_command="private-box-77 --password=hidden",
            physics=dict(complete=True,run_id=run_id,contact_steps=0,invalid_values=0,steps=1000,mujoco_version="3.3.0"))
        return dict(fixed_case_raw=fixed_raw,trace_raw=trace_raw,verification_raw=encoded(verification),
                    session_raw=encoded(session),model_manifest_raw=manifest_raw,unindexed_trace_raw=None)

    def install(self,index,outcome):
        row=self.catalog[index]
        source=self.sources[row["row_id"]]=self.artifacts(row,outcome)
        fixed=json.loads(source["fixed_case_raw"])
        original=self.rows[index]
        original.update(status="aborted" if outcome is None else "completed",
            request_id=None if fixed["scored"] is None else fixed["scored"]["request_id"],
            actual_request_status=None if fixed["scored"] is None else fixed["scored"]["status"],outcome=outcome,
            fixed_case_result_sha256=digest(source["fixed_case_raw"]),verification_result_sha256=digest(source["verification_raw"]),
            trace_sha256=None if source["trace_raw"] is None else digest(source["trace_raw"]),
            fixed_case_result_path="/home/private-user-77/runtime/fixed-case.json",error=fixed["error"])

    def refresh(self):
        compact=[summarize_row(self.suite,row,**{key:self.sources[row["row_id"]][key]
            for key in ("fixed_case_raw","trace_raw","verification_raw")}) for row in self.catalog]
        counts=dict(planned=20,selected=4,completed=sum(row["status"]=="completed" for row in self.rows),
            aborted=sum(row["status"]=="aborted" for row in self.rows),not_run=sum(row["status"]=="not_run" for row in self.rows),
            motion_succeeded=sum(row.get("outcome")=="motion_succeeded" and row["status"]=="completed" for row in self.rows),
            planning_failed=sum(row.get("outcome")=="planning_failed" and row["status"]=="completed" for row in self.rows),
            expected_invalid_goal=sum(row.get("outcome")=="expected_invalid_goal" and row["status"]=="completed" for row in self.rows))
        self.raw=dict(schema_version=1,id="fixed-motion-raw-v1",status="aborted",suite_sha256=suite_hash(self.suite),
            compute_budget_wall_sec=5.,initial_tolerance_rad=.0015,repeats=1,requested_case_ids=["local_small","invalid_goal"],
            domain_start=171,domain_range=dict(start=171,end=174,count=4),rows=self.rows,results=compact,
            summary=aggregate_results(self.suite,self.catalog,compact),counts=counts,source_sha256=self.code,
            raw_process_command="private-box-77 /home/private-user-77/command")

    def export(self):
        return export_fixed_motion(encoded(self.suite),encoded(self.raw),self.sources,expected_suite=self.suite,
                                   forbidden_tokens=("private-user-77","private-box-77"))

    def test_projection_removes_private_metadata_messages_and_preserves_actual_commit(self):
        public=self.export()
        text=json.dumps(public)
        for private in ("private-user-77","private-box-77","/home/","password=","process_command","error\""):
            self.assertNotIn(private,text)
        self.assertEqual(public["measurement"]["code_commit"],"1"*40)
        self.assertTrue(public["measurement"]["measurement_worktree_dirty"])
        self.assertEqual(public["measurement"]["environment"]["cpu_model"],"Public Test CPU")
        self.assertTrue(public["privacy_audit"]["passed"])
        self.assertEqual(public["source_sha256"]["benchmark"],digest(encoded(self.raw)))
        self.assertEqual(public["rows"][1]["source_sha256"]["session"],digest(self.sources[self.catalog[1]["row_id"]]["session_raw"]))

    def test_all_unrun_rows_and_expected_invalid_denominators_remain_visible(self):
        public=self.export()
        self.assertEqual(len(public["planned_catalog"]),20)
        self.assertEqual(len(public["rows"]),20)
        self.assertEqual(public["summary"]["all_rows"]["not_run"],17)
        self.assertEqual(public["summary"]["valid_goal"]["motion_denominators"],dict(all_planned=18,selected=2,scored=2))
        self.assertEqual(public["summary"]["expected_invalid_goal"]["invalid_goal_denominators"],dict(all_planned=2,selected=2,scored=1))
        self.assertEqual(public["summary"]["valid_goal"]["motion_success_rate_selected"],.5)
        self.assertEqual(public["scope"]["sample_role"],"partial_or_diagnostic")
        self.assertEqual(public["scope"]["repeats"],1)
        self.assertEqual(public["rows"][0]["metrics"]["planned"]["prefix_minimum_distance_m"],.029)
        self.assertIsNone(public["rows"][0]["metrics"]["planned"]["minimum_distance_m"])
        self.assertNotIn("p95",public["summary"])

    def test_source_sha_and_stored_summary_tampering_fail_closed(self):
        original=deepcopy(self.sources)
        key=self.catalog[0]["row_id"]
        self.sources[key]["trace_raw"]+=b" "
        with self.assertRaisesRegex(ValueError,"SHA256 mismatch"):
            self.export()
        self.sources=original
        self.raw["summary"]["valid_goal"]["motion_success_rate_selected"]=.99
        with self.assertRaisesRegex(ValueError,"recomputation"):
            self.export()

    def test_dropping_or_reordering_catalog_or_compact_rows_is_rejected(self):
        original=deepcopy(self.raw)
        for change in (lambda:self.raw["rows"].pop(),lambda:self.raw["results"].pop(),
                       lambda:self.raw["rows"].__setitem__(slice(0,2),list(reversed(self.raw["rows"][:2])))):
            self.raw=deepcopy(original);change()
            with self.assertRaises(ValueError):
                self.export()

    def test_measured_code_or_model_mismatch_is_rejected_without_using_current_head(self):
        key=self.catalog[1]["row_id"]
        original=deepcopy(self.sources[key])
        session=json.loads(original["session_raw"])
        session["code_commit"]="2"*40
        self.sources[key]["session_raw"]=encoded(session)
        with self.assertRaisesRegex(ValueError,"changed between rows"):
            self.export()
        self.sources[key]=deepcopy(original)
        manifest=json.loads(original["model_manifest_raw"])
        manifest["workcell_sha256"]="0"*64
        self.sources[key]["model_manifest_raw"]=encoded(manifest)
        with self.assertRaisesRegex(ValueError,"identity mismatch"):
            self.export()

    def test_incomplete_implementation_provenance_is_rejected(self):
        key=self.catalog[0]["row_id"]
        session=json.loads(self.sources[key]["session_raw"])
        session["implementation_sha256"].pop("plan_selection.py")
        self.sources[key]["session_raw"]=encoded(session)
        with self.assertRaisesRegex(ValueError,"Missing measured implementation"):
            self.export()

    def test_report_id_is_reusable_and_plain(self):
        public=export_fixed_motion(encoded(self.suite),encoded(self.raw),self.sources,
            expected_suite=self.suite,report_id="2026-11-01-fixed-motion")
        self.assertEqual(public["report_id"],"2026-11-01-fixed-motion")
        for identifier in ("../private", "x/../y", "", "a"*101):
            with self.subTest(identifier=identifier), self.assertRaisesRegex(ValueError,"report ID"):
                export_fixed_motion(encoded(self.suite),encoded(self.raw),self.sources,
                    expected_suite=self.suite,report_id=identifier)

    def test_private_identity_in_a_curated_environment_field_is_rejected(self):
        key=self.catalog[0]["row_id"]
        session=json.loads(self.sources[key]["session_raw"])
        session["host"]["cpu"]="private-box-77"
        # Apply the same private value to every session so the privacy audit,
        # rather than cross-row configuration equality, is what rejects it.
        for row in self.catalog:
            source=self.sources[row["row_id"]]
            if source["session_raw"] is not None:
                value=json.loads(source["session_raw"]);value["host"]["cpu"]="private-box-77";source["session_raw"]=encoded(value)
        with self.assertRaisesRegex(ValueError,"identity token"):
            self.export()

    def test_orphan_bytes_remain_hash_only_and_do_not_become_a_scored_row(self):
        index=19;key=self.catalog[index]["row_id"]
        source=self.sources[key]
        fixed=json.loads(source["fixed_case_raw"])
        fixed.update(scored_attempted=True,scored_request_id="f"*32)
        source["fixed_case_raw"]=encoded(fixed)
        verification=json.loads(source["verification_raw"])
        verification["fixed_case_result_sha256"]=digest(source["fixed_case_raw"])
        source["verification_raw"]=encoded(verification)
        source["unindexed_trace_raw"]=b'\x00truncated private_geometry /home/private-user-77/model.step'
        self.rows[index].update(fixed_case_result_sha256=digest(source["fixed_case_raw"]),
            verification_result_sha256=digest(source["verification_raw"]),unindexed_trace_present=True,
            unindexed_trace_sha256=digest(source["unindexed_trace_raw"]))
        self.refresh()
        public=self.export();row=public["rows"][index]
        self.assertTrue(row["attempted"] and row["delivery_unknown"] and row["unindexed_trace_present"])
        self.assertFalse(row["scored"])
        self.assertIsNone(row["request_id"])
        self.assertEqual(row["unindexed_trace_sha256"],digest(source["unindexed_trace_raw"]))
        self.assertNotIn("private_geometry",json.dumps(public))


if __name__=="__main__":
    unittest.main()
