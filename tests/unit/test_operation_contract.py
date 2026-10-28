"""Observational phases do not admit commands or conflate request/physical outcomes."""
from copy import deepcopy
import json
import os
from pathlib import Path
import subprocess
import sys
import unittest

from mujoco_ros2_manipulation.workcells.operation_contract import MAX_EVENTS, describe_operation


class OperationContractTests(unittest.TestCase):
    def record(self, **values):
        return dict(request_id="a"*32, session_id="session-a", status="running", stage="planning_both_arms", **values)

    def test_action_correlation_uses_only_bounded_controller_and_confirmed_uuid(self):
        source = self.record(events=[dict(name="goal_acceptance", arm="fr5",
            controller="fr5_controller", goal_uuid="a"*32, accepted=True)])
        event = describe_operation(source)["events"]["items"][0]
        self.assertEqual(event["controller"], "fr5_controller")
        self.assertEqual(event["goal_uuid"], "a"*32)
        for invalid in ("", "a"*31, "x"*32, "path/to/private"):
            source["events"][0]["goal_uuid"] = invalid
            with self.assertRaises(ValueError):
                describe_operation(source)

    def test_explicit_phase_mapping_distinguishes_planning_virtual_review_and_actual_execution(self):
        phases = {
            "admission": ("submitted", "received", "queued", "state_snapshot"),
            "planning": ("computation_started", "endpoint_clearance", "endpoint_validity", "planning_both_arms",
                         "planning_left_arm", "planning_right_arm", "spatial_candidate_search", "departure_search",
                         "safety_checks", "computation_done", "state_recheck"),
            "review": ("plan_ready", "plan_revoked", "approval_validating", "approved_plan_recheck"),
            "dispatch": ("final_state_recheck", "dispatch", "goal_sent", "goal_acceptance"),
            "executing": ("first_motion", "first_motion_left", "first_motion_right", "goal_result", "actions_done"),
            "settling": ("settling", "settled"), "terminal": ("terminal", "plan_not_required"),
        }
        for phase, stages in phases.items():
            for stage in stages:
                source=self.record();source["stage"]=stage
                with self.subTest(stage=stage):
                    result=describe_operation(source)
                    self.assertEqual(result["phase"], phase)
                    self.assertEqual(result["source_stage"], stage)
                    self.assertTrue(result["read_only"])
                    self.assertEqual(result["admission_owned_by"], "existing_command_server")

    def test_unknown_stage_is_preserved_without_name_heuristics_or_terminal_phase_override(self):
        for stage in ("future_planning_magic", "first_motion_new_robot", "cleanup_has_finished", "terminal_v2"):
            source=self.record();source.update(stage=stage,status="cancelled")
            result=describe_operation(source)
            self.assertEqual(result["source_stage"], stage)
            self.assertEqual(result["phase"], "unknown")
            self.assertTrue(result["request_terminal"])
            self.assertFalse(result["cancellation"]["implies_physical_stop"])

    def test_legacy_mode_and_plan_execution_links_remain_request_identity_not_dispatch_evidence(self):
        automatic=describe_operation(self.record())
        self.assertEqual(automatic["mode"], "automatic")
        self.assertEqual(automatic["mode_origin"], "legacy_default")
        self.assertEqual(automatic["execution_request_id"], "a"*32)
        source=self.record(mode="plan_only");source["stage"]="plan_ready"
        planned=describe_operation(source)
        self.assertEqual(planned["plan_request_id"], "a"*32)
        self.assertIsNone(planned["execution_request_id"])
        source=self.record(mode="approved_plan",approval=dict(plan_request_id="b"*32, secret="ignored"))
        approved=describe_operation(source)
        self.assertEqual(approved["plan_request_id"], "b"*32)
        self.assertEqual(approved["execution_request_id"], "a"*32)
        self.assertNotIn("secret", json.dumps(approved))
        future=describe_operation(self.record(mode="future_mode"))
        self.assertFalse(future["mode_known"])
        self.assertEqual(future["mode"], "future_mode")
        self.assertIsNone(future["execution_request_id"])
        with self.assertRaisesRegex(ValueError, "Conflicting"):
            describe_operation(self.record(plan_request_id="c"*32,approval=dict(plan_request_id="b"*32)))

    def test_explicit_stage_wins_while_full_record_can_use_last_event_name(self):
        events=[dict(name="received", wall_sec=0,simulation_sec=50),
                dict(name="plan_ready",wall_sec=.1,simulation_sec=50.1)]
        source=self.record(events=events)
        self.assertEqual(describe_operation(source)["phase"], "planning")
        del source["stage"]
        result=describe_operation(source)
        self.assertEqual(result["phase"], "review")
        self.assertEqual(result["source_stage"], "plan_ready")
        self.assertEqual(result["events"]["items"][-1]["phase"], "review")

    def test_cancel_submission_acknowledgement_and_terminal_request_outcomes_do_not_confirm_stop(self):
        for submission, expected in (("submitting","submission_in_progress"),("submitted","submitted"),
                                     ("unconfirmed","submission_unconfirmed")):
            result=describe_operation(self.record(cancel_submission=submission))
            self.assertEqual(result["cancellation"]["state"], expected)
            self.assertIsNone(result["cancellation"]["server_acknowledged"])
            self.assertFalse(result["cancellation"]["implies_physical_stop"])
            self.assertIsNone(result["physical_guard"])
        source=self.record();source["stage"]="cancel_received"
        result=describe_operation(source)
        self.assertEqual(result["phase"], "unknown")
        self.assertEqual(result["cancellation"]["state"], "server_acknowledged")
        self.assertTrue(result["cancellation"]["server_acknowledged"])
        for status, expected in (("cancelled","request_cancelled"),("failed","request_failed_after_cancel"),
                                 ("succeeded","request_completed_after_cancel")):
            source=self.record(events=[dict(name="cancel_received",wall_sec=.1)],contact_guard=None)
            source.update(status=status,stage="terminal")
            result=describe_operation(source)
            self.assertEqual(result["cancellation"]["state"], expected)
            self.assertFalse(result["cancellation"]["implies_physical_stop"])
            self.assertIsNone(result["physical_guard"])

    def test_physical_guard_and_owner_cleanup_are_independent_of_request_terminal(self):
        for phase, reason, confirmed in (("stopping","contact",False),("stopped","contact",True),
                                         ("stopped","clock_stall",True),("stopped","unknown",False),
                                         ("stopped","",False)):
            source=self.record(contact_guard=dict(latched=True,phase=phase,reason=reason,controllers={"private":"ignored"}),
                               cleanup_clean=True,process_group_gone=False)
            source.update(status="failed",stage="terminal")
            result=describe_operation(source)
            self.assertEqual(result["physical_guard"]["stop_confirmation"]=="reported_stopped", confirmed)
            self.assertEqual(result["status"], "failed")
            self.assertTrue(result["owner_cleanup"]["clean"])
            self.assertFalse(result["owner_cleanup"]["process_group_gone"])
            self.assertIsNone(result["owner_cleanup"]["shutdown_clean"])
            self.assertNotIn("controllers",result["physical_guard"])

    def test_clock_domains_source_stamps_and_snapshot_ages_are_never_subtracted_or_retimed(self):
        source=self.record(events=[dict(name="first_motion",wall_sec=.2,simulation_sec=500,
            joint_state_stamp_simulation_sec=499.98,arm="left",status=2,accepted=True,error_code=0,
            host="ignored",raw_payload={"private":"ignored"})],joint_state_stamp_simulation_sec=499.98,
            state_receive_age_wall_sec=.01,clock_receive_age_wall_sec=.02)
        result=describe_operation(source)
        self.assertFalse(result["clocks"]["cross_domain_durations_valid"])
        self.assertEqual(result["events"]["items"][0]["wall_sec"], .2)
        self.assertEqual(result["events"]["items"][0]["simulation_sec"], 500)
        self.assertEqual(result["freshness"]["joint_state_source_stamp_simulation_sec"], 499.98)
        self.assertEqual(result["freshness"]["joint_state_age_wall_sec"], .01)
        self.assertEqual(result["freshness"]["scope"], "recorded_snapshot_ages_only")
        self.assertNotIn("ignored",json.dumps(result))

    def test_bounded_event_tail_reports_lost_history_and_cannot_assert_cancel_ack_absence(self):
        events=[dict(name="cancel_received",wall_sec=0)]+[dict(name="goal_result",wall_sec=i) for i in range(1,MAX_EVENTS+2)]
        source=self.record(events=events)
        result=describe_operation(source)
        self.assertEqual(result["events"]["retained_count"],MAX_EVENTS)
        self.assertEqual(result["events"]["omitted_prefix_count"],2)
        self.assertEqual(result["events"]["source_count"],MAX_EVENTS+2)
        self.assertIsNone(result["cancellation"]["server_acknowledged"])
        self.assertEqual(result["cancellation"]["state"],"not_observed")
        missing=describe_operation(self.record())
        self.assertFalse(missing["events"]["history_provided"])
        self.assertIsNone(missing["events"]["source_count"])
        self.assertIsNone(missing["cancellation"]["server_acknowledged"])
        empty=describe_operation(self.record(events=[]))
        self.assertFalse(empty["cancellation"]["server_acknowledged"])
        terminal_summary=self.record();terminal_summary.update(stage="terminal",status="cancelled")
        cancelled=describe_operation(terminal_summary)
        self.assertEqual(cancelled["cancellation"]["state"],"request_cancelled")
        self.assertIsNone(cancelled["cancellation"]["server_acknowledged"])

    def test_projection_is_detached_and_json_finite(self):
        source=self.record(events=[dict(name="received",wall_sec=0)],guard=dict(phase="ready",reason="",latched=False))
        before=deepcopy(source)
        result=describe_operation(source)
        result["events"]["items"][0]["name"]="changed"
        result["physical_guard"]["reason"]="changed"
        self.assertEqual(source,before)
        json.dumps(describe_operation(source),allow_nan=False)

    def test_malformed_unbounded_or_nonfinite_observations_fail_without_admitting_any_command(self):
        for changes in (dict(stage="x"*129),dict(request_id="/private/path"),dict(events={}),dict(mode="x"*65),
                        dict(cancel_submission="cancelled"),dict(cleanup_clean=1),
                        dict(state_receive_age_wall_sec=-1),dict(clock_receive_age_wall_sec=float("inf")),
                        dict(events=[dict(name="received",wall_sec=float("nan"))]),
                        dict(events=[dict(name="goal_result",status=True)]),dict(guard=dict(latched="true"))):
            source=self.record();source.update(changes)
            with self.subTest(changes=changes),self.assertRaises(ValueError):
                describe_operation(source)

    def test_import_does_not_require_ros_physics_or_configuration_dependencies(self):
        code="""
import sys
class Deny:
    def find_spec(self, name, path=None, target=None):
        if name.split('.')[0] in ('rclpy','mujoco','numpy','yaml'):
            raise RuntimeError('forbidden import '+name)
sys.meta_path.insert(0,Deny())
from mujoco_ros2_manipulation.workcells.operation_contract import describe_operation
assert describe_operation({'request_id':'a'*32,'stage':'received'})['read_only']
"""
        subprocess.run([sys.executable,"-c",code],check=True,capture_output=True,text=True,
            env=dict(os.environ,PYTHONPATH=str(Path(__file__).resolve().parents[2]/"src")))


if __name__ == "__main__":
    unittest.main()
