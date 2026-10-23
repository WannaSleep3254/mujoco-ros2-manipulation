"""Fixed-row policy boundaries exercised without importing or starting ROS."""
import ast
from contextlib import contextmanager
from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path
import re
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock
import uuid

from mujoco_ros2_manipulation.paths import ROOT
from mujoco_ros2_manipulation.workcells.motion_request import command


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def atomic_json(path, data):
    Path(path).write_text(json.dumps(data, allow_nan=False) + "\n")


def execution_class():
    """Load only the real class and policy constants, replacing its ROS base.

    Methods under test are unchanged. Constructor, transport, engine and inherited
    execution services are not run. The namespace is local; sys.modules and ROS
    imports in other tests are unaffected.
    """
    source = ROOT / "src/mujoco_ros2_manipulation/workcells/fixed_execution.py"
    tree = ast.parse(source.read_text(), filename=str(source))
    constants = {"SCORED_BUDGET_SEC", "INITIAL_TOLERANCE_RAD", "MAINTENANCE_BUDGET_SEC",
                 "MAINTENANCE_GOAL_TOLERANCE_RAD", "MAX_CORRECTIONS", "MAX_CORRECTION_RAD", "PLANNING_FAILURE_KINDS"}
    body = [node for node in tree.body if isinstance(node, ast.Assign)
            and all(isinstance(target, ast.Name) and target.id in constants for target in node.targets)]
    klass = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == "FixedCaseExecution")
    klass.bases = [ast.Name(id="object", ctx=ast.Load())]
    body.append(klass)
    namespace = dict(contextmanager=contextmanager, deepcopy=deepcopy, json=json, math=math,
                     Path=Path, re=re, uuid=uuid, command=command, require=require,
                     file_sha256=sha, atomic_json=atomic_json,
                     String=lambda **kwargs: SimpleNamespace(**kwargs), COMMAND_TOPIC="unused-test-topic")
    exec(compile(ast.fix_missing_locations(ast.Module(body=body, type_ignores=[])), str(source), "exec"), namespace)
    return namespace["FixedCaseExecution"]


Execution = execution_class()


def failed_record(kind="candidate_exhausted", *, initial=0.0):
    return dict(request_id="a"*32, status="failed", initial_state_rad={"joint":initial},
        command=dict(target_rad={"joint":.2}, strategy="coordinated"),
        failure_kind=kind, error=dict(type="PlanningFailure", message="Recorded planning failure"),
        metrics=dict(received_to_first_motion_wall_sec=None, received_to_terminal_wall_sec=.5),
        compute_budget=dict(limit_wall_sec=5.0, exceeded=kind == "budget_exhausted"),
        dispatched_goals=0, action_results=[])


def successful_record():
    return dict(request_id="b"*32, status="succeeded", initial_state_rad={"joint":0.0},
        metrics=dict(received_to_first_motion_wall_sec=.25),
        compute_budget=dict(limit_wall_sec=5.0, exceeded=False),
        dispatched_goals=2, action_results=[dict(status=4,error_code=0),dict(status=4,error_code=0)],
        planned_clearance=dict(complete=True,minimum=dict(distance_m=.03)),
        observed_clearance=dict(complete=True,minimum=dict(distance_m=.02)))


class FixedExecutionTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.node = Execution.__new__(Execution)
        node = self.node
        node.directory = Path(self.temporary.name)
        node.joint_names = ["joint"]
        node.metadata = dict(joint_limits=dict(joint=dict(lower=-10.0,upper=10.0)))
        node.policy = dict(min_inter_arm_distance_m=.02,tracking_reserve_m=.01)
        node.case = dict(case_id="local_small", expectation="valid_goal", initial_rad=dict(joint=0.0),target_rad=dict(joint=.2))
        node.result_record = dict(status="running",scored_attempted=False,scored_request_id=None,
                                 scored=None,maintenance=[],preparation_stages=[],error=None)
        node.commanded, node.actual = dict(joint=0.0), dict(joint=0.0)
        node.compute_budget_sec, node.max_plan_candidates, node.goal_tolerance_rad = 5.0, 1, .003
        node.preparation = None
        node._scored_started = False
        node.blocked, node.contact_guard = False, dict(latched=False)
        node.check_abort = Mock()
        node.quiet = Mock()
        node.state = lambda:dict(node.actual)
        node.command_publisher = SimpleNamespace(publish=Mock())
        node.pending, node.replies = None, {}
        node.until, node.process_next = Mock(), Mock()

    def install_transport(self, *, publish_error=None):
        node = self.node
        node.sent = []
        def publish(message):
            node.sent.append(json.loads(message.data))
            if publish_error is not None:
                raise publish_error
            node.pending = SimpleNamespace(data=dict(request_id=node.sent[-1]["request_id"]))
        node.command_publisher.publish.side_effect = publish
        def until(predicate, *args):
            self.assertTrue(predicate())
        node.until.side_effect = until
        def process():
            payload = node.sent[-1]
            trace = failed_record()
            trace.update(request_id=payload["request_id"], command=deepcopy(payload))
            (node.directory/(payload["request_id"]+".json")).write_text(json.dumps(trace))
            node.pending = None
        node.process_next.side_effect = process

    def test_maintenance_settings_restore_on_success_and_primary_exception(self):
        node = self.node
        original = (node.compute_budget_sec,node.max_plan_candidates,node.goal_tolerance_rad)
        with node._maintenance_settings():
            self.assertEqual((node.compute_budget_sec,node.max_plan_candidates,node.goal_tolerance_rad),(30.0,12,.00001))
        self.assertEqual((node.compute_budget_sec,node.max_plan_candidates,node.goal_tolerance_rad),original)
        error = KeyboardInterrupt("stop preparation")
        with self.assertRaises(KeyboardInterrupt) as caught:
            with node._maintenance_settings():
                raise error
        self.assertIs(caught.exception,error)
        self.assertEqual((node.compute_budget_sec,node.max_plan_candidates,node.goal_tolerance_rad),original)

    def test_failed_maintenance_restores_replay_adapter_and_scored_settings(self):
        node = self.node
        old = node.preparation = dict(source="outer")
        failure = RuntimeError("preparation planning failed")
        def fail(*args, **kwargs):
            self.assertEqual((node.compute_budget_sec,node.max_plan_candidates,node.goal_tolerance_rad),(30.0,12,.00001))
            self.assertEqual(node.preparation,dict(source="saved fourth path"))
            raise failure
        node.submit = Mock(side_effect=fail)
        with self.assertRaises(RuntimeError) as caught:
            node._maintenance_request(dict(joint=0.0),role="setup",preparation=dict(source="saved fourth path"))
        self.assertIs(caught.exception,failure)
        self.assertIs(node.preparation,old)
        self.assertEqual((node.compute_budget_sec,node.max_plan_candidates,node.goal_tolerance_rad),(5.0,1,.003))
        self.assertIsNone(node.result_record["scored"])

    def test_feedback_corrects_actual_bias_with_a_normal_bounded_command(self):
        node, desired, targets = self.node, dict(joint=0.0), []
        def move(target, **kwargs):
            targets.append(deepcopy(target))
            node.commanded = dict(target)
            node.actual = dict(joint=target["joint"]-.006)
            return dict(request_id=str(len(targets)))
        node._maintenance_request = Mock(side_effect=move)
        stage = node.drive_to_state(desired,"initial")
        self.assertEqual(targets,[dict(joint=0.0),dict(joint=.006)])
        self.assertEqual(stage["status"],"converged")
        self.assertEqual(stage["correction_requests"],1)
        self.assertEqual(stage["final_error_rad"],0.0)
        self.assertEqual(desired,dict(joint=0.0))
        self.assertIsNone(node.result_record["scored"])

    def test_nonconverging_feedback_stops_after_three_corrections_and_retains_stage(self):
        node, targets = self.node, []
        def move(target, **kwargs):
            targets.append(deepcopy(target))
            node.commanded, node.actual = dict(target),dict(joint=-.005)
            return dict(request_id=str(len(targets)))
        node._maintenance_request = Mock(side_effect=move)
        with self.assertRaisesRegex(RuntimeError,"within three corrections"):
            node.drive_to_state(dict(joint=0.0),"initial")
        self.assertEqual(len(targets),4)  # One nominal request and three corrections.
        stage = node.result_record["preparation_stages"][-1]
        self.assertEqual(stage["correction_requests"],3)
        self.assertEqual(stage["request_ids"],["1","2","3","4"])
        self.assertEqual(stage["status"],"not_converged")
        self.assertEqual(stage["final_error_rad"],.005)
        self.assertIsNone(node.result_record["scored"])

    def test_feedback_larger_than_point_zero_three_never_submits_a_correction(self):
        node = self.node
        node.actual = dict(joint=-.0301)
        node._maintenance_request = Mock()
        with self.assertRaisesRegex(RuntimeError,"exceeds 0.03rad"):
            node.drive_to_state(dict(joint=0.0),"initial",initial_request=False)
        node._maintenance_request.assert_not_called()
        self.assertEqual(node.result_record["preparation_stages"][-1]["request_ids"],[])

    def test_feedback_target_remains_subject_to_joint_limits_before_publish(self):
        node = self.node
        node.commanded, node.actual = dict(joint=9.999),dict(joint=-.002)
        with self.assertRaisesRegex(RuntimeError,"out-of-limit"):
            node.drive_to_state(dict(joint=0.0),"initial",initial_request=False)
        node.command_publisher.publish.assert_not_called()
        node.process_next.assert_not_called()
        self.assertEqual((node.compute_budget_sec,node.max_plan_candidates,node.goal_tolerance_rad),(5.0,1,.003))

    def test_a_second_scored_request_is_rejected_before_publish_even_without_a_trace(self):
        node = self.node
        node._scored_started = True
        self.assertIsNone(node.result_record["scored"])
        with self.assertRaisesRegex(RuntimeError,"only one scored"):
            node.submit(dict(joint=.2),role="scored",scored=True)
        node.command_publisher.publish.assert_not_called()
        node.process_next.assert_not_called()

    def test_failed_first_publish_consumes_the_row_and_cannot_be_retried(self):
        node = self.node
        error = RuntimeError("publisher transport failed")
        self.install_transport(publish_error=error)
        with self.assertRaises(RuntimeError) as caught:
            node.submit(dict(joint=.2),role="scored",scored=True)
        self.assertIs(caught.exception,error)
        self.assertTrue(node._scored_started)
        self.assertTrue(node.result_record["scored_attempted"])
        saved = json.loads(node.result_path().read_text())
        self.assertTrue(saved["scored_attempted"])
        self.assertIsNone(saved["scored"])
        self.assertRegex(saved["scored_request_id"],r"^[0-9a-f]{32}$")
        self.assertEqual(saved["scored_request_id"],node.sent[0]["request_id"])
        self.assertIsNone(node.result_record["scored"])
        with self.assertRaisesRegex(RuntimeError,"only one scored"):
            node.submit(dict(joint=.2),role="scored",scored=True)
        self.assertEqual(node.command_publisher.publish.call_count,1)
        node.process_next.assert_not_called()

    def test_attempt_identity_is_durable_before_the_first_publisher_call(self):
        node = self.node
        def observe_before_publish(message):
            payload = json.loads(message.data)
            saved = json.loads(node.result_path().read_text())
            self.assertTrue(saved["scored_attempted"])
            self.assertEqual(saved["scored_request_id"],payload["request_id"])
            self.assertIsNone(saved["scored"])
            raise RuntimeError("fail after observing durable attempt")
        node.command_publisher.publish.side_effect = observe_before_publish
        with self.assertRaisesRegex(RuntimeError,"durable attempt"):
            node.submit(dict(joint=.2),role="scored",scored=True)
        self.assertEqual(node.command_publisher.publish.call_count,1)

    def test_terminal_raw_failure_is_saved_once_with_its_content_hash(self):
        node = self.node
        self.install_transport()
        record, entry = node.submit(dict(joint=.2),role="scored",scored=True)
        self.assertIs(node.result_record["scored"],entry)
        self.assertEqual(entry["status"],"failed")
        self.assertEqual(entry["trace_sha256"],sha(node.directory/entry["trace_file"]))
        self.assertEqual(entry["request_id"],record["request_id"])
        self.assertEqual(node.command_publisher.publish.call_count,1)
        with self.assertRaisesRegex(RuntimeError,"only one scored"):
            node.submit(dict(joint=.2),role="scored",scored=True)
        self.assertEqual(node.command_publisher.publish.call_count,1)

    def test_each_expected_planning_failure_is_a_retained_scored_outcome(self):
        node = self.node
        node.submit = Mock()
        for kind in ("candidate_exhausted","budget_exhausted","planner_failed"):
            record = failed_record(kind)
            before = deepcopy(record)
            entry = dict(request_id=record["request_id"],status=record["status"],failure_kind=kind)
            node.result_record["scored"] = entry
            node._judge_scored(record,entry)
            self.assertEqual(entry["outcome"],"planning_failed")
            self.assertEqual(record,before)
        node.submit.assert_not_called()  # Scored planning failures never trigger retries.

    def test_transport_or_state_failure_is_not_relabelled_as_an_expected_planning_failure(self):
        for kind in ("planning_error","state_changed","invalid_start","cancelled","execution_failed"):
            record = failed_record(kind)
            entry = dict(status=record["status"],failure_kind=kind)
            self.node.result_record["scored"] = entry
            with self.subTest(kind=kind), self.assertRaisesRegex(RuntimeError,"aborted"):
                self.node._judge_scored(record,entry)
            self.assertNotIn("outcome",entry)

    def test_dispatch_or_motion_cannot_be_hidden_inside_planning_failure_statistics(self):
        for field,value in (("dispatched_goals",2),("action_results",[dict(status=5,error_code=0)])):
            record = failed_record()
            record[field] = value
            with self.subTest(field=field), self.assertRaisesRegex(RuntimeError,"dispatched, moved"):
                self.node._judge_scored(record,{})
        record = failed_record()
        record["metrics"]["received_to_first_motion_wall_sec"] = .1
        with self.assertRaisesRegex(RuntimeError,"dispatched, moved"):
            self.node._judge_scored(record,{})

    def test_invalid_goal_requires_its_production_rejection_and_is_separate_from_motion_success(self):
        node = self.node
        node.case["expectation"] = "invalid_goal"
        entry = {}
        node._judge_scored(failed_record("invalid_goal"),entry)
        self.assertEqual(entry["outcome"],"expected_invalid_goal")
        for record in (failed_record("budget_exhausted"),successful_record()):
            with self.subTest(status=record["status"]), self.assertRaises(RuntimeError):
                node._judge_scored(record,{})

    def test_current_start_error_is_checked_and_never_treated_as_a_solver_failure(self):
        node = self.node
        with self.assertRaisesRegex(RuntimeError,"fixed state"):
            node._judge_scored(failed_record(initial=.00151),{})
        entry = {}
        node._judge_scored(failed_record(initial=.0015),entry)
        self.assertEqual(entry["initial_error_rad"],.0015)
        self.assertEqual(entry["outcome"],"planning_failed")

    def test_execution_interlock_or_cancel_prevents_any_success_classification(self):
        node = self.node
        for kind in ("blocked","latched","cancel"):
            node.blocked = kind == "blocked"
            node.contact_guard = dict(latched=kind == "latched")
            node.check_abort.side_effect = RuntimeError("cancel received") if kind == "cancel" else None
            entry = {}
            with self.subTest(kind=kind), self.assertRaises(RuntimeError): node._judge_scored(successful_record(),entry)
            self.assertNotIn("outcome",entry)

    def test_motion_success_requires_both_actions_complete_clearance_and_budget(self):
        node = self.node
        entry = {}
        node._judge_scored(successful_record(),entry)
        self.assertEqual(entry["outcome"],"motion_succeeded")
        variants = []
        record = successful_record(); record["compute_budget"]["exceeded"] = True; variants.append(record)
        record = successful_record(); record["action_results"][1]["status"] = 5; variants.append(record)
        record = successful_record(); record["planned_clearance"]["complete"] = False; variants.append(record)
        record = successful_record(); record["observed_clearance"]["minimum"]["distance_m"] = .0199; variants.append(record)
        record = successful_record(); record["no_motion_required"] = True; record["dispatched_goals"] = 0; variants.append(record)
        for record in variants:
            with self.subTest(record=record), self.assertRaises(RuntimeError): node._judge_scored(record,{})

    def install_environment_probe(self, *, contact_body="workcell_evaluation_post", goal_valid=True):
        node = self.node
        node.case.update(case_id="environment_post", provenance=dict(parameters=dict(fixed_obstacle=dict(name="evaluation_post"))))
        node.evaluator = SimpleNamespace(at=Mock(return_value=dict(distance_m=.2,bodies=["left","right"])))
        calls = []
        def validity(state):
            calls.append(deepcopy(state))
            if len(calls) == 2 and not goal_valid:
                return SimpleNamespace(valid=False,contacts=[])
            hit = len(calls) > 2 and abs(state["joint"]-.1) < 1e-12 and contact_body is not None
            contacts = [SimpleNamespace(contact_body_1=contact_body,contact_body_2="left_wrist2_link")] if hit else []
            return SimpleNamespace(valid=not hit,contacts=contacts)
        node.validity_of = Mock(side_effect=validity)
        return calls

    def test_environment_preflight_requires_a_real_moveit_post_witness_and_remains_unscored(self):
        node = self.node
        calls = self.install_environment_probe()
        preflight = node._endpoint_preflight(dict(joint=0.0))
        probe = preflight["environment_direct_probe"]
        self.assertEqual(len(calls),23)  # Both endpoints followed by the 21 diagnostic states.
        self.assertFalse(preflight["scored"])
        self.assertFalse(probe["scored"])
        self.assertEqual(probe["obstacle_link"],"workcell_evaluation_post")
        self.assertEqual(len(probe["samples"]),21)
        self.assertEqual(probe["post_contact_samples"],1)
        witness = next(sample for sample in probe["samples"] if not sample["valid"])
        self.assertEqual(witness["alpha"],.5)
        self.assertEqual(witness["contacts"][0]["body1"],"workcell_evaluation_post")
        self.assertEqual(node.evaluator.at.call_count,2)  # Native scope remains endpoints/cross-arm only.
        self.assertFalse(node.result_record["scored_attempted"])
        self.assertIsNone(node.result_record["scored"])
        node.command_publisher.publish.assert_not_called()

    def test_environment_probe_uses_the_declared_object_name_instead_of_a_fixed_literal(self):
        node = self.node
        self.install_environment_probe(contact_body="workcell_another_post")
        node.case["provenance"]["parameters"]["fixed_obstacle"]["name"] = "another_post"
        probe = node._endpoint_preflight(dict(joint=0.0))["environment_direct_probe"]
        self.assertEqual(probe["obstacle_link"],"workcell_another_post")
        self.assertEqual(probe["post_contact_samples"],1)

    def test_missing_or_unrelated_moveit_contact_cannot_validate_the_post_case(self):
        node = self.node
        for contact_body in (None,"workcell_other_obstacle"):
            self.install_environment_probe(contact_body=contact_body)
            with self.subTest(body=contact_body), self.assertRaisesRegex(RuntimeError,"did not detect"):
                node._endpoint_preflight(dict(joint=0.0))
            probe = node.result_record["endpoint_preflight"]["environment_direct_probe"]
            self.assertEqual(probe["post_contact_samples"],0)
            self.assertEqual(len(probe["samples"]),21)
            self.assertFalse(node.result_record["scored_attempted"])
            node.command_publisher.publish.assert_not_called()

    def test_invalid_environment_endpoint_stops_before_any_direct_probe_or_scored_request(self):
        node = self.node
        calls = self.install_environment_probe(goal_valid=False)
        with self.assertRaisesRegex(RuntimeError,"valid goal is invalid"):
            node._endpoint_preflight(dict(joint=0.0))
        self.assertEqual(len(calls),2)
        self.assertNotIn("environment_direct_probe",node.result_record["endpoint_preflight"])
        self.assertFalse(node.result_record["scored_attempted"])
        node.command_publisher.publish.assert_not_called()

    def run_stubs(self, record=None, *, preparation_error=None):
        node = self.node
        node.cell = SimpleNamespace(home=[0.0],instances=[SimpleNamespace(joints=["joint"])])
        node.command_publisher.get_subscription_count = Mock(return_value=1)
        node.prepare = Mock(side_effect=preparation_error)
        node.drive_to_state = Mock()
        node._prepare_case_initial = Mock()
        node._endpoint_preflight = Mock()
        node.progress = Mock()
        if record is not None:
            def submit(*args,**kwargs):
                node._scored_started = True
                node.result_record["scored_attempted"] = True
                node.result_record["scored_request_id"] = record["request_id"]
                entry = dict(request_id=record["request_id"],status=record["status"],
                             failure_kind=record.get("failure_kind"),metrics=deepcopy(record["metrics"]))
                node.result_record["scored"] = entry
                return deepcopy(record),entry
            node.submit = Mock(side_effect=submit)
        else:
            node.submit = Mock()

    def test_preparation_failure_preserves_unstarted_scored_row_for_not_run_accounting(self):
        node = self.node
        self.run_stubs(preparation_error=RuntimeError("initial preparation failed"))
        with self.assertRaisesRegex(RuntimeError,"preparation failed"): node.run()
        self.assertEqual(node.result_record["status"],"aborted")
        self.assertIsNone(node.result_record["scored"])
        self.assertFalse(node.result_record["scored_attempted"])
        node.submit.assert_not_called()
        saved = json.loads(node.result_path().read_text())
        self.assertIsNone(saved["scored"])
        self.assertEqual(saved["error"]["message"],"initial preparation failed")

    def test_publish_failure_is_not_indistinguishable_from_preparation_not_run(self):
        node = self.node
        self.run_stubs()
        node.submit = Execution.submit.__get__(node,Execution)
        self.install_transport(publish_error=RuntimeError("scored publish failed"))
        with self.assertRaisesRegex(RuntimeError,"publish failed"):
            node.run()
        self.assertEqual(node.result_record["status"],"aborted")
        self.assertTrue(node.result_record["scored_attempted"])
        self.assertIsNone(node.result_record["scored"])
        saved = json.loads(node.result_path().read_text())
        self.assertTrue(saved["scored_attempted"])
        self.assertIsNotNone(saved["scored_request_id"])
        self.assertIsNone(saved["scored"])

    def test_expected_planning_failure_completes_the_evaluation_row_without_retry(self):
        node = self.node
        self.run_stubs(failed_record("budget_exhausted"))
        result = node.run()
        self.assertEqual(node.result_record["status"],"completed")
        self.assertEqual(node.result_record["scored"]["status"],"failed")
        self.assertEqual(node.result_record["scored"]["outcome"],"planning_failed")
        node.submit.assert_called_once_with(dict(joint=.2),role="scored",scored=True)
        self.assertEqual(result["scored_request_id"],"a"*32)
        self.assertEqual(result["fixed_case_result_sha256"],sha(node.result_path()))

    def test_failure_after_a_scored_request_retains_that_request_instead_of_becoming_not_run(self):
        node = self.node
        record = failed_record("execution_failed")
        record["dispatched_goals"] = 2
        self.run_stubs(record)
        with self.assertRaises(RuntimeError): node.run()
        self.assertEqual(node.result_record["status"],"aborted")
        self.assertIsNotNone(node.result_record["scored"])
        self.assertEqual(node.result_record["scored"]["request_id"],record["request_id"])
        self.assertEqual(node.result_record["scored"]["status"],"failed")
        self.assertNotIn("outcome",node.result_record["scored"])
        node.submit.assert_called_once()


if __name__ == "__main__":
    unittest.main()
