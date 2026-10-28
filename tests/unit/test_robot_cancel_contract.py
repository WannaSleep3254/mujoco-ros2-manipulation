"""Fresh post-cancel stopping evidence without importing or starting ROS."""
from copy import deepcopy
from array import array
import ast
import hashlib
import json
import math
import os
from pathlib import Path
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest.mock import patch

import numpy as np

from mujoco_ros2_manipulation.verification.verify_robot_cancel import (
    StopEvidence, HomeRecoveryEvidence, capture_confirmation, completion_flags,
    cancel_target, moving_observation, normalized_state, normalized_goal_uuid,
    cancel_diagnostic_environment)
from mujoco_ros2_manipulation.verification import verify_robot_cancel as implementation
from mujoco_ros2_manipulation.verification.reports import atomic_json


class StopEvidenceTests(unittest.TestCase):
    names = ["j1", "j2"]

    def evidence(self, **options):
        value = StopEvidence(self.names, **options)
        value.begin(terminal_received_ns=1_000_000_000, minimum_source_sec=10., minimum_clock_sec=10.,
            goal_accepted=True, movement_observed=True, cancel_response_accepted=True, action_status=5)
        return value

    def sample(self, index, **changes):
        elapsed = index*.1
        receive = 1_100_000_000+round(elapsed*1e9)
        return dict(names=list(self.names), positions=[.1, .2], velocities=[.0, .0],
            source_sec=10.1+elapsed, clock_sec=10.1+elapsed, received_ns=receive,
            clock_received_ns=receive, now_ns=receive, **changes) if not changes else dict(
                self.sample(index), **changes)

    def test_actual_fresh_position_and_velocity_window_confirms_stop(self):
        evidence = self.evidence()
        self.assertTrue(all(not evidence.observe(**self.sample(index)) for index in range(4)))
        self.assertTrue(evidence.observe(**self.sample(4)))
        snapshot = evidence.snapshot(now_ns=self.sample(4)["now_ns"])
        self.assertTrue(snapshot["physical_stop_observed"])
        self.assertEqual(snapshot["window_samples"], 5)
        self.assertGreaterEqual(snapshot["source_window_sec"], .4-1e-9)
        self.assertEqual(snapshot["maximum_velocity_rad_sec"], 0.)

    def test_partial_observation_window_never_implies_stop(self):
        evidence = self.evidence()
        for index in range(3):
            self.assertFalse(evidence.observe(**self.sample(index)))
        self.assertFalse(evidence.snapshot(now_ns=self.sample(2)["now_ns"])["physical_stop_observed"])

    def test_stream_stopping_after_a_valid_window_revokes_current_freshness(self):
        evidence = self.evidence()
        for index in range(5):
            evidence.observe(**self.sample(index))
        snapshot = evidence.snapshot(now_ns=self.sample(4)["now_ns"]+600_000_000)
        self.assertFalse(snapshot["fresh"])
        self.assertFalse(snapshot["physical_stop_observed"])

    def test_repeated_source_stamps_or_clock_are_not_new_stationary_evidence(self):
        for field in ("source_sec", "clock_sec", "received_ns"):
            evidence = self.evidence()
            for index in range(6):
                row = self.sample(index)
                row[field] = self.sample(0)[field]
                if field == "source_sec":
                    row["clock_sec"] = self.sample(0)["clock_sec"]+.01
                if field == "clock_sec":
                    row["source_sec"] = self.sample(0)["source_sec"]+.01
                evidence.observe(**row)
            with self.subTest(field=field):
                self.assertFalse(evidence.snapshot(now_ns=self.sample(5)["now_ns"])["physical_stop_observed"])
                self.assertGreater(len(evidence.rejections), 0)

    def test_old_pre_terminal_or_stale_delivery_cannot_pass(self):
        for changes in (dict(received_ns=900_000_000), dict(source_sec=9.9), dict(clock_sec=9.9),
                        dict(clock_received_ns=100_000_000), dict(now_ns=9_000_000_000)):
            evidence = self.evidence()
            self.assertFalse(evidence.observe(**self.sample(0, **changes)))
            self.assertFalse(evidence.snapshot(now_ns=9_000_000_000)["physical_stop_observed"])

    def test_position_drift_or_nonzero_velocity_resets_the_stop_window(self):
        for kind in ("position", "velocity"):
            evidence = self.evidence()
            for index in range(5):
                row = self.sample(index)
                if kind == "position":
                    row["positions"][0] += .001*index
                else:
                    row["velocities"][0] = .1
                self.assertFalse(evidence.observe(**row))
            self.assertFalse(evidence.snapshot(now_ns=self.sample(4)["now_ns"])["physical_stop_observed"])

    def test_observation_gap_clears_prior_stationary_samples(self):
        evidence = self.evidence()
        for index in range(4):
            evidence.observe(**self.sample(index))
        self.assertFalse(evidence.observe(**self.sample(10)))
        self.assertEqual(len(evidence.window), 1)
        self.assertIn("observation_gap", evidence.rejections)

    def test_backlog_burst_cannot_substitute_for_a_wall_observation_window(self):
        evidence = self.evidence()
        for index in range(5):
            row = self.sample(index)
            row["received_ns"] = 1_100_000_000+index*1_000_000
            row["now_ns"] = row["clock_received_ns"] = row["received_ns"]
            evidence.observe(**row)
        self.assertFalse(evidence.snapshot(now_ns=row["now_ns"])["physical_stop_observed"])

    def test_wall_time_alone_without_a_complete_source_and_clock_window_is_insufficient(self):
        evidence = self.evidence()
        for index in range(5):
            row = self.sample(index)
            row["source_sec"] = row["clock_sec"] = 10.1+index*.01
            evidence.observe(**row)
        snapshot = evidence.snapshot(now_ns=row["now_ns"])
        self.assertAlmostEqual(snapshot["wall_window_sec"], .4)
        self.assertAlmostEqual(snapshot["source_window_sec"], .04)
        self.assertFalse(snapshot["physical_stop_observed"])

    def test_mapping_partial_duplicate_and_nonfinite_samples_fail_closed(self):
        for changes in (dict(names=["j1"]), dict(names=["j1", "j1"]), dict(names=["j1", "other"]),
                        dict(positions=[.1]), dict(velocities=[]), dict(positions=[float("nan"), 0.]),
                        dict(velocities=[float("inf"), 0.]), dict(source_sec=float("nan")),
                        dict(clock_sec=float("inf")), dict(received_ns=True)):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                self.evidence().observe(**self.sample(0, **changes))

    def test_reordered_complete_joint_mapping_is_normalized_correctly(self):
        evidence = self.evidence()
        for index in range(5):
            row = self.sample(index, names=["j2", "j1"], positions=[.2, .1], velocities=[0., 0.])
            evidence.observe(**row)
        self.assertEqual(evidence.samples[-1]["positions_rad"], [.1, .2])
        self.assertTrue(evidence.snapshot(now_ns=row["now_ns"])["physical_stop_observed"])

    def test_no_active_moving_canceled_goal_or_acknowledgement_means_no_evidence(self):
        valid = dict(terminal_received_ns=1_000_000_000, minimum_source_sec=10., minimum_clock_sec=10.,
            goal_accepted=True, movement_observed=True, cancel_response_accepted=True, action_status=5)
        for change in (dict(goal_accepted=False), dict(movement_observed=False), dict(cancel_response_accepted=False),
                       dict(action_status=4), dict(action_status=True)):
            evidence = StopEvidence(self.names)
            with self.subTest(change=change), self.assertRaises(ValueError):
                evidence.begin(**dict(valid, **change))
        with self.assertRaisesRegex(ValueError, "No actual active cancellation"):
            StopEvidence(self.names).observe(**self.sample(0))

    def test_no_samples_are_never_zero_filled_into_physical_success(self):
        snapshot = self.evidence().snapshot(now_ns=2_000_000_000)
        self.assertFalse(snapshot["physical_stop_observed"])
        self.assertFalse(snapshot["fresh"])
        self.assertIsNone(snapshot["maximum_velocity_rad_sec"])
        self.assertEqual(snapshot["samples"], [])

    def test_source_clock_disagreement_is_rejected(self):
        evidence = self.evidence()
        self.assertFalse(evidence.observe(**self.sample(0, clock_sec=11.)))
        self.assertEqual(evidence.rejections, ["source_clock_disagreement"])

    def test_confirmation_freezes_the_accepted_snapshot_across_freshness_boundary(self):
        evidence = self.evidence()
        for index in range(5):
            evidence.observe(**self.sample(index))
        accepted = capture_confirmation(evidence, now_ns=1_999_999_999)
        self.assertTrue(accepted["physical_stop_observed"])
        self.assertEqual(accepted["confirmed_at_wall_ns"], 1_999_999_999)
        self.assertFalse(evidence.snapshot(now_ns=2_000_000_001)["physical_stop_observed"])
        self.assertIsNone(capture_confirmation(evidence, now_ns=2_000_000_001))
        evidence.samples[0]["positions_rad"][0] = 99.
        self.assertEqual(accepted["samples"][0]["positions_rad"], [.1, .2])


class HomeRecoveryTests(StopEvidenceTests):
    def evidence(self, **options):
        value = HomeRecoveryEvidence(self.names, [.1, .2], tracking_tolerance_rad=.01,
            minimum_samples=options.pop("minimum_samples", 5), **options)
        value.begin(sent_ns=800_000_000, terminal_received_ns=1_000_000_000,
            source_before_send=9.5, clock_before_send=9.5, minimum_source_sec=10., minimum_clock_sec=10.,
            goal_accepted=True, action_status=4, error_code=0)
        return value

    def test_only_fresh_advancing_post_home_target_window_confirms_recovery(self):
        evidence = self.evidence()
        for index in range(5):
            observed = evidence.observe(**self.sample(index))
        self.assertTrue(observed)
        value = capture_confirmation(evidence, now_ns=self.sample(4)["now_ns"], field="home_recovery_observed")
        self.assertTrue(value["home_recovery_observed"])
        self.assertEqual(value["maximum_target_error_rad"], 0.)
        self.assertEqual(value["action_context"]["source_at_terminal_sec"], 10.)

    def test_fresh_stationary_positions_at_the_wrong_target_are_not_home_recovery(self):
        evidence = self.evidence()
        for index in range(8):
            self.assertFalse(evidence.observe(**self.sample(index, positions=[.12, .2])))
        self.assertFalse(evidence.snapshot(now_ns=self.sample(7)["now_ns"])["home_recovery_observed"])
        self.assertIn("home_target_outside_tracking_tolerance", evidence.rejections)

    def test_stale_terminal_or_backwards_home_action_watermarks_are_rejected(self):
        valid = dict(sent_ns=800_000_000, terminal_received_ns=1_000_000_000,
            source_before_send=9.5, clock_before_send=9.5, minimum_source_sec=10., minimum_clock_sec=10.,
            goal_accepted=True, action_status=4, error_code=0)
        for change in (dict(goal_accepted=False), dict(action_status=5), dict(error_code=-5),
                       dict(action_status=True), dict(terminal_received_ns=700_000_000),
                       dict(minimum_source_sec=9.4), dict(minimum_clock_sec=9.4)):
            evidence = HomeRecoveryEvidence(self.names, [.1, .2], tracking_tolerance_rad=.01)
            with self.subTest(change=change), self.assertRaises(ValueError):
                evidence.begin(**dict(valid, **change))

    def test_completion_flags_never_convert_missing_or_false_proof_to_success(self):
        stop = StopEvidenceTests.evidence(self)
        home = self.evidence()
        for index in range(5):
            stop.observe(**self.sample(index)); home.observe(**self.sample(index))
        valid = dict(cancellation_response=dict(own_goal_acknowledged=True),
            action_terminal=dict(status=5, canceled=True),
            physical_stop=capture_confirmation(stop, now_ns=self.sample(4)["now_ns"]),
            recovery=dict(action_succeeded=True, goal_accepted=True, action_status=4, error_code=0,
                sent_wall_ns=800_000_000, terminal_received_wall_ns=1_000_000_000,
                fresh_state=capture_confirmation(home, now_ns=self.sample(4)["now_ns"], field="home_recovery_observed")))
        self.assertTrue(all(completion_flags(valid).values()))
        self.assertFalse(any(completion_flags({}).values()))
        cases = [("physical_stop", "physical_stop_observed", False, "physical_stop_observed"),
                 ("physical_stop", "confirmed_at_wall_ns", None, "physical_stop_observed"),
                 ("recovery", "action_status", 5, "home_recovery_succeeded"),
                 ("recovery", "terminal_received_wall_ns", 700_000_000, "home_recovery_succeeded")]
        for key, field, value, flag in cases:
            changed = deepcopy(valid); changed[key][field] = value
            with self.subTest(field=field):
                self.assertFalse(completion_flags(changed)[flag])
        changed = deepcopy(valid); changed["recovery"]["fresh_state"]["home_recovery_observed"] = False
        self.assertFalse(completion_flags(changed)["home_recovery_succeeded"])


class TargetAndMovementTests(unittest.TestCase):
    def setUp(self):
        self.names = ["j1", "j2"]
        self.limits = {name: dict(lower=-1., upper=1., velocity=1.) for name in self.names}

    def test_small_target_uses_actual_state_and_generated_joint_limits_without_mutation(self):
        actual = [.9, .1]
        before = deepcopy((actual, self.limits))
        target = cancel_target(self.names, actual, self.limits)
        self.assertAlmostEqual(target[0], .75)
        self.assertEqual(target[1], .1)
        self.assertEqual((actual, self.limits), before)

    def test_incomplete_nonfinite_or_outside_limit_target_inputs_fail(self):
        for positions in ([0.], [float("nan"), 0.], [2., 0.]):
            with self.subTest(positions=positions), self.assertRaises(ValueError):
                cancel_target(self.names, positions, self.limits)
        narrow = deepcopy(self.limits)
        narrow["j1"].update(lower=-.02, upper=.02)
        with self.assertRaises(ValueError):
            cancel_target(self.names, [0., 0.], narrow)
        with self.assertRaises(ValueError):
            cancel_target(["j1", "j1"], [0., 0.], {"j1": self.limits["j1"]})

    def test_movement_requires_fresh_new_actual_displacement_and_velocity(self):
        sample = dict(positions=[.01, 0.], velocities=[.03, 0.], received_ns=2_000_000_000, source_sec=10.2)
        options = dict(sent_ns=1_000_000_000, source_before=10., now_ns=2_000_000_001)
        self.assertTrue(moving_observation([0., 0.], sample, **options))
        for change in (dict(positions=[0., 0.]), dict(velocities=[0., 0.]), dict(source_sec=10.),
                       dict(received_ns=500_000_000)):
            self.assertFalse(moving_observation([0., 0.], dict(sample, **change), **options))
        self.assertFalse(moving_observation([0., 0.], sample, **dict(options, now_ns=3_000_000_000)))

    def test_malformed_movement_cannot_be_treated_as_motion(self):
        sample = dict(positions=[float("nan")], velocities=[.1], received_ns=2, source_sec=1.)
        with self.assertRaises(ValueError):
            moving_observation([0.], sample, sent_ns=1, source_before=0., now_ns=3)


class GoalUuidTests(unittest.TestCase):
    def test_numpy_uint8_and_ros_byte_array_become_native_json_integer_lists(self):
        expected = list(range(15)) + [255]
        for raw in (np.array(expected, dtype=np.uint8), array('B', expected), tuple(expected)):
            with self.subTest(transport=type(raw).__name__):
                result = normalized_goal_uuid(raw)
                self.assertEqual(result, expected)
                self.assertTrue(all(type(value) is int for value in result))
                self.assertEqual(json.loads(json.dumps(result)), expected)
                self.assertIsNot(result, raw)

    def test_uuid_length_is_exact_and_no_out_of_range_or_fractional_truncation(self):
        for raw in ([], [0] * 15, [0] * 17, [0] * 15 + [-1], [0] * 15 + [256],
                    [0] * 15 + [.5], [0] * 15 + [True], [0] * 15 + [np.bool_(True)],
                    np.array([0.] * 16), None):
            with self.subTest(raw=repr(raw)), self.assertRaises(ValueError):
                normalized_goal_uuid(raw)


class CancelDiagnosticEnvironmentTests(unittest.TestCase):
    def test_owned_launch_pins_guard_and_diagnostic_identity_without_mutating_parent(self):
        report = NS(directory=Path("/owned/new-verification-run"), run_id="owned-new-run")
        inherited = dict(MANIPULATION_CONTACT_STOP="0", MANIPULATION_DIAGNOSTICS_PATH="/stale/physics.json",
            MANIPULATION_DIAGNOSTICS_RUN_ID="another-run", MANIPULATION_STEP_WITNESS="1", CUSTOM_ENV="preserved")
        with patch.dict(os.environ, inherited, clear=True):
            result = cancel_diagnostic_environment(report, NS())
            self.assertEqual(result["MANIPULATION_CONTACT_STOP"], "1")
            self.assertEqual(result["MANIPULATION_DIAGNOSTICS_PATH"], str(report.directory / "physics.json"))
            self.assertEqual(result["MANIPULATION_DIAGNOSTICS_RUN_ID"], report.run_id)
            self.assertEqual(result["MANIPULATION_STEP_WITNESS_PATH"], str(report.directory / "step-witness.json"))
            self.assertEqual(result["CUSTOM_ENV"], "preserved")
            self.assertEqual(dict(os.environ), inherited)


def wrapper_class(*, stale_home=False, post_stop_delay_ns=0, uuid_factory=None):
    """Load the actual lazy ROS wrapper with transports/physics replaced.

    This verifies orchestration and proof persistence, not actual ROS, controller
    timing, native physics or the legacy baseline verification itself.
    """
    wall = NS(now=1_000_000_000)
    uuid_factory = uuid_factory or (lambda values: np.array(values, dtype=np.uint8))
    class Future:
        def __init__(self, value): self.value, self.finished = value, False
        def done(self): return self.finished
    class Point:
        def __init__(self):
            self.positions, self.velocities = [], []
            self.time_from_start = NS(sec=0, nanosec=0)
    class Goal:
        def __init__(self): self.trajectory = NS(joint_names=[], points=[])
    class Handle:
        def __init__(self, number):
            self.accepted, self.goal_id = True, NS(uuid=uuid_factory([0] * 15 + [number]))
            self.future = Future(NS(status=5 if number == 1 else 4, result=NS(error_code=0, error_string="")))
        def get_result_async(self): return self.future
        def cancel_goal_async(self):
            return Future(NS(return_code=0, goals_canceling=[NS(goal_id=NS(uuid=array('B', self.goal_id.uuid)))]))
    class Client:
        def __init__(self, *_): self.number = 0
        def server_is_ready(self): return True
        def send_goal_async(self, _):
            self.number += 1
            return Future(Handle(self.number))
    class Base:
        def __init__(self, profile, progress):
            self.profile, self.joint_names, self.settings = profile, profile.joints, profile.data["verification"]
            self.progress = progress or (lambda stage: None)
            self.positions, self.joint_count, self.clock_count = [0., 0.], 1, 1
            self.clock_last, self.clock_backwards = 10., False
            self.clock_received_ns = wall.now
            self.latest_sample = dict(names=list(self.joint_names), positions=[0., 0.], velocities=[0., 0.],
                source_sec=10., received_ns=wall.now)
        def run(self): return dict(baseline="transport/physics replaced in this unit test")
        def clock(self, message):
            value = message.clock.sec + message.clock.nanosec * 1e-9
            self.clock_backwards |= value < self.clock_last
            self.clock_last = value; self.clock_count += 1
        def joints(self, message):
            mapping = dict(zip(message.name, message.position))
            self.positions = [mapping[name] for name in self.joint_names]; self.joint_count += 1
        def result(self, future, *_):
            wall.now += 50_000_000; future.finished = True
            return future.value
        def feed(self, positions, velocities, *, advance_source=True):
            wall.now += 100_000_000
            value = self.clock_last + (.1 if advance_source else 0.)
            sec = math.floor(value); stamp = NS(sec=sec, nanosec=round((value - sec) * 1e9))
            self.clock(NS(clock=stamp))
            self.joints(NS(name=list(self.joint_names), position=positions, velocity=velocities,
                           header=NS(stamp=stamp)))
        def until(self, predicate, seconds, description):
            if description == "fresh actual movement before cancellation":
                self.feed([.01, 0.], [.03, 0.])
            for _ in range(20):
                if "joint position/velocity stop window" in description:
                    self.feed([.01, 0.], [0., 0.])
                elif "home target/velocity/clock window" in description:
                    self.feed([0., 0.], [0., 0.], advance_source=not stale_home)
                if predicate():
                    if "joint position/velocity stop window" in description:
                        wall.now += post_stop_delay_ns
                    return
            raise RuntimeError("Timeout: " + description)
    source = Path(implementation.__file__)
    tree = ast.parse(source.read_text(), filename=str(source))
    factory = next(item for item in tree.body if isinstance(item, ast.FunctionDef) and item.name == "node_class")
    definition = next(item for item in factory.body if isinstance(item, ast.ClassDef))
    namespace = dict(__name__="cancel_wrapper_unit_fixture", __file__=str(source),
        Verification=Base, ActionClient=Client, GoalStatus=NS(STATUS_CANCELED=5, STATUS_SUCCEEDED=4),
        FollowJointTrajectory=NS(Goal=Goal, Result=NS(SUCCESSFUL=0)), JointTrajectoryPoint=Point,
        _finite=implementation._finite, _require=implementation._require,
        StopEvidence=StopEvidence, HomeRecoveryEvidence=HomeRecoveryEvidence,
        normalized_state=normalized_state, capture_confirmation=capture_confirmation,
        normalized_goal_uuid=normalized_goal_uuid,
        completion_flags=completion_flags, cancel_target=cancel_target, moving_observation=moving_observation,
        Path=Path, hashlib=hashlib, atomic_json=atomic_json, json=json, deepcopy=deepcopy,
        time=NS(monotonic_ns=lambda: wall.now))
    exec(compile(ast.fix_missing_locations(ast.Module(body=[definition], type_ignores=[])), str(source), "exec"), namespace)
    return namespace["RobotCancelVerification"]


class WrapperOrchestrationTests(unittest.TestCase):
    def profile(self, root):
        model = root / "model"; model.mkdir()
        (model / "source.json").write_text(json.dumps(dict(joint_limits={
            name: dict(lower=-1., upper=1., velocity=1.) for name in ("j1", "j2")})))
        return NS(id="fr3", output=model, joints=["j1", "j2"], home=[0., 0.],
            ros=dict(trajectory_controller="arm_controller"), data=dict(verification=dict(
                tracking_tolerance_rad=.01, settle_messages=5, trajectory_duration_sec=3.)))

    def test_actual_wrapper_keeps_authoritative_stop_across_post_predicate_delay(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            phases = []
            node = wrapper_class(post_stop_delay_ns=600_000_000)(self.profile(root), phases.append, output=root / "receipt")
            result = node.run()
            self.assertTrue(result["physical_stop_observed"])
            self.assertTrue(result["home_recovery_succeeded"])
            saved = json.loads((root / "receipt/cancel-verification.json").read_text())
            self.assertEqual(saved["status"], "completed")
            self.assertTrue(saved["physical_stop"]["physical_stop_observed"])
            self.assertTrue(saved["recovery"]["fresh_state"]["home_recovery_observed"])
            self.assertGreater(saved["recovery"]["sent_wall_ns"] - saved["physical_stop"]["confirmed_at_wall_ns"], 500_000_000)
            self.assertNotEqual(saved["recovery"]["goal_uuid"], saved["cancellation_response"]["own_goal_uuid"])
            self.assertEqual(phases, ["active_cancel_goal", "cancel_response", "action_terminal",
                "physical_stop_window", "home_recovery"])
            for value in (saved["recovery"]["goal_uuid"], saved["cancellation_response"]["own_goal_uuid"]):
                self.assertEqual(len(value), 16)
                self.assertTrue(all(type(byte) is int for byte in value))

    def test_actual_wrapper_accepts_ros_array_byte_goal_uuids(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            node = wrapper_class(uuid_factory=lambda values: array('B', values))(
                self.profile(root), output=root / "receipt")
            self.assertTrue(node.run()["home_recovery_succeeded"])
            saved = json.loads((root / "receipt/cancel-verification.json").read_text())
            self.assertEqual(saved["cancellation_response"]["own_goal_uuid"], [0] * 15 + [1])
            self.assertEqual(saved["recovery"]["goal_uuid"], [0] * 15 + [2])

    def test_actual_wrapper_cannot_finish_home_from_repeated_source_clock_states(self):
        with tempfile.TemporaryDirectory() as directory:
            root = Path(directory)
            phases = []
            node = wrapper_class(stale_home=True)(self.profile(root), phases.append, output=root / "receipt")
            with self.assertRaisesRegex(RuntimeError, "fresh advancing home"):
                node.run()
            saved = json.loads((root / "receipt/cancel-verification.json").read_text())
            self.assertEqual(saved["status"], "failed")
            self.assertTrue(saved["recovery"]["action_succeeded"])
            self.assertFalse(saved["recovery"]["fresh_state"]["home_recovery_observed"])
            self.assertFalse(completion_flags(saved)["home_recovery_succeeded"])
            self.assertEqual(phases[-1], "home_recovery")
            self.assertEqual(saved["phase"], "home_recovery")


if __name__ == "__main__":
    unittest.main()
