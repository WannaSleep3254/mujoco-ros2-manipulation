"""Exercise production owner-thread observation intake without ROS or processes."""
import ast
from copy import deepcopy
import io
from pathlib import Path
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from mujoco_ros2_manipulation.grasping import ros_admission
from mujoco_ros2_manipulation.grasping.ros_observation import ObservationUnavailable
from mujoco_ros2_manipulation.grasping.ros_task import (
    CONTROLLERS, MAX_TASK_WALL_SEC, OBSERVATION_INTAKE_SEC, STALE_OBSERVATION_REASONS,
    RosGraspCancelled, RosGraspTaskError, _require, admitted_action_submission,
)
from . import test_ros_grasp_observation as observation_fixtures
from .test_ros_grasp_observation import guard, sample


SOURCE = Path(__file__).resolve().parents[2] / 'src/mujoco_ros2_manipulation/grasping/ros_task.py'


def owner_functions():
    run = next(node for node in ast.parse(SOURCE.read_text()).body
               if isinstance(node, ast.FunctionDef) and node.name == 'run_ros_grasp')
    names = {'admission_context', 'check_control', 'current', 'abort', 'spin',
             'check_file_identity', 'scene_mutation_guard', 'scene_observation'}
    return ast.Module(body=[deepcopy(node) for node in run.body
                           if isinstance(node, ast.FunctionDef) and node.name in names], type_ignores=[])


class ProofPath:
    def __init__(self, fixture):
        self.fixture = fixture

    def __truediv__(self, name):
        return self

    def exists(self):
        return self.fixture.proof_ready


class IntakeFixture:
    def __init__(self):
        self.wall = 10.61
        self.spin_cost = .001
        self.queue = []
        self.calls = []
        self.cancelled = False
        self.launch_exit = None
        self.proof_ready = False
        self.member_scan = None
        self.store = observation_fixtures.RosGraspObservationTests().store()
        self.store.ingest(sample(), receipt_sec=10.)
        self.store.ingest(sample(2, .05), receipt_sec=10.1)
        self.store.update_guard(guard(), receipt_sec=10.3)
        self.latest = dict(error=None, guard=guard(), accepted_context=dict(
            native_sequence=2, capture_monotonic_ns=10_050_000_000, received_monotonic_ns=10_100_000_000))
        self.record = dict(cleanup={'status': 'not_started'}, observation_intake=dict(
            budget_sec=.01, maximum_rows=32, rows=[], dropped_rows=0))
        self.worker = ros_admission.ReadOnlyAdmissionWorker.__new__(ros_admission.ReadOnlyAdmissionWorker)
        self.worker._clock = lambda: self.wall
        self.worker._closed = self.worker._abandoned = False
        self.worker._pending = None
        self.worker._generation = 0
        self.worker._result_consumed = False
        self.worker._token = 'offline-validation-owner'
        self.worker.directory = ProofPath(self)
        self.worker.process = SimpleNamespace(stdin=io.BytesIO(), poll=lambda: None)
        self.worker._remember_members = self.remember_members
        self.worker._interrupt = Mock()
        self.namespace = dict(
            time=SimpleNamespace(monotonic=lambda: self.wall, monotonic_ns=lambda: round(self.wall*1e9)),
            store=self.store, latest=self.latest, record=self.record,
            process=SimpleNamespace(poll=lambda: self.launch_exit),
            executor=SimpleNamespace(spin_once=self.spin_once),
            cancel_event=SimpleNamespace(is_set=lambda: self.cancelled),
            change_phase=lambda phase: self.calls.append(('phase', phase)),
            ready=True, start_wall=10., baseline_right=None, monitor_held=False,
            MAX_TASK_WALL_SEC=MAX_TASK_WALL_SEC, CONTROLLERS=CONTROLLERS,
            OBSERVATION_INTAKE_SEC=OBSERVATION_INTAKE_SEC,
            STALE_OBSERVATION_REASONS=STALE_OBSERVATION_REASONS,
            _require=_require, RosGraspCancelled=RosGraspCancelled, ObservationUnavailable=ObservationUnavailable,
            identity_worker=self.worker, phase=[2], manifest_sha256='1'*64, input_hashes={}, identity_paths=[],
            verify_identity_proof=Mock())
        exec(compile(owner_functions(), str(SOURCE), 'exec'), self.namespace)

    def spin_once(self, *, timeout_sec):
        self.calls.append(('spin', timeout_sec))
        self.wall += self.spin_cost
        if self.queue:
            self.queue.pop(0)()

    def fresh_native(self):
        received_ns = round(self.wall*1e9)
        sequence = self.store._latest.payload['sequence'] + 1
        value = sample(sequence, (sequence-1)*.05, capture_sec=self.wall-.01)
        accepted = self.store.ingest(value, receipt_sec=self.wall)
        if not accepted.accepted:
            self.latest['error'] = accepted.reason
            return
        self.latest['accepted_context'] = dict(native_sequence=sequence,
            capture_monotonic_ns=int(value['capture_monotonic_ns']), received_monotonic_ns=received_ns)

    def fresh_guard(self):
        self.latest['guard'] = guard()
        self.store.update_guard(self.latest['guard'], receipt_sec=self.wall)

    def remember_members(self):
        if self.member_scan is not None:
            self.member_scan()
        self.proof_ready = True

    def call(self, name, **kwargs):
        return self.namespace[name](**kwargs)

    def proof(self, path):
        return dict(token=self.worker._token, generation=self.worker._generation)

    @property
    def spins(self):
        return [entry for entry in self.calls if entry[0] == 'spin']


class ObservationIntakeTests(unittest.TestCase):
    def test_queued_fresh_native_is_admitted_with_its_original_capture(self):
        fixture = IntakeFixture()
        fixture.queue = [fixture.fresh_native]
        value = fixture.call('current')
        self.assertEqual(value.capture_monotonic_ns, 10_601_000_000)
        self.assertAlmostEqual(value.receipt_sec, 10.611)
        row = fixture.record['observation_intake']['rows'][0]
        self.assertTrue(row['recovered'])
        self.assertEqual(row['before']['native_sequence'], 2)
        self.assertEqual(row['after']['native_sequence'], 3)
        self.assertEqual(row['after']['capture_monotonic_ns'], value.capture_monotonic_ns)
        self.assertEqual(fixture.spins, [('spin', 0)])

    def test_timer_and_guard_callbacks_ahead_of_native_are_drained(self):
        fixture = IntakeFixture()
        fixture.queue = [lambda: None, fixture.fresh_guard, lambda: None, fixture.fresh_native]
        value = fixture.call('current')
        self.assertEqual(value.payload['sequence'], 3)
        self.assertEqual(len(fixture.spins), 4)
        self.assertTrue(all(timeout == 0 for _, timeout in fixture.spins))

    def test_already_fresh_current_performs_no_executor_intake(self):
        fixture = IntakeFixture()
        fixture.fresh_native()
        fixture.call('current')
        self.assertEqual(fixture.spins, [])
        self.assertEqual(fixture.record['observation_intake']['rows'], [])

    def test_actual_stale_and_held_clock_exhaust_budget_without_freshening(self):
        fixture = IntakeFixture()
        with self.assertRaises(ObservationUnavailable) as failure:
            fixture.call('current')
        self.assertEqual(failure.exception.reason, 'clock_stale')
        self.assertEqual(fixture.store._latest.capture_monotonic_ns, 10_050_000_000)
        self.assertLessEqual(len(fixture.spins), 11)
        row = fixture.record['observation_intake']['rows'][0]
        self.assertTrue(row['budget_exhausted'])
        self.assertFalse(row['recovered'])
        self.assertEqual(row['failure']['reason'], 'clock_stale')

    def test_native_phase_eight_does_not_enter_stale_recovery(self):
        fixture = IntakeFixture()
        fixture.store.ingest(sample(3, .1, phase=8, capture_sec=10.6), receipt_sec=10.61)
        with self.assertRaises(ObservationUnavailable) as failure:
            fixture.call('current')
        self.assertEqual(failure.exception.reason, 'native_held')
        self.assertEqual(fixture.spins, [])

    def test_future_receipt_and_invalid_input_are_not_recoverable(self):
        fixture = IntakeFixture()
        fixture.wall = 10.09
        with self.assertRaises(ObservationUnavailable) as failure:
            fixture.call('current')
        self.assertEqual(failure.exception.reason, 'future_receipt')
        self.assertEqual(fixture.spins, [])
        fixture = IntakeFixture()
        fixture.latest['error'] = 'invalid native identity'
        with self.assertRaisesRegex(RosGraspTaskError, 'invalid native'):
            fixture.call('current')
        self.assertEqual(fixture.spins, [])

    def test_expired_queued_source_is_rejected_at_its_original_capture(self):
        fixture = IntakeFixture()
        def expired():
            accepted = fixture.store.ingest(sample(3, .1, capture_sec=10.1), receipt_sec=fixture.wall)
            fixture.latest['error'] = accepted.reason
        fixture.queue = [expired]
        with self.assertRaisesRegex(RosGraspTaskError, 'already stale'):
            fixture.call('current')
        self.assertEqual(fixture.store._latest.capture_monotonic_ns, 10_050_000_000)

    def test_cancel_process_death_and_known_guard_stop_precede_intake(self):
        for failure in ('cancel', 'process', 'latch', 'inactive'):
            with self.subTest(failure=failure):
                fixture = IntakeFixture()
                fixture.queue = [fixture.fresh_native]
                if failure == 'cancel':
                    fixture.cancelled = True
                elif failure == 'process':
                    fixture.launch_exit = 0
                elif failure == 'latch':
                    fixture.latest['guard']['latched'] = True
                else:
                    fixture.latest['guard']['controllers']['gripper_controller'] = 'inactive'
                with self.assertRaises((RosGraspTaskError, ObservationUnavailable)):
                    fixture.call('current')
                self.assertEqual(fixture.spins, [])

    def test_cancel_latch_and_process_death_inside_intake_cannot_return_authority(self):
        for failure in ('cancel', 'latch', 'process'):
            with self.subTest(failure=failure):
                fixture = IntakeFixture()
                def callback():
                    fixture.fresh_native()
                    if failure == 'cancel':
                        fixture.cancelled = True
                    elif failure == 'latch':
                        fixture.latest['guard']['latched'] = True
                    else:
                        fixture.launch_exit = 1
                fixture.queue = [callback]
                with self.assertRaises((RosGraspTaskError, ObservationUnavailable)):
                    fixture.call('current')
                self.assertFalse(fixture.record['observation_intake']['rows'][0]['recovered'])

    def test_callback_exceeding_budget_cannot_supply_physical_authority(self):
        fixture = IntakeFixture()
        fixture.spin_cost = .02
        fixture.queue = [fixture.fresh_native]
        with self.assertRaises(ObservationUnavailable) as failure:
            fixture.call('current')
        self.assertEqual(failure.exception.reason, 'clock_stale')
        self.assertTrue(fixture.record['observation_intake']['rows'][0]['budget_exhausted'])
        self.assertFalse(fixture.record['observation_intake']['rows'][0]['recovered'])

    def test_spin_shares_its_original_budget_with_additional_ready_callbacks(self):
        fixture = IntakeFixture()
        fixture.queue = [lambda: None, fixture.fresh_guard, fixture.fresh_native]
        fixture.call('spin')
        self.assertLessEqual(fixture.spins[0][1], .01)
        self.assertTrue(all(timeout == 0 for _, timeout in fixture.spins[1:]))
        self.assertLess(fixture.wall-10.61, .01)

    def test_spin_wait_consuming_budget_does_not_start_another_budget(self):
        fixture = IntakeFixture()
        fixture.spin_cost = .011
        fixture.queue = [lambda: None, fixture.fresh_native]
        with self.assertRaises(ObservationUnavailable):
            fixture.call('spin')
        self.assertEqual(len(fixture.spins), 1)
        self.assertEqual(len(fixture.queue), 1)

    def test_scene_suppliers_and_actual_validator_abort_hooks_use_common_intake(self):
        for name in ('scene_observation', 'scene_mutation_guard', 'check_file_identity'):
            with self.subTest(name=name):
                fixture = IntakeFixture()
                fixture.queue = [fixture.fresh_native]
                with patch.object(ros_admission, '_read_json', side_effect=fixture.proof):
                    fixture.call(name)
                self.assertGreater(len(fixture.spins), 0)
                if name != 'scene_observation':
                    self.assertTrue(fixture.worker._result_consumed)
                    self.assertFalse(fixture.worker._abandoned)

    def test_validator_scan_delay_is_followed_by_intake_before_snapshot(self):
        fixture = IntakeFixture()
        fixture.fresh_native()
        def scan():
            fixture.wall += .51
            fixture.queue.extend([fixture.fresh_guard, fixture.fresh_native])
        fixture.member_scan = scan
        with patch.object(ros_admission, '_read_json', side_effect=fixture.proof):
            fixture.call('check_file_identity')
        self.assertTrue(fixture.worker._result_consumed)
        self.assertFalse(fixture.worker._abandoned)

    def test_action_admission_sends_exactly_once_after_fresh_intake(self):
        fixture = IntakeFixture()
        fixture.queue = [fixture.fresh_native]
        goals = []
        admitted_action_submission(fixture.namespace['abort'], lambda: None, lambda: None,
                                   lambda: goals.append('one'))
        self.assertEqual(goals, ['one'])

    def test_failed_action_admission_never_sends_or_retries(self):
        for failure in ('stale', 'cancel', 'latch', 'process', 'budget', 'future', 'held', 'invalid'):
            with self.subTest(failure=failure):
                fixture = IntakeFixture()
                if failure == 'cancel':
                    fixture.cancelled = True
                elif failure == 'latch':
                    fixture.latest['guard']['latched'] = True
                elif failure == 'process':
                    fixture.launch_exit = 1
                elif failure == 'future':
                    fixture.wall = 10.09
                elif failure == 'held':
                    fixture.store.ingest(sample(3, .1, phase=8, capture_sec=10.6), receipt_sec=10.61)
                elif failure == 'invalid':
                    fixture.latest['error'] = 'invalid source'
                elif failure == 'budget':
                    fixture.spin_cost = .02
                    fixture.queue = [fixture.fresh_native]
                goals = []
                with self.assertRaises((RosGraspTaskError, ObservationUnavailable)):
                    admitted_action_submission(fixture.namespace['abort'], lambda: None, lambda: None,
                                               lambda: goals.append('forbidden'))
                self.assertEqual(goals, [])

    def test_intake_diagnostics_are_bounded_and_do_not_invoke_save(self):
        fixture = IntakeFixture()
        for _ in range(34):
            with self.assertRaises(ObservationUnavailable):
                fixture.call('current')
        trace = fixture.record['observation_intake']
        self.assertEqual(len(trace['rows']), 32)
        self.assertEqual(trace['dropped_rows'], 2)
        self.assertEqual(trace['rows'][0]['before']['capture_monotonic_ns'], 10_050_000_000)

    def test_budget_expiry_preserves_the_initial_exception_object(self):
        fixture = IntakeFixture()
        original = ObservationUnavailable('original stale field', reason='capture_stale')
        with patch.object(fixture.store, 'snapshot', side_effect=original):
            with self.assertRaises(ObservationUnavailable) as failure:
                fixture.call('current')
        self.assertIs(failure.exception, original)

    def test_legacy_unclassified_error_does_not_enter_intake(self):
        fixture = IntakeFixture()
        with patch.object(fixture.store, 'snapshot', side_effect=ObservationUnavailable('legacy')):
            with self.assertRaisesRegex(ObservationUnavailable, 'legacy'):
                fixture.call('current')
        self.assertEqual(fixture.spins, [])

    def test_right_arm_hold_and_physical_contact_checks_remain_after_intake(self):
        for failure in ('right_arm', 'physical_hold'):
            with self.subTest(failure=failure):
                fixture = IntakeFixture()
                if failure == 'right_arm':
                    fixture.namespace['baseline_right'] = {'right_shoulder_pan_joint': 0.08 - .02}
                else:
                    fixture.namespace['monitor_held'] = True
                    fixture.namespace['held_reference'] = ([0, 0, 0], [1, 0, 0, 0])
                    fixture.namespace['contact_evidence'] = lambda *args: dict(
                        bilateral_contact=False, relative_position_error_m=0., relative_angle_error_rad=0.)
                fixture.queue = [fixture.fresh_native]
                goals = []
                with self.assertRaisesRegex(RosGraspTaskError, 'hold tolerance|slipped'):
                    admitted_action_submission(fixture.namespace['abort'], lambda: None, lambda: None,
                                               lambda: goals.append('forbidden'))
                self.assertEqual(goals, [])

    def test_cleanup_spin_does_not_reenter_cancelled_motion_intake(self):
        fixture = IntakeFixture()
        fixture.cancelled = True
        fixture.launch_exit = 0
        fixture.record['cleanup']['status'] = 'running'
        fixture.call('spin', health=False)
        self.assertEqual(len(fixture.spins), 1)
        self.assertEqual(fixture.record['observation_intake']['rows'], [])

    def test_store_reasons_distinguish_stale_future_invalid_and_stopped(self):
        fixture = IntakeFixture()
        fixture.store.ingest(sample(3, .1, capture_sec=10.1), receipt_sec=10.55)
        fixture.store.update_guard(guard(), receipt_sec=10.55)
        with self.assertRaises(ObservationUnavailable) as failure:
            fixture.store.snapshot(now_sec=10.61, require_physical=True)
        self.assertEqual(failure.exception.reason, 'capture_stale')
        fixture = IntakeFixture()
        fixture.fresh_native()
        fixture.store.update_guard(guard(), receipt_sec=10.7)
        with self.assertRaises(ObservationUnavailable) as failure:
            fixture.store.snapshot(now_sec=10.61, require_physical=True)
        self.assertEqual(failure.exception.reason, 'future_guard')
        fixture = IntakeFixture()
        fixture.fresh_native()
        self.assertTrue(fixture.store.update_guard(guard(latched=True), receipt_sec=10.61).accepted)
        with self.assertRaises(ObservationUnavailable) as failure:
            fixture.store.snapshot(now_sec=10.61, require_physical=True)
        self.assertEqual(failure.exception.reason, 'guard_stopped')
        bad = sample(4, .15, capture_sec=10.61)
        bad['model_sha256'] = 'f'*64
        self.assertFalse(fixture.store.ingest(bad, receipt_sec=10.61).accepted)
        with self.assertRaises(ObservationUnavailable) as failure:
            fixture.store.snapshot(now_sec=10.61)
        self.assertEqual(failure.exception.reason, 'invalid_native')

    def test_nonphysical_snapshot_keeps_optional_guard_and_held_recording_behavior(self):
        fixture = IntakeFixture()
        fixture.store.ingest(sample(3, .1, phase=8, capture_sec=10.6), receipt_sec=10.61)
        fixture.store.update_guard(guard(latched=True), receipt_sec=10.61)
        value = fixture.store.snapshot(now_sec=10.61)
        self.assertEqual(value.payload['phase'], 8)
        self.assertEqual(value.capture_monotonic_ns, 10_600_000_000)
        with self.assertRaises(ObservationUnavailable) as failure:
            fixture.store.snapshot(now_sec=10.61, require_physical=True)
        self.assertEqual(failure.exception.reason, 'native_held')

    def test_native_callback_then_stale_guard_drains_the_queued_guard(self):
        fixture = IntakeFixture()
        fixture.wall = 10.85
        fixture.queue = [fixture.fresh_native, fixture.fresh_guard]
        fixture.call('current')
        row = fixture.record['observation_intake']['rows'][0]
        self.assertEqual(row['spin_once_calls'], 2)
        self.assertEqual(row['last_unavailable_reason'], 'guard_stale')
        self.assertTrue(row['recovered'])

    def test_future_source_arriving_inside_intake_cannot_submit_a_goal(self):
        fixture = IntakeFixture()
        def future():
            accepted = fixture.store.ingest(sample(3, .1, capture_sec=fixture.wall+.01), receipt_sec=fixture.wall)
            fixture.latest['error'] = accepted.reason
        fixture.queue = [future]
        goals = []
        with self.assertRaises(RosGraspTaskError):
            admitted_action_submission(fixture.namespace['abort'], lambda: None, lambda: None,
                                       lambda: goals.append('forbidden'))
        self.assertEqual(goals, [])
        self.assertFalse(fixture.record['observation_intake']['rows'][0]['recovered'])

    def test_controller_inactivation_inside_intake_cannot_submit_a_goal(self):
        fixture = IntakeFixture()
        def inactive():
            fixture.latest['guard']['controllers']['left_controller'] = 'inactive'
        fixture.queue = [inactive]
        goals = []
        with self.assertRaises(ObservationUnavailable) as failure:
            admitted_action_submission(fixture.namespace['abort'], lambda: None, lambda: None,
                                       lambda: goals.append('forbidden'))
        self.assertEqual(failure.exception.reason, 'guard_inactive')
        self.assertEqual(goals, [])

    def test_legacy_exception_construction_keeps_args_and_has_no_recovery_reason(self):
        failure = ObservationUnavailable('legacy error')
        self.assertEqual(str(failure), 'legacy error')
        self.assertEqual(failure.args, ('legacy error',))
        self.assertIsNone(failure.reason)
