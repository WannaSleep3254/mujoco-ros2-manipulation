"""Action ownership/failure interleavings, without ROS or a running simulator."""
from array import array
from concurrent.futures import Future
import json
from types import SimpleNamespace as NS
import unittest

from mujoco_ros2_manipulation.workcells.execution_adapter import (
    ExecutionCleanupError, ExecutionLedger, TrajectoryExecutionAdapter)


def completed(value):
    future = Future()
    future.set_result(value)
    return future


def answer(status=4, error_code=0):
    return NS(status=status, result=NS(error_code=error_code))


class Handle:
    def __init__(self, *, accepted=True, result=None, cancel_completes=True,
                 cancel_error=None, uuid=None, result_failures=0):
        self.accepted = accepted
        self.result_future = Future() if result is None else completed(result)
        self.cancel_future = completed(NS(goals_canceling=[]))
        self.cancel_completes = cancel_completes
        self.cancel_error = cancel_error
        self.cancel_calls = 0
        self.result_calls = 0
        self.result_failures = result_failures
        if uuid is not None:
            self.goal_id = NS(uuid=uuid)

    def get_result_async(self):
        self.result_calls += 1
        if self.result_failures:
            self.result_failures -= 1
            raise RuntimeError("result future unavailable")
        return self.result_future

    def cancel_goal_async(self):
        self.cancel_calls += 1
        if self.cancel_error:
            raise self.cancel_error
        if self.cancel_completes and not self.result_future.done():
            self.result_future.set_result(answer(5))
        return self.cancel_future


class Client:
    def __init__(self, future):
        self.future = future
        self.goals = []

    def send_goal_async(self, goal):
        self.goals.append(goal)
        return self.future


class Harness:
    def __init__(self, left=None, right=None, *, until=None, abort=None,
                 event=None, on_sent=None):
        self.left = left or Handle(result=answer())
        self.right = right or Handle(result=answer())
        self.clients = dict(left=Client(self.left if isinstance(self.left, Future) else completed(self.left)),
                            right=Client(self.right if isinstance(self.right, Future) else completed(self.right)),
                            unrelated=Client(completed(Handle())))
        self.waits = []
        self.events = []
        self.now = 10.0
        self.until_hook = until
        self.wait_hook = None
        self.external_event = event
        self.adapter = TrajectoryExecutionAdapter(self.clients,
            goal_factory=lambda trajectory: NS(trajectory=trajectory),
            wait_result=self.result, wait_until=self.until,
            simulation_now=lambda: self.now, event=self.event, abort=abort,
            on_sent=on_sent)
        self.trajectories = [(NS(id=arm, controller=arm + "_controller"), NS(
            joint_names=[arm + "_j1"], points=[NS(positions=[.2])]))
            for arm in ("left", "right")]

    def result(self, future, seconds, description):
        self.waits.append((seconds, description, future))
        if self.wait_hook:
            self.wait_hook(future, description)
        if not future.done():
            raise TimeoutError(description)
        result = future.result()
        if result is None:
            raise RuntimeError("No result: " + description)
        return result

    def until(self, predicate, seconds, description):
        self.waits.append((seconds, description, None))
        if self.until_hook:
            self.until_hook()
        if not predicate():
            raise TimeoutError(description)

    def event(self, name, arm, **details):
        self.events.append((name, arm, details))
        if self.external_event:
            self.external_event(name, arm, **details)


class ExecutionAdapterTests(unittest.TestCase):
    def test_success_preserves_trajectories_bounds_counts_uuid_and_visible_handles(self):
        import numpy as np
        left = Handle(result=answer(), uuid=np.arange(16, dtype=np.uint8))
        right = Handle(result=answer(), uuid=array('B', range(16, 32)))
        observed = []
        ledger = ExecutionLedger(total_dispatched=7)
        harness = Harness(left, right, on_sent=lambda current: observed.append(
            (current.total_dispatched, len(current.handles))))
        visible = ledger.handles
        self.assertIs(harness.adapter.dispatch(harness.trajectories, ledger, scheduled_start=10.5), ledger)
        self.assertIs(visible, ledger.handles)
        self.assertEqual(ledger.total_dispatched, 9)
        self.assertEqual(observed, [(8, 0), (9, 0)])
        self.assertEqual(len(visible), 2)
        self.assertEqual(ledger.acceptance_margins, [.5])
        self.assertFalse(ledger.cleanup_requested)
        self.assertEqual(left.cancel_calls + right.cancel_calls, 0)
        for index, arm in enumerate(('left', 'right')):
            self.assertIs(harness.clients[arm].goals[0].trajectory, harness.trajectories[index][1])
        self.assertEqual([attempt.goal_uuid for attempt in ledger.attempts],
                         [bytes(range(16)).hex(), bytes(range(16, 32)).hex()])
        self.assertEqual([name for name, _, _ in harness.events],
            ['goal_sent', 'goal_sent', 'goal_acceptance', 'goal_acceptance', 'goal_result', 'goal_result'])
        self.assertEqual([(seconds, description) for seconds, description, _ in harness.waits],
            [(5, 'arm goal acceptance'), (5, 'arm goal acceptance'), (45, 'all arm results')])
        self.assertEqual(harness.clients['unrelated'].goals, [])

    def test_rejected_peer_cancels_only_the_accepted_owned_goal(self):
        left, right = Handle(), Handle(accepted=False)
        harness, ledger = Harness(left, right), ExecutionLedger()
        with self.assertRaisesRegex(RuntimeError, "Arm goal rejected") as raised:
            harness.adapter.dispatch(harness.trajectories, ledger)
        self.assertNotIsInstance(raised.exception, ExecutionCleanupError)
        self.assertEqual(left.cancel_calls, 1)
        self.assertEqual(right.cancel_calls, 0)
        self.assertFalse(ledger.cleanup_uncertain)
        self.assertTrue(ledger.attempts[0].terminal_seen)
        self.assertEqual(ledger.cleanup_errors, [])
        self.assertEqual(harness.clients['unrelated'].goals, [])

    def test_failed_arm_cancels_active_peer_and_reports_failure_once(self):
        left, right = Handle(result=answer(6, -4)), Handle()
        harness, ledger = Harness(left, right), ExecutionLedger()
        with self.assertRaisesRegex(RuntimeError, "An arm failed"):
            harness.adapter.dispatch(harness.trajectories, ledger)
        self.assertEqual(left.cancel_calls, 0)
        self.assertEqual(right.cancel_calls, 1)
        results = [event for event in harness.events if event[0] == 'goal_result']
        self.assertEqual([(arm, detail['status']) for _, arm, detail in results], [('left', 6)])
        self.assertTrue(all(attempt.terminal_seen for attempt in ledger.attempts))

    def test_partial_success_and_terminal_timeout_never_report_complete_success(self):
        left, right = Handle(result=answer()), Handle()
        harness, ledger = Harness(left, right), ExecutionLedger()
        with self.assertRaises(TimeoutError) as raised:
            harness.adapter.dispatch(harness.trajectories, ledger)
        self.assertEqual(str(raised.exception), 'all arm results')
        self.assertEqual(left.cancel_calls, 0)
        self.assertEqual(right.cancel_calls, 1)
        self.assertEqual(ledger.total_dispatched, 2)
        self.assertTrue(any(seconds == 45 for seconds, _, _ in harness.waits))

    def test_cancel_acknowledgement_is_not_terminal_confirmation(self):
        left, right = Handle(cancel_completes=False), Handle(cancel_completes=False)
        original = RuntimeError('operator requested cancel')
        harness = Harness(left, right, until=lambda: (_ for _ in ()).throw(original))
        ledger = ExecutionLedger()
        with self.assertRaises(ExecutionCleanupError) as raised:
            harness.adapter.dispatch(harness.trajectories, ledger)
        self.assertIs(raised.exception.__cause__, original)
        self.assertIs(raised.exception.primary, original)
        self.assertTrue(all(attempt.cancel_acknowledged for attempt in ledger.attempts))
        self.assertFalse(any(attempt.terminal_seen for attempt in ledger.attempts))
        self.assertTrue(ledger.cleanup_uncertain)
        self.assertEqual(left.cancel_calls + right.cancel_calls, 2)
        self.assertTrue(all(seconds == 5 for seconds, description, _ in harness.waits
                            if description.startswith('peer ') or description.startswith('pending ')))
        self.assertTrue(any(error['stage'] == 'cancel_terminal' for error in ledger.cleanup_errors))

    def test_late_acceptance_after_drain_timeout_remains_owned_and_cancelled_asynchronously(self):
        pending = Future()
        right = Handle()
        harness, ledger = Harness(pending, right), ExecutionLedger()
        visible = ledger.handles
        with self.assertRaises(ExecutionCleanupError) as raised:
            harness.adapter.dispatch(harness.trajectories, ledger)
        original = raised.exception.__cause__
        self.assertIsInstance(original, TimeoutError)
        self.assertEqual(str(original), 'arm goal acceptance')
        self.assertTrue(ledger.cleanup_uncertain)
        waits_before = len(harness.waits)
        left = Handle(cancel_completes=False, uuid=array('B', range(16)))
        pending.set_result(left)
        self.assertEqual(len(harness.waits), waits_before, 'Future callback must not wait/spin')
        self.assertIs(visible, ledger.handles)
        self.assertEqual(len(visible), 2)
        self.assertEqual(sum(handle is left for handle, _ in visible), 1)
        self.assertEqual(left.cancel_calls, 1)
        self.assertEqual(left.result_calls, 1)
        self.assertTrue(ledger.cleanup_uncertain, 'Ack alone must not clear uncertainty')
        left.result_future.set_result(answer(5))
        self.assertFalse(ledger.cleanup_uncertain)
        self.assertIs(raised.exception.__cause__, original)
        self.assertEqual(harness.clients['unrelated'].goals, [])

    def test_late_callback_failure_cannot_replace_original_error_or_escape_callback(self):
        pending = Future()
        harness, ledger = Harness(pending, Handle()), ExecutionLedger()
        with self.assertRaises(ExecutionCleanupError) as raised:
            harness.adapter.dispatch(harness.trajectories, ledger)
        original = raised.exception.__cause__
        broken = Handle(cancel_error=RuntimeError('late cancel transport failure'))
        pending.set_result(broken)  # No callback exception escapes set_result.
        self.assertIs(raised.exception.__cause__, original)
        self.assertIs(raised.exception.primary, original)
        self.assertEqual(broken.cancel_calls, 1)
        self.assertTrue(any(error['message'] == 'late cancel transport failure'
                            for error in ledger.cleanup_errors))
        self.assertTrue(ledger.cleanup_uncertain)

    def test_late_result_factory_failure_still_sends_owned_cancel(self):
        pending = Future()
        harness, ledger = Harness(pending, Handle()), ExecutionLedger()
        with self.assertRaises(ExecutionCleanupError):
            harness.adapter.dispatch(harness.trajectories, ledger)
        left = Handle(result_failures=1)
        pending.set_result(left)
        self.assertEqual(left.cancel_calls, 1)
        self.assertTrue(any(attempt.accepted_handle is left for attempt in ledger.attempts))
        self.assertTrue(any(error['stage'] == 'late_result_acquisition' for error in ledger.cleanup_errors))

    def test_abort_before_next_send_preserves_primary_and_does_not_abort_cleanup(self):
        original = ValueError('cancel before second send')
        abort_calls = []
        def abort():
            abort_calls.append(True)
            if len(abort_calls) >= 2:
                raise original
        left, right = Handle(), Handle()
        harness, ledger = Harness(left, right, abort=abort), ExecutionLedger()
        with self.assertRaises(ValueError) as raised:
            harness.adapter.dispatch(harness.trajectories, ledger)
        self.assertIs(raised.exception, original)
        self.assertEqual(len(abort_calls), 2)
        self.assertEqual(ledger.total_dispatched, 1)
        self.assertEqual(left.cancel_calls, 1)
        self.assertEqual(harness.clients['right'].goals, [])
        self.assertFalse(ledger.cleanup_uncertain)

    def test_on_sent_failure_already_owns_submission_and_is_visible_before_waits(self):
        original = ValueError('counter observer failed')
        seen = []
        harness = Harness(Handle(), Handle())
        def sent(ledger):
            seen.append((ledger.total_dispatched, ledger.attempts[0].send_future,
                         len(harness.waits)))
            raise original
        harness.adapter.on_sent = sent
        ledger = ExecutionLedger(total_dispatched=4)
        with self.assertRaises(ValueError) as raised:
            harness.adapter.dispatch(harness.trajectories, ledger)
        self.assertIs(raised.exception, original)
        self.assertEqual(seen[0][0], 5)
        self.assertIsNotNone(seen[0][1])
        self.assertEqual(seen[0][2], 0)
        self.assertEqual(harness.left.cancel_calls, 1)
        self.assertEqual(harness.clients['right'].goals, [])

    def test_failed_ack_still_waits_for_terminal_and_preserves_primary_cause(self):
        left = Handle()
        left.cancel_future = Future()
        original = ValueError('result poll failed')
        harness = Harness(left, Handle(), until=lambda: (_ for _ in ()).throw(original))
        ledger = ExecutionLedger()
        with self.assertRaises(ExecutionCleanupError) as raised:
            harness.adapter.dispatch(harness.trajectories, ledger)
        self.assertIs(raised.exception.__cause__, original)
        self.assertTrue(ledger.attempts[0].terminal_seen)
        self.assertTrue(any(description == 'peer cancellation result'
                            for _, description, _ in harness.waits))
        self.assertEqual(harness.right.cancel_calls, 1)

    def test_start_margin_refusal_and_injected_cancel_use_same_owned_cleanup(self):
        for margin in (.1, 0):
            with self.subTest(margin=margin):
                harness, ledger = Harness(Handle(), Handle()), ExecutionLedger()
                with self.assertRaisesRegex(RuntimeError, '0.1s'):
                    harness.adapter.dispatch(harness.trajectories, ledger,
                                             scheduled_start=harness.now + margin)
                self.assertEqual(ledger.acceptance_margins, [])
                self.assertEqual(harness.left.cancel_calls + harness.right.cancel_calls, 2)
        harness, ledger = Harness(Handle(), Handle()), ExecutionLedger()
        with self.assertRaisesRegex(RuntimeError, 'An arm failed'):
            harness.adapter.dispatch(harness.trajectories, ledger, inject_cancel_at=10)
        self.assertEqual(harness.left.cancel_calls + harness.right.cancel_calls, 2)

    def test_explicit_legacy_owned_handles_can_be_cancelled_without_discovering_other_clients(self):
        left, right = Handle(), Handle(result=answer())
        harness = Harness()
        ledger = ExecutionLedger(handles=[(left, left.result_future), (right, right.result_future)])
        self.assertIs(harness.adapter.cancel_actions(ledger), ledger)
        self.assertEqual(left.cancel_calls, 1)
        self.assertEqual(right.cancel_calls, 0)
        self.assertEqual(ledger.total_dispatched, 0)
        self.assertFalse(ledger.cleanup_uncertain)
        self.assertTrue(all(not client.goals for client in harness.clients.values()))

    def test_observer_failure_after_acceptance_keeps_handle_owned_and_original_exception(self):
        original = ValueError('acceptance observer failed')
        def event(name, arm, **details):
            if name == 'goal_acceptance' and arm == 'left':
                raise original
        harness, ledger = Harness(Handle(), Handle(), event=event), ExecutionLedger()
        with self.assertRaises(ValueError) as raised:
            harness.adapter.dispatch(harness.trajectories, ledger)
        self.assertIs(raised.exception, original)
        self.assertEqual(harness.left.cancel_calls + harness.right.cancel_calls, 2)
        self.assertEqual(len(ledger.handles), 2)

    def test_execution_boundary_rejects_empty_oversized_and_reused_ledgers_before_send(self):
        harness = Harness()
        for trajectories in ([], harness.trajectories * 17):
            with self.assertRaises(ValueError):
                harness.adapter.dispatch(trajectories, ExecutionLedger())
        ledger = ExecutionLedger()
        harness.adapter.dispatch(harness.trajectories, ledger)
        with self.assertRaisesRegex(ValueError, 'fresh'):
            harness.adapter.dispatch(harness.trajectories, ledger)
        self.assertEqual(ledger.total_dispatched, 2)

    def test_snapshot_is_detached_bounded_json_and_separates_cumulative_from_request_counts(self):
        harness = Harness(Handle(result=answer(), uuid=array('B', range(16))),
                          Handle(result=answer()))
        ledger = ExecutionLedger(total_dispatched=20)
        harness.adapter.dispatch(harness.trajectories, ledger)
        snapshot = ledger.snapshot()
        self.assertEqual(json.loads(json.dumps(snapshot, allow_nan=False)), snapshot)
        self.assertEqual(snapshot['total_dispatched_cumulative'], 22)
        self.assertEqual(snapshot['request_attempt_count'], 2)
        self.assertEqual(snapshot['request_sent_count'], 2)
        self.assertEqual(snapshot['attempts'][0]['confirmed_goal_uuid'], bytes(range(16)).hex())
        self.assertNotIn('send_future', snapshot['attempts'][0])
        self.assertNotIn('accepted_handle', snapshot['attempts'][0])
        snapshot['attempts'][0]['arm'] = 'changed copy'
        self.assertEqual(ledger.attempts[0].arm, 'left')
        ledger.attempts[0].controller = 'x' * 300
        ledger.cleanup_errors.append(dict(arm='left', stage='transport', type='Error', message='x' * 3000))
        bounded = ledger.snapshot()
        self.assertEqual(len(bounded['attempts'][0]['controller']), 128)
        self.assertEqual(len(bounded['cleanup_errors'][0]['message']), 2000)
        self.assertTrue(bounded['cleanup_errors'][0]['message_truncated'])
        self.assertEqual(bounded['truncation']['strings'], 2)
        bounded['cleanup_errors'][0]['message'] = 'changed copy'
        self.assertEqual(len(ledger.cleanup_errors[0]['message']), 3000)

    def test_snapshot_cancel_ack_without_terminal_stays_uncertain(self):
        harness = Harness(Handle(cancel_completes=False), Handle(cancel_completes=False))
        ledger = ExecutionLedger()
        with self.assertRaises(ExecutionCleanupError):
            harness.adapter.dispatch(harness.trajectories, ledger)
        snapshot = ledger.snapshot()
        self.assertTrue(snapshot['cleanup_requested'])
        self.assertTrue(snapshot['cleanup_uncertain'])
        self.assertTrue(all(item['cancel_requested'] and item['cancel_acknowledged']
                            and not item['terminal_seen'] for item in snapshot['attempts']))
        self.assertIn('does not prove physical stop', snapshot['terminal_semantics'])
        json.dumps(snapshot, allow_nan=False)

    def test_submission_exception_or_missing_future_is_unknown_without_retry_or_global_cancel(self):
        for outcome in ('raises', 'none'):
            with self.subTest(outcome=outcome):
                harness, ledger = Harness(), ExecutionLedger()
                primary = OSError('transport submission outcome unknown')
                calls = []
                def send(goal):
                    calls.append(goal)
                    if outcome == 'raises':
                        raise primary
                    return None
                harness.clients['left'].send_goal_async = send
                with self.assertRaises(ExecutionCleanupError) as raised:
                    harness.adapter.dispatch(harness.trajectories, ledger)
                if outcome == 'raises':
                    self.assertIs(raised.exception.primary, primary)
                    self.assertIs(raised.exception.__cause__, primary)
                else:
                    self.assertRegex(str(raised.exception.__cause__), 'no acceptance future')
                self.assertEqual(len(calls), 1)
                self.assertEqual(ledger.total_dispatched, 0)
                self.assertEqual(harness.clients['right'].goals, [])
                self.assertEqual(harness.clients['unrelated'].goals, [])
                self.assertEqual(harness.left.cancel_calls + harness.right.cancel_calls, 0)
                self.assertTrue(ledger.cleanup_uncertain)
                snapshot = ledger.snapshot()
                self.assertEqual(snapshot['request_sent_count'], 0)
                self.assertEqual(snapshot['request_submission_started_count'], 1)
                self.assertTrue(snapshot['attempts'][0]['submission_started'])
                self.assertTrue(snapshot['attempts'][0]['submission_uncertain'])
                self.assertFalse(snapshot['attempts'][0]['terminal_seen'])
                # Explicit cleanup cannot turn the missing ownership evidence
                # into a claim that no action may have reached the controller.
                with self.assertRaises(ExecutionCleanupError):
                    harness.adapter.cancel_actions(ledger)
                self.assertTrue(ledger.cleanup_uncertain)

    def test_malformed_or_exceptional_result_never_clears_terminal_uncertainty(self):
        for result in (NS(status=4, result=NS()), answer(status=2), answer(status=4.0)):
            with self.subTest(result=result):
                harness = Harness(Handle(result=result, cancel_completes=False), Handle())
                ledger = ExecutionLedger()
                with self.assertRaises(ExecutionCleanupError):
                    harness.adapter.dispatch(harness.trajectories, ledger)
                self.assertFalse(ledger.attempts[0].terminal_seen)
                self.assertTrue(ledger.cleanup_uncertain)
        pending = Future()
        harness, ledger = Harness(pending, Handle()), ExecutionLedger()
        with self.assertRaises(ExecutionCleanupError) as raised:
            harness.adapter.dispatch(harness.trajectories, ledger)
        primary = raised.exception.__cause__
        late = Handle(cancel_completes=False)
        pending.set_result(late)
        late.result_future.set_exception(RuntimeError('result transport closed'))
        self.assertFalse(ledger.attempts[0].terminal_seen)
        self.assertTrue(ledger.cleanup_uncertain)
        self.assertIs(raised.exception.__cause__, primary)
        self.assertTrue(any(error['stage'] == 'terminal_callback' for error in ledger.cleanup_errors))

    def test_snapshot_retains_error_truncation_from_callback_storage(self):
        pending = Future()
        harness, ledger = Harness(pending, Handle()), ExecutionLedger()
        with self.assertRaises(ExecutionCleanupError):
            harness.adapter.dispatch(harness.trajectories, ledger)
        pending.set_result(Handle(cancel_error=RuntimeError('x' * 3000)))
        stored = [error for error in ledger.cleanup_errors if error['message_truncated']]
        self.assertEqual(len(stored), 1)
        self.assertEqual(len(stored[0]['message']), 2000)
        snapshot = ledger.snapshot()
        self.assertEqual(snapshot['truncation']['strings'], 1)
        self.assertTrue(any(error['message_truncated'] for error in snapshot['cleanup_errors']))
