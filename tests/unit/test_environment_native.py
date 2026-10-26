"""Native helper lifecycle boundaries with mocked owned children; no real process."""
import signal
import subprocess
import unittest
from unittest.mock import Mock,call,patch,sentinel

from mujoco_ros2_manipulation.workcells import environment_native


class OwnedNativeProcessTests(unittest.TestCase):
    def setUp(self):
        self.argv=['owned-fixture-tool','--input','owned-synthetic-fixture']
        self.child=Mock(pid=123456)
        self.child.poll.return_value=None

    def invoke(self):
        return environment_native._owned(self.argv,timeout=.25,stdout=sentinel.output,stderr=sentinel.error)

    def test_invocation_owns_a_new_session_and_reaps_the_exact_child(self):
        self.child.wait.return_value=0
        with patch.object(environment_native.subprocess,'Popen',return_value=self.child) as start,patch.object(environment_native.os,'killpg') as kill:
            self.assertIsNone(self.invoke())
        start.assert_called_once_with(self.argv,stdin=subprocess.DEVNULL,stdout=sentinel.output,
            stderr=sentinel.error,start_new_session=True)
        self.child.wait.assert_called_once_with(timeout=.25)
        kill.assert_not_called()

    def test_nonzero_exit_is_not_success_or_a_reason_to_signal_a_reaped_group(self):
        self.child.wait.return_value=7
        with patch.object(environment_native.subprocess,'Popen',return_value=self.child),patch.object(environment_native.os,'killpg') as kill:
            with self.assertRaises(subprocess.CalledProcessError) as result:self.invoke()
        self.assertEqual(result.exception.returncode,7)
        self.assertEqual(result.exception.cmd,self.argv)
        self.child.wait.assert_called_once_with(timeout=.25)
        kill.assert_not_called()

    def test_timeout_terminates_only_the_new_owned_group_and_preserves_the_timeout(self):
        original=subprocess.TimeoutExpired(self.argv,.25)
        self.child.wait.side_effect=[original,-signal.SIGTERM]
        with patch.object(environment_native.subprocess,'Popen',return_value=self.child),patch.object(environment_native.os,'killpg') as kill:
            with self.assertRaises(subprocess.TimeoutExpired) as result:self.invoke()
        self.assertIs(result.exception,original)
        self.assertEqual(kill.call_args_list,[call(self.child.pid,signal.SIGTERM)])
        self.assertEqual(self.child.wait.call_args_list,[call(timeout=.25),call(timeout=3)])

    def test_keyboard_interrupt_cleans_owned_group_before_reraising_the_same_interrupt(self):
        original=KeyboardInterrupt('synthetic caller interruption')
        self.child.wait.side_effect=[original,-signal.SIGTERM]
        with patch.object(environment_native.subprocess,'Popen',return_value=self.child),patch.object(environment_native.os,'killpg') as kill:
            with self.assertRaises(KeyboardInterrupt) as result:self.invoke()
        self.assertIs(result.exception,original)
        kill.assert_called_once_with(self.child.pid,signal.SIGTERM)
        self.assertEqual(self.child.wait.call_count,2)

    def test_term_ignoring_owned_child_gets_kill_and_a_final_bounded_reap(self):
        original=subprocess.TimeoutExpired(self.argv,.25)
        self.child.wait.side_effect=[original,subprocess.TimeoutExpired(self.argv,3),-signal.SIGKILL]
        with patch.object(environment_native.subprocess,'Popen',return_value=self.child),patch.object(environment_native.os,'killpg') as kill:
            with self.assertRaises(subprocess.TimeoutExpired) as result:self.invoke()
        self.assertIs(result.exception,original)
        self.assertEqual(kill.call_args_list,[call(self.child.pid,signal.SIGTERM),call(self.child.pid,signal.SIGKILL)])
        self.assertEqual(self.child.wait.call_args_list,[call(timeout=.25),call(timeout=3),call(timeout=3)])

    def test_repeated_interrupt_during_term_wait_still_kills_reaps_and_preserves_original(self):
        original=KeyboardInterrupt('original caller interruption')
        self.child.wait.side_effect=[original,KeyboardInterrupt('second interruption'),-signal.SIGKILL]
        with patch.object(environment_native.subprocess,'Popen',return_value=self.child),patch.object(environment_native.os,'killpg') as kill:
            with self.assertRaises(KeyboardInterrupt) as result:self.invoke()
        self.assertIs(result.exception,original)
        self.assertEqual(kill.call_args_list,[call(self.child.pid,signal.SIGTERM),call(self.child.pid,signal.SIGKILL)])
        self.assertEqual(self.child.wait.call_count,3)

    def test_group_exit_between_poll_and_term_is_reaped_without_an_unrelated_kill(self):
        original=subprocess.TimeoutExpired(self.argv,.25)
        self.child.wait.side_effect=[original,0]
        with patch.object(environment_native.subprocess,'Popen',return_value=self.child),patch.object(environment_native.os,'killpg',side_effect=ProcessLookupError) as kill:
            with self.assertRaises(subprocess.TimeoutExpired) as result:self.invoke()
        self.assertIs(result.exception,original)
        kill.assert_called_once_with(self.child.pid,signal.SIGTERM)
        self.assertEqual(self.child.wait.call_count,2)

    def test_a_child_already_reaped_by_poll_is_never_signaled(self):
        original=subprocess.TimeoutExpired(self.argv,.25)
        self.child.wait.side_effect=original;self.child.poll.return_value=0
        with patch.object(environment_native.subprocess,'Popen',return_value=self.child),patch.object(environment_native.os,'killpg') as kill:
            with self.assertRaises(subprocess.TimeoutExpired) as result:self.invoke()
        self.assertIs(result.exception,original)
        kill.assert_not_called()
        self.child.wait.assert_called_once_with(timeout=.25)

    def test_failed_spawn_has_no_owned_pid_to_signal(self):
        original=OSError('synthetic spawn failure')
        with patch.object(environment_native.subprocess,'Popen',side_effect=original),patch.object(environment_native.os,'killpg') as kill:
            with self.assertRaises(OSError) as result:self.invoke()
        self.assertIs(result.exception,original)
        kill.assert_not_called()

    def test_cleanup_failure_cannot_turn_timeout_into_a_success_or_hide_the_original_cause(self):
        original=subprocess.TimeoutExpired(self.argv,.25)
        self.child.wait.side_effect=[original,subprocess.TimeoutExpired(self.argv,3),OSError('synthetic reap failure')]
        with patch.object(environment_native.subprocess,'Popen',return_value=self.child),patch.object(environment_native.os,'killpg') as kill:
            with self.assertRaises(subprocess.TimeoutExpired) as result:self.invoke()
        self.assertIs(result.exception,original)
        self.assertEqual(kill.call_args_list,[call(self.child.pid,signal.SIGTERM),call(self.child.pid,signal.SIGKILL)])


if __name__=='__main__':unittest.main()
