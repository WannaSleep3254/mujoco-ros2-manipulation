"""Owned comparison startup proofs and deadline-bound non-ROS cleanup."""
from copy import deepcopy
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock, patch

from mujoco_ros2_manipulation.operator_console.demo_worker import DemoCancelled, DemoJob
from mujoco_ros2_manipulation.verification.reports import VerificationReport
from mujoco_ros2_manipulation.verification.startup import move_group_readiness

WORKER = 'mujoco_ros2_manipulation.operator_console.demo_worker'
HELPERS = ''.join(
    f'[INFO] [spawner-{index}]: process started with pid [{40+index}]\n'
    f'[spawner-{index}] [INFO] [1.0] [spawner_{role}]: Configured and activated {role}\n'
    f'[INFO] [spawner-{index}]: process has finished cleanly [pid {40+index}]\n'
    for index, role in enumerate(('joint_state_broadcaster', 'left_controller', 'right_controller'), 3))
MOVE_START = '[INFO] [move_group-6]: process started with pid [9000]\n'
MOVE_CONTEXT = '[move_group-6] [INFO] [1.0] [moveit_move_group_capabilities_base.move_group_context]: MoveGroup context initialization complete\n'
MOVE_READY = '[move_group-6] You can start planning now!\n'
IDENTITY = dict(pid=9000, start_ticks=100, ppid=8000, pgid=8000, sid=8000)


class MoveGroupReadinessTests(unittest.TestCase):
    def test_final_markers_require_one_live_owned_process_and_ordered_initialization(self):
        identify = Mock(return_value=IDENTITY)
        pending = move_group_readiness(MOVE_START+MOVE_CONTEXT, 8000, identify)
        self.assertFalse(pending['ready'])
        self.assertEqual(pending['start_ticks'], 100)
        proof = move_group_readiness(MOVE_START+MOVE_CONTEXT+MOVE_READY, 8000, identify)
        self.assertTrue(proof['ready'])
        self.assertLess(proof['start_line'], proof['context_ready_line'])
        self.assertLess(proof['context_ready_line'], proof['planning_ready_line'])
        for log in (MOVE_START+MOVE_START+MOVE_CONTEXT+MOVE_READY,
                    MOVE_START+MOVE_CONTEXT+MOVE_CONTEXT+MOVE_READY,
                    MOVE_START+MOVE_READY+MOVE_CONTEXT,
                    MOVE_START+MOVE_CONTEXT+MOVE_READY+'[INFO] [move_group-6]: process has finished cleanly [pid 9000]\n'):
            with self.subTest(log=log), self.assertRaises(RuntimeError):
                move_group_readiness(log, 8000, identify)
        for identity in (None, dict(IDENTITY, ppid=7000), dict(IDENTITY, sid=7000)):
            with self.subTest(identity=identity), self.assertRaises(RuntimeError):
                move_group_readiness(MOVE_START+MOVE_CONTEXT+MOVE_READY, 8000, lambda _pid:identity)


class ComparisonStartupTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.root = Path(temporary.name)
        console = self.root/'runtime/operator/session'
        self.job = DemoJob(console/'demos'/('a'*32), 'contact_comparison', 91, console,
                           render=False, root=self.root)
        self.job._cancel_stacks.start = Mock()
        self.addCleanup(self.job._cancel_stacks.close)
        self.report = VerificationReport(self.root, 'comparison-fixture', 'headless', 91)
        self.job.active_report = self.report
        self.job.launch_process = Mock(pid=8000)
        self.job.launch_process.poll.return_value = None
        self.job.launch_cell = NS(instances=[NS(controller='left_controller'), NS(controller='right_controller')])
        self.job.data['active_launch'] = dict(run_id=self.report.run_id, pid=8000, start_ticks=90)
        self.log = HELPERS+MOVE_START+MOVE_CONTEXT+MOVE_READY
        self.report.log_path.write_text(self.log)
        self.owned = patch(WORKER+'.matching_process', return_value=True)
        self.identify = patch(WORKER+'.process_identity', return_value=IDENTITY)
        self.owned.start(); self.identify.start()
        self.addCleanup(self.owned.stop); self.addCleanup(self.identify.stop)

    def test_proof_waits_for_clean_helpers_and_rejects_child_failure_or_stopping(self):
        self.report.log_path.write_text(self.log.replace('[INFO] [spawner-5]: process has finished cleanly [pid 45]\n', ''))
        self.assertFalse(self.job._startup_proof()['ready'])
        self.report.log_path.write_text(self.log)
        proof = self.job._startup_proof()
        self.assertTrue(proof['ready'])
        self.assertEqual(proof['run_id'], self.report.run_id)
        for suffix in ("[ERROR] [spawner-5]: process has died [pid 45, exit code -2]\n",
                       "[ERROR] [move_group-6]: process has died [pid 9000, exit code -6]\n",
                       "[ERROR] [ros2_control_node-1]: process has died [pid 44, exit code -11]\n",
                       "[INFO] [spawner-5]: sending signal 'SIGINT' to process[spawner-5]\n"):
            self.report.log_path.write_text(self.log+suffix)
            with self.subTest(suffix=suffix), self.assertRaises(RuntimeError):
                self.job._startup_proof()

    def test_report_domain_live_launch_and_process_reuse_are_bound_to_each_proof(self):
        self.assertTrue(self.job._startup_proof()['ready'])
        with patch(WORKER+'.process_identity', return_value=dict(IDENTITY, start_ticks=101)):
            with self.assertRaisesRegex(RuntimeError, 'identity changed'):
                self.job._startup_proof()
        self.report.data['ros_domain_id'] = 92
        with self.assertRaisesRegex(RuntimeError, 'live owned launch'):
            self.job._startup_proof()
        self.report.data['ros_domain_id'] = 91
        with patch(WORKER+'.matching_process', return_value=False):
            with self.assertRaisesRegex(RuntimeError, 'live owned launch'):
                self.job._startup_proof()

    def test_normal_barrier_cannot_complete_before_the_last_helper_has_exited(self):
        pending = self.log.replace('[INFO] [spawner-5]: process has finished cleanly [pid 45]\n', '')
        self.report.log_path.write_text(pending)
        observed = []
        def until(predicate, seconds, description):
            observed.append(predicate())
            self.report.log_path.write_text(self.log)
            observed.append(predicate())
        self.job._startup_barrier(NS(until=until))
        self.assertEqual(observed, [False, True])
        self.assertTrue(self.job.startup_complete)
        saved = json.loads(self.report.path.read_text())['comparison_startup']
        self.assertEqual(saved['status'], 'passed')
        self.assertEqual(len(saved['proof']['controllers']['spawners']), 3)

    def test_cancel_drain_without_a_node_observes_progress_then_uses_existing_budget(self):
        now = [1_000_000_000]
        observed = []
        self.job.cancel_deadline_ns = 45_000_000_000
        def proof():
            observed.append(now[0])
            return dict(ready=len(observed) == 2, synthetic=True)
        with patch.object(self.job, '_startup_proof', side_effect=proof), \
             patch(WORKER+'.time.monotonic_ns', side_effect=lambda:now[0]), \
             patch(WORKER+'.time.sleep', side_effect=lambda seconds:now.__setitem__(0, now[0]+int(seconds*1e9))):
            self.job._drain_cancelled_startup(self.report, None, None, self.job.launch_process, None, DemoCancelled('stop'))
        evidence = json.loads(self.report.path.read_text())['comparison_startup_drain']
        self.assertEqual(evidence['status'], 'passed')
        self.assertFalse(evidence['node_was_created'])
        self.assertEqual(evidence['deadline_monotonic_ns'], 33_000_000_000)
        self.assertEqual(self.job.cancel_deadline_ns, 45_000_000_000)
        self.assertEqual(len(observed), 2)

    def test_late_success_or_real_predicate_failure_cannot_make_drain_clean(self):
        for failure in (False, True):
            now = [32_000_000_000]
            self.job.startup_complete = False
            self.job.cancel_deadline_ns = 45_000_000_000
            def proof():
                if failure:
                    raise ValueError('original decoder or service failure')
                now[0] = 34_000_000_000
                return dict(ready=True)
            with patch.object(self.job, '_startup_proof', side_effect=proof), \
                 patch(WORKER+'.time.monotonic_ns', side_effect=lambda:now[0]):
                with self.subTest(failure=failure), self.assertRaises((TimeoutError, ValueError)):
                    self.job._drain_cancelled_startup(self.report, None, None, self.job.launch_process, None, DemoCancelled('stop'))
            self.assertFalse(self.job.startup_complete)
            saved = json.loads(self.report.path.read_text())['comparison_startup_drain']
            self.assertEqual(saved['status'], 'failed')
            self.assertEqual(self.job.cancel_deadline_ns, 45_000_000_000)

    def test_completed_startup_or_non_cancel_error_skips_drain(self):
        with patch.object(self.job, '_startup_proof') as proof:
            self.job.startup_complete = True
            self.job._drain_cancelled_startup(self.report, None, None, self.job.launch_process, None, DemoCancelled('stop'))
            self.job.startup_complete = False
            self.job._drain_cancelled_startup(self.report, None, None, self.job.launch_process, None, RuntimeError('real failure'))
        proof.assert_not_called()
        self.assertNotIn('comparison_startup_drain', json.loads(self.report.path.read_text()))


if __name__ == '__main__':
    unittest.main()
