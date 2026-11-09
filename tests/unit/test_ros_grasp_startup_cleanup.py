"""Exercise the actual startup-stop block without launching ROS or processes."""
import ast
from copy import deepcopy
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest

from mujoco_ros2_manipulation.grasping.ros_task import (
    CONTROLLERS, MAX_STARTUP_LOG_BYTES, RosGraspTaskError, moveit_startup_proof,
)

SOURCE = Path(__file__).resolve().parents[2] / 'src/mujoco_ros2_manipulation/grasping/ros_task.py'
START = '[INFO] [move_group-7]: process started with pid [42]'
CONTEXT = ('[move_group-7] [INFO] [1.2] [moveit_move_group_capabilities_base.move_group_context]: '
           'MoveGroup context initialization complete')
READY = '[move_group-7] You can start planning now!'
COMPLETE = '\n'.join((START, CONTEXT, READY))


def identity(pid):
    return dict(pid=pid, ppid=10, pgrp=10, session=10, starttime=123)


def cleanup_statements():
    """Select the production nested wait, stop block and launch-stop call."""
    tree = ast.parse(SOURCE.read_text())
    run = next(node for node in tree.body if isinstance(node, ast.FunctionDef) and node.name == 'run_ros_grasp')
    waiter = next(node for node in run.body if isinstance(node, ast.FunctionDef) and node.name == 'until')
    final = next(node.finalbody for node in run.body if isinstance(node, ast.Try) and node.finalbody)
    stop = next(node for node in final if isinstance(node, ast.If)
                and any(isinstance(item, ast.Name) and item.id == 'publisher' for item in ast.walk(node.test)))
    launch_stop = next(node for node in final if isinstance(node, ast.If)
                       and any(isinstance(item, ast.Call) and isinstance(item.func, ast.Name)
                               and item.func.id == 'stop_process' for item in ast.walk(node)))
    return waiter, [stop, launch_stop]


class MoveItStartupProofTests(unittest.TestCase):
    def proof(self, log=COMPLETE, process_identity=identity):
        return moveit_startup_proof(log, 10, process_identity=process_identity)

    def test_completed_markers_are_bound_to_one_live_owned_process(self):
        proof = self.proof()
        self.assertEqual(proof, dict(node='move_group-7', pid=42, launch_pid=10, starttime=123,
                                     start_line=1, context_ready_line=2, planning_ready_line=3))
        colored = COMPLETE.replace('initialization complete', '\x1b[92minitialization complete\x1b[0m')
        self.assertEqual(self.proof(colored), proof)

    def test_partial_or_foreign_ready_markers_cannot_finish_initialization(self):
        for log in ('', START, START+'\n'+CONTEXT, START+'\n'+READY,
                    START+'\n'+CONTEXT.replace('move_group-7', 'move_group-8')+'\n'+READY):
            with self.subTest(log=log):
                self.assertIsNone(self.proof(log))

    def test_duplicate_reordered_or_unowned_process_proof_is_rejected(self):
        for log in (COMPLETE+'\n'+START, COMPLETE+'\n'+CONTEXT, COMPLETE+'\n'+READY,
                    '\n'.join((START, READY, CONTEXT))):
            with self.subTest(log=log), self.assertRaises(RosGraspTaskError):
                self.proof(log)
        for field in ('pid', 'ppid', 'pgrp', 'session'):
            changed = identity(42)
            changed[field] += 1
            with self.subTest(field=field), self.assertRaises(RosGraspTaskError):
                self.proof(process_identity=lambda _pid: changed)
        with self.assertRaises(RosGraspTaskError):
            self.proof(process_identity=lambda _pid: None)

    def test_stopping_launch_or_actual_child_crash_never_authorizes_clean_drain(self):
        crash = "[ERROR] [move_group-7]: process has died [pid 42, exit code -6, cmd '/owned/move_group']."
        stopped = "[INFO] [move_group-7]: sending signal 'SIGINT' to process[move_group-7]"
        for suffix in (crash, stopped, '[WARNING] [launch]: user interrupted with ctrl-c (SIGINT)',
                       '[INFO] [move_group-7]: process has finished cleanly [pid 42]'):
            with self.subTest(suffix=suffix), self.assertRaises(RosGraspTaskError):
                self.proof(COMPLETE+'\n'+suffix)


class StartupCleanupTests(unittest.TestCase):
    def scenario(self, *, actions=None, cancelled=True, ready_at=.75,
                 native_stop_at=.25, stale_guard=False, reactivated_until=0., crash=False, proof_cost=0.):
        with tempfile.TemporaryDirectory() as directory:
            log_path = Path(directory)/'launch.log'
            log_path.write_text(START)
            events, checkpoints, budgets = [], [], []
            wall = SimpleNamespace(value=0.)
            clock = SimpleNamespace(monotonic=lambda: wall.value,
                                    monotonic_ns=lambda: int(wall.value*1e9))
            guard = dict(latched=False, phase='starting', controllers={name:'active' for name in CONTROLLERS})
            latest = dict(guard=guard, guard_observed_monotonic_ns=0)
            record = dict(status='cancelled' if cancelled else 'failed', actions=deepcopy(actions or []),
                          error=dict(type='RosGraspCancelled' if cancelled else 'OriginalError'),
                          cleanup=dict(status='running'), cleanup_errors=[])
            process = SimpleNamespace(pid=10, poll=lambda: None)
            published_ready = False

            def spin(*, health=True):
                nonlocal published_ready
                self.assertFalse(health, 'cleanup must not re-enter cancelled motion admission')
                wall.value = round(wall.value+.25, 8)
                stopped = wall.value >= native_stop_at and not (.25 < wall.value < reactivated_until)
                latest['guard'] = dict(latched=stopped, phase='stopped' if stopped else 'starting',
                                      controllers={name:'inactive' if stopped else 'active' for name in CONTROLLERS})
                latest['guard_observed_monotonic_ns'] = 0 if stale_guard else clock.monotonic_ns()
                if wall.value >= ready_at and not published_ready:
                    log_path.write_text(COMPLETE)
                    events.append('moveit_ready')
                    published_ready = True
                if crash:
                    log_path.write_text(START+"\n[ERROR] [move_group-7]: process has died [pid 42, exit code -6, cmd '/owned/move_group'].")

            def proof(log, launch):
                value = moveit_startup_proof(log, launch, process_identity=identity)
                if value is not None:
                    wall.value += proof_cost
                return value

            def stop_process(owned):
                self.assertIs(owned, process)
                events.append('launch_sigint')
                return False

            namespace = dict(process=process, publisher=object(), time=clock, latest=latest,
                record=record, CONTROLLERS=CONTROLLERS, MAX_STARTUP_LOG_BYTES=MAX_STARTUP_LOG_BYTES,
                life=None, cancel_event=SimpleNamespace(is_set=lambda: cancelled),
                change_phase=lambda phase: events.append(('phase', phase)), deepcopy=deepcopy,
                save=lambda: checkpoints.append(deepcopy(record)), log_path=log_path, spin=spin,
                moveit_startup_proof=proof, _require=lambda value,message: self.require(value,message),
                stop_process=stop_process, ik_client=None, fk_client=None, plan_client=None,
                get_scene=None, apply_scene=None)
            for name in ('ik_client', 'fk_client', 'plan_client', 'get_scene', 'apply_scene'):
                namespace[name] = SimpleNamespace(service_is_ready=lambda: wall.value >= ready_at)
            waiter, statements = cleanup_statements()
            exec(compile(ast.Module(body=[waiter], type_ignores=[]), str(SOURCE), 'exec'), namespace)
            real_wait = namespace['until']
            def observed_wait(predicate, seconds, label, *, health=True):
                budgets.append((label, seconds))
                return real_wait(predicate, seconds, label, health=health)
            namespace['until'] = observed_wait
            exec(compile(ast.Module(body=statements, type_ignores=[]), str(SOURCE), 'exec'), namespace)
            return record, events, checkpoints, budgets, wall.value

    @staticmethod
    def require(value, message):
        if not value:
            raise RosGraspTaskError(message)

    def test_physics_stop_then_constructor_completion_precedes_launch_sigint(self):
        record, events, checkpoints, budgets, elapsed = self.scenario()
        self.assertEqual(events, [('phase', 8), 'moveit_ready', 'launch_sigint'])
        self.assertTrue(record['cleanup']['native_stop_observed'])
        drain = record['cleanup']['moveit_startup_drain']
        self.assertEqual(drain['status'], 'passed')
        self.assertEqual(drain['proof']['pid'], 42)
        self.assertTrue(drain['proof']['required_services_ready'])
        self.assertEqual(record['actions'], [])
        self.assertEqual(record['cleanup_errors'], [])
        self.assertEqual(checkpoints[0]['cleanup']['moveit_startup_drain']['status'], 'waiting')
        self.assertEqual(budgets[0][1], 8.)
        self.assertEqual(budgets[1][1], 7.75)
        self.assertLessEqual(elapsed, 8.)

    def test_missing_initialization_consumes_only_the_remaining_eight_second_budget(self):
        record, events, _, budgets, elapsed = self.scenario(ready_at=100., native_stop_at=7.5)
        self.assertEqual(budgets[0][1], 8.)
        self.assertEqual(budgets[1][1], .5)
        self.assertEqual(elapsed, 8.)
        self.assertEqual(record['cleanup']['moveit_startup_drain']['status'], 'failed')
        self.assertIn('startup MoveIt drain:', record['cleanup_errors'][0])
        self.assertEqual(events[-1], 'launch_sigint')

    def test_fresh_stopped_guard_is_rechecked_after_late_controller_activation(self):
        record, _, _, _, elapsed = self.scenario(ready_at=.5, reactivated_until=1.25)
        proof = record['cleanup']['moveit_startup_drain']['proof']
        self.assertEqual(elapsed, 1.25)
        self.assertTrue(all(value=='inactive' for value in proof['guard']['controllers'].values()))
        self.assertEqual(proof['guard_age_sec'], 0.)

    def test_stale_guard_or_crash_remains_a_cleanup_failure(self):
        for values in (dict(stale_guard=True), dict(crash=True)):
            with self.subTest(values=values):
                record, events, _, _, _ = self.scenario(**values)
                self.assertEqual(record['cleanup']['moveit_startup_drain']['status'], 'failed')
                self.assertTrue(record['cleanup_errors'])
                self.assertEqual(events[-1], 'launch_sigint')

    def test_completed_owned_initialization_requires_no_additional_wait(self):
        record, events, _, budgets, elapsed = self.scenario(ready_at=0.)
        self.assertEqual(record['cleanup']['moveit_startup_drain']['status'], 'passed')
        self.assertEqual(len(budgets), 1)
        self.assertEqual(elapsed, .25)
        self.assertEqual(events[-1], 'launch_sigint')

    def test_readiness_work_finishing_after_deadline_cannot_authorize_clean_drain(self):
        record, events, _, _, _ = self.scenario(ready_at=0., native_stop_at=7.75, proof_cost=.3)
        self.assertEqual(record['cleanup']['moveit_startup_drain']['status'], 'failed')
        self.assertIn('existing cleanup deadline', record['cleanup_errors'][0])
        self.assertEqual(events[-1], 'launch_sigint')

    def test_transport_cancellation_and_non_cancel_errors_skip_startup_drain(self):
        for values in (dict(actions=[dict(stage='transport')]), dict(cancelled=False)):
            with self.subTest(values=values):
                record, events, _, budgets, _ = self.scenario(**values)
                self.assertNotIn('moveit_startup_drain', record['cleanup'])
                self.assertEqual(len(budgets), 1)
                self.assertEqual(events, [('phase', 8), 'launch_sigint'])


if __name__ == '__main__':
    unittest.main()
