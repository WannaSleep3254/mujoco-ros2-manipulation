import json
import signal
import subprocess
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest
from unittest.mock import patch
from unittest.mock import Mock

from mujoco_ros2_manipulation.workcells.import_step import import_step,run_freecad
from tests.unit.test_environment_assets import CONVERTER,spec,tessellation


class StepImportTests(unittest.TestCase):
    def test_source_change_during_owned_conversion_aborts_without_output(self):
        with TemporaryDirectory() as temporary:
            root=Path(temporary);source=root/'local.step';source.write_bytes(b'original STEP')
            def converter(job,directory,**kwargs):
                self.assertNotEqual(Path(job['input']),source)
                self.assertEqual(Path(job['input']).read_bytes(),b'original STEP')
                (directory/'converted.json').write_text(json.dumps(dict(tessellation=tessellation(),version='0.0-test',occt_version='0.0-test')))
                source.write_bytes(b'changed STEP')
                return {key:CONVERTER[key] for key in ('name','script_sha256','executable_sha256')}
            with patch('mujoco_ros2_manipulation.workcells.import_step.run_freecad',converter),self.assertRaisesRegex(ValueError,'source changed'):
                import_step(source,root/'never_published',spec())
            self.assertFalse((root/'never_published').exists())
            self.assertEqual(source.read_bytes(),b'changed STEP')

    def owned(self,root,wait):
        executable=root/'freecad';executable.write_bytes(b'owned converter fixture')
        process=Mock(pid=4567);process.poll.return_value=None;process.wait.side_effect=wait
        return executable,process

    def test_keyboard_interrupt_reaps_owned_group_and_preserves_original_exception(self):
        with TemporaryDirectory() as temporary:
            root=Path(temporary);interrupted=KeyboardInterrupt('requested stop')
            executable,process=self.owned(root,[interrupted,0])
            with patch('mujoco_ros2_manipulation.workcells.import_step.subprocess.Popen',return_value=process),patch('mujoco_ros2_manipulation.workcells.import_step.os.killpg') as kill:
                with self.assertRaises(KeyboardInterrupt) as caught:run_freecad({},root,executable=executable)
            self.assertIs(caught.exception,interrupted)
            kill.assert_called_once_with(process.pid,signal.SIGTERM)
            self.assertEqual(process.wait.call_count,2)

    def test_timeout_escalates_to_kill_reaps_and_preserves_timeout(self):
        with TemporaryDirectory() as temporary:
            root=Path(temporary);timeout=subprocess.TimeoutExpired('fixture',.1)
            executable,process=self.owned(root,[timeout,subprocess.TimeoutExpired('fixture',3),0])
            with patch('mujoco_ros2_manipulation.workcells.import_step.subprocess.Popen',return_value=process),patch('mujoco_ros2_manipulation.workcells.import_step.os.killpg') as kill:
                with self.assertRaises(subprocess.TimeoutExpired) as caught:run_freecad({},root,executable=executable,timeout=.1)
            self.assertIs(caught.exception,timeout)
            self.assertEqual([call.args for call in kill.call_args_list],[(process.pid,signal.SIGTERM),(process.pid,signal.SIGKILL)])
            self.assertEqual(process.wait.call_count,3)

    def test_success_never_signals_owned_process(self):
        with TemporaryDirectory() as temporary:
            root=Path(temporary);executable,process=self.owned(root,[0])
            with patch('mujoco_ros2_manipulation.workcells.import_step.subprocess.Popen',return_value=process),patch('mujoco_ros2_manipulation.workcells.import_step.os.killpg') as kill:
                result=run_freecad({},root,executable=executable)
            kill.assert_not_called();process.wait.assert_called_once_with(timeout=30.)
            self.assertEqual(result['name'],'FreeCAD')

    def test_cleanup_failure_cannot_replace_triggering_error(self):
        with TemporaryDirectory() as temporary:
            root=Path(temporary);interrupted=RuntimeError('wait transport failed')
            executable,process=self.owned(root,[interrupted,subprocess.TimeoutExpired('fixture',3),RuntimeError('reap failed')])
            with patch('mujoco_ros2_manipulation.workcells.import_step.subprocess.Popen',return_value=process),patch('mujoco_ros2_manipulation.workcells.import_step.os.killpg'):
                with self.assertRaises(RuntimeError) as caught:run_freecad({},root,executable=executable)
            self.assertIs(caught.exception,interrupted)
            self.assertIn('reap failed',(root/'cleanup-error.json').read_text())

    def test_timeout_must_be_finite_positive_bounded_before_spawn(self):
        for value in (0,-1,True,float('nan'),float('inf'),121,'30'):
            with self.subTest(value=value),patch('mujoco_ros2_manipulation.workcells.import_step.subprocess.Popen') as spawn:
                with self.assertRaisesRegex(ValueError,'timeout'):run_freecad({},Path('/unused'),timeout=value)
                spawn.assert_not_called()

if __name__=='__main__':unittest.main()
