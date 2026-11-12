import asyncio
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import Mock, patch

from mujoco_ros2_manipulation import launch_cli


class LaunchCliTests(unittest.TestCase):
    def setUp(self):
        self.manager = SimpleNamespace(_AsyncSafeSignalManager__current=object())

    def invoke(self, cli, argv=None):
        package = ModuleType('ros2cli')
        module = ModuleType('ros2cli.cli')
        module.main = cli
        signal_management = ModuleType('launch.utilities.signal_management')
        signal_management.AsyncSafeSignalManager = self.manager
        modules = {'ros2cli': package, 'ros2cli.cli': module,
                   'launch.utilities.signal_management': signal_management}
        with patch.dict(sys.modules, modules):
            return launch_cli.main(argv)

    def test_forwards_original_argv_and_return_value(self):
        argv = ['launch', '/project/launch/workcell.launch.py',
                'workcell:=fr5_ur5e', 'label:=two words']
        cli = Mock(return_value=17)
        with patch.object(signal, 'getsignal', return_value=signal.default_int_handler):
            with patch.object(signal, 'signal') as install:
                self.assertEqual(self.invoke(cli, argv), 17)
        cli.assert_called_once_with(argv=argv)
        self.assertIs(cli.call_args.kwargs['argv'], argv)
        self.assertEqual(install.call_args_list[0].args[0], signal.SIGINT)
        self.assertEqual(install.call_args_list[-1].args,
                         (signal.SIGINT, signal.default_int_handler))

    def test_none_argv_retains_ros_cli_sys_argv_handling(self):
        cli = Mock(return_value=0)
        with patch.object(signal, 'getsignal', return_value=signal.SIG_IGN):
            with patch.object(signal, 'signal') as install:
                self.assertEqual(self.invoke(cli), 0)
        cli.assert_called_once_with(argv=None)
        install.assert_not_called()

    def test_custom_handler_is_preserved(self):
        handler = Mock()
        cli = Mock(return_value=0)
        with patch.object(signal, 'getsignal', return_value=handler):
            with patch.object(signal, 'signal') as install:
                self.assertEqual(self.invoke(cli, ['launch']), 0)
        install.assert_not_called()

    def test_restores_default_handler_on_cli_error(self):
        for error in (RuntimeError('cli failure'), KeyboardInterrupt(), SystemExit(2)):
            with self.subTest(error=type(error).__name__):
                cli = Mock(side_effect=error)
                with patch.object(signal, 'getsignal', return_value=signal.default_int_handler):
                    with patch.object(signal, 'signal') as install:
                        with self.assertRaises(type(error)):
                            self.invoke(cli)
                self.assertEqual(install.call_args_list[-1].args,
                                 (signal.SIGINT, signal.default_int_handler))

    def test_cli_initialization_sigint_is_not_swallowed(self):
        with patch.object(signal, 'getsignal', return_value=signal.default_int_handler):
            with patch.object(signal, 'signal') as install:
                def initialize(**kwargs):
                    install.call_args.args[1](signal.SIGINT, None)
                with self.assertRaises(KeyboardInterrupt):
                    self.invoke(initialize, ['launch'])
        self.assertEqual(install.call_args_list[-1].args,
                         (signal.SIGINT, signal.default_int_handler))

    def test_running_registered_manager_neither_interrupts_nor_replays(self):
        async def action(handler):
            handler(signal.SIGINT, None)
            handler(signal.SIGINT, None)
            await asyncio.sleep(0)
            return 'action finished'
        with patch.object(signal, 'getsignal', return_value=signal.default_int_handler):
            with patch.object(signal, 'signal') as install, patch.object(signal, 'raise_signal') as replay:
                def cli(**kwargs):
                    return asyncio.run(action(install.call_args.args[1]))
                self.assertEqual(self.invoke(cli), 'action finished')
                replay.assert_not_called()

    def test_first_loop_turn_replays_once_after_manager_registers(self):
        self.manager._AsyncSafeSignalManager__current = None
        with patch.object(signal, 'getsignal', return_value=signal.default_int_handler):
            with patch.object(signal, 'signal') as install, patch.object(signal, 'raise_signal') as replay:
                def cli(**kwargs):
                    handler = install.call_args.args[1]
                    async def initialize():
                        handler(signal.SIGINT, None)
                        self.manager._AsyncSafeSignalManager__current = object()
                        await asyncio.sleep(0)
                    replay.side_effect = lambda signum: handler(signum, None)
                    asyncio.run(initialize())
                self.invoke(cli)
                replay.assert_called_once_with(signal.SIGINT)

    def test_first_loop_turn_coalesces_repeated_sigints(self):
        self.manager._AsyncSafeSignalManager__current = None
        with patch.object(signal, 'getsignal', return_value=signal.default_int_handler):
            with patch.object(signal, 'signal') as install, patch.object(signal, 'raise_signal') as replay:
                def cli(**kwargs):
                    handler = install.call_args.args[1]
                    async def initialize():
                        try:
                            handler(signal.SIGINT, None)
                            handler(signal.SIGINT, None)
                        except KeyboardInterrupt as error:
                            raise AssertionError('Repeated startup SIGINT interrupted the task') from error
                        self.manager._AsyncSafeSignalManager__current = object()
                        await asyncio.sleep(0)
                    replay.side_effect = lambda signum: handler(signum, None)
                    asyncio.run(initialize())
                self.invoke(cli)
                replay.assert_called_once_with(signal.SIGINT)

    def test_missing_manager_on_replay_preserves_default_interrupt(self):
        self.manager._AsyncSafeSignalManager__current = None
        with patch.object(signal, 'getsignal', return_value=signal.default_int_handler):
            with patch.object(signal, 'signal') as install, patch.object(signal, 'raise_signal') as replay:
                def cli(**kwargs):
                    handler = install.call_args.args[1]
                    async def initialize():
                        handler(signal.SIGINT, None)
                        await asyncio.sleep(0)
                    replay.side_effect = lambda signum: handler(signum, None)
                    asyncio.run(initialize())
                with self.assertRaises(KeyboardInterrupt):
                    self.invoke(cli)
                replay.assert_called_once_with(signal.SIGINT)
        self.assertEqual(install.call_args_list[-1].args,
                         (signal.SIGINT, signal.default_int_handler))

    def test_both_shell_entrypoints_preserve_launch_arguments(self):
        root = Path(__file__).resolve().parents[2]
        entries = [('run_scene.sh', 'workcell.launch.py'),
                   ('run_sim.sh', 'manipulation.launch.py')]
        for filename, launch_file in entries:
            with self.subTest(entrypoint=filename), tempfile.TemporaryDirectory() as temporary:
                folder = Path(temporary)
                (folder/filename).write_bytes((root/filename).read_bytes())
                (folder/'manipulation_env.sh').write_text('# disposable argument-only fixture\n')
                executable = folder/'python3'
                executable.write_text('#!/usr/bin/python3\nimport json,sys\nprint(json.dumps(sys.argv[1:]))\n')
                executable.chmod(0o700)
                options = ['robot:=fr5', 'label:=two words', 'empty:=', 'path:=a$b']
                command = ['/bin/bash', str(folder/filename), *options]
                result = subprocess.run(command,
                    env=dict(os.environ, PATH=str(folder)+os.pathsep+'/usr/bin:/bin'),
                    check=True, stdout=subprocess.PIPE, text=True)
                self.assertEqual(json.loads(result.stdout), [
                    '-m', 'mujoco_ros2_manipulation.launch_cli', 'launch',
                    str(folder/'launch'/launch_file), *options])


if __name__ == '__main__':
    unittest.main()
