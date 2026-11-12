"""Exercise an owned ROS launch CLI using two non-ROS Python child processes."""
import argparse
import atexit
import asyncio
import hashlib
import json
import logging
import os
from pathlib import Path
import selectors
import signal
import subprocess
import sys
import time

ROOT = Path(__file__).resolve().parents[2]


def process_identity(pid):
    raw = Path(f'/proc/{pid}/stat').read_text()
    fields = raw[raw.rfind(')') + 2:].split()
    return dict(pid=pid, ppid=int(fields[1]), group=int(fields[2]),
                session=int(fields[3]), start_ticks=int(fields[19]))


def publish(path, value):
    with path.open('x') as stream:
        json.dump(value, stream, indent=2)
        stream.write('\n')


def create_fixture(output, notification_fd, boundary):
    """Only the diagnostic process installs these memory-only observations."""
    import launch
    from launch.actions import ExecuteProcess, OpaqueFunction
    from launch.actions.execute_local import ExecuteLocal

    rows = []
    tasks = []
    actions = []
    service_events = getattr(sys.modules.get('sitecustomize'), '_recorded_events', None)
    notified = False
    saved = False
    context = None

    def future_state(future):
        if future is None:
            return None
        return dict(done=future.done(), cancelled=future.cancelled())

    def action_state(action):
        protocol = getattr(action, '_subprocess_protocol', None)
        return dict(name=(action.process_details or {}).get('name'),
                    pid=(action.process_details or {}).get('pid'),
                    completed=future_state(getattr(action, '_ExecuteLocal__completed_future', None)),
                    protocol_complete=future_state(getattr(protocol, 'complete', None)))

    def trace(kind, **values):
        if len(rows) < 1000:
            rows.append(dict(monotonic_ns=str(time.monotonic_ns()), kind=kind, **values))

    def task_state(task):
        value = dict(task_id=id(task), done=task.done(), cancelled=task.cancelled(),
                     coroutine=repr(task.get_coro()))
        if task.done() and not task.cancelled():
            error = task.exception()
            if error is not None:
                import traceback
                value['error'] = dict(type=type(error).__name__,
                    traceback=''.join(traceback.format_exception(type(error), error, error.__traceback__)))
        return value

    def save_snapshot(signum=None, frame=None):
        nonlocal saved
        if saved:
            return
        saved = True
        publish(output/'fixture-observations.json', dict(
            trace=rows, tasks=[task_state(task) for task in tasks],
            actions=[action_state(action) for action in actions],
            clock='same process CLOCK_MONOTONIC',
            diagnostic_gate='An external SIGINT is delivered during a bounded100ms clean-log callback.',
            ros_contexts_or_nodes_created=0,
            service_events=service_events or []))

    def factory(loop, coroutine):
        task = asyncio.Task(coroutine, loop=loop)
        tasks.append(task)
        task.add_done_callback(lambda done: trace('task_done', task=task_state(done)))
        return task

    class ExitLogHandler(logging.Handler):
        def emit(self, record):
            nonlocal notified
            message = record.getMessage()
            if 'finished cleanly' in message or 'failed to terminate' in message:
                trace('process_log', logger=record.name, message=message,
                      actions=[action_state(action) for action in actions])
            if boundary == 'clean_exit' and not notified and record.name.startswith('boundary_fast') and 'finished cleanly' in message:
                notified = True
                os.write(notification_fd, b'E')
                # Widen the real clean-log/cleanup boundary without an asyncio yield.
                time.sleep(.1)

    if service_events is None:
        from launch.launch_service import LaunchService
        service_events = []
        original_event = LaunchService._LaunchService__process_event
        async def observe_event(service, event):
            service_events.append(dict(type=type(event).__name__,
                monotonic_ns=str(time.monotonic_ns()), reason=getattr(event, 'reason', None)))
            return await original_event(service, event)
        LaunchService._LaunchService__process_event = observe_event

    handler = ExitLogHandler()
    original_logger = launch.logging.get_logger
    def get_logger(name=None):
        logger = original_logger(name)
        if handler not in logger.handlers:
            logger.addHandler(handler)
        return logger
    launch.logging.get_logger = get_logger

    original_cleanup = ExecuteLocal._ExecuteLocal__cleanup
    def cleanup(action):
        trace('cleanup_enter', action=action_state(action))
        result = original_cleanup(action)
        trace('cleanup_return', action=action_state(action))
        return result
    ExecuteLocal._ExecuteLocal__cleanup = cleanup

    def observe(launch_context):
        nonlocal context
        context = launch_context
        launch_context.asyncio_loop.set_task_factory(factory)
        trace('sigint_handler', handler=repr(signal.getsignal(signal.SIGINT)))
        signal.signal(signal.SIGUSR1, save_snapshot)
        atexit.register(save_snapshot)
        return None

    child = ('import signal,sys,time; '
             'signal.signal(signal.SIGINT,lambda signum,frame:sys.exit(0)); '
             'time.sleep(float(sys.argv[1]))')
    fast = ExecuteProcess(cmd=[sys.executable, '-c', child, '.2'], name='boundary_fast', output='screen')
    slow_duration = '.25' if boundary == 'after_signal_manager_fd_removed' else '20'
    slow = ExecuteProcess(cmd=[sys.executable, '-c', child, slow_duration], name='boundary_slow', output='screen')
    actions.extend((fast, slow))
    return launch.LaunchDescription([OpaqueFunction(function=observe), fast, slow])


def group_exists(group):
    try:
        os.killpg(group, 0)
        return True
    except ProcessLookupError:
        return False


def verify(args):
    output = args.output.absolute()
    if output.exists() or not output.is_relative_to(ROOT/'runtime'):
        raise ValueError('Select a new project runtime evidence directory')
    output.mkdir(parents=True)
    read_fd, write_fd = os.pipe()
    fixture = output/'boundary.launch.py'
    fixture.write_text('from pathlib import Path\n'
        'from tests.integration.verify_launch_signal_boundary import create_fixture\n'
        'def generate_launch_description():\n'
        f'    return create_fixture(Path({str(output)!r}), {write_fd}, {args.boundary!r})\n')
    if args.boundary != 'clean_exit':
        method = {'before_signal_manager':'__enter__',
                  'before_signal_manager_chain':'_AsyncSafeSignalManager__chain',
                  'after_signal_manager_fd_removed':'_AsyncSafeSignalManager__unchain'}[args.boundary]
        bootstrap = output/'sitecustomize.py'
        bootstrap.write_text(
            'import asyncio,os,time\n'
            'from launch.utilities.signal_management import AsyncSafeSignalManager\n'
            'from launch.launch_service import LaunchService\n'
            f'_original_boundary=getattr(AsyncSafeSignalManager,{method!r})\n'
            '_notified=False\n'
            '_recorded_events=[]\n'
            '_original_event=LaunchService._LaunchService__process_event\n'
            'async def _event(service,event):\n'
            '    _recorded_events.append(dict(type=type(event).__name__,monotonic_ns=str(time.monotonic_ns()),reason=getattr(event,"reason",None)))\n'
            '    return await _original_event(service,event)\n'
            'LaunchService._LaunchService__process_event=_event\n'
            'def _boundary(manager,*args):\n'
            '    global _notified\n'
            '    try: asyncio.get_running_loop()\n'
            '    except RuntimeError: return _original_boundary(manager,*args)\n'
            '    if not _notified:\n'
            '        _notified=True\n'
            f'        os.write({write_fd},b"E")\n'
            '        time.sleep(.1)\n'
            '    return _original_boundary(manager,*args)\n'
            f'setattr(AsyncSafeSignalManager,{method!r},_boundary)\n')
    if args.entry == 'original':
        command = ['/usr/bin/python3', '/opt/ros/humble/bin/ros2', 'launch', str(fixture)]
    else:
        command = ['/usr/bin/python3', '-m', 'mujoco_ros2_manipulation.launch_cli', 'launch', str(fixture)]
    env = dict(os.environ, PYTHONDONTWRITEBYTECODE='1', ROS_LOG_DIR=str(output/'ros-logs'),
        PYTHONPATH=os.pathsep.join([str(output), str(ROOT/'src'), str(ROOT),
            '/opt/ros/humble/lib/python3.10/site-packages',
            '/opt/ros/humble/local/lib/python3.10/dist-packages']))
    process = None
    forced = False
    result = dict(entry=args.entry, boundary=args.boundary, command=command, status='running', automatic_retry=False,
                  diagnostic_only=True, robot_commands=0)
    try:
        with (output/'launcher.log').open('xb') as log:
            process = subprocess.Popen(command, cwd=ROOT, env=env, stdout=log,
                stderr=subprocess.STDOUT, start_new_session=True, pass_fds=(write_fd,))
            os.close(write_fd)
            write_fd = None
            owned = process_identity(process.pid)
            result['owned_launcher'] = owned
            if owned['pid'] != owned['group'] or owned['pid'] != owned['session']:
                raise AssertionError('Launcher is not in its independently owned group')
            with selectors.DefaultSelector() as selector:
                selector.register(read_fd, selectors.EVENT_READ)
                if not selector.select(timeout=5) or os.read(read_fd, 1) != b'E':
                    raise AssertionError('Owned clean-exit boundary was not observed')
            if process_identity(process.pid) != owned:
                raise AssertionError('Launcher identity changed before SIGINT')
            cleanup_deadline = time.monotonic()+12
            result['signals_monotonic_ns'] = []
            for index in range(args.signal_count):
                if process_identity(process.pid) != owned:
                    raise AssertionError('Launcher identity changed between diagnostic signals')
                result['signals_monotonic_ns'].append(str(time.monotonic_ns()))
                os.kill(owned['pid'], signal.SIGINT)
                if index+1 < args.signal_count:
                    time.sleep(.02)
            result['signal_monotonic_ns'] = result['signals_monotonic_ns'][0]
            try:
                process.wait(timeout=max(0., cleanup_deadline-time.monotonic()))
            except subprocess.TimeoutExpired:
                forced = True
                if process_identity(process.pid)['start_ticks'] != owned['start_ticks']:
                    raise AssertionError('Launcher PID was reused before diagnostic snapshot')
                os.kill(owned['pid'], signal.SIGUSR1)
                deadline = time.monotonic()+1
                while not (output/'fixture-observations.json').exists() and time.monotonic() < deadline:
                    time.sleep(.01)
                os.killpg(owned['group'], signal.SIGTERM)
                try:
                    process.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    os.killpg(owned['group'], signal.SIGKILL)
                    process.wait(timeout=3)
        result.update(return_code=process.returncode, forced_cleanup=forced,
                      process_group_gone=not group_exists(owned['group']))
        log_text = (output/'launcher.log').read_text(errors='replace')
        result['sigint_messages'] = [line for line in log_text.splitlines() if 'user interrupted' in line]
        result['escalation_messages'] = [line for line in log_text.splitlines() if 'failed to terminate' in line]
        observations = json.loads((output/'fixture-observations.json').read_text())
        result['observations_sha256'] = hashlib.sha256((output/'fixture-observations.json').read_bytes()).hexdigest()
        result['shutdown_events'] = [event for event in observations.get('service_events', [])
                                     if event['type']=='Shutdown']
        result['spawned_child_pids'] = [action['pid'] for action in observations['actions']
                                       if action['pid'] is not None]
        if args.entry == 'original':
            interrupted = [task for task in observations['tasks'] if task.get('error',{}).get('type')=='KeyboardInterrupt']
            orphan = [a for a in observations['actions'] if a['protocol_complete']['done'] and not a['completed']['done']]
            passed = forced and len(interrupted)==1 and len(orphan)==1 and result['process_group_gone']
            result['status'] = 'expected_original_failure_observed' if passed else 'failed'
        else:
            passed = (not forced and process.returncode==0 and result['process_group_gone']
                      and all(a['completed']['done'] for a in observations['actions'])
                      and not any(t.get('error') for t in observations['tasks']))
            result['status'] = 'passed' if passed else 'failed'
        return 0 if passed else 1
    except BaseException as error:
        result.update(status='failed', error=dict(type=type(error).__name__, message=str(error)))
        return 1
    finally:
        if write_fd is not None:
            os.close(write_fd)
        os.close(read_fd)
        if process is not None:
            if process.poll() is None or group_exists(process.pid):
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                process.wait(timeout=3)
                result['emergency_cleanup'] = True
            cleanup_deadline = time.monotonic()+2
            while group_exists(process.pid) and time.monotonic()<cleanup_deadline:
                time.sleep(.02)
            result['process_group_gone'] = not group_exists(process.pid)
        if (output/'launcher.log').exists():
            result['launcher_log_sha256'] = hashlib.sha256((output/'launcher.log').read_bytes()).hexdigest()
        publish(output/'result.json', result)
        print(json.dumps(result))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output',type=Path,required=True)
    parser.add_argument('--entry',choices=('original','project'),default='project')
    parser.add_argument('--signal-count',type=int,choices=(1,2),default=1)
    parser.add_argument('--boundary',choices=('clean_exit','before_signal_manager','before_signal_manager_chain','after_signal_manager_fd_removed'),default='clean_exit')
    return verify(parser.parse_args())


if __name__=='__main__':
    raise SystemExit(main())
