"""Testable verification lifecycle, including preflight and cleanup failures."""

import os
import re
import signal
import subprocess
import time
from contextlib import contextmanager

from mujoco_ros2_manipulation.profiles import check_generated_model, load_profile
from mujoco_ros2_manipulation.model_build.integrity import file_sha256
from mujoco_ros2_manipulation.verification.reports import VerificationReport, utc_now
from mujoco_ros2_manipulation.verification.process_faults import process_failures


def group_gone(pid):
    try:
        os.killpg(pid, 0)
        return False
    except ProcessLookupError:
        return True


def stop_process(process):
    forced = False
    if process.poll() is None:
        process.send_signal(signal.SIGINT)
        try:
            process.wait(timeout=12)
        except subprocess.TimeoutExpired:
            forced = True
            os.killpg(process.pid, signal.SIGTERM)
            try:
                process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5)
    if not group_gone(process.pid):
        forced = True
        for sig in (signal.SIGTERM, signal.SIGKILL):
            os.killpg(process.pid, sig)
            deadline = time.monotonic() + 2
            while not group_gone(process.pid) and time.monotonic() < deadline:
                time.sleep(0.05)
            if group_gone(process.pid):
                break
    return forced


def _save_cleanup_diagnostics(report, diagnostics):
    """Diagnostics must not prevent cleanup or replace its primary error."""
    try:
        report.update(cleanup_diagnostics=diagnostics)
    except Exception as error:
        # A later normal report write can preserve this failed write attempt.
        diagnostics.setdefault('write_errors', []).append(
            dict(type=type(error).__name__, message=str(error), utc=utc_now()))


@contextmanager
def _cleanup_step(report, diagnostics, name, *, applicable=True):
    """Persist entry before the call, so a blocked call leaves its last boundary."""
    step = dict(name=name, status='running' if applicable else 'skipped',
                started_utc=utc_now(), started_monotonic_ns=time.monotonic_ns(),
                completed_utc=None, completed_monotonic_ns=None,
                elapsed_sec=None, error=None)
    diagnostics['steps'].append(step)
    _save_cleanup_diagnostics(report, diagnostics)
    try:
        yield step
    except BaseException as error:
        step.update(status='failed', error=dict(type=type(error).__name__, message=str(error)))
        raise
    else:
        if step['status'] == 'running':
            step['status'] = 'completed'
    finally:
        step.update(completed_utc=utc_now(), completed_monotonic_ns=time.monotonic_ns())
        step['elapsed_sec'] = (step['completed_monotonic_ns'] - step['started_monotonic_ns']) / 1e9
        _save_cleanup_diagnostics(report, diagnostics)


def run_verification(args, root, ros, factory, trigger_type, *, profile_loader=None,
                     validator=None, command_builder=None, report_prefix="",
                     launch_environment=None, finalize=None, expected_crash=None, on_launch=None):
    mode = 'gui' if args.gui else 'headless'
    if args.stop_mode == 'window':
        mode += '_window'
    scene_id = getattr(args, 'scene', 'default')
    report_id = args.robot if scene_id == 'default' else f'{args.robot}_{scene_id}'
    report = VerificationReport(root, report_prefix + report_id, mode, args.domain)
    process = node = profile = None
    failed = False
    interrupted = False
    forced = False
    cleanup = []
    expected_exits = []
    os.environ['ROS_DOMAIN_ID'] = str(args.domain)
    os.environ['MANIPULATION_ROS_DOMAIN_ID'] = str(args.domain)
    try:
        profile = (profile_loader or load_profile)(args.robot, root=root, scene=scene_id)
        (validator or check_generated_model)(profile)
        report.update(scene_id=scene_id, robot=profile.label, robot_profile_sha256=profile.fingerprint,
                      model_manifest_sha256=file_sha256(profile.output / 'source.json'),
                      gui=args.gui, stop_mode=args.stop_mode)
        report.stage('launch')
        with report.log_path.open('w') as log:
            command = command_builder(profile, args) if command_builder else ['bash', str(root / 'run_sim.sh'), f'robot:={profile.id}', f'scene:={scene_id}',
                 f'gui:={str(args.gui).lower()}', f'rviz:={str(args.gui).lower()}', 'moveit:=true']
            process = subprocess.Popen(command,
                cwd=root, stdout=log, stderr=subprocess.STDOUT, start_new_session=True,
                env=launch_environment(report, args) if launch_environment else None)
        if on_launch is not None:
            on_launch(report, profile, args, process)
        report.stage('ros_initialization')
        ros.init()
        node = factory(profile, progress=report.stage)
        node.process_group = process.pid
        node.launch_process = process
        node.launch_log_path = report.log_path
        report.stage('motion')
        payload = node.run()
        if expected_crash is not None:
            expected_exits = expected_crash.verify(report.log_path.read_text(), process.poll())
        else:
            if process.poll() is not None:
                raise RuntimeError('Launch process exited unexpectedly')
            if 'process has died' in report.log_path.read_text():
                raise RuntimeError('A launch node failed; see the log')
        report.update(**payload, motion_checks_passed=True)
        if args.gui:
            report.stage('gui')
            ready_marker = f'Ready to take commands for planning group {profile.ros["planning_group"]}.'
            node.until(lambda: ready_marker in report.log_path.read_text(), 15, 'RViz MoveIt panel ready')
            if ready_marker not in report.log_path.read_text():
                raise RuntimeError('RViz MoveIt panel is not ready')
            gpu = subprocess.run(['nvidia-smi'], capture_output=True, text=True, timeout=5)
            (report.directory / 'nvidia_smi.txt').write_text(gpu.stdout + gpu.stderr)
            report.update(mujoco_gpu_process_seen='ros2_control_node' in gpu.stdout,
                          rviz_gpu_process_seen='/rviz2' in gpu.stdout)
            if getattr(args, 'require_gpu', False):
                from mujoco_ros2_manipulation.verification.diagnostics import process_sample, host_info
                owned = process_sample(process.pid)
                owned_gpu = [line for line in gpu.stdout.splitlines()
                             if any(re.search(r'\b' + str(pid) + r'\b', line) for pid in owned)]
                if gpu.returncode != 0 or not all(any(name in line for line in owned_gpu)
                        for name in ('ros2_control_node', '/rviz2')):
                    raise RuntimeError('Owned MuJoCo/RViz processes were not both seen on the NVIDIA GPU')
                gl = subprocess.run(['glxinfo', '-B'], capture_output=True, text=True, timeout=5)
                (report.directory / 'glxinfo.txt').write_text(gl.stdout + gl.stderr)
                if gl.returncode or 'NVIDIA' not in gl.stdout:
                    raise RuntimeError('Hardware NVIDIA OpenGL context was not confirmed')
                report.update(host=host_info(), graphics=dict(owned_gpu_processes=owned_gpu,
                              opengl=gl.stdout, nvidia_smi=gpu.stdout))
            if args.screenshot_dir:
                from mujoco_ros2_manipulation.verification.images import capture_gui_windows
                report.update(screenshots=capture_gui_windows(profile, process.pid, args.screenshot_dir))
        if args.stop_mode == 'window':
            report.stage('window_close')
            closer = node.create_client(trigger_type, '/mujoco_ros2_control_node/close_window')
            node.until(closer.service_is_ready, 5, 'window close service')
            closed = node.result(closer.call_async(trigger_type.Request()), 5, 'window close request')
            if not closed.success:
                raise RuntimeError(closed.message)
            process.wait(timeout=15)
    except (Exception, KeyboardInterrupt) as error:
        failed = True
        interrupted = isinstance(error, KeyboardInterrupt)
        report.update(error={'type': type(error).__name__, 'message': str(error)},
                      failure_stage=report.data['stage'])
    finally:
        # Cleanup errors must never erase the primary verification failure.
        diagnostics = dict(schema_version=1, steps=[])
        try:
            with _cleanup_step(report, diagnostics, 'destroy_node', applicable=node is not None):
                if node is not None:
                    node.destroy_node()
        except Exception as error:
            cleanup.append(f'destroy_node: {error}')
        try:
            with _cleanup_step(report, diagnostics, 'ros_shutdown') as step:
                if ros.ok():
                    ros.shutdown()
                else:
                    step['status'] = 'skipped'
        except Exception as error:
            cleanup.append(f'ROS shutdown: {error}')
        try:
            with _cleanup_step(report, diagnostics, 'launch_stop', applicable=process is not None):
                if process is not None:
                    forced = stop_process(process)
        except Exception as error:
            cleanup.append(f'launch cleanup: {error}')
        try:
            with _cleanup_step(report, diagnostics, 'finalize',
                               applicable=finalize is not None and process is not None):
                if finalize is not None and process is not None:
                    finalize(report, profile, args)
        except Exception as error:
            report.update(physics_checks_passed=False, diagnostics_error=str(error))
            if not failed:
                failed = True
                report.update(error={'type': type(error).__name__, 'message': str(error)},
                              failure_stage='physics_diagnostics')
        log = report.log_path.read_text()
        observed_failures = process_failures(log)
        failures = [entry for entry in observed_failures if entry not in expected_exits]
        # Recheck the complete log: a late extra crash must fail the run too.
        if expected_exits and observed_failures != expected_exits:
            failures = observed_failures
        try:
            gone = process is None or group_gone(process.pid)
        except OSError as error:
            gone = False
            cleanup.append(f'process group check: {error}')
        clean = not (cleanup or failures or forced or 'failed to terminate' in log or
                     (process is not None and process.returncode != 0) or not gone)
        if not clean and not failed:
            failed = True
            report.update(error={'type': 'ShutdownError', 'message': 'Launch cleanup failed; see shutdown fields'},
                          failure_stage='shutdown')
        report.update(status='interrupted' if interrupted else ('failed' if failed else 'passed'),
                      completed_utc=utc_now(), stage='complete', cleanup_errors=cleanup,
                      shutdown_failures=failures, expected_process_exits=expected_exits, shutdown_timed_out=forced or 'failed to terminate' in log,
                      launch_return_code=process.returncode if process is not None else None,
                      launch_started=process is not None, process_group_gone=gone, shutdown_clean=clean)
    print(f'{report.data["status"].upper()}: {report.path}', flush=True)
    if report.data['error']:
        print(f'{report.data["failure_stage"]}: {report.data["error"]}', flush=True)
    return 130 if interrupted else (1 if failed else 0)
