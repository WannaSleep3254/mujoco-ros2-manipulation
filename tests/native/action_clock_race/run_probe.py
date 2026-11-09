"""Retain one actual installed action-server timer race attempt without retries."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import re
import resource
import signal
import subprocess
import time
from mujoco_ros2_manipulation.paths import ROOT
from mujoco_ros2_manipulation.build import check_action_clock_runtime
from mujoco_ros2_manipulation.verification.runner import group_gone
from mujoco_ros2_manipulation.workcells.verify_motion_reproduction import require_unused_domain


def sha(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def require(value, message):
    if not value:
        raise AssertionError(message)


def library(path):
    path = path.resolve()
    notes = subprocess.check_output(['readelf', '-n', str(path)], text=True)
    match = re.search(r'Build ID: ([a-f0-9]+)', notes)
    return dict(path=str(path), sha256=sha(path), build_id=match.group(1) if match else None)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--variant', choices=('legacy', 'fixed'), required=True)
    parser.add_argument('--binary', type=Path, required=True)
    parser.add_argument('--action-library', type=Path, required=True)
    parser.add_argument('--server-source', type=Path)
    parser.add_argument('--domain', type=int, required=True)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    output = args.output.resolve()
    require(output.is_relative_to(ROOT / 'runtime') and not output.exists(), 'Select a new project runtime output')
    if args.variant == 'fixed':
        require(args.action_library.resolve() == (ROOT/'ros_ws/install/rclcpp_action/lib/librclcpp_action.so').resolve(),
                'The fixed check requires this project action clock overlay')
        check_action_clock_runtime(ROOT)
    require_unused_domain(args.domain)
    output.mkdir(parents=True, mode=0o700)
    expected = dict(rclcpp_action=library(args.action_library),
                    rcl=library(Path('/opt/ros/humble/lib/librcl.so')),
                    rclcpp=library(Path('/opt/ros/humble/lib/librclcpp.so')))
    source = Path(__file__).with_name('clock_action_race.cpp')
    record = dict(schema_version=1, kind='actual-action-clock-race-v1', status='running',
        variant=args.variant, domain=args.domain, automatic_retry=False,
        started_monotonic_ns=time.monotonic_ns(), binary=library(args.binary),
        driver_sha256=sha(Path(__file__)),
        harness_source=dict(path=str(source), sha256=sha(source)), libraries=expected,
        no_external_constructor_lock=True, physical_trajectory_contact_scope_unchanged=True)
    if args.server_source:
        record['server_source'] = dict(path=str(args.server_source.resolve()), sha256=sha(args.server_source.resolve()))
    receipt = output / 'result.json'
    def save():
        receipt.write_text(json.dumps(record, indent=2) + '\n')
    save()
    process = None
    try:
        environment = dict(os.environ, ROS_DOMAIN_ID=str(args.domain), MANIPULATION_ROS_DOMAIN_ID=str(args.domain))
        environment['LD_LIBRARY_PATH'] = str(args.action_library.resolve().parent) + os.pathsep + environment.get('LD_LIBRARY_PATH', '')
        environment.pop('LD_PRELOAD', None)
        command = [str(args.binary.resolve()), args.variant]
        record['command'] = command
        with (output / 'probe.log').open('wb') as log:
            process = subprocess.Popen(command, cwd=output, env=environment, stdout=log,
                stderr=subprocess.STDOUT, start_new_session=True,
                preexec_fn=lambda: resource.setrlimit(resource.RLIMIT_CORE, (0, 0)))
            record.update(owned_pid=process.pid, owned_process_group=os.getpgid(process.pid))
            save()
            process.wait(timeout=20)
        record['return_code'] = process.returncode
        log_path = output / 'probe.log'
        events = [json.loads(line[len('CLOCK_RACE '):]) for line in log_path.read_text(errors='replace').splitlines()
                  if line.startswith('CLOCK_RACE ')]
        record.update(events=events, log_sha256=sha(log_path))
        require(events and all(event['pid'] == process.pid for event in events), 'Event process identity differs')
        require(all(a['monotonic_ns'] < b['monotonic_ns'] for a, b in zip(events, events[1:])), 'Nonmonotonic event sequence')
        actual = {event['key']: library(Path(event['path'])) for event in events if event['event'] == 'library'}
        require(actual == expected, 'Actual mapped library identity differs from the explicit variant')
        names = [event['event'] for event in events if event['event'] != 'library']
        if args.variant == 'legacy':
            require(process.returncode == -signal.SIGSEGV and names == [
                'registered_timer_impl_null', 'clock_mutex_acquired', 'update_timer_impl_null'],
                'Original actual timer initialization race did not produce SIGSEGV at the proven gate')
            record['status'] = 'expected_failure_observed'
        else:
            required = ('registered_timer_impl_null', 'clock_mutex_blocked', 'registration_released',
                        'update_timer_impl_ready', 'time_update_returned')
            require(process.returncode == 0 and names[-1] == 'passed'
                    and all(names.count(name) == 1 for name in required)
                    and [names.index(name) for name in required] == sorted(names.index(name) for name in required),
                    'Actual patched action constructor did not serialize timer initialization and ROS time update')
            record['status'] = 'passed'
    except BaseException as error:
        record.update(status='failed', error=dict(type=type(error).__name__, message=str(error)))
    finally:
        if process is not None and process.poll() is None:
            os.killpg(process.pid, signal.SIGKILL)
            process.wait(timeout=5)
            record['forced_cleanup'] = True
            record['status'] = 'failed'
        record.update(completed_monotonic_ns=time.monotonic_ns(),
                      owned_process_group_gone=process is None or group_gone(process.pid))
        if not record['owned_process_group_gone']:
            record.update(status='failed', cleanup_error='Owned test process group remains')
        save()
    print(record['status'].upper() + ' ' + str(receipt))
    return 0 if record['status'] in ('passed', 'expected_failure_observed') else 1


if __name__ == '__main__':
    raise SystemExit(main())
