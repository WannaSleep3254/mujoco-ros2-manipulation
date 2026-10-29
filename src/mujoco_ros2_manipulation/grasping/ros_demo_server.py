"""Loopback viewer for one explicitly selected native ROS/MoveIt grasp task.

HTTP selects only start/cancel of the startup-pinned FR5 task. It never selects a
model, executable, ROS endpoint, goal, gripper action or attachment operation.
"""
import argparse
from copy import deepcopy
from dataclasses import dataclass, field
import hashlib
import hmac
from http.server import ThreadingHTTPServer
from importlib.resources import files
import os
from pathlib import Path
import re
import secrets
import signal
import stat
import subprocess
import threading
import time
from urllib.parse import parse_qs, urlsplit
import uuid

from ..operator_console.recorded_server import RecordingHandler
from ..operator_console.scene_host import _canonical, _json, _regular, _require
from ..operator_console.view_binding import binding_id, envelope, make_binding
from ..model_build.integrity import file_sha256
from ..verification.reports import atomic_json
from ..workcells.config import load_workcell
from .ros_display import build_scene
from .ros_observation import GraspObservation
from .ros_runtime import check_grasp_runtime
from .ros_target import load_ros_grasp_target


MAX_JOBS = 8  # Includes every reserved request, including a failed spawn.
MAX_LATEST_BYTES = 512 * 1024
MAX_RESULT_BYTES = 8 * 1024 * 1024
MAX_JOB_BYTES = 96 * 1024 * 1024
MAX_STDOUT_BYTES = 4 * 1024 * 1024
MAX_WALL_SEC = 300.
CANCEL_GRACE_SEC = 50.
SOURCE_ID = 'scene:ros_fr5_grasp'
_ID = re.compile(r'[0-9a-f]{32}')
_SHA = re.compile(r'[0-9a-f]{64}')


def _sha(raw):
    return hashlib.sha256(raw).hexdigest()


def _unused_domain(domain):
    from ..workcells.verify_motion_reproduction import require_unused_domain
    require_unused_domain(domain)


def _group_gone(pid):
    try:
        os.killpg(pid, 0)
        return False
    except ProcessLookupError:
        return True
    except PermissionError:
        return False


def _proc_identity(pid):
    try:
        raw = Path('/proc', str(pid), 'stat').read_text()
        fields = raw[raw.rfind(')') + 2:].split()
        return dict(ppid=int(fields[1]), group=int(fields[2]), session=int(fields[3]), start=int(fields[19]))
    except (OSError, ValueError, IndexError):
        return None


def _members(group):
    result = {}
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit():
            continue
        pid = int(entry.name)
        identity = _proc_identity(pid)
        if identity is not None and identity['group'] == group and identity['session'] == group:
            result[pid] = identity['start']
    return result


def _directory_bytes(root):
    size, count = 0, 0
    for path in root.rglob('*'):
        try:
            mode = path.lstat().st_mode
            _require(not stat.S_ISLNK(mode), 'Owned task output cannot contain linked files')
            if stat.S_ISREG(mode):
                count += 1
                _require(count <= 256, 'Owned task file count exceeded')
                size += path.stat().st_size
        except FileNotFoundError:  # Atomic temporary files may already be replaced.
            continue
    return size


class _BoundedOutput:
    """Drain worker stdout; bound the retained log without blocking cleanup."""
    def __init__(self, pipe, path):
        self.pipe, self.path = pipe, path
        self.overflow = False
        self.error = None
        self.thread = threading.Thread(target=self._drain, daemon=True)
        self.thread.start()

    def _drain(self):
        written = 0
        try:
            with self.path.open('wb') as stream:
                while chunk := self.pipe.read(8192):
                    keep = chunk[:max(0, MAX_STDOUT_BYTES - written)]
                    stream.write(keep)
                    written += len(keep)
                    if len(keep) != len(chunk):
                        self.overflow = True
        except Exception as error:
            self.error = str(error)[:512]
        finally:
            self.pipe.close()


@dataclass
class RosJob:
    identifier: str
    request_id: str
    generation: int
    directory: Path
    binding: dict
    binding_id: str
    display: object
    started: float
    process: object = None
    output: object = None
    status: str = 'starting'
    finished: bool = False
    cancel_requested: bool = False
    cancel_at: object = None
    cancel_delivery: str = 'not_requested'
    escalation: int = 0
    timed_out: bool = False
    update_digest: object = None
    update: object = None
    observation: object = None
    observed_ns: object = None
    progress_ns: object = None
    capture_ns: object = None
    pose: object = None
    error: object = None
    display_error: object = None
    result: object = None
    result_sha256: object = None
    native_group: object = None
    native_leader_start: object = None
    native_members: dict = field(default_factory=dict)
    members_checked: float = 0.
    owner_cleanup_clean: object = None
    owner_errors: list = field(default_factory=list)


class RosDemoApplication:
    """One owned ROS task at a time; frozen geometry and retained evidence."""
    def __init__(self, target, output_root, *, domain, clock=time.monotonic,
                 _process_factory=subprocess.Popen, _domain_check=_unused_domain,
                 _scene_builder=build_scene, _timeout_sec=MAX_WALL_SEC):
        _require(type(domain) is int and 0 <= domain <= 232, 'Select a fixed unused ROS domain 0..232')
        _require(type(_timeout_sec) in (int, float) and 300 <= _timeout_sec <= 360,
                 'Owned ROS task timeout must include at least 300 seconds')
        target.validate()
        check_grasp_runtime(target.root)
        self.target, self.workspace, self.domain = target, target.root, domain
        self.clock, self.process_factory = clock, _process_factory
        self.domain_check, self.scene_builder, self.timeout_sec = _domain_check, _scene_builder, float(_timeout_sec)
        self.preview = self.scene_builder(target)
        self.source_pin = self._pin()
        self.domain_check(domain)  # The probe is closed before the idle HTTP server starts.
        self.output_root = Path(output_root).resolve()
        _require(self.output_root.is_relative_to((self.workspace / 'runtime').resolve()),
                 'Use a new retained output root under this worktree runtime/')
        self.output_root.mkdir(parents=True, exist_ok=False)
        atomic_json(self.output_root / 'source-pin.json', self.source_pin)
        self.session_id = uuid.uuid4().hex
        self.preview_binding = make_binding(self.session_id, 0, SOURCE_ID, 'model_preview')
        self.preview_id = binding_id(self.preview_binding)
        self.generation, self.jobs, self.requests, self.current = 0, {}, {}, None
        self.lock, self.closing = threading.RLock(), threading.Event()
        self.source = dict(id=SOURCE_ID, label='FR5 + UR5e · 검증용 평행 그리퍼 · 자유 블록',
            model_sha256=self.preview.manifest['model_sha256'], manifest_sha256=target.manifest_sha256,
            engine='MuJoCo 3.12.0 · ROS 2 Humble', method='MoveIt 2 계획 · ROS 궤적 제어',
            domain=domain, simulation_only=True,
            scope='FR5의 집기·운반·놓기; UR5e는 같은 물리 환경에서 현재 자세 유지')
        self.monitor = threading.Thread(target=self._watch, daemon=True)
        self.monitor.start()

    def _pin(self):
        paths = set((self.workspace / 'src/mujoco_ros2_manipulation/grasping').glob('*.py'))
        paths.update((self.workspace / 'src/mujoco_ros2_manipulation/grasping/static').glob('ros_demo.*'))
        paths.update((self.workspace / 'native').glob('grasp*.*'))
        paths.update((self.workspace / 'native/grasp_ros2_plugin').rglob('*'))
        paths.update(self.workspace / name for name in ('native/step_monitor.hpp', 'native/control_loop_shutdown.hpp',
            'native/moveit_plugin_lifetime.c', 'runtime/native/grasp_scene_export', 'runtime/native/grasp_distance.so', 'config/build.yaml',
            'runtime/native/moveit_plugin_lifetime.so',
            'ros_ws/install/mujoco_ros2_control/lib/libmujoco_ros2_control.so',
            'ros_ws/install/mujoco_ros2_control/lib/mujoco_ros2_control/ros2_control_node',
            'ros_ws/install/manipulation_grasp_plugin/lib/libmujoco_ros2_control_grasp_observer.so'))
        return {str(path.relative_to(self.workspace)): file_sha256(path) for path in sorted(paths) if path.is_file()}

    def _identity(self, model_sha):
        _require(type(model_sha) is str and model_sha == self.source['model_sha256'], '선택한 ROS 실행 모델이 다릅니다.')

    def start(self, value):
        _require(type(value) is dict and set(value) == {'request_id', 'model_sha256'}, '새 요청 ID와 고정 모델 SHA만 받습니다.')
        _require(type(value['request_id']) is str and _ID.fullmatch(value['request_id']), '올바른 새 요청 ID가 필요합니다.')
        self._identity(value['model_sha256'])
        with self.lock:
            if self.closing.is_set():
                raise RuntimeError('ROS 시연 서버를 종료하고 있습니다.')
            identifier = value['request_id']
            if identifier in self.requests:
                state = self.snapshot()
                state['request_replay'] = dict(job_id=self.requests[identifier], new_job_started=False)
                return state
            if self.current is not None and (not self.current.finished or self.current.owner_cleanup_clean is not True):
                raise RuntimeError('이전 ROS 작업의 소유 프로세스 정리가 확인될 때까지 새 작업을 시작하지 않습니다.')
            if len(self.requests) >= MAX_JOBS:
                raise RuntimeError('실패한 요청을 포함한 작업 보존 상한에 도달했습니다. 새 서버 작업 공간이 필요합니다.')
            _require(self._pin() == self.source_pin, '실행 코드가 바뀌었습니다. 이 서버를 다시 시작하세요.')
            self.target.validate()
            check_grasp_runtime(self.workspace)
            self.domain_check(self.domain)
            display = self.scene_builder(self.target, run_id=(run_id := uuid.uuid4().hex))
            self.generation += 1
            binding = make_binding(self.session_id, self.generation, SOURCE_ID, 'live', job_id=run_id, run_id=run_id)
            directory = self.output_root / run_id
            directory.mkdir(mode=0o700)
            self.requests[identifier] = run_id
            job = RosJob(run_id, identifier, self.generation, directory, binding, binding_id(binding), display, self.clock())
            self.jobs[run_id] = self.current = job
            task_output = directory / 'run'
            command = ['bash', '--noprofile', '--norc', '-c',
                'source "$1"; shift; exec /usr/bin/python3 -m mujoco_ros2_manipulation.grasping.ros_task "$@"',
                'owned-ros-grasp', str(self.workspace / 'manipulation_env.sh'),
                '--domain', str(self.domain), '--target', str(self.target.output), '--sha', self.target.manifest_sha256,
                '--output', str(task_output), '--run-id', run_id, '--updates', str(task_output / 'latest.json')]
            environment = dict(os.environ, MANIPULATION_ROS_DOMAIN_ID=str(self.domain), ROS_DOMAIN_ID=str(self.domain),
                               PYTHONUNBUFFERED='1', PYTHONDONTWRITEBYTECODE='1')
            try:
                atomic_json(directory / 'source-pin.json', self.source_pin)
                job.process = self.process_factory(command, cwd=self.workspace, env=environment,
                    stdout=subprocess.PIPE, stderr=subprocess.STDOUT, start_new_session=True)
                if job.process.stdout is not None:
                    job.output = _BoundedOutput(job.process.stdout, directory / 'stdout.log')
            except Exception as error:
                if job.process is None:
                    # Popen never returned a child: there is no owned worker or
                    # nested ROS launch to clean. Keep the failed reservation.
                    job.status, job.finished, job.owner_cleanup_clean = 'failed', True, True
                    job.error = dict(type=type(error).__name__, message='ROS 작업 프로세스를 시작하지 못했습니다.')
                    self._checkpoint(job)
                else:
                    # A drainer/setup failure after Popen must not abandon a
                    # live worker or declare cleanup before observing its exit.
                    self._owner_error(job, 'worker setup', error)
                    self._begin_cancel(job, reason='ROS 작업의 출력 준비에 실패해 소유 프로세스를 정리하고 있습니다.')
                raise RuntimeError('ROS 작업을 준비하지 못했습니다. 이 요청은 자동 재실행하지 않습니다.') from error
            self._checkpoint(job)
            return self.snapshot()

    def _signal_worker(self, job, sig):
        if job.process is None or job.process.poll() is not None:
            return
        try:
            # SIGINT targets the worker's cancellation handler. Its ROS launch
            # is a separate owned session and must be allowed to stop in order.
            os.kill(job.process.pid, sig)
        except ProcessLookupError:
            pass

    def _bind_native_group(self, job, group):
        if group is None:
            return
        _require(type(group) is int and group > 1, 'Invalid native process-group identity')
        if job.native_group is not None:
            _require(job.native_group == group, 'Native process group changed within one run')
            return
        identity = _proc_identity(group)
        if identity is not None and identity['ppid'] == job.process.pid and identity['group'] == group and identity['session'] == group:
            job.native_group, job.native_leader_start = group, identity['start']
            job.native_members.update(_members(group))

    def _signal_native_group(self, job, sig):
        if job.native_group is None or _group_gone(job.native_group):
            return
        leader = _proc_identity(job.native_group)
        bound = (leader is not None and leader['start'] == job.native_leader_start
                 and leader['group'] == leader['session'] == job.native_group)
        if not bound:
            bound = any((identity := _proc_identity(pid)) is not None and identity['start'] == started
                        and identity['group'] == identity['session'] == job.native_group
                        for pid, started in job.native_members.items())
        _require(bound, 'Refusing to signal a native group without retained process-instance ownership')
        try:
            os.killpg(job.native_group, sig)
        except ProcessLookupError:
            pass

    @staticmethod
    def _owner_error(job, stage, error):
        message = (stage + ': ' + str(error))[:512]
        if message not in job.owner_errors and len(job.owner_errors) < 8:
            job.owner_errors.append(message)
        if job.error is None:
            job.error = dict(type=type(error).__name__, message=message)

    def _checkpoint(self, job):
        # Disk failures must never disable cancellation, deadlines or the
        # monitor. Keep a bounded in-memory error until a later write can work.
        try:
            self._save(job)
        except Exception as error:
            self._owner_error(job, 'owner receipt', error)

    def _begin_cancel(self, job, *, reason=None, timeout=False):
        job.timed_out = job.timed_out or timeout
        if not job.cancel_requested:
            job.cancel_requested, job.cancel_at = True, self.clock()
            job.cancel_delivery, job.status = 'worker_signal_sent', 'cancelling'
            if reason is not None and job.error is None:
                job.error = dict(type='OwnerStop', message=reason)
            try:
                self._signal_worker(job, signal.SIGINT)
            except Exception as error:
                self._owner_error(job, 'worker cancel signal', error)
            self._checkpoint(job)

    @staticmethod
    def _checked_task(task):
        _require(type(task) is dict and task.get('status') in ('running', 'succeeded', 'failed', 'cancelled'),
                 'ROS 작업 상태 형식이 다릅니다.')
        _require(type(task.get('stage')) is str and len(task['stage']) <= 128,
                 'ROS 작업 단계 형식이 다릅니다.')
        _require(type(task.get('cleanup')) is dict and type(task['cleanup'].get('clean')) is bool,
                 'ROS 정리 상태 형식이 다릅니다.')
        for name in ('lifecycle', 'planning_scene', 'action_terminal'):
            _require(task.get(name) is None or type(task[name]) is dict,
                     'ROS 작업의 상세 상태 형식이 다릅니다: ' + name)
        lifecycle = task.get('lifecycle')
        if lifecycle is not None:
            events = lifecycle.get('events', [])
            _require(type(events) is list and len(events) <= 128 and all(type(item) is dict for item in events),
                     'ROS 작업 이벤트 형식이 다릅니다.')
        if 'actions' in task:
            _require(type(task['actions']) is list and len(task['actions']) <= 128
                     and all(type(item) is dict for item in task['actions']), 'ROS Action 목록 형식이 다릅니다.')
        group = task.get('owned_process_group')
        _require(group is None or (type(group) is int and group > 1), 'ROS 소유 프로세스 그룹 형식이 다릅니다.')

    @staticmethod
    def _checked_guard(guard):
        if guard is None:
            return
        _require(type(guard) is dict and type(guard.get('latched')) is bool
                 and type(guard.get('phase')) is str and type(guard.get('reason')) is str
                 and type(guard.get('controllers')) is dict
                 and all(type(name) is str and type(state) is str for name, state in guard['controllers'].items()),
                 'Native 정지 감시 형식이 다릅니다.')

    def cancel(self, value):
        _require(type(value) is dict and set(value) == {'job_id', 'generation', 'model_sha256'}, '현재 작업·세대·모델 SHA만 받습니다.')
        self._identity(value['model_sha256'])
        with self.lock:
            job = self.current
            _require(job is not None and value['job_id'] == job.identifier and type(value['generation']) is int
                     and value['generation'] == job.generation, '취소할 현재 ROS 작업의 식별자가 다릅니다.')
            if not job.finished:
                self._begin_cancel(job)
            return self.snapshot()

    def _read_update(self, job):
        path = job.directory / 'run/latest.json'
        if not path.exists() or job.display_error is not None:
            return
        try:
            raw = _regular(path, MAX_LATEST_BYTES)
            digest = _sha(raw)
            if digest == job.update_digest:
                return
            value = _json(raw)
            _require(type(value) is dict and value.get('schema_version') == 1 and value.get('kind') in ('task', 'observation')
                     and value.get('run_id') == job.identifier, 'ROS 관측의 실행 식별자가 다릅니다.')
            task, native = value.get('task'), value.get('native_observation')
            self._checked_task(task)
            self._checked_guard(task['cleanup'].get('guard'))
            self._checked_guard(value.get('native_guard'))
            _require(native is None or type(native) is dict, 'Native 관측은 객체여야 합니다.')
            for key, expected in (('model_sha256', self.source['model_sha256']),
                                  ('plugin_sha256', self.source_pin['native/grasp_ros2_plugin/src/observation.cpp'])):
                _require(value.get(key) == expected or (value.get(key) is None and native is None and task.get('stage') == 'preflight'),
                         'ROS 관측의 모델·플러그인 식별자가 다릅니다.')
            sequence = value.get('sequence')
            _require(type(sequence) is int and 1 <= sequence <= 100000, 'ROS 작업 관측 순번이 다릅니다.')
            if job.update is not None and sequence <= job.update['sequence']:
                # Old atomic snapshots cannot replace a later accepted state.
                _require(sequence < job.update['sequence'] or digest == job.update_digest,
                         '서로 다른 ROS 상태가 같은 순번을 사용했습니다.')
                return
            self._bind_native_group(job, task.get('owned_process_group'))
            if native is not None:
                observed, progressed = value.get('observed_monotonic_ns'), value.get('clock_progress_observed_monotonic_ns')
                now = time.monotonic_ns()
                _require(type(observed) is int and type(progressed) is int and 0 < progressed <= observed <= now + 100_000_000
                         and type(value.get('native_clock_has_advanced')) is bool, 'ROS 원본 관측 시각이 다릅니다.')
                captured = native.get('capture_monotonic_ns')
                _require(type(captured) is str and re.fullmatch(r'[1-9][0-9]{0,18}', captured) is not None
                         and native.get('capture_clock_semantics') == 'linux_CLOCK_MONOTONIC',
                         'Native 원본 수집 시각이 다릅니다.')
                captured = int(captured)
                _require(captured <= min(observed, now), 'Native 수집 시각이 원본 수신 시각보다 미래입니다.')
                if job.observation is not None:
                    previous = job.observation.payload
                    _require(native.get('sequence', 0) >= previous['sequence'] and native.get('time_sec', -1) >= previous['time_sec'],
                             'Native 자세가 역행했습니다.')
                    if native.get('sequence') == previous['sequence']:
                        _require(native == previous and observed == job.observed_ns and progressed == job.progress_ns,
                                 '작업 상태 갱신이 원본 자세나 수신 시각을 바꿨습니다.')
                    elif native.get('time_sec') == previous['time_sec']:
                        _require(progressed == job.progress_ns, '정지한 native 시계의 관측 나이를 갱신했습니다.')
                observation = GraspObservation(observed * 1e-9, progressed * 1e-9,
                                               value['native_clock_has_advanced'], _canonical(native))
                pose = job.display.pose(observation)
                job.observation, job.observed_ns, job.progress_ns, job.capture_ns, job.pose = observation, observed, progressed, captured, pose
            job.update, job.update_digest = value, digest
            if task['status'] == 'cancelled':
                job.cancel_delivery = 'worker_observed'
            if not job.cancel_requested:
                job.status = 'running'
        except Exception as error:
            job.display_error = str(error)[:512]
            self._begin_cancel(job, reason='ROS 관측의 식별·시계 검사를 통과하지 못했습니다.')

    def _finish(self, job):
        if job.finished or job.process is None or job.process.poll() is None:
            return
        path = job.directory / 'run/result.json'
        if path.exists():
            try:
                raw = _regular(path, MAX_RESULT_BYTES)
                result = _json(raw)
                _require(type(result) is dict and result.get('kind') == 'ros-grasp-task-v1'
                         and result.get('run_id') == job.identifier and result.get('model_sha256') == self.source['model_sha256']
                         and result.get('status') in ('succeeded', 'failed', 'cancelled'), '최종 ROS 결과의 실행·모델 식별자가 다릅니다.')
                self._checked_task(result)
                self._checked_guard(result.get('cleanup', {}).get('guard'))
                self._bind_native_group(job, result.get('owned_process_group'))
                job.result, job.result_sha256 = result, _sha(raw)
                job.status = result['status']
                if result['status'] == 'cancelled':
                    job.cancel_delivery = 'worker_observed'
                job.error = job.error or deepcopy(result.get('error'))
            except (ValueError, TypeError, KeyError, OSError) as error:
                job.status, job.error = 'failed', dict(type=type(error).__name__, message=str(error)[:512])
        else:
            job.status, job.error = 'failed', dict(type='MissingWorkerResult', message='ROS 작업 프로세스의 최종 결과를 확인하지 못했습니다.')
        native_group = (job.result or {}).get('owned_process_group')
        if job.native_group is not None and not _group_gone(job.native_group):
            self._begin_cancel(job, reason='ROS 작업 종료 후 소유 시뮬레이션 프로세스를 정리하고 있습니다.')
            job.status = 'cancelling'
            return
        unknown_live_group = native_group is not None and job.native_group is None and not _group_gone(native_group)
        clean = bool(not unknown_live_group and _group_gone(job.process.pid) and job.escalation == 0
                     and (job.result or {}).get('cleanup', {}).get('clean') is True)
        job.owner_cleanup_clean = clean
        if job.status == 'succeeded' and (job.process.returncode != 0 or not clean or job.display_error or job.error):
            job.status, job.error = 'failed', job.error or dict(type='IncompleteCleanup', message='정상 ROS 종료와 프로세스 정리를 확인하지 못했습니다.')
        if job.timed_out or job.escalation or unknown_live_group:
            job.status = 'failed'
            job.error = job.error or dict(type='ForcedCleanup', message='제한시간 안에 정상적인 ROS 정리를 확인하지 못했습니다.')
        job.finished = True
        if job.output is not None:
            job.output.thread.join(timeout=.1)
        if job.output is None and job.process.stdout is not None:
            try:
                job.process.stdout.close()
            except Exception as error:
                self._owner_error(job, 'worker output close', error)
        self._checkpoint(job)

    def _advance_cancel(self, job):
        if job.finished or job.cancel_at is None:
            return
        elapsed = self.clock() - job.cancel_at
        if elapsed >= CANCEL_GRACE_SEC + 7 and job.escalation < 2:
            level, sig = 2, signal.SIGKILL
        elif elapsed >= CANCEL_GRACE_SEC and job.escalation < 1:
            level, sig = 1, signal.SIGTERM
        else:
            return
        # A failed native ownership check must not prevent signalling the
        # separately owned worker. Never signal an unproved native process.
        for label, send in (('native escalation', self._signal_native_group),
                            ('worker escalation', self._signal_worker)):
            try:
                send(job, sig)
            except Exception as error:
                self._owner_error(job, label, error)
        job.escalation = level
        self._checkpoint(job)

    def _update(self, job):
        if job.finished:
            return
        try:
            self._read_update(job)
            if job.native_group is not None and self.clock() - job.members_checked >= .5:
                leader = _proc_identity(job.native_group)
                if leader is not None and leader['start'] == job.native_leader_start:
                    job.native_members.update(_members(job.native_group))
                job.members_checked = self.clock()
            if job.process is not None and job.process.poll() is not None:
                self._finish(job)
                if job.finished:
                    return
            if job.output is not None and (job.output.overflow or job.output.error):
                self._begin_cancel(job, reason='ROS 작업 로그의 보존 한도를 초과했습니다.')
            if _directory_bytes(job.directory) > MAX_JOB_BYTES:
                self._begin_cancel(job, reason='ROS 작업 자료의 보존 한도를 초과했습니다.')
        except Exception as error:
            # Quarantine malformed IPC/retention input, preserve the last
            # accepted frame, and keep the independent stop driver alive.
            job.display_error = job.display_error or str(error)[:512]
            self._owner_error(job, 'owned task observation', error)
            self._begin_cancel(job)
        finally:
            if not job.finished:
                if self.clock() - job.started >= self.timeout_sec:
                    self._begin_cancel(job, reason='ROS 작업의 제한시간을 초과했습니다.', timeout=True)
                self._advance_cancel(job)

    def _watch(self):
        while not self.closing.wait(.1):
            with self.lock:
                if self.current is not None and not self.current.finished:
                    try:
                        self._update(self.current)
                    except Exception as error:
                        self._owner_error(self.current, 'owner monitor', error)
                        self._begin_cancel(self.current)
                        self._advance_cancel(self.current)

    @staticmethod
    def _task_summary(record):
        if not record:
            return None
        lifecycle = record.get('lifecycle') or {}
        return dict(stage=lifecycle.get('stage', record.get('stage')), events=deepcopy(lifecycle.get('events', [])[-32:]),
                    physical_grasp=record.get('physical_grasp', 'unconfirmed'),
                    planning_attachment=record.get('planning_attachment', 'uninitialized'))

    def _snapshot_job(self, job):
        task = job.result or (job.update or {}).get('task') or {}
        native = job.observation.payload if job.observation is not None else None
        age = self._source_age(job)
        guard = (job.update or {}).get('native_guard') or task.get('cleanup', {}).get('guard')
        forces = [0., 0.] if native is not None else None
        if native is not None:
            for contact in native['contacts']:
                pair = {contact['geom1_name'], contact['geom2_name']}
                for index, finger in enumerate(('fr5_left_finger_geom', 'fr5_right_finger_geom')):
                    if pair == {finger, 'fr5_block_geom'}:
                        forces[index] += contact['normal_force_n']
        evidence = dict(finger_force_n=forces, tcp_position_m=(native or {}).get('tcp', {}).get('position'),
                        block_position_m=(native or {}).get('block', {}).get('position'),
                        relative_position_error_m=None, relative_angle_error_rad=None)
        for event in reversed((task.get('lifecycle') or {}).get('events', [])):
            if 'relative_position_error_m' in event:
                evidence.update(relative_position_error_m=event['relative_position_error_m'],
                                relative_angle_error_rad=event.get('relative_angle_error_rad'))
                break
        return dict(job_id=job.identifier, generation=job.generation, model_sha256=self.source['model_sha256'],
            status=job.status, process_running=job.process is not None and job.process.poll() is None,
            finished=job.finished, cancel_requested=job.cancel_requested, cancel_delivery=job.cancel_delivery,
            timed_out=job.timed_out, phase=task.get('phase', task.get('stage', 'starting')),
            sequence=(native or {}).get('sequence', 0), source_age_sec=age,
            freshness='unknown' if age is None or not job.observation.clock_has_advanced else 'fresh' if age <= .5 and not job.display_error else 'stale',
            simulation_sec=(native or {}).get('time_sec'), steps=None, task=self._task_summary(task), evidence=evidence,
            action_terminal=deepcopy(task.get('action_terminal')), native_guard=deepcopy(guard),
            cleanup=deepcopy(task.get('cleanup', {})), owner_cleanup_clean=job.owner_cleanup_clean,
            error=job.display_error or deepcopy(job.error) or deepcopy(task.get('error')),
            worker_result_observed=job.result is not None, worker_result_sha256=job.result_sha256,
            process_exit_code=None if job.process is None else job.process.poll(),
            process_group_gone=None if not job.finished else job.process is None or _group_gone(job.process.pid),
            automatic_release=False, automatic_detach=False, automatic_retry=False)

    @staticmethod
    def _source_age(job):
        if job.observation is None:
            return None
        # Delivery and IPC polling must not make a queued old capture current.
        # Keep the stopped-clock age as an independent lower freshness bound.
        return max(0., (time.monotonic_ns() - min(job.capture_ns, job.observed_ns, job.progress_ns)) / 1e9)

    def _save(self, job):
        atomic_json(job.directory / 'owner-receipt.json', dict(schema_version=1, kind='owned-ros-grasp-viewer-v1',
            source=deepcopy(self.source), job=self._snapshot_job(job),
            native_group=job.native_group, native_leader_start=job.native_leader_start,
            physical_stop_inferred_from_process_exit=False, stdout_truncated=bool(job.output and job.output.overflow),
            owner_errors=list(job.owner_errors)))

    def _view(self, job):
        live = job is not None and job.pose is not None
        display = job.display if live else self.preview
        identifier, binding = (job.binding_id, job.binding) if live else (self.preview_id, self.preview_binding)
        manifest = display.manifest
        return dict(binding_id=identifier, binding=deepcopy(binding), mode='live' if live else 'model_preview',
            manifest_descriptor=dict(sha256=_sha(display.manifest_raw), byte_length=len(display.manifest_raw),
                model_sha256=manifest['model_sha256'], scene_sha256=manifest['scene_sha256']),
            sequence=job.observation.sequence if live else 1, pose_ready=True)

    def snapshot(self):
        with self.lock:
            if self.current is not None and not self.current.finished:
                self._update(self.current)
            return dict(schema_version=1, source=deepcopy(self.source),
                        job=None if self.current is None else self._snapshot_job(self.current), view=self._view(self.current))

    def manifest_bytes(self, identifier):
        with self.lock:
            if identifier == self.preview_id:
                return self.preview.manifest_raw
            _require(self.current is not None and identifier == self.current.binding_id, '현재 ROS 표시 식별자가 다릅니다.')
            return self.current.display.manifest_raw

    def asset_bytes(self, digest):
        return self.preview.asset(digest)

    def pose_envelope(self, identifier):
        with self.lock:
            if identifier == self.preview_id:
                return envelope(self.preview_binding, self.preview.static_preview_pose, 1)
            job = self.current
            _require(job is not None and identifier == job.binding_id, '현재 ROS 표시 식별자가 다릅니다.')
            if not job.finished:
                self._update(job)
            _require(job.pose is not None and job.observation is not None,
                     '검증한 실제 native 자세를 기다리고 있습니다.')
            native = job.observation.payload
            age = self._source_age(job)
            result = envelope(job.binding, job.pose, native['sequence'], source_state_age_sec=age,
                              simulation_sec=native['time_sec'])
            result.update(schema_version=2, source=dict(session_id=job.identifier,
                clock_domain='mujoco-native:' + job.identifier, stamp_sec=native['time_sec'],
                stamp_ns=native['stamp_ns'], stamp_semantics=native['stamp_semantics'],
                sequence=native['sequence'], joint_positions_rad=deepcopy(native['arm_positions_rad'])))
            return result

    def close(self):
        self.closing.set()
        with self.lock:
            job = self.current
            if job is not None and not job.finished:
                self._begin_cancel(job)
        self.monitor.join(timeout=1.)
        deadline = time.monotonic() + CANCEL_GRACE_SEC + 9
        while job is not None and not job.finished and time.monotonic() < deadline:
            with self.lock:
                self._update(job)
            time.sleep(.1)
        if job is not None and not job.finished:
            raise RuntimeError('Owned ROS cleanup did not finish; retained process ownership needs inspection')


class RosDemoHandler(RecordingHandler):
    def _origin(self, required=False):
        values = self.headers.get_all('Origin', [])
        port = self.server.server_address[1]
        return ((not required and not values) or (len(values) == 1 and values[0] in
                (f'http://127.0.0.1:{port}', f'http://localhost:{port}')))

    def _host(self):
        return len(self.headers.get_all('Host', [])) == 1 and self.valid_host()

    def do_GET(self):
        if not self._host() or not self._origin():
            return self.respond(dict(error='허용되지 않은 ROS 시연 주소입니다.'), 403)
        parsed, app = urlsplit(self.path), self.server.app
        try:
            if parsed.path == '/api/ros-demo' and not parsed.query:
                return self.respond(dict(app.snapshot(), token=self.server.token))
            if parsed.path in ('/api/view/manifest', '/api/view/pose'):
                query = parse_qs(parsed.query, keep_blank_values=True)
                _require(set(query) == {'binding_id'} and len(query['binding_id']) == 1, '표시 식별자가 필요합니다.')
                return self.respond((app.manifest_bytes if parsed.path.endswith('manifest') else app.pose_envelope)(query['binding_id'][0]))
            if parsed.path.startswith('/api/view/assets/') and not parsed.query:
                digest = parsed.path.removeprefix('/api/view/assets/')
                _require(_SHA.fullmatch(digest), '잘못된 표시 자산 식별자입니다.')
                return self.respond(app.asset_bytes(digest), mime='application/octet-stream')
            local = {'/': 'ros_demo.html', '/ros_demo.html': 'ros_demo.html', '/ros_demo.js': 'ros_demo.js', '/ros_demo.css': 'ros_demo.css'}
            shared = {'/viewer.js': 'viewer.js', '/viewer_contract.mjs': 'viewer_contract.mjs',
                '/vendor/three.module.js': 'vendor/three.module.js', '/vendor/three.core.js': 'vendor/three.core.js',
                '/vendor/OrbitControls.js': 'vendor/OrbitControls.js'}
            if not parsed.query and (parsed.path in local or parsed.path in shared):
                name = local.get(parsed.path, shared.get(parsed.path))
                package = __package__ if parsed.path in local else 'mujoco_ros2_manipulation.operator_console'
                raw = files(package).joinpath('static', name).read_bytes()
                if name == 'viewer.js': raw = raw.replace(b"from 'three'", b"from './vendor/three.module.js'")
                elif name == 'vendor/OrbitControls.js': raw = raw.replace(b"from 'three'", b"from './three.module.js'")
                mime = 'text/javascript' if name.endswith(('.js', '.mjs')) else 'text/css' if name.endswith('.css') else 'text/html'
                return self.respond(raw, mime=mime + '; charset=utf-8')
            return self.respond(dict(error='ROS 시연 자료를 찾을 수 없습니다.'), 404)
        except (ValueError, KeyError, OSError, TypeError) as error:
            return self.respond(dict(error=str(error)), 409)

    def do_POST(self):
        tokens = self.headers.get_all('X-Operator-Token', [])
        if (not self._host() or not self._origin(required=True) or len(tokens) != 1
                or not tokens[0].isascii() or not hmac.compare_digest(tokens[0], self.server.token)):
            return self.respond(dict(error='이 전용 화면의 ROS 시연 요청만 허용합니다.'), 403)
        parsed = urlsplit(self.path)
        if parsed.query or parsed.path not in ('/api/ros-demo/start', '/api/ros-demo/cancel'):
            return self.respond(dict(error='정해진 ROS 작업 시작과 취소만 지원합니다.'), 404)
        try:
            lengths = self.headers.get_all('Content-Length', [])
            _require(len(lengths) == 1 and lengths[0].isascii() and lengths[0].isdigit(), 'JSON 요청 길이가 필요합니다.')
            size = int(lengths[0])
            _require(0 < size <= 1024 and len(self.headers.get_all('Content-Type', [])) == 1
                     and self.headers.get_content_type() == 'application/json' and not self.headers.get('Transfer-Encoding'),
                     '작은 JSON 요청이 필요합니다.')
            self.connection.settimeout(3.)
            body = _json(self.rfile.read(size))
            action = self.server.app.start if parsed.path.endswith('start') else self.server.app.cancel
            return self.respond(dict(action(body), token=self.server.token))
        except (ValueError, TypeError, KeyError, OSError, UnicodeError) as error:
            return self.respond(dict(error=str(error)), 400)
        except RuntimeError as error:
            return self.respond(dict(error=str(error)), 409)


def serve(app, port):
    server = ThreadingHTTPServer(('127.0.0.1', port), RosDemoHandler)
    server.daemon_threads = True
    server.app, server.token = app, secrets.token_urlsafe(32)
    return server


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execution-environment', type=Path, required=True)
    parser.add_argument('--manifest-sha256', required=True)
    parser.add_argument('--domain', type=int, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--port', type=int, required=True)
    args = parser.parse_args(argv)
    if not 0 <= args.domain <= 232 or not 1024 <= args.port <= 65535:
        parser.error('Expected unused ROS domain 0..232 and loopback port 1024..65535')
    target = load_ros_grasp_target(load_workcell('fr5_ur5e'), args.execution_environment, args.manifest_sha256)
    app = server = None
    previous = signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    try:
        app = RosDemoApplication(target, args.output, domain=args.domain)
        server = serve(app, args.port)
        print(f'NATIVE_ROS_GRASP_VIEWER_READY http://127.0.0.1:{args.port}/ domain={args.domain}', flush=True)
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        signal.signal(signal.SIGTERM, previous)
        if server is not None: server.server_close()
        if app is not None: app.close()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
