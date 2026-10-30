"""Console-owned choice of a fixed motion or native grasp execution owner.

This adapter never changes native grasp admission, timestamps, controllers or
process cleanup. It arbitrates UI ownership and routes the one shared 3D view to
an explicitly constructed owner. No browser-provided path/domain is accepted.
"""
from copy import deepcopy
from contextlib import contextmanager
import threading
import re
import uuid


_ID = re.compile(r'[0-9a-f]{32}')


class WorkspaceTasks:
    def __init__(self, app, grasp=None):
        self.app, self.grasp = app, grasp
        self._transport = threading.local()
        self.selected = 'motion'
        self.selection_id = uuid.uuid4().hex
        if grasp is not None:
            if not app.render_enabled or getattr(app, 'execution_target', None) is not None:
                raise ValueError('집기 작업은 기본 작업 환경의 웹 3D 화면에서 명시적으로 연결하세요.')
            if grasp.domain == app.state.config.get('domain'):
                raise ValueError('일반 조작과 집기는 서로 다른 명시적 ROS 도메인을 사용하세요.')

    def require_motion(self):
        expected = getattr(self._transport, 'selection_id', None)
        if expected is not None and expected != self.selection_id:
            raise ValueError('작업 선택이 바뀌었습니다. 이전 일반 조작 요청을 재사용하지 않습니다.')
        if self.selected != 'motion':
            raise RuntimeError('집기 작업이 선택되어 있습니다. 정리 완료 후 일반 조작으로 돌아오세요.')

    @contextmanager
    def motion_request(self, headers):
        """Recheck a captured HTTP epoch at each normal admission point.

        The request thread alone carries this epoch. Never hold the application
        lock across slow model hashing, playback I/O or a response write.
        """
        if self.grasp is None:
            yield
            return
        if (type(headers) is not list or len(headers) != 1 or type(headers[0]) is not str
                or not _ID.fullmatch(headers[0])):
            raise ValueError('현재 작업 선택의 X-Workspace-Selection 헤더가 필요합니다.')
        previous = getattr(self._transport, 'selection_id', None)
        self._transport.selection_id = headers[0]
        try:
            with self.app.lock:
                self.require_motion()
            yield
        finally:
            if previous is None:
                del self._transport.selection_id
            else:
                self._transport.selection_id = previous

    def _normal_reason(self):
        app = self.app
        if app.closing.is_set():
            return '화면을 종료하고 있습니다.'
        if app.phase == 'failed' and app.error:
            return '일반 실행의 종료 오류를 확인한 뒤 서버를 복구하고 작업을 바꾸세요.'
        if app.demos.busy or app.phase in ('starting', 'running', 'stopping', 'demo'):
            return '일반 시뮬레이션과 시연을 종료한 뒤 작업을 바꾸세요.'
        if any(process.poll() is None for kind, process in app.children.items() if kind != 'renderer'):
            return '일반 조작의 소유 프로세스가 종료된 뒤 작업을 바꾸세요.'
        if app.state.active or not app.bridge.outbox.empty():
            return '현재 요청과 전송 대기가 끝난 뒤 작업을 바꾸세요.'
        review = app.state.review_snapshot()
        if review and review.get('state') == 'awaiting_approval':
            return '승인 대기 계획을 폐기한 뒤 작업을 바꾸세요.'
        if app.views.preview:
            return '미리보기를 종료하고 실시간 보기로 돌아온 뒤 작업을 바꾸세요.'
        capture = getattr(getattr(app, 'sync_recordings', None), 'current', {})
        if capture.get('status') in ('preparing', 'recording', 'saving'):
            return '현재 센서 기록의 저장이 끝난 뒤 작업을 바꾸세요.'
        if any(app.state.graph.get(name, 0) for name in ('clock', 'joints', 'status', 'guards')):
            return '일반 ROS 도메인에 실행이 남아 있습니다. 해당 소유자의 종료를 확인하세요.'
        return None

    @staticmethod
    def _grasp_reason(grasp):
        job = None if grasp is None else grasp.get('job')
        if job and (not job.get('finished') or job.get('owner_cleanup_clean') is not True):
            return '집기 작업의 취소·종료와 소유 프로세스 정리를 확인하고 있습니다.'
        return None

    def _identity(self, value, fields):
        if type(value) is not dict or set(value) != set(fields) | {'selection_id'}:
            raise ValueError('현재 작업 선택과 정해진 요청 필드만 지정하세요.')
        selection = value['selection_id']
        if type(selection) is not str or not _ID.fullmatch(selection) or selection != self.selection_id:
            raise ValueError('작업 선택이 바뀌었습니다. 현재 상태를 다시 확인하세요.')

    def select(self, value):
        app = self.app
        with app.lock, app.state.lock:
            self._identity(value, {'task'})
            task = value['task']
            if type(task) is not str or task not in ('motion', 'grasp'):
                raise ValueError('목록에 등록된 작업만 선택하세요.')
            if task == self.selected:
                return self.snapshot()
            if task == 'grasp' and self.grasp is None:
                raise ValueError('이 서버에 집기 실행 환경이 등록되지 않았습니다.')
            native = None if self.grasp is None else self.grasp.snapshot()
            reason = self._grasp_reason(native) or self._normal_reason()
            if reason:
                raise RuntimeError(reason)
            if self.grasp is not None and task == 'grasp' and getattr(self.grasp, 'closing', None) is not None and self.grasp.closing.is_set():
                raise RuntimeError('집기 실행 소유자가 종료 중입니다.')
            # Clear the old normal binding even on a round trip. Late normal
            # poses cannot become current merely by returning to the task.
            if app.views.host is not None:
                app.views.host.invalidate()
            app.views.key = app.views.source = None
            app.views.stamp_key = app.views.last_accepted_stamp = None
            app.views.revision += 1
            self.selected, self.selection_id = task, uuid.uuid4().hex
            if task == 'motion':
                app.views.tick()
            app.audit('workspace_task_selected', task=task, selection_id=self.selection_id)
            return self.snapshot()

    def start_grasp(self, value):
        app = self.app
        with app.lock, app.state.lock:
            self._identity(value, {'request_id', 'model_sha256'})
            if self.selected != 'grasp' or self.grasp is None or app.closing.is_set():
                raise RuntimeError('현재 화면에서 등록된 집기 작업을 선택하세요.')
            reason = self._normal_reason()
            if reason:
                raise RuntimeError(reason)
            # The native owner handles duplicate request IDs, model identity,
            # retained limits and finished+clean admission itself, unchanged.
            result = self.grasp.start({key: value[key] for key in ('request_id', 'model_sha256')})
            app.audit('workspace_grasp_requested', request_id=value['request_id'])
            return dict(selection_id=self.selection_id, task='grasp', grasp=result)

    def cancel_grasp(self, value):
        with self.app.lock:
            self._identity(value, {'job_id', 'generation', 'model_sha256'})
            if self.grasp is None:
                raise RuntimeError('이 서버의 집기 실행 소유자가 없습니다.')
            # Target the exact native owner/job, not a normal request or a
            # loosely inferred currently displayed robot.
            result = self.grasp.cancel({key: value[key] for key in ('job_id', 'generation', 'model_sha256')})
            self.app.audit('workspace_grasp_cancel_requested', job_id=value['job_id'], generation=value['generation'])
            return dict(selection_id=self.selection_id, task='grasp', grasp=result)

    def _capacity_reason(self):
        if self.grasp is None:
            return '이 서버에 집기 실행 환경이 등록되지 않았습니다.'
        closing = getattr(self.grasp, 'closing', None)
        if closing is not None and closing.is_set():
            return '집기 실행 소유자가 종료 중입니다.'
        # Read the native owner's own retained-request limit; do not reset it
        # when the page changes task or a failed start reserves a request.
        from ..grasping.ros_demo_server import MAX_JOBS
        if len(getattr(self.grasp, 'requests', {})) >= MAX_JOBS:
            return '집기 작업 보존 상한에 도달했습니다. 새 서버 작업 공간이 필요합니다.'
        return None

    def snapshot(self):
        with self.app.lock, self.app.state.lock:
            native = None if self.grasp is None else self.grasp.snapshot()
            reason = self._grasp_reason(native) or self._normal_reason()
            available = self.grasp is not None
            start_reason = reason or self._capacity_reason()
            if not available:
                reason = start_reason
            return dict(selected=self.selected, selection_id=self.selection_id,
                options=[dict(id='motion', label='관절·TCP 조작', available=True),
                         dict(id='grasp', label='집기·운반·놓기', available=available)],
                can_select=available and reason is None,
                can_start=self.selected == 'grasp' and available and start_reason is None,
                reason=start_reason if self.selected == 'grasp' else reason, grasp=native)

    def view(self, native=None):
        if self.selected != 'grasp' or self.grasp is None:
            raise RuntimeError('현재 집기 표시가 선택되지 않았습니다.')
        native = self.grasp.snapshot() if native is None else native
        result = deepcopy(native['view'])
        source = deepcopy(native['source'])
        result.update(enabled=True, status='ready' if result.get('pose_ready') else 'waiting',
            source=source, config_source_id=source['id'],
            catalog=[dict(id=source['id'], label=source['label'], available=True)],
            preview_active=result.get('mode') != 'live', review_plan=False,
            duration_sec=None, clearance_minimum_time_sec=None, preview_time_sec=None,
            error=None, display_mode='web3d', task_selected='grasp')
        return result

    def _current(self, binding):
        if self.selected != 'grasp' or self.grasp is None:
            raise ValueError('현재 집기 표시가 아닙니다.')
        current = self.grasp.snapshot()['view']
        if binding != current.get('binding_id'):
            raise ValueError('현재 선택한 집기 표시 식별자가 다릅니다.')

    def manifest_bytes(self, binding):
        with self.app.lock:
            self._current(binding)
            return self.grasp.manifest_bytes(binding)

    def pose_envelope(self, binding):
        with self.app.lock:
            self._current(binding)
            return self.grasp.pose_envelope(binding)

    def asset_bytes(self, digest):
        with self.app.lock:
            if self.selected != 'grasp' or self.grasp is None:
                raise ValueError('현재 집기 표시가 아닙니다.')
            return self.grasp.asset_bytes(digest)

    def hierarchy_bytes(self, binding):
        with self.app.lock:
            self._current(binding)
            return None  # Geometry picking remains exact; no invented body roles.

    def close(self):
        if self.grasp is not None:
            self.grasp.close()
            reason = self._grasp_reason(self.grasp.snapshot())
            if reason:
                raise RuntimeError(reason)
        return True
