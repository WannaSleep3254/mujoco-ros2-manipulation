"""Console-owned scene selection and read-only poses, independent of ROS entities."""
from copy import deepcopy
import hashlib
import json
import math
from pathlib import Path
import time
from ..model_build.integrity import file_sha256
from ..profiles import load_profile, check_generated_model
from ..workcells.integrity import check_workcell
from ..workcells.motion_request import request_id
from .scene_host import SceneHost, SceneSource
from .display_contract import DisplayHostAdapter
from .view_binding import make_binding
from .view_plan import SavedPlan, ReviewPlan
from .demo_jobs import matching_process


# Checked display profiles never alter the configured execution workcell.
READ_ONLY_PROFILE_IDS = ('fr5', 'fr10', 'ur5e', 'fr3')


def _extra_view_sources(values, cell, *, enabled):
    """Freeze trusted startup-only preview entries before starting a worker.

    Browser selection receives declared IDs only. This does not change the base
    model, ROS hardware identity, command limits or admission suspension state.
    """
    if type(values) not in (tuple,list) or len(values)>8:
        raise ValueError('읽기 전용 작업 환경은 명시적인 목록으로 최대 8개까지 등록하세요.')
    if values and not enabled:
        raise ValueError('화면 표시가 비활성화된 실행에는 추가 작업 환경을 등록할 수 없습니다.')
    if not values:
        return ()
    names=tuple(cell.joints);home=dict(zip(names,cell.home));seen=set();result=[]
    for source in values:
        if (not isinstance(source,SceneSource) or not source.id.startswith('scene:')
                or source.id in seen):
            raise ValueError('중복되지 않는 scene: 식별자의 검증된 작업 환경만 등록하세요.')
        if (source.joint_names!=names or type(source.home) is not dict or source.home!=home
                or any(type(value) not in (int,float) or not math.isfinite(value) for value in source.home.values())):
            raise ValueError('추가 작업 환경은 현재 작업 환경과 정확히 같은 관절·홈 자세를 사용해야 합니다.')
        seen.add(source.id)
        token=deepcopy(source.validate())
        # Source validators may return a token or None; SceneHost independently
        # checks model bytes. A JSON token must be finite and detached here.
        token_json=json.dumps(token,sort_keys=True,separators=(',',':'),allow_nan=False)
        contract=(source.id,source.label,source.xml_path,source.joint_names,deepcopy(source.home),source.robot_roots)
        def validate(source=source,expected=deepcopy(token),token_json=token_json,contract=contract):
            current=(source.id,source.label,source.xml_path,source.joint_names,source.home,source.robot_roots)
            if current!=contract or type(source.home) is not dict or any(type(value) not in (int,float) or not math.isfinite(value) for value in source.home.values()):
                raise ValueError('등록한 읽기 전용 작업 환경의 모델 계약이 바뀌었습니다.')
            current_token=source.validate()
            if json.dumps(current_token,sort_keys=True,separators=(',',':'),allow_nan=False)!=token_json:
                raise ValueError('등록한 읽기 전용 작업 환경이 바뀌었습니다. 시작 시 다시 검증하세요.')
            return deepcopy(expected)
        result.append(SceneSource(source.id,source.label,source.xml_path,source.joint_names,deepcopy(source.home),validate,robot_roots=source.robot_roots))
    return tuple(result)


class ConsoleViews:
    def __init__(self, app, *, enabled=True, host_factory=SceneHost, extra_sources=()):
        extra_sources=_extra_view_sources(extra_sources,app.cell,enabled=enabled)
        self.app, self.enabled = app, enabled
        self.mode = 'live'
        self.revision = 0
        self.key = None
        self.epoch = time.monotonic()
        self.source = None
        self.plan = None
        self.review_plan = False
        self.plan_time = 0.
        self.preview_source = None
        self.plan_request_id = self.plan_motion_session_id = None
        self.error = None
        self.stamp_key = None
        self.last_accepted_stamp = None
        self.run_reset_blocked = None
        self.run_reset_generation = None
        self.registry = {}
        self.catalog = []
        self.metadata = self.base = self.host = None
        if not enabled:
            return
        self.metadata = check_workcell(app.cell)
        self.base = self._cell_source()
        self.registry[self.base.id] = self.base
        self.catalog.append(dict(id=self.base.id,label=app.cell.label,available=True))
        for identifier in READ_ONLY_PROFILE_IDS:
            name = 'profile:'+identifier
            try:
                profile = load_profile(identifier,root=app.cell.root)
                # The profile validator returns None. Freeze the checked source
                # bytes explicitly rather than comparing None with None forever.
                check_generated_model(profile)
                metadata_path = profile.output/'source.json'
                metadata_raw = metadata_path.read_bytes()
                metadata = json.loads(metadata_raw)
                metadata_sha256 = hashlib.sha256(metadata_raw).hexdigest()
                def validate(profile=profile, metadata_path=metadata_path,
                             expected_sha256=metadata_sha256, expected=deepcopy(metadata)):
                    check_generated_model(profile)
                    raw = metadata_path.read_bytes()
                    current = json.loads(raw)
                    if hashlib.sha256(raw).hexdigest() != expected_sha256 or current != expected:
                        raise ValueError('Selected robot model changed')
                    return current
                source = SceneSource(name,profile.label,profile.scene_path,tuple(profile.joints),dict(zip(profile.joints,profile.home)),validate)
                self.registry[name] = source
                self.catalog.append(dict(id=name,label=profile.label,available=True))
            except (OSError,ValueError,KeyError):
                self.catalog.append(dict(id=name,label=identifier.upper(),available=False))
        for source in extra_sources:
            if source.id in self.registry or any(item['id']==source.id for item in self.catalog):
                raise ValueError('기존 모델과 중복된 읽기 전용 작업 환경 식별자입니다.')
            self.registry[source.id]=source
            self.catalog.append(dict(id=source.id,label=source.label,available=True))
        self.host = host_factory(app.directory/'scene-view',app.cell.root/'.venv/bin/python',cwd=app.cell.root)
        if host_factory is SceneHost:
            self.host = DisplayHostAdapter(self.host)

    @property
    def preview(self):
        return self.mode != 'live'

    def _cell_source(self):
        target = getattr(self.app, "execution_target", None)
        if target is not None:
            target.validate()
            cell = self.app.cell
            return SceneSource("scene:"+target.id, cell.label+" · "+target.id,
                target.model_path, tuple(cell.joints), dict(zip(cell.joints,cell.home)),
                target.validate, robot_roots=tuple((i.id, i.prefix+i.profile.ros["base_frame"]) for i in getattr(cell,"instances",())))
        def validate():
            current = check_workcell(self.app.cell)
            if current != self.metadata:
                raise ValueError('Selected workcell model changed')
            return current
        cell = self.app.cell
        return SceneSource('workcell:'+cell.id,cell.label,cell.output/'cell.xml',tuple(cell.joints),dict(zip(cell.joints,cell.home)),validate,robot_roots=tuple((i.id,i.prefix+i.profile.ros['base_frame']) for i in getattr(cell,"instances",())))

    def _activate(self, key, source, *, job_id=None, run_id=None, motion_session_id=None, request_id=None):
        self.revision += 1
        try:
            binding = make_binding(self.app.session_id,self.revision,source.id,self.mode,
                job_id=job_id,run_id=run_id,motion_session_id=motion_session_id,request_id=request_id)
        except ValueError:
            # A malformed observed session must not leave the previous binding
            # current just because the replacement failed validation.
            self.epoch = time.monotonic()
            self.key = self.source = None
            self.host.invalidate()
            raise
        self.epoch = time.monotonic()
        self.key, self.source = key, source
        self.stamp_key, self.last_accepted_stamp = key, None
        self.host.activate(binding,source)
        self.error = None

    def _invalidate(self, key):
        """Start a blank epoch before a new launch/model can claim live state."""
        self.revision += 1
        binding = make_binding(self.app.session_id,self.revision,self.base.id,'live')
        self.epoch = time.monotonic()
        self.key, self.source = key, None
        self.stamp_key = self.last_accepted_stamp = None
        self.host.invalidate(binding)
        self.error = None

    def _idle(self):
        app = self.app
        if app.closing.is_set() or app.demos.busy or app.state.active or app.phase in ('starting','stopping','demo'):
            raise RuntimeError('현재 요청이나 시연이 끝난 뒤 미리보기를 선택하세요.')
        if not self.enabled:
            raise RuntimeError('이 실행은 화면 표시가 비활성화되어 있습니다.')
        if app.display_mode != 'web3d':
            raise RuntimeError('로컬 3D 표시에서 미리보기를 선택하세요.')

    def _context(self):
        return self.app.generation,self.revision

    def _unchanged(self, context):
        self._idle()
        if self._context() != context:
            raise RuntimeError('검증 중 실행이나 표시 대상 선택이 바뀌었습니다. 다시 선택하세요.')

    @staticmethod
    def _read_record(path, maximum):
        # This bounded read is deliberately outside application/state locks.
        with path.open('rb') as stream:
            raw = stream.read(maximum+1)
        if len(raw) > maximum:
            raise ValueError('저장된 요청 자료의 크기 제한을 초과했습니다.')
        return raw

    def select(self, payload):
        app = self.app
        with app.lock, app.state.lock:
            self._idle()
            if type(payload) is not dict or set(payload) != {'mode','source_id'} or payload['mode'] not in ('live','model_preview'):
                raise ValueError('지원하는 화면 보기 모드를 선택하세요.')
            if not isinstance(payload['source_id'],str) or payload['source_id'] not in self.registry:
                raise ValueError('준비된 목록에서 모델을 선택하세요.')
            source = self.registry[payload['source_id']]
            if payload['mode'] == 'live' and source.id != self.base.id:
                raise ValueError('실시간 제어는 현재 작업 환경에서만 지원합니다.')
            mode = payload['mode']
            context = self._context()
        # Hashing/validation may be slow and must not delay state callbacks or
        # admission. A concurrent command/start is handled by the commit recheck.
        source.validate()
        with app.lock, app.state.lock:
            self._unchanged(context)
            if self.registry.get(source.id) is not source:
                raise RuntimeError('검증 중 모델 목록이 바뀌었습니다. 다시 선택하세요.')
            if mode == 'live':
                motion = (app.state.server or {}).get('session_id')
                make_binding(app.session_id,self.revision+1,source.id,'live',motion_session_id=motion)
            else:
                make_binding(app.session_id,self.revision+1,source.id,'model_preview')
            self.mode, self.plan, self.plan_time = mode, None, 0.
            self.review_plan = False
            self.preview_source = source if mode == 'model_preview' else None
            self.plan_request_id = self.plan_motion_session_id = None
            if self.mode == 'model_preview':
                self._activate(('model_preview',source.id,self.revision+1),source)
            else:
                self.key = None
                self.tick()
            app.audit('view_selected',mode=self.mode,source_id=source.id)
            return self.snapshot()

    def select_plan(self, identifier):
        return self._select_plan(identifier, review=False)

    def select_review_plan(self, identifier):
        return self._select_plan(identifier, review=True)

    def _select_plan(self, identifier, *, review):
        app = self.app
        with app.lock, app.state.lock:
            self._idle()
            # Only IDs already observed by this console are eligible. Never accept
            # a client path or scan arbitrary saved runtime sessions.
            request_id(identifier)
            record = deepcopy(app.state.history.get(identifier))
            if record is None or record.get('status') != 'succeeded' or not record.get('trace_path'):
                raise ValueError('이 화면에서 완료한 요청의 계획만 다시 볼 수 있습니다.')
            context = self._context()
            pending = app.state.review_snapshot() if review else None
            if review and (pending is None or pending.get('plan_request_id') != identifier
                           or pending.get('state') != 'awaiting_approval' or pending.get('remaining_sec', 0) <= 0
                           or pending.get('session_id') != (app.state.server or {}).get('session_id')):
                raise ValueError('현재 서버에서 승인 대기 중인 계획만 검토할 수 있습니다.')
            root = app.cell.root/'runtime'
            base = self.base
            model_manifest = deepcopy(self.metadata)
            joint_names = tuple(app.cell.joints)
            joint_limits = deepcopy(app.state.config['joint_limits'])
        path = Path(record['trace_path']).resolve()
        root = root.resolve()
        if not path.is_relative_to(root) or path.name != identifier+'.json' or not path.is_file() or path.stat().st_size > 32*1024*1024:
            raise ValueError('요청의 저장 경로를 확인할 수 없습니다.')
        session_path = (path.parent/'session.json').resolve()
        if not session_path.is_relative_to(root) or not session_path.is_file() or session_path.stat().st_size > 4*1024*1024:
            raise ValueError('요청의 모델 출처를 확인할 수 없습니다.')
        base.validate()
        plan_options = dict(review=pending) if review else {}
        plan_type = ReviewPlan if review else SavedPlan
        plan = plan_type(self._read_record(path,32*1024*1024),self._read_record(session_path,4*1024*1024),
            request_id=identifier,motion_session_id=record['session_id'],model_manifest=model_manifest,
            joint_names=joint_names,joint_limits=joint_limits, **plan_options)
        initial_positions = plan.positions(0.)
        with app.lock, app.state.lock:
            self._unchanged(context)
            if (self.base is not base or self.metadata != model_manifest
                    or tuple(app.cell.joints) != joint_names or app.state.config['joint_limits'] != joint_limits
                    or app.state.history.get(identifier) != record):
                raise RuntimeError('검증 중 요청이나 모델 출처가 바뀌었습니다. 다시 선택하세요.')
            if review:
                current = app.state.review_snapshot()
                if current is None or current.get('plan_sha256') != pending['plan_sha256'] or current.get('state') != 'awaiting_approval':
                    raise RuntimeError('검토 중 계획이 바뀌었거나 만료되었습니다. 다시 계획하세요.')
            self.mode, self.plan, self.plan_time = 'plan_preview', plan, 0.
            self.review_plan = review
            self.preview_source = self.base
            self.plan_request_id,self.plan_motion_session_id = identifier,record['session_id']
            self._activate(('plan_preview',identifier,self.revision+1),self.base,
                motion_session_id=record['session_id'],request_id=identifier)
            self.host.submit(initial_positions,preview_time_sec=0.)
            app.audit('plan_preview_selected',request_id=identifier)
            return self.snapshot()

    def approve_review(self, identifier):
        """Leave the reviewed display before dispatch; backend still revalidates."""
        app = self.app
        with app.lock, app.state.lock:
            self._idle()
            current = app.state.review_snapshot()
            if (not self.review_plan or self.mode != 'plan_preview' or self.plan_request_id != identifier
                    or current is None or current.get('plan_request_id') != identifier
                    or current.get('state') != 'awaiting_approval' or current.get('remaining_sec', 0) <= 0
                    or current.get('session_id') != self.plan_motion_session_id):
                raise RuntimeError('현재 계획을 다시 검토한 뒤 실행하세요.')
            self.mode, self.plan, self.review_plan, self.plan_time = 'live', None, False, 0.
            self.preview_source = None
            self.plan_request_id = self.plan_motion_session_id = None
            self.key = None
            self.tick()

    def seek(self, stamp, *, binding_id):
        with self.app.lock, self.app.state.lock:
            self._idle()
            if self.mode != 'plan_preview' or self.plan is None:
                raise RuntimeError('저장된 계획을 먼저 선택하세요.')
            current = self.host.snapshot().get('binding_id')
            if not isinstance(binding_id,str) or current is None or binding_id != current:
                raise ValueError('계획 미리보기가 바뀌었습니다. 현재 경로에서 다시 선택하세요.')
            positions = self.plan.positions(stamp)
            self.plan_time = stamp
            self.host.submit(positions,preview_time_sec=stamp)
            return dict(preview_time_sec=stamp,duration_sec=self.plan.duration)

    def tick(self):
        with self.app.lock:
            self._tick()

    def _tick(self):
        if not self.enabled:
            return
        app, now = self.app, time.monotonic()
        if app.image_render_enabled:
            if self.key != ('image',):
                self.revision += 1
                self.epoch = now
                self.key,self.source = ('image',),None
                self.stamp_key = self.last_accepted_stamp = None
                self.host.invalidate()
            return
        if self.preview:
            if self.key == ('image',):
                if self.mode == 'plan_preview' and self.plan is not None:
                    self._activate(('plan_preview',self.plan_request_id,self.revision+1),self.base,
                        motion_session_id=self.plan_motion_session_id,request_id=self.plan_request_id)
                else:
                    self._activate(('model_preview',self.preview_source.id,self.revision+1),self.preview_source)
            if self.mode == 'plan_preview' and self.plan is not None:
                self.host.submit(self.plan.positions(self.plan_time),preview_time_sec=self.plan_time)
            return
        job = app.demos.current or {}
        launch = job.get('active_launch')
        if self.run_reset_blocked is not None:
            observed_run = ((job.get('job_id'),launch.get('run_id'))
                if app.demos.busy and launch and launch.get('run_id') else None)
            if (app.generation != self.run_reset_generation
                    or (observed_run is not None and observed_run != self.run_reset_blocked)):
                self.run_reset_blocked = self.run_reset_generation = None
            else:
                waiting = ('clock_reset_wait',self.run_reset_blocked,app.generation,
                    job.get('stage'),job.get('variant'),job.get('run_id'))
                if waiting != self.key:
                    self._invalidate(waiting)
                return
        waiting = ('demo_wait',job.get('job_id'),job.get('stage'),job.get('variant'),
            (launch or {}).get('run_id') or job.get('run_id'),(launch or {}).get('model_sha256'))
        known_run = (self.key is not None and len(self.key) == 5 and self.key[0] == 'demo'
            and self.source is not None
            and self.key[1:4] == (job.get('job_id'),job.get('variant'),job.get('run_id'))
            and (launch is None or (launch.get('run_id') == self.key[3]
                and launch.get('model_sha256') == self.key[4]
                and Path(launch.get('model_path','')).resolve() == self.source.xml_path)))
        if app.demos.busy:
            # Preparation for a different run must start blank. Cleanup of this
            # exact already-observed run may retain its aging last actual pose.
            if not launch or not launch.get('model_sha256'):
                if launch is None and known_run:
                    return  # No submission: the original receiver age keeps growing.
                if waiting != self.key:
                    self._invalidate(waiting)
                return
            digest = launch['model_sha256']
            key = ('demo',job['job_id'],job.get('variant'),launch['run_id'],digest)
            if key != self.key:
                # Clear old poses before validation, including its failure path.
                if waiting != self.key:
                    self._invalidate(waiting)
                app.demos.validate_launch(launch)
                if not matching_process(launch):
                    return
                model = Path(launch['model_path']).resolve()
                def validate(model=model, digest=digest):
                    if file_sha256(model) != digest:
                        raise ValueError('Demonstration model changed')
                    self.base.validate()
                    return dict(model_sha256=digest,base=self.metadata)
                source = SceneSource(self.base.id,self.base.label,model,self.base.joint_names,self.base.home,validate)
                self._activate(key,source,job_id=job['job_id'],run_id=launch['run_id'])
            elif not matching_process(launch):
                if known_run:
                    return  # Retain only this same validated run/model; stop feeding poses.
                self._invalidate(waiting)
                return
        elif self.key and self.key[0] in ('demo','demo_wait') and app.phase not in ('starting','running'):
            if known_run:
                return  # Preserve the final pose and free camera with a growing stale age.
            if waiting != self.key:
                self._invalidate(waiting)
            return
        else:
            motion = (app.state.server or {}).get('session_id')
            key = ('live',app.generation,motion)
            if self.key != key:
                self._activate(key,self.base,motion_session_id=motion)
        with app.state.lock:
            state = app.state
            stamp = state.joint_state_stamp_simulation_sec
            valid_stamp = type(stamp) in (int,float) and math.isfinite(stamp) and stamp >= 0
            if (valid_stamp and self.stamp_key == self.key and self.last_accepted_stamp is not None
                    and stamp < self.last_accepted_stamp):
                # A same-path new simulator can publish before the demo manager
                # has learned its new launch. Keep this view blank until that
                # run identity changes or the owner explicitly changes lifecycle.
                self.run_reset_blocked = ((job.get('job_id'),launch.get('run_id'))
                    if app.demos.busy and launch else (None,None))
                self.run_reset_generation = app.generation
                self._invalidate(('clock_reset_wait',self.run_reset_blocked,app.generation,
                    job.get('stage'),job.get('variant'),job.get('run_id')))
                return
            # Display admission differs from command admission: an active request,
            # blocked guard, or non-ready server still has a meaningful live pose.
            if state.graph['clock'] != 1 or state.graph['joints'] != 1 or state.positions is None or state.joint_at is None:
                return
            if state.joint_at < self.epoch or now-state.joint_at > .5 or state.clock_at is None or now-state.clock_at > .5:
                return
            requested = getattr(state,'identity_requested_at',None)
            checked = getattr(state,'identity_checked_at',None)
            if (self.source is None or state.observed_model_path != str(self.source.xml_path)
                    or requested is None or checked is None
                    or requested < self.epoch or checked < self.epoch):
                return
            if state.clock_advanced_at is None or now-state.clock_advanced_at > .5:
                return
            if not valid_stamp:
                return
            self.host.submit(deepcopy(state.positions),source_received_monotonic=state.joint_at,
                simulation_sec=stamp,source_stamp_ns=getattr(state,'joint_state_stamp_ns',None))
            self.stamp_key, self.last_accepted_stamp = self.key, stamp

    def snapshot(self):
        with self.app.lock:
            result = self.host.snapshot() if self.host else dict(status='disabled',binding_id=None,binding=None,manifest_descriptor=None)
            result.update(enabled=self.enabled,mode=self.mode,catalog=deepcopy(self.catalog),
                review_plan=self.review_plan,
                preview_active=self.preview,duration_sec=None if self.plan is None else self.plan.duration,
                clearance_minimum_time_sec=None if self.plan is None else self.plan.clearance_minimum_time_sec,
                preview_time_sec=self.plan_time if self.plan else None)
            if self.error:
                result['error'] = self.error
            return result

    def close(self):
        if self.host:
            self.host.close()
