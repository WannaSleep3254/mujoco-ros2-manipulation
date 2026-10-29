import {LocalSceneView} from './viewer.js';

const $ = id => document.getElementById(id);
const TIMEOUT_MS = 3000;
const FRESHNESS_SEC = .5;
const ACTIVE_STATUSES = new Set(['starting', 'running', 'cancelling']);
const SHA256 = /^[0-9a-f]{64}$/;
const STATUS_LABELS = {starting: '시작 중', running: '실행 중', cancelling: '취소 처리 중',
  succeeded: '시연 완료', failed: '시연 실패', cancelled: '취소됨', interrupted: '실행 중단'};
const STAGE_LABELS = {approaching: '접근', closing: '그리퍼 닫기', extracting: '들어 올리기',
  holding: '집기 확인', carrying: '이송', releasing: '놓기', settling: '안정화',
  succeeded: '배치 완료', failed: '실패', cancelled: '취소됨'};
const PHASE_LABELS = {initial_settle: '초기 안정화', approach: '접근', close: '그리퍼 닫기',
  extract: '들어 올리기', carry: '이송', lower: '내려놓기', open: '그리퍼 열기',
  retract: '물러나기', final: '최종 관측', cancelled_hold: '취소 후 유지 관측',
  preflight: '실행 대상 확인', unused_domain: 'ROS 환경 확인', launch: '시뮬레이션 시작',
  verification_lift: '들어 올려 집기 확인', transport: '운반', place: '내려놓기',
  settle: '배치 안정화', cleanup: '실행 정리', complete: '작업 종료'};
const PHYSICAL_LABELS = {unconfirmed:'집기 미확인',confirmed:'물리 집기 확인',released:'놓기 관측',slipped:'미끄러짐 관측'};
const ATTACHMENT_LABELS = {world:'MoveIt 작업공간 객체',attached:'MoveIt 부착 확인',uninitialized:'장면 연결 대기',unknown:'부착 상태 확인 불가'};
const EVENT_LABELS = {started: '작업 시작', approach_complete: '접근 완료',
  bilateral_contact_confirmed: '양쪽 손가락 접촉 확인', physical_grasp_confirmed: '물리 집기 확인',
  logical_attachment_recorded: 'MoveIt 부착 기록', carry_started: '이송 시작',
  release_started: '놓기 시작', physical_release_confirmed: '물리 놓기 확인',
  placement_confirmed: '목표 배치 확인', failed: '작업 실패', cancelled: '작업 취소',
  cancelled_hold_observed: '취소 후 유지 상태 관측'};
const EVENT_FIELDS = {left_force_n: '왼쪽 접촉력 N', right_force_n: '오른쪽 접촉력 N',
  observed_sec: '관측 기간 s', lift_m: '들어 올린 높이 m', stable_sec: '안정 관측 s',
  relative_position_error_m: '상대 위치 오차 m', relative_angle_error_rad: '상대 각도 오차 rad',
  bilateral_fraction: '양쪽 접촉 비율', bilateral_contact_absent_sec: '양쪽 접촉 없음 s',
  open_gap_m: '열림 간격 m', table_contact_sec: '테이블 접촉 s', position_error_m: '위치 오차 m',
  speed_m_s: '속도 m/s', stop_observed: '물리 정지 관측', grasp_still_observed: '집기 유지 관측',
  automatic_release: '자동 놓기', automatic_resume: '자동 재개', explicit_recovery_required: '명시적 복구 필요',
  reason: '사유', object_id: '객체', owner: '소유자', link: '링크', simulation_sec: '시뮬레이션 s'};

let latest = null, source = null, token = null, identity = null;
let epoch = 0, bindingID = null, loadedBinding = null, poseDirty = true;
let stopped = false, updating = false, commandBusy = false, pendingAction = null;
let updateTimer = null, pendingImmediate = false, stateRead = null, commandRead = null;
let stateHealthy = false, needsFreshState = true, observationSample = null, eventsKey = null;
let viewer = null, viewerError = null, poseReadError = false;

function text(id, value) {
  const node = $(id), next = String(value ?? '—');
  if (node.textContent !== next) node.textContent = next;
}
function notice(id, message) {
  text(id, message ?? '');
  $(id).hidden = !message;
}
function printable(value) {
  if (value === null || value === undefined) return '—';
  if (typeof value === 'boolean') return value ? '예' : '아니요';
  if (typeof value === 'number') return Number.isFinite(value) ? String(Number(value.toFixed(4))) : '—';
  return typeof value === 'string' ? value : JSON.stringify(value);
}
function number(value, decimals = 3, suffix = '') {
  return Number.isFinite(value) ? value.toFixed(decimals) + suffix : '—';
}
function vector(value) {
  return Array.isArray(value) && value.length === 3 && value.every(Number.isFinite)
    ? value.map(item => item.toFixed(4)).join('  /  ') : '—';
}
function activeJob(job = latest?.job) {
  return Boolean(job && (ACTIVE_STATUSES.has(job.status) || job.process_running === true));
}
function cadence() {return document.hidden ? 1000 : 200;}
function abortReason(message) {return new DOMException(message, 'AbortError');}

function observationAge(now = performance.now()) {
  return Number.isFinite(observationSample?.ageSec)
    ? observationSample.ageSec + Math.max(0, now - observationSample.receivedAt) / 1000 : null;
}

/** A successful status GET is not a new source observation. */
function acceptObservation(next) {
  if (!next.job) {observationSample = null; return;}
  const now = performance.now();
  const key = JSON.stringify([next.source.model_sha256, next.job.job_id,
    next.job.generation, next.view?.binding_id ?? null]);
  const sequence = Number.isSafeInteger(next.job.sequence) ? next.job.sequence : next.view?.sequence;
  const reportedAge = Number.isFinite(next.job.source_age_sec) && next.job.source_age_sec >= 0
    ? next.job.source_age_sec : null;
  const newer = !observationSample || observationSample.key !== key ||
    (Number.isSafeInteger(sequence) && (!Number.isSafeInteger(observationSample.sequence) || sequence > observationSample.sequence));
  if (newer) {
    observationSample = {key, sequence, ageSec: reportedAge, receivedAt: now};
  } else {
    const elapsedAge = observationAge(now);
    // Retain elapsed age even if a duplicate GET repeats an old small age.
    observationSample.ageSec = elapsedAge === null ? reportedAge : reportedAge === null
      ? elapsedAge : Math.max(elapsedAge, reportedAge);
    observationSample.receivedAt = now;
  }
}

/** A single deadline covers response headers and the complete JSON body. */
async function requestJSON(url, {method = 'GET', body, authToken, slot = 'state'} = {}) {
  const controller = new AbortController();
  let rejectAbort;
  const aborted = new Promise((_, reject) => {rejectAbort = reject;});
  const current = {cancel(reason) {
    if (controller.signal.aborted) return;
    controller.abort(reason);
    rejectAbort(reason);
  }};
  if (slot === 'state') stateRead = current;
  else commandRead = current;
  const timer = setTimeout(() => current.cancel(new DOMException(
    method === 'GET' ? '상태 조회 시간이 3초를 초과했습니다.' : '명령 응답 시간이 3초를 초과했습니다. 실행 결과는 상태 조회에서 확인합니다.',
    'TimeoutError')), TIMEOUT_MS);
  try {
    return await Promise.race([(async () => {
      const options = {method, signal: controller.signal, cache: 'no-store', credentials: 'same-origin', redirect: 'error'};
      if (method === 'POST') {
        options.headers = {'Content-Type': 'application/json', 'X-Operator-Token': authToken};
        options.body = JSON.stringify(body);
      }
      const response = await fetch(url, options);
      let result;
      try {result = await response.json();}
      catch (error) {
        if (!response.ok) throw new Error('HTTP ' + response.status + ' · 오류 응답을 읽지 못했습니다.');
        throw error;
      }
      if (!response.ok) throw new Error('HTTP ' + response.status + ' · ' + printable(result?.error ?? '요청이 거부되었습니다.'));
      return result;
    })(), aborted]);
  } finally {
    clearTimeout(timer);
    if (slot === 'state' && stateRead === current) stateRead = null;
    if (slot === 'command' && commandRead === current) commandRead = null;
  }
}

function validateSnapshot(value) {
  if (!value || value.schema_version !== 1 || typeof value.token !== 'string' || !value.token ||
    value.source?.id !== 'scene:ros_fr5_grasp' || !SHA256.test(value.source?.model_sha256) ||
    !SHA256.test(value.source?.manifest_sha256) || !Object.hasOwn(value, 'job') || !Object.hasOwn(value, 'view')) {
    throw new Error('시연 상태 응답의 대상·모델 식별을 확인할 수 없습니다.');
  }
  if (value.job && (typeof value.job.job_id !== 'string' || !value.job.job_id ||
    !['string', 'number'].includes(typeof value.job.generation) || value.job.model_sha256 !== value.source.model_sha256)) {
    throw new Error('실행 작업의 모델·세대 식별이 일치하지 않습니다.');
  }
  if (value.view?.pose_ready && ((value.view.mode !== 'model_preview' && !value.job) || typeof value.view.binding_id !== 'string' || !value.view.binding_id ||
    !SHA256.test(value.view.manifest_descriptor?.sha256) ||
    value.view.manifest_descriptor?.model_sha256 !== value.source.model_sha256 ||
    !SHA256.test(value.view.manifest_descriptor?.scene_sha256))) {
    throw new Error('3D 관측의 바인딩·모델 식별을 확인할 수 없습니다.');
  }
  return value;
}

function clearView(reason) {
  bindingID = loadedBinding = null;
  poseDirty = true;
  poseReadError = false;
  viewer?.clearBinding(reason);
  if (viewer) viewer.canvas.hidden = true;
}

/** Token, job generation and scene binding form one display epoch. */
function acceptSnapshot(value) {
  const next = validateSnapshot(value);
  const nextIdentity = JSON.stringify([next.token, next.source.model_sha256,
    next.job?.job_id ?? null, next.job?.generation ?? null, next.view?.binding_id ?? null]);
  if (nextIdentity !== identity) {
    epoch++;
    identity = nextIdentity;
    clearView(next.job ? '시연의 첫 관측 대기' : '시연 실행 전 · 모델 미실행');
  }
  latest = next;
  source = next.source;
  token = next.token;
  stateHealthy = true;
  needsFreshState = false;
  acceptObservation(next);
  const view = next.view;
  if (!view?.pose_ready) {
    if (bindingID !== null || viewer?.hasPose) clearView(next.job ? '시연의 첫 관측 대기' : '시연 실행 전 · 모델 미실행');
  } else if (bindingID !== view.binding_id) {
    bindingID = view.binding_id;
    loadedBinding = null;
    poseDirty = true;
    viewer?.setExpectedBinding(bindingID, view.binding);
    if (viewer) viewer.canvas.hidden = true;
  }
  notice('connection-error', null);
  renderUI();
  return epoch;
}

function controlsAvailable() {
  const available = !stopped && !commandBusy && stateHealthy && !needsFreshState && Boolean(token && source);
  $('demo-start').disabled = !available || activeJob() || !latest;
  $('demo-cancel').disabled = !available || !activeJob() || latest?.job?.cancel_requested === true || latest?.job?.status === 'cancelling';
}

function renderSource() {
  if (!source) return;
  text('source-label', source.label);
  text('source-engine', source.engine);
  text('source-method', source.method);
  text('model-sha', source.model_sha256.slice(0, 12));
}

function renderStages(job) {
  const task = job?.task;
  const observedStages = new Set((task?.events ?? []).map(event => event.stage));
  for (const item of $('stage-list').children) {
    const current = item.dataset.stage === task?.stage;
    item.dataset.state = current ? 'current' : observedStages.has(item.dataset.stage) ? 'observed' : 'waiting';
    if (current) item.setAttribute('aria-current', 'step');
    else item.removeAttribute('aria-current');
  }
  text('job-phase', job?.phase ? (PHASE_LABELS[job.phase] ?? job.phase) : '—');
  text('task-stage', task?.stage ? '작업 단계: ' + (STAGE_LABELS[task.stage] ?? task.stage) : '단계 관측 대기');
  const physical = task?.physical_grasp;
  text('physical-grasp', physical ? (PHYSICAL_LABELS[physical] ?? physical) : '관측 없음');
  $('physical-grasp').dataset.state = physical ?? 'waiting';
  const attachment = task?.planning_attachment;
  text('planning-attachment', attachment ? (ATTACHMENT_LABELS[attachment] ?? attachment) : '관측 없음');
  $('planning-attachment').dataset.state = attachment ?? 'waiting';
}

function renderEvidence(job) {
  const evidence = job?.evidence;
  text('left-force', number(evidence?.finger_force_n?.[0]));
  text('right-force', number(evidence?.finger_force_n?.[1]));
  text('relative-position', number(evidence?.relative_position_error_m, 5, ' m'));
  text('relative-angle', number(evidence?.relative_angle_error_rad, 4, ' rad'));
  text('tcp-position', vector(evidence?.tcp_position_m));
  text('block-position', vector(evidence?.block_position_m));
}

function renderEvents(job) {
  const events = Array.isArray(job?.task?.events) ? job.task.events.slice(-32) : [];
  const key = JSON.stringify([job?.job_id, job?.generation, events]);
  if (key === eventsKey) return;
  eventsKey = key;
  text('event-count', events.length + '건');
  const list = $('event-list');
  list.replaceChildren();
  if (!events.length) {
    const empty = document.createElement('li');
    empty.className = 'empty-events';
    empty.textContent = job ? '아직 작업자가 보고한 이벤트가 없습니다.' : '실행 프로세스가 보고한 이벤트가 여기에 쌓입니다.';
    list.append(empty);
    return;
  }
  events.forEach((event, index) => {
    const item = document.createElement('li');
    const count = document.createElement('span');
    count.className = 'event-number';
    count.textContent = String(index + 1).padStart(2, '0');
    const body = document.createElement('div');
    const heading = document.createElement('div');
    heading.className = 'event-header';
    const title = document.createElement('strong');
    title.textContent = EVENT_LABELS[event.event] ?? printable(event.event);
    heading.append(title);
    const meta = document.createElement('p');
    meta.className = 'event-meta';
    const values = [];
    if (event.stage) values.push('단계 ' + (STAGE_LABELS[event.stage] ?? event.stage));
    if (event.physical_grasp) values.push(PHYSICAL_LABELS[event.physical_grasp] ?? event.physical_grasp);
    if (event.planning_attachment) values.push(ATTACHMENT_LABELS[event.planning_attachment] ?? event.planning_attachment);
    const additional = Object.entries(event).filter(([name]) => Object.hasOwn(EVENT_FIELDS, name))
      .map(([name, value]) => (EVENT_FIELDS[name] ?? name) + ': ' + printable(value));
    meta.textContent = values.join(' · ') + (additional.length ? '\n' + additional.join(' · ') : '');
    body.append(heading, meta);
    item.append(count, body);
    list.append(item);
  });
}

function renderExecution(job) {
  text('process-state', !job ? (pendingAction === 'start' ? '실행 응답 대기' : '실행 전') :
    job.process_running === true ? '실행 중' : job.process_running === false ? '종료 / 실행 중 아님' : '관측 미확인');
  text('cancel-requested', job?.timed_out === true ? '제한시간 초과로 중단 요청' : job ? job.cancel_requested === true ? '요청됨' : job.cancel_requested === false ? '요청 없음' : '미확인' : '—');
  text('cancel-delivery', ({not_requested:'요청 없음',signal_sent:'취소 신호 전달',worker_observed:'작업자 수신 확인',process_exited:'프로세스 종료'}[job?.cancel_delivery] ?? printable(job?.cancel_delivery)));
  const guard = job?.native_guard;
  const stopped = guard?.latched === true && guard?.phase === 'stopped' &&
    ['left_controller', 'right_controller', 'gripper_controller'].every(name => guard.controllers?.[name] === 'inactive');
  text('physical-stop', !guard ? '정지 관측 대기' : stopped ? '제어기 3개 정지 · ' + ({grasp_cancel:'종료 요청',grasp_heartbeat:'작업 신호 끊김',contact:'접촉'}[guard.reason] ?? printable(guard.reason)) : guard.latched ? '제어기 정지 처리 중' : guard.phase === 'ready' ? '정지 감시 중' : '제어기 준비 중');
  $('physical-stop').dataset.state = stopped ? 'confirmed' : 'waiting';
  const action = job?.action_terminal;
  text('action-state', !action ? '종료 관측 대기' : (PHASE_LABELS[action.stage] ?? printable(action.stage)) + ' · ' + ({4:'성공',5:'취소',6:'중단'}[action.status] ?? '종료 확인 중') + (Number.isFinite(action.error_code) && action.error_code !== 0 ? ' · 코드 ' + printable(action.error_code) : ''));
  const cleanup = job?.cleanup;
  const clean = cleanup?.clean === true && job?.owner_cleanup_clean === true;
  text('process-cleanup', !job ? '실행 전' : clean ? 'Native 종료·소유 프로세스 정리 확인' : job.finished ? '정리 결과 확인 필요' : cleanup?.status === 'not_started' ? '작업 종료 후 확인' : '정리 중');
  $('process-cleanup').dataset.state = clean ? 'confirmed' : 'waiting';
  text('job-id', printable(job?.job_id));
  text('job-generation', printable(job?.generation));
  text('worker-result', job ? job.worker_result_observed === true ? '수신함' : job.worker_result_observed === false ? '수신하지 않음' : '미확인' : '—');
  text('process-exit', printable(job?.process_exit_code));
  const expectedCancel = job?.error?.type === 'RosGraspCancelled';
  notice('job-error', expectedCancel
    ? job.status === 'failed' || job.finished && job.owner_cleanup_clean !== true
      ? '취소 후 정리를 확인하지 못했습니다. 정지 상태와 실행 기록을 확인하세요.' : null
    : job?.error ? '실행 오류 · ' + printable(job.error) : null);
}

function renderViewStatus() {
  const job = latest?.job;
  const unavailable = Boolean(viewerError || (job && !viewer) || viewer?.contextLost || viewer?.sceneError);
  const showingPose = Boolean(viewer?.hasPose && viewer?.world.visible && !unavailable && bindingID && latest?.view?.pose_ready);
  const age = observationAge();
  const poseAge = Number.isFinite(viewer?.sourceAgeSec) && Number.isFinite(viewer?.receivedAt)
    ? viewer.sourceAgeSec + Math.max(0, performance.now() - viewer.receivedAt) / 1000 : null;
  const behind = Number.isSafeInteger(job?.sequence) && Number.isSafeInteger(viewer?.sequence) && viewer.sequence < job.sequence;
  const stale = Boolean(job && (!stateHealthy || poseReadError || viewer?.manualStaleReason ||
    job.freshness !== 'fresh' || age === null || age > FRESHNESS_SEC ||
    poseAge === null || poseAge > FRESHNESS_SEC || behind || viewer?.label?.dataset?.state === 'stale'));
  if (viewer) viewer.canvas.hidden = !showingPose;
  $('scene-placeholder').hidden = showingPose;
  if (!showingPose) {
    text('placeholder-title', unavailable ? '3D 표시 연결 확인' : pendingAction === 'start' ? '시연 시작 응답 대기' : job ? activeJob(job) ? '첫 자세 관측 대기' : '표시할 자세 관측 없음' : '시연 실행 전');
    text('placeholder-copy', unavailable ? '아래 오류를 확인해 주세요. 작업 단계와 증거는 상태 조회로 표시합니다.' :
      job ? '실제 시뮬레이션의 자세가 도착하면 3D 화면을 표시합니다.' : '실행한 시뮬레이션의 첫 자세가 도착하면\nFR5와 그리퍼, 블록을 여기에 표시합니다.');
  }
  text('pose-sequence', '자세 # ' + (showingPose ? viewer.sequence : '—'));
  const preview = latest?.view?.mode === 'model_preview';
  const viewState = unavailable ? 'unavailable' : document.hidden ? 'hidden' : preview && showingPose ? 'preview' : !showingPose ? 'waiting' : stale ? 'stale' : activeJob(job) ? 'fresh' : 'ended';
  text('view-state', {unavailable: '3D 표시 연결 확인', hidden: '탭 숨김 · 자세 조회 중지',
    preview: job ? '실행 전 모델 · 시뮬레이션 관측 대기' : '실행 전 모델', waiting: '관측 대기', stale: '관측 지연 · 마지막 수신 자세', fresh: '실시간 관측', ended: '마지막 관측'}[viewState]);
  $('view-state').dataset.state = viewState;
  text('observation-state', latest?.view?.mode === 'model_preview' ? '실행 전 모델 · 실시간 관측 없음' : !job ? '아직 수신한 관측 없음' : age === null || job.freshness === 'unknown'
    ? '관측 시각 미확인' : stale ? '관측 지연 · 마지막 수신 상태' : activeJob(job) ? '실행 중인 시뮬레이션' : '종료된 시연의 마지막 관측');
  text('source-age', number(age, 2, ' s'));
  text('simulation-time', number(job?.simulation_sec, 3, ' s'));
  text('simulation-steps', 'ROS 궤적 제어');
}

function renderUI() {
  renderSource();
  const job = latest?.job;
  const timedOut = job?.timed_out === true;
  const status = pendingAction === 'start' ? 'starting' : timedOut ? 'interrupted' : pendingAction === 'cancel' ? 'cancelling' : job?.status ?? 'waiting';
  text('job-status', pendingAction === 'start' ? '실행 요청 중' : timedOut ? '제한시간 초과 중단' : pendingAction === 'cancel' ? '취소 요청 중' : STATUS_LABELS[status] ?? (job ? status : '실행 전'));
  $('job-status').dataset.state = status;
  let hint = '실행 전에는 모델을 표시합니다. 실행하면 시뮬레이션 관측과 실제 작업 상태를 확인합니다.';
  if (pendingAction === 'start') hint = '서버의 실행 응답을 기다리고 있습니다.';
  else if (timedOut) hint = '실행 제한시간을 초과했습니다. 프로세스 종료와 물리 정지 관측을 확인합니다.';
  else if (pendingAction === 'cancel') hint = '취소를 요청했습니다. 전달 결과와 물리 정지 관측을 확인합니다.';
  else if (job?.cancel_requested || job?.status === 'cancelling') hint = '취소 전달·프로세스 종료·물리 정지 관측을 각각 확인합니다.';
  else if (activeJob(job)) hint = 'ROS 작업을 실행 중입니다. 오른쪽에서 실제 단계와 집기 증거를 확인하세요.';
  else if (job) hint = '시연이 종료되었습니다. 마지막 관측과 이벤트를 표시합니다.';
  text('command-hint', hint);
  const connectionState = !stateHealthy ? 'error' : document.hidden ? 'hidden' : 'ready';
  $('connection-dot').dataset.state = connectionState;
  text('connection-status', !stateHealthy ? source ? '상태 연결 확인 중' : '상태 연결 중' : document.hidden ? '탭 숨김 · 상태 1초 조회' : '로컬 상태 연결됨');
  renderStages(job);
  renderEvidence(job);
  renderEvents(job);
  renderExecution(job);
  renderViewStatus();
  controlsAvailable();
}

function scheduleUpdate(delay = cadence()) {
  if (stopped) return;
  if (updateTimer !== null) {clearTimeout(updateTimer); updateTimer = null;}
  if (updating || commandBusy) {
    if (delay === 0) pendingImmediate = true;
    return;
  }
  updateTimer = setTimeout(() => {updateTimer = null; update();}, delay);
}

async function command(action) {
  if (stopped || commandBusy || !stateHealthy || needsFreshState || !token || !source) return;
  if (action === 'start' ? activeJob() || !latest : !activeJob() || latest?.job?.cancel_requested || latest?.job?.status === 'cancelling') return;
  const authToken = token;
  let body;
  try {
    body = action === 'start'
      ? {request_id: crypto.randomUUID().replaceAll('-', ''), model_sha256: source.model_sha256}
      : {job_id: latest.job.job_id, generation: latest.job.generation, model_sha256: latest.job.model_sha256};
  } catch (error) {notice('command-error', error.message); return;}
  commandBusy = true;
  pendingAction = action;
  needsFreshState = true;
  const commandEpoch = ++epoch;
  token = null;
  stateRead?.cancel(abortReason('새 명령으로 이전 상태 조회를 종료했습니다.'));
  viewer?.cancelPoseRead();
  viewer?.loadController?.abort();
  poseDirty = true;
  if (updateTimer !== null) {clearTimeout(updateTimer); updateTimer = null;}
  if (action === 'start') {
    identity = null;
    latest = null;
    observationSample = null;
    clearView('새 시연 실행 중 · 첫 관측 대기');
  }
  notice('command-error', null);
  renderUI();
  try {
    const result = await requestJSON('/api/ros-demo/' + action, {method: 'POST', body, authToken, slot: 'command'});
    if (stopped || epoch !== commandEpoch) return;
    acceptSnapshot(result);
  } catch (error) {
    if (!stopped && epoch === commandEpoch && error.name !== 'AbortError') {
      notice('command-error', error.message + ' · 명령은 자동 재전송하지 않습니다. 아래 상태 조회로 결과를 확인합니다.');
    }
  } finally {
    commandBusy = false;
    pendingAction = null;
    if (!stopped) {renderUI(); scheduleUpdate(document.hidden ? 1000 : 0);}
  }
}

async function update() {
  if (stopped) return;
  if (updating || commandBusy) {pendingImmediate = true; return;}
  if (updateTimer !== null) {clearTimeout(updateTimer); updateTimer = null;}
  updating = true;
  pendingImmediate = false;
  const readEpoch = epoch;
  let currentEpoch = readEpoch;
  let readingState = true;
  try {
    const result = await requestJSON('/api/ros-demo');
    if (stopped || commandBusy || readEpoch !== epoch) return;
    currentEpoch = acceptSnapshot(result);
    readingState = false;
    const view = latest.view;
    if (document.hidden || !viewer || viewerError || !view?.pose_ready || bindingID === null) return;
    const id = bindingID;
    if (loadedBinding !== id) {
      const descriptor = view.manifest_descriptor;
      await viewer.loadManifest('/api/view/manifest?binding_id=' + encodeURIComponent(id),
        {binding_id: id, binding: view.binding, sha256: descriptor.sha256,
          model_sha256: descriptor.model_sha256, scene_sha256: descriptor.scene_sha256});
      if (stopped || commandBusy || currentEpoch !== epoch || bindingID !== id) return;
      loadedBinding = id;
      poseDirty = true;
    }
    if (stopped || commandBusy || currentEpoch !== epoch || document.hidden || bindingID !== id) return;
    const newer = Number.isSafeInteger(view.sequence) && view.sequence > viewer.sequence;
    if (poseDirty || activeJob() || newer || !viewer.hasPose) {
      const previousSequence = viewer.sequence;
      const applied = await viewer.pollPose('/api/view/pose?binding_id=' + encodeURIComponent(id),
        {accept: () => !stopped && !document.hidden && !commandBusy && currentEpoch === epoch && bindingID === id});
      if (stopped || commandBusy || currentEpoch !== epoch || bindingID !== id) return;
      if (applied && viewer.hasPose && viewer.sequence > previousSequence) poseReadError = false;
      poseDirty = !(viewer.hasPose && (!Number.isSafeInteger(view.sequence) || viewer.sequence >= view.sequence));
    }
    if (latest.job && view.mode !== 'model_preview' && !activeJob()) viewer.markStale('시연 종료 · 마지막 수신 자세');
    if (!viewerError) notice('view-error', null);
    renderViewStatus();
  } catch (error) {
    if (!stopped && !commandBusy && currentEpoch === epoch && error.name !== 'AbortError') {
      if (!readingState) {
        poseDirty = true;
        poseReadError = true;
        viewer?.markStale('관측 연결 중단 · 마지막 수신 자세');
        notice('view-error', '3D 관측 갱신을 확인할 수 없습니다. ' + error.message);
      } else {
        stateHealthy = false;
        notice('connection-error', '상태 갱신을 확인할 수 없습니다. ' + error.message);
      }
      renderUI();
    }
  } finally {
    updating = false;
    if (!stopped && !commandBusy) {
      const delay = pendingImmediate && !document.hidden ? 0 : cadence();
      pendingImmediate = false;
      scheduleUpdate(delay);
    }
  }
}

try {
  viewer = new LocalSceneView($('demo-scene'), {background: 0x14232d, requestTimeoutMs: TIMEOUT_MS,
    onMetrics: () => {if (!stopped) renderViewStatus();},
    onContextLost: () => {
      viewerError = '브라우저의 3D 표시 연결을 잃었습니다. 페이지를 다시 열어 확인하세요.';
      notice('view-error', viewerError);
      renderViewStatus();
    }});
  clearView('시연 실행 전 · 모델 미실행');
} catch (error) {
  viewerError = error.message;
  notice('view-error', '3D 표시 오류 · ' + viewerError);
}
$('camera-zoom-in').addEventListener('click', () => viewer?.zoomCamera?.('in'));
$('camera-zoom-out').addEventListener('click', () => viewer?.zoomCamera?.('out'));
$('camera-fit').addEventListener('click', () => viewer?.fitCamera?.());
$('camera-reset').addEventListener('click', () => viewer?.resetCamera?.());
$('camera-sensitivity').addEventListener('input', event => {
  const value = Number(event.target.value);
  viewer?.setControlSensitivity?.({rotate: value, zoom: value, pan: value});
  text('camera-sensitivity-value', value.toFixed(2) + '×');
});
$('demo-start').addEventListener('click', () => command('start'));
$('demo-cancel').addEventListener('click', () => command('cancel'));
document.addEventListener('visibilitychange', () => {
  if (stopped) return;
  if (!commandBusy) epoch++;
  stateRead?.cancel(abortReason('탭 표시 상태가 바뀌어 최신 상태를 다시 조회합니다.'));
  viewer?.cancelPoseRead();
  viewer?.loadController?.abort();
  poseDirty = true;
  pendingImmediate = !document.hidden;
  renderUI();
  scheduleUpdate(document.hidden ? 1000 : 0);
});
window.addEventListener('pagehide', () => {
  stopped = true;
  epoch++;
  pendingImmediate = false;
  if (updateTimer !== null) {clearTimeout(updateTimer); updateTimer = null;}
  stateRead?.cancel(abortReason('시연 화면을 종료했습니다.'));
  commandRead?.cancel(abortReason('명령 응답 관측을 종료했습니다.'));
  viewer?.cancelPoseRead();
  viewer?.destroy();
});
renderUI();
update();
