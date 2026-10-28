/** Demo freshness and stop-cause regressions without a browser, GL or server. */
import test from 'node:test';
import assert from 'node:assert/strict';
import {readFile} from 'node:fs/promises';

const source = (await readFile(new URL('../../src/mujoco_ros2_manipulation/grasping/static/demo.js', import.meta.url), 'utf8'))
  .replace(/^import[^\n]*\n/, '').replace(/\nupdate\(\);\s*$/, '');

function deferred() {
  let resolve, reject;
  const promise = new Promise((yes, no) => {resolve = yes; reject = no;});
  return {promise, resolve, reject};
}
class Element {
  constructor() {this.textContent = ''; this.hidden = false; this.disabled = false; this.dataset = {}; this.children = []; this.attributes = {};}
  addEventListener() {}
  setAttribute(name, value) {this.attributes[name] = value;}
  removeAttribute(name) {delete this.attributes[name];}
  append(...children) {this.children.push(...children);}
  replaceChildren(...children) {this.children = children;}
}
function snapshot(overrides = {}) {
  return {schema_version: 1, token: 'local-demo-token',
    source: {id: 'scene:fr5_grasp_live', label: 'FR5 synthetic grasp', model_sha256: 'a'.repeat(64),
      manifest_sha256: 'b'.repeat(64), engine: 'MuJoCo 3.14.0', method: 'local site-Jacobian DLS'},
    job: {job_id: 'job-one', generation: 1, model_sha256: 'a'.repeat(64), status: 'running', process_running: true,
      source_age_sec: .01, freshness: 'fresh', sequence: 1, simulation_sec: 1, steps: 100,
      cancel_requested: false, timed_out: false, physical_stop_confirmed: null, action_state: 'not_applicable',
      task: {stage: 'approaching', physical_grasp: 'unconfirmed', planning_attachment: 'world', events: []},
      evidence: {}, ...overrides},
    view: {binding_id: 'c'.repeat(64), binding: {generation: 1}, sequence: overrides.sequence ?? 1, pose_ready: true,
      manifest_descriptor: {sha256: 'd'.repeat(64), model_sha256: 'a'.repeat(64), scene_sha256: 'e'.repeat(64)}}};
}
function harness() {
  let now = 0, nextTimer = 0;
  const elements = new Map(), requests = [], timers = new Map();
  const element = id => {
    if (!elements.has(id)) elements.set(id, new Element());
    return elements.get(id);
  };
  const document = {hidden: false, getElementById: element, createElement: () => new Element(), addEventListener() {}};
  class FakeView {
    constructor(_container, options) {
      this.options = options; this.canvas = {}; this.world = {visible: false}; this.label = {dataset: {state: 'waiting'}};
      this.hasPose = false; this.sequence = -1; this.sourceAgeSec = null; this.receivedAt = null;
      this.manualStaleReason = null; this.sceneError = null; this.contextLost = false;
    }
    clearBinding() {this.hasPose = false; this.world.visible = false; this.sequence = -1; this.manualStaleReason = null;}
    setExpectedBinding(id) {this.bindingID = id; this.clearBinding();}
    cancelPoseRead() {}
    async loadManifest() {}
    async pollPose(_url, {accept} = {}) {
      if (this.nextError) {const error = this.nextError; this.nextError = null; throw error;}
      if (!this.nextPose || (accept && !accept({}))) return false;
      const pose = this.nextPose;
      this.nextPose = null;
      if (pose.sequence <= this.sequence) return false;
      this.setPose(pose.sequence, pose.age ?? .01);
      return true;
    }
    setPose(sequence, age = .01) {
      this.sequence = sequence; this.hasPose = true; this.world.visible = true;
      this.sourceAgeSec = age; this.receivedAt = now; this.manualStaleReason = null;
      this.label.dataset.state = 'fresh';
    }
    markStale(reason) {this.manualStaleReason = reason; this.label.dataset.state = 'stale';}
    destroy() {}
  }
  const api = new Function('LocalSceneView', 'document', 'window', 'performance', 'fetch', 'setTimeout', 'clearTimeout',
    source + '\nreturn {acceptSnapshot, renderViewStatus, renderUI, update, viewer};')(
    FakeView, document, {addEventListener() {}}, {now: () => now},
    (url, options) => {const pending = {...deferred(), url, options}; requests.push(pending); return pending.promise;},
    (callback, delay) => {const id = ++nextTimer; timers.set(id, {callback, delay}); return id;}, id => timers.delete(id));
  return {...api, element, requests, document,
    advance(milliseconds) {now += milliseconds;},
    display(value = snapshot(), poseAge = .01) {
      api.acceptSnapshot(value); api.viewer.setPose(value.view.sequence, poseAge); api.renderViewStatus();
    },
    async refresh(value = snapshot()) {
      const pending = api.update();
      requests.at(-1).resolve({ok: true, json: async () => value});
      await pending;
    }};
}

test('live wording requires both source and displayed pose to be fresh', () => {
  for (const kind of ['elapsed', 'server-age', 'server-freshness', 'pose-age', 'unknown-age']) {
    const h = harness();
    h.display(snapshot(kind === 'server-age' ? {source_age_sec: .7} : kind === 'server-freshness'
      ? {freshness: 'stale'} : kind === 'unknown-age' ? {source_age_sec: null, freshness: 'unknown'} : {}), kind === 'pose-age' ? .7 : .01);
    if (kind === 'elapsed') {h.advance(600); h.renderViewStatus();}
    assert.equal(h.element('view-state').textContent, '관측 지연 · 마지막 수신 자세', kind);
    assert.doesNotMatch(h.element('view-state').textContent, /실시간/);
  }
  const fresh = harness(); fresh.display();
  assert.equal(fresh.element('view-state').textContent, '실시간 관측');
});

test('successful GETs for an unchanged observation never reset its age or displayed sequence', async () => {
  const h = harness(); h.display();
  const poseReceived = h.viewer.receivedAt;
  h.advance(400); await h.refresh(snapshot({source_age_sec: .01}));
  assert.equal(h.element('source-age').textContent, '0.41 s');
  h.advance(200); await h.refresh(snapshot({source_age_sec: .01}));
  assert.equal(h.element('source-age').textContent, '0.61 s');
  assert.equal(h.viewer.sequence, 1);
  assert.equal(h.viewer.receivedAt, poseReceived, 'Status connectivity cannot refresh the pose observation');
  assert.equal(h.element('view-state').textContent, '관측 지연 · 마지막 수신 자세');
  assert.equal(h.element('connection-status').textContent, '로컬 상태 연결됨');
});

test('a genuinely newer source observation and verified pose can become live again', async () => {
  const h = harness(); h.display(); h.advance(900); h.renderViewStatus();
  assert.equal(h.element('view-state').dataset.state, 'stale');
  h.viewer.nextPose = {sequence: 2, age: .02};
  await h.refresh(snapshot({sequence: 2, source_age_sec: .02}));
  assert.equal(h.viewer.sequence, 2);
  assert.equal(h.element('source-age').textContent, '0.02 s');
  assert.equal(h.element('view-state').textContent, '실시간 관측');
});

test('a failed pose GET stays stale across successful status GETs and duplicate poses', async () => {
  const h = harness(); h.display();
  h.viewer.nextError = new Error('Pose transport unavailable');
  await h.refresh();
  assert.equal(h.element('view-state').textContent, '관측 지연 · 마지막 수신 자세');
  assert.match(h.element('view-error').textContent, /Pose transport unavailable/);
  await h.refresh();
  assert.equal(h.element('connection-status').textContent, '로컬 상태 연결됨');
  assert.equal(h.viewer.sequence, 1);
  assert.equal(h.element('view-state').textContent, '관측 지연 · 마지막 수신 자세');
  h.viewer.nextPose = {sequence: 2};
  await h.refresh(snapshot({sequence: 2}));
  assert.equal(h.element('view-state').textContent, '실시간 관측');
});

test('a failed status GET cannot leave a last pose labelled live', async () => {
  const h = harness(); h.display();
  const pending = h.update(); h.requests.at(-1).reject(new Error('Status unavailable')); await pending;
  assert.equal(h.element('view-state').textContent, '관측 지연 · 마지막 수신 자세');
  assert.equal(h.viewer.sequence, 1);
});

test('lost graphics or a scene error explicitly require checking the 3D connection', () => {
  for (const kind of ['context', 'scene']) {
    const h = harness(); h.display();
    if (kind === 'context') {h.viewer.contextLost = true; h.viewer.options.onContextLost();}
    else {h.viewer.sceneError = 'Manifest unavailable'; h.renderViewStatus();}
    assert.equal(h.element('view-state').textContent, '3D 표시 연결 확인');
    assert.equal(h.element('scene-placeholder').hidden, false);
    assert.equal(h.viewer.canvas.hidden, true);
    assert.doesNotMatch(h.element('view-state').textContent, /실시간/);
  }
});

test('a watchdog timeout is distinct from user cancellation without inferring a physical stop', () => {
  const timed = harness();
  timed.display(snapshot({status: 'cancelled', process_running: false, cancel_requested: true, timed_out: true}));
  assert.equal(timed.element('job-status').textContent, '제한시간 초과 중단');
  assert.match(timed.element('command-hint').textContent, /제한시간/);
  assert.equal(timed.element('cancel-requested').textContent, '제한시간 초과로 중단 요청');
  assert.equal(timed.element('physical-stop').textContent, '물리 정지 관측 없음');
  const cancelled = harness();
  cancelled.display(snapshot({status: 'cancelled', process_running: false, cancel_requested: true, timed_out: false}));
  assert.equal(cancelled.element('job-status').textContent, '취소됨');
  assert.equal(cancelled.element('cancel-requested').textContent, '요청됨');
  assert.doesNotMatch(cancelled.element('command-hint').textContent, /제한시간/);
});
