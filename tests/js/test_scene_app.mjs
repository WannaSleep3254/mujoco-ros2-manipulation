/** Real SceneControls methods with deferred requests; no DOM, Three or WebGL. */
import test from 'node:test';
import assert from 'node:assert/strict';
import {readFile} from 'node:fs/promises';

const source = (await readFile(new URL('../../src/mujoco_ros2_manipulation/operator_console/static/scene_app.js', import.meta.url), 'utf8'))
  .replace(/^import[^\n]*\n/gm, '')
  .replace('export class SceneControls', 'class SceneControls');

function deferred() {
  let resolve, reject;
  const promise = new Promise((yes, no) => {resolve = yes; reject = no;});
  return {promise, resolve, reject};
}

async function drain() {for (let index = 0; index < 12; index++) await Promise.resolve();}

class Element {
  constructor(tag = 'div') {this.tagName = tag.toUpperCase(); this._value = ''; this.hidden = false; this.disabled = false; this.children = []; this.listeners = {};}
  get value() {return this._value;}
  set value(value) {this._value = this.tagName === 'SELECT' && !this.children.some(child => child.value === String(value)) ? '' : String(value);}
  addEventListener(name, callback) {this.listeners[name] = callback;}
  append(...children) {for (const child of children) {child.parentElement = this; this.children.push(child); if (this.tagName === 'SELECT' && !this._value && !child.disabled) this._value = child.value;}}
  replaceChildren() {this.children = []; if (this.tagName === 'SELECT') this._value = '';}
}

function state(character = 'a', generation = 1, overrides = {}) {
  const binding_id = character.repeat(64);
  const binding = {operator_session_id: 'd'.repeat(32), generation, mode: 'plan_preview',
    source_id: 'workcell:manual', job_id: null, run_id: null, motion_session_id: 'e'.repeat(32), request_id: 'f'.repeat(32)};
  return {active_request_id: null, lifecycle: {phase: 'idle'}, demo: {current: null},
    view: {enabled: true, mode: 'plan_preview', preview_active: true, binding_id, binding,
      source: {id: 'workcell:manual'}, config_source_id: 'workcell:manual',
      catalog: [{id: 'workcell:manual', label: 'Manual', available: true}], display_mode: 'web3d',
      duration_sec: 10, preview_time_sec: 0, status: 'ready',
      manifest_descriptor: {sha256: '1'.repeat(64), model_sha256: '2'.repeat(64), scene_sha256: '3'.repeat(64)},
      ...overrides}};
}

function harness({manifest} = {}) {
  let now = 0, nextTimer = 0, active = 0, maximumActive = 0;
  const elements = new Map(), timers = new Map(), requests = [], notices = [];
  const element = id => {
    if (!elements.has(id)) elements.set(id, new Element(id === 'scene-source' ? 'select' : 'div'));
    return elements.get(id);
  };
  element('demos').hidden = true;
  const document = {getElementById: element, createElement: tag => new Element(tag), activeElement: null};
  class FakeViewer {
    constructor(container, options) {this.onMetrics = options.onMetrics; this.loads = []; this.poses = []; this.bindings = []; this.clears = 0; this.aborts = 0; this.destroyed = 0;}
    setExpectedBinding(id, binding) {this.bindings.push({id, binding});}
    clearBinding() {this.clears++; this.aborts += this.loads.filter(load => !load.finished).length;}
    loadManifest(url, options) {
      const item = {url, options, finished: false}; this.loads.push(item);
      const promise = manifest ? manifest(item) : Promise.resolve({});
      return promise.finally(() => {item.finished = true;});
    }
    pollPose(url) {this.poses.push(url); return Promise.resolve(false);}
    markStale(reason) {this.staleReason = reason;}
    setCollisionVisible() {}
    resize() {}
    destroy() {this.destroyed++;}
  }
  const SceneControls = new Function('LocalSceneView', 'document', 'performance', 'setTimeout', 'clearTimeout',
    source + '\nreturn SceneControls;')(FakeViewer, document, {now: () => now},
      (callback, delay) => {const id = ++nextTimer; timers.set(id, {callback, delay}); return id;},
      id => timers.delete(id));
  const controls = new SceneControls({request: (path, payload) => {
    const request = {...deferred(), path, payload}; requests.push(request);
    active++; maximumActive = Math.max(maximumActive, active);
    return request.promise.finally(() => active--);
  }, notify: message => notices.push(message), showOperate: () => {controls.shown = (controls.shown || 0)+1;}});
  return {controls, element, requests, notices, timers, advance: milliseconds => {now += milliseconds;},
    get maximumActive() {return maximumActive;},
    reply(index, value = {}) {requests[index].resolve(value);},
    fail(index, message = 'Synthetic request failure') {requests[index].reject(new Error(message));}};
}

test('an old seek response cannot change a new preview slider', async () => {
  const h = harness(); h.controls.update(state()); await drain();
  const result = h.controls.seek(4);
  assert.equal(h.requests[0].payload.binding_id, 'a'.repeat(64));
  h.controls.update(state('b', 2, {preview_time_sec: 1}));
  h.reply(0, {preview_time_sec: 4});
  assert.equal(await result, false);
  assert.equal(h.element('plan-seek').value, '1');
  assert.equal(h.controls.data.binding_id, 'b'.repeat(64));
});

test('pause and restart while an old seek awaits leaves one owned playback chain', async () => {
  const h = harness(); h.controls.update(state()); await drain();
  h.controls.play(); assert.equal(h.requests.length, 1);
  h.controls.pause(); h.advance(1000); h.element('plan-seek').value = '3'; h.controls.play();
  assert.equal(h.requests.length, 1, 'Restart queues behind the existing seek');
  h.reply(0); await drain();
  assert.equal(h.controls.playing, true, 'Old frame must not pause the new playback');
  assert.equal(h.requests.length, 2);
  assert.equal(h.requests[1].payload.time_sec, 3);
  assert.equal(h.element('plan-seek').value, '3', 'Old response must not rewind the new start');
  assert.equal(h.timers.size, 0, 'Old frame must not schedule an extra timer');
  h.reply(1); await drain();
  assert.equal(h.timers.size, 1);
  h.controls.pause(); assert.equal(h.timers.size, 0);
});

test('changing directly between plan bindings pauses playback and discards pending seeks', async () => {
  const h = harness(); h.controls.update(state()); await drain();
  h.controls.play();
  const pending = h.controls.seek(6);
  h.controls.update(state('b', 2, {preview_time_sec: .75, duration_sec: 20}));
  assert.equal(await pending, false);
  assert.equal(h.controls.playing, false);
  assert.equal(h.element('plan-seek').value, '.75'.replace(/^\./,'0.'));
  h.reply(0); await drain();
  assert.equal(h.requests.length, 1);
  assert.equal(h.timers.size, 0);
  assert.equal(h.element('plan-seek').value, '0.75');
});

test('rapid scrub A B C sends only A then C and finishes at the last server time', async () => {
  const h = harness(); h.controls.update(state()); await drain();
  const first = h.controls.seek(1), second = h.controls.seek(2), third = h.controls.seek(3);
  assert.equal(await second, false, 'Superseded pending input resolves without submission');
  assert.equal(h.requests.length, 1);
  h.reply(0, {preview_time_sec: 1}); await first; await drain();
  assert.equal(h.requests.length, 2);
  assert.equal(h.requests[1].payload.time_sec, 3);
  assert.equal(h.element('plan-seek').value, '0', 'Earlier response is not the latest UI value');
  h.reply(1, {preview_time_sec: 3}); assert.equal(await third, true);
  assert.equal(h.element('plan-seek').value, '3');
  assert.equal(h.maximumActive, 1);
});

test('a current playback seek failure stops once instead of posting forever', async () => {
  const h = harness(); h.controls.update(state()); await drain();
  h.controls.play(); h.fail(0); await drain();
  assert.equal(h.controls.playing, false);
  assert.equal(h.timers.size, 0);
  assert.equal(h.notices.length, 1);
  for (let index = 0; index < 10; index++) h.controls.update(state());
  assert.equal(h.requests.length, 1);
});

test('manifest failures obey backoff and a finite attempt limit, notifying once', async () => {
  const h = harness({manifest: () => Promise.reject(new Error('Synthetic manifest error'))});
  for (let attempt = 0; attempt < 5; attempt++) {
    h.controls.update(state()); await drain();
    assert.equal(h.controls.viewer.loads.length, attempt+1);
    for (let poll = 0; poll < 20; poll++) h.controls.update(state());
    assert.equal(h.controls.viewer.loads.length, attempt+1);
    h.advance(1000 * 2**attempt);
  }
  h.advance(1000000);
  for (let poll = 0; poll < 20; poll++) h.controls.update(state());
  assert.equal(h.controls.viewer.loads.length, 5);
  assert.equal(h.notices.length, 1);
  h.controls.update(state('b', 2)); await drain();
  assert.equal(h.controls.viewer.loads.length, 6, 'New binding can attempt its own manifest');
  assert.equal(h.notices.length, 2);
});

test('null binding clears and aborts a pending manifest and rejects its late success', async () => {
  const load = deferred(), h = harness({manifest: () => load.promise});
  h.controls.update(state());
  assert.equal(h.controls.viewer.loads.length, 1);
  h.controls.update(state('a', 2, {binding_id: null, binding: null, mode: 'live', preview_active: false,
    duration_sec: null, preview_time_sec: null, manifest_descriptor: null}));
  assert.equal(h.controls.viewer.clears, 1);
  assert.equal(h.controls.viewer.aborts, 1);
  assert.equal(h.controls.expected, null);
  load.resolve({}); await drain();
  assert.equal(h.controls.loadedBinding, null);
  assert.equal(h.controls.loading, null);
  assert.equal(h.controls.viewer.poses.length, 0);
});

test('a new binding reloads its manifest even when the previous view had one', async () => {
  const h = harness(); h.controls.update(state()); await drain();
  assert.equal(h.controls.loadedBinding, 'a'.repeat(64));
  h.controls.viewer.manifest = {previous: true};
  h.controls.update(state('b', 2)); await drain();
  assert.equal(h.controls.viewer.loads.length, 2);
  assert.equal(h.controls.loadedBinding, 'b'.repeat(64));
});

test('an older same-session state response cannot restore the previous binding', async () => {
  const h = harness(); h.controls.update(state('b', 2, {preview_time_sec: 2})); await drain();
  const epoch = h.controls.playEpoch;
  h.controls.update(state('a', 1, {preview_time_sec: 1}));
  assert.equal(h.controls.expected, 'b'.repeat(64));
  assert.equal(h.element('plan-seek').value, '2');
  assert.equal(h.controls.playEpoch, epoch);
  assert.equal(h.controls.viewer.loads.length, 1);
});

test('selection is serialized and destroy discards pending playback work', async () => {
  const h = harness(); h.controls.update(state()); await drain();
  const selection = h.controls.select('model_preview', 'profile:fr5');
  await h.controls.previewRequest('f'.repeat(32));
  assert.equal(h.requests.length, 1);
  assert.equal(h.requests[0].path, '/api/view/select');
  h.reply(0); await selection;
  h.controls.play(); const pending = h.controls.seek(7);
  h.controls.destroy(); assert.equal(await pending, false);
  h.reply(1); await drain();
  assert.equal(h.controls.viewer.destroyed, 1);
  assert.equal(h.requests.length, 2);
  assert.equal(h.timers.size, 0);
});

for (const errorState of [{status: 'error'}, {error: {code: 'scene_unavailable'}}]) {
  test(`a same-binding backend ${errorState.status ? 'error status' : 'error payload'} stops playback and discards late seeks`, async () => {
    const h = harness(); h.controls.update(state()); await drain();
    h.controls.play(); const pending = h.controls.seek(6);
    h.controls.update(state('a', 1, errorState));
    assert.equal(await pending, false);
    assert.equal(h.controls.playing, false);
    assert.equal(h.element('plan-play').disabled, true);
    assert.equal(h.element('plan-seek').disabled, true);
    assert.equal(h.controls.expected, null);
    assert.equal(h.controls.viewer.clears, 1);
    for (let poll = 0; poll < 10; poll++) h.controls.update(state('a', 1, errorState));
    assert.equal(h.controls.viewer.clears, 1, 'Repeated error polls do not restart clearing/aborting');
    h.controls.play(); assert.equal(await h.controls.seek(7), false);
    assert.equal(h.requests.length, 1);
    // Even recovery to the same binding cannot revive a pre-error request.
    h.controls.update(state('a', 1, {preview_time_sec: 2})); await drain();
    h.reply(0); await drain();
    assert.equal(h.requests.length, 1);
    assert.equal(h.timers.size, 0);
    assert.equal(h.element('plan-seek').value, '2');
    assert.equal(h.element('plan-play').disabled, false);
  });
}

test('an absent or unready plan binding cannot submit playback or a null-binding seek', async () => {
  const h = harness();
  for (const overrides of [
    {binding_id: null, binding: null, manifest_descriptor: null},
    {status: 'preparing', manifest_descriptor: null},
    {status: 'error', duration_sec: null, error: {code: 'unavailable'}},
  ]) {
    assert.doesNotThrow(() => h.controls.update(state('a', 1, overrides)));
    h.controls.play(); assert.equal(await h.controls.seek(2), false);
    assert.equal(h.element('plan-play').disabled, true);
    assert.equal(h.element('plan-seek').disabled, true);
  }
  assert.equal(h.requests.length, 0);
  assert.equal(h.timers.size, 0);
});

for (const lateOutcome of ['success', 'failure']) {
  test(`a pre-error manifest ${lateOutcome} cannot overwrite a same-binding recovery load`, async () => {
    const first = deferred(), second = deferred(); let count = 0;
    const h = harness({manifest: () => (++count === 1 ? first : second).promise});
    h.controls.update(state());
    h.controls.update(state('a', 1, {status: 'error'}));
    h.controls.update(state());
    const current = h.controls.loading;
    assert.equal(h.controls.viewer.loads.length, 2);
    if (lateOutcome === 'success') first.resolve({});
    else first.reject(new Error('Superseded manifest failure'));
    await drain();
    assert.equal(h.controls.loading, current, 'Late completion cannot release the new load guard');
    assert.equal(h.controls.loadedBinding, null);
    assert.equal(h.controls.manifestFailures, 0);
    assert.equal(h.notices.length, 0);
    h.controls.update(state());
    assert.equal(h.controls.viewer.loads.length, 2, 'No duplicate load starts during recovery');
    second.resolve({}); await drain();
    assert.equal(h.controls.loadedBinding, 'a'.repeat(64));
    assert.equal(h.controls.loading, null);
  });
}

test('loss of view state clears the scene and pending playback immediately', async () => {
  const h = harness(); h.controls.update(state()); await drain();
  const first = h.controls.seek(1), pending = h.controls.seek(2);
  h.controls.update({view: null});
  assert.equal(h.element('plan-play').disabled, true);
  assert.equal(h.element('plan-seek').disabled, true);
  assert.equal(await pending, false);
  assert.equal(h.controls.viewer.clears, 1);
  assert.equal(h.controls.expected, null);
  h.reply(0); assert.equal(await first, false); await drain();
  assert.equal(h.requests.length, 1);
  assert.equal(h.timers.size, 0);
});

const sourceCatalog = [
  {id: 'workcell:manual', label: 'FR5 + UR5e', available: true},
  {id: 'profile:fr10', label: 'FR10', available: true},
  {id: 'profile:ur5e', label: 'UR5e', available: true},
];
function selectedSource(id, generation = 1, catalog = sourceCatalog) {
  return state(generation === 1 ? 'a' : 'b', generation, {
    mode: id.startsWith('profile:') ? 'model_preview' : 'live', preview_active: id.startsWith('profile:'),
    source: {id}, catalog, duration_sec: null, preview_time_sec: null,
  });
}

test('server source changes synchronize the dropdown without a catalog rebuild', async () => {
  const h = harness();
  h.controls.update(selectedSource('profile:fr10')); await drain();
  const select = h.element('scene-source'), options = [...select.children];
  assert.equal(select.value, 'profile:fr10');
  assert.deepEqual(select.children.map(option => [option.value, option.textContent]),
    sourceCatalog.map(item => [item.id, item.label]));
  h.controls.update(selectedSource('workcell:manual', 2)); await drain();
  assert.equal(select.value, 'workcell:manual', 'Live binding must show the actual workcell');
  assert.deepEqual(select.children, options, 'Changing the source does not require replacing the catalog');
  h.controls.update(selectedSource('profile:ur5e', 2));
  assert.equal(select.value, 'profile:ur5e');
});

test('same-source polling preserves a user selection that has not been applied', async () => {
  const h = harness(); h.controls.update(selectedSource('workcell:manual')); await drain();
  const select = h.element('scene-source'); select.value = 'profile:fr10';
  for (let poll = 0; poll < 5; poll++) h.controls.update(selectedSource('workcell:manual'));
  assert.equal(select.value, 'profile:fr10');
  assert.equal(h.controls.appliedSource, 'workcell:manual');
  assert.equal(h.requests.length, 0, 'Selecting an option alone never applies it');
  h.controls.update(selectedSource('profile:fr10', 2));
  assert.equal(select.value, 'profile:fr10');
  select.value = 'profile:ur5e'; h.controls.update(selectedSource('profile:fr10', 2));
  assert.equal(select.value, 'profile:ur5e');
});

test('catalog replacement preserves an available draft and resets a removed or disabled draft', async () => {
  const h = harness(); h.controls.update(selectedSource('workcell:manual')); await drain();
  const select = h.element('scene-source'); select.value = 'profile:fr10';
  const renamed = sourceCatalog.map(item => ({...item, label: item.label+' updated'}));
  h.controls.update(selectedSource('workcell:manual', 1, renamed));
  assert.equal(select.value, 'profile:fr10', 'A rebuild must not lose the pending available choice');
  assert.equal(select.children[1].textContent, 'FR10 updated');
  const disabled = renamed.map(item => ({...item, available: item.id !== 'profile:fr10'}));
  h.controls.update(selectedSource('workcell:manual', 1, disabled));
  assert.equal(select.children[1].disabled, true);
  assert.equal(select.value, 'workcell:manual');
  select.value = 'profile:ur5e';
  h.controls.update(selectedSource('workcell:manual', 1, disabled.slice(0, 2)));
  assert.equal(select.value, 'workcell:manual', 'A removed draft must not leave a blank selection');
});

test('a catalog rebuilt with a changed server source initializes to the applied source', async () => {
  const h = harness(); h.controls.update(selectedSource('workcell:manual')); await drain();
  const select = h.element('scene-source'); select.value = 'profile:ur5e';
  const reordered = [...sourceCatalog].reverse();
  h.controls.update(selectedSource('profile:fr10', 2, reordered));
  assert.equal(select.children[0].value, 'profile:ur5e');
  assert.equal(select.value, 'profile:fr10', 'Catalog order and old draft cannot override the actual source');
});

test('frame age distinguishes live waiting, live telemetry, previews and no active mode', async () => {
  const h = harness(); h.controls.update(state()); await drain();
  for (const [mode, source_age_ms, expected] of [
    ['live', null, '상태 수신 대기'], ['live', undefined, '상태 수신 대기'],
    ['live', 12.6, '13 ms 전 상태'], ['model_preview', null, '읽기 전용 보기'],
    ['plan_preview', 10, '읽기 전용 보기'], [null, null, '장면 준비 대기'],
  ]) {
    h.controls.viewer.onMetrics({binding_id: 'a'.repeat(64), mode, source_age_ms, view_state: 'waiting'});
    assert.equal(h.element('frame-age').textContent, expected);
    assert.equal(h.element('demo-frame-age').textContent, expected);
  }
  h.controls.viewer.onMetrics({binding_id: 'b'.repeat(64), mode: 'live', source_age_ms: 100, view_state: 'fresh'});
  assert.equal(h.element('frame-age').textContent, '장면 준비 대기', 'Stale binding metrics cannot overwrite the current label');
});
