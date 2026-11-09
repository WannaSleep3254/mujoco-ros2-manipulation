import test from 'node:test';
import assert from 'node:assert/strict';
import {readFile} from 'node:fs/promises';
import vm from 'node:vm';

const source = (await readFile(new URL('../../src/mujoco_ros2_manipulation/operator_console/static/app.js', import.meta.url), 'utf8'))
 .replace(/boot\(\)\.catch\(error=>\{notice\('조작 화면을 불러오지 못했습니다: '\+error\.message\);\}\);\s*$/, '');
const ID = 'a'.repeat(32);
const deferred = () => {
 let resolve, reject;
 const promise = new Promise((accept, refuse) => { resolve = accept; reject = refuse; });
 return {promise, resolve, reject};
};
function harness(fetcher) {
 let now = 10;
 const elements = new Map();
 const context = vm.createContext({
  document: {getElementById(id) {
   if (!elements.has(id)) elements.set(id, {textContent: ''});
   return elements.get(id);
  }},
  performance: {now() { return typeof now === 'function' ? now() : now; }},
  fetch: fetcher, AbortController, setTimeout, clearTimeout,
 });
 vm.runInContext(source + `
this.timingTest = {
  request, observeBrowserRequestTimings,
  samples: () => JSON.parse(JSON.stringify(browserRequestTimings)),
 };`, context);
 return {api: context.timingTest, elements, now(value) { now = value; }};
}
const reply = (data, status = 200) => ({ok: status < 400, status, json: async () => data});
const flush = async () => { for (let index = 0; index < 5; index++) await Promise.resolve(); };

test('ACK completes after JSON decoding and before a later state poll', async () => {
 const headers = deferred(), body = deferred();
 const h = harness(() => headers.promise);
 const pending = h.api.request('/api/plan', {request_id: ID});
 h.now(12); headers.resolve({ok: true, status: 202, json: () => body.promise});
 await flush();
 assert.equal(h.api.samples()[0].headers_ms, 12);
 assert.equal(h.api.samples()[0].json_complete_ms, null);
 h.now(18); body.resolve({request_id: ID}); await pending;
 assert.equal(h.api.samples()[0].elapsed_ms, 8);
 h.now(42); h.api.observeBrowserRequestTimings({history: [{request_id: ID, status: 'running'}]});
 h.now(88); h.api.observeBrowserRequestTimings({history: [{request_id: ID, status: 'succeeded'}]});
 const sample = h.api.samples()[0];
 assert.equal(sample.json_complete_ms, 18);
 assert.equal(sample.first_state_observed_ms, 42);
 assert.equal(sample.first_state_status, 'running');
 assert.equal(sample.terminal_state_observed_ms, 88);
 assert.equal(sample.elapsed_ms, 8);
});

test('state receipt may precede a slow HTTP acknowledgement', async () => {
 const body = deferred(), h = harness(() => Promise.resolve({ok: true, status: 202, json: () => body.promise}));
 const pending = h.api.request('/api/cancel', {request_id: ID}); await flush();
 h.now(15); h.api.observeBrowserRequestTimings({history: [{request_id: ID, status: 'cancelled'}]});
 h.now(20); body.resolve({request_id: ID}); await pending;
 assert.equal(h.api.samples()[0].terminal_state_observed_ms, 15);
 assert.equal(h.api.samples()[0].json_complete_ms, 20);
});

test('GET polls are excluded and POST samples remain bounded at forty', async () => {
 const h = harness(async () => reply({request_id: ID}));
 await h.api.request('/api/state'); assert.equal(h.api.samples().length, 0);
 for (let index = 0; index < 43; index++) await h.api.request('/api/plan', {request_id: ID});
 const samples = h.api.samples();
 assert.equal(samples.length, 40); assert.equal(samples[0].sequence, 4);
 assert.equal(samples.at(-1).sequence, 43);
 assert.equal(JSON.parse(h.elements.get('request-metrics').textContent).samples.length, 40);
});

test('HTTP rejection and unknown delivery stay separate without a retry', async () => {
 let posts = 0;
 const h = harness(async () => { posts++; return reply({error: 'refused'}, 409); });
 await assert.rejects(h.api.request('/api/plan', {request_id: ID}), error => error.httpStatus === 409);
 assert.equal(h.api.samples()[0].outcome, 'http_error');
 assert.equal(h.api.samples()[0].http_status, 409); assert.equal(posts, 1);
 const fault = harness(async () => { throw Error('lost response'); });
 await assert.rejects(fault.api.request('/api/cancel', {request_id: ID}), /lost response/);
 assert.equal(fault.api.samples()[0].outcome, 'transport_or_decode_error');
 assert.equal(fault.api.samples()[0].json_complete_ms, null);
});

for (const broken of [() => { throw Error('clock unavailable'); }, () => NaN]) {
 test('optional clock failure preserves the command response and leaves metrics unavailable', async () => {
  let posts = 0;
  const h = harness(async () => { posts++; return reply({request_id: ID}); });
  h.now(broken);
  assert.equal((await h.api.request('/api/approve', {request_id: ID})).request_id, ID);
  const sample = h.api.samples()[0];
  assert.equal(sample.started_ms, null); assert.equal(sample.elapsed_ms, null);
  assert.equal(sample.outcome, 'accepted'); assert.equal(posts, 1);
 });
}

test('clock reversal cannot produce a negative duration or a fabricated identity', async () => {
 const body = deferred(), h = harness(async () => ({ok: true, status: 202, json: () => body.promise}));
 const pending = h.api.request('/api/task/select', {task: 'grasp'}); await flush();
 h.now(1); body.resolve({request_id: 'untrusted'}); await pending;
 assert.equal(h.api.samples()[0].elapsed_ms, null);
 assert.equal(h.api.samples()[0].request_id, null);
});

test('late task acknowledgement preserves analysis and review navigation', () => {
 assert.match(source, /showOperate:\(\)=>\{if\(!\$\('operate'\)\.hidden&&\$\('operate'\)\.dataset\.workspaceMode==='operate'\)show\('operate',\{focusTask:true\}\);\}/);
 const callback = source.match(/showOperate:(\(\)=>\{if\(!\$\('operate'\)\.hidden[^}]+\}\);\})/)[1];
 const pane = {hidden: true, dataset: {workspaceMode: 'operate'}};
 let shown = 0;
 const context = vm.createContext({$: () => pane, show() { shown++; }});
 const invoke = vm.runInContext(callback, context);
 invoke(); assert.equal(shown, 0);
 pane.hidden = false; pane.dataset.workspaceMode = 'review'; invoke(); assert.equal(shown, 0);
 pane.dataset.workspaceMode = 'operate'; invoke(); assert.equal(shown, 1);
});


test('first observed statuses keep missing original times after the optional clock recovers', async () => {
 const h = harness(async () => reply({request_id: ID}));
 await h.api.request('/api/plan', {request_id: ID});
 h.now(() => { throw Error('clock unavailable on first state'); });
 h.api.observeBrowserRequestTimings({history: [{request_id: ID, status: 'running'}]});
 h.api.observeBrowserRequestTimings({history: [{request_id: ID, status: 'succeeded'}]});
 h.now(42);
 h.api.observeBrowserRequestTimings({history: [{request_id: ID, status: 'succeeded'}]});
 const sample = h.api.samples()[0];
 assert.equal(sample.first_state_status, 'running');
 assert.equal(sample.first_state_observed_ms, null);
 assert.equal(sample.terminal_state_observed_ms, null);
});
