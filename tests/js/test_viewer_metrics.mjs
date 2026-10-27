/** CPU submission instrumentation must not claim GPU/physical paint timing. */
import test from 'node:test';
import assert from 'node:assert/strict';
import {readFile} from 'node:fs/promises';
import * as contract from '../../src/mujoco_ros2_manipulation/operator_console/static/viewer_contract.mjs';

const source = await readFile(new URL('../../src/mujoco_ros2_manipulation/operator_console/static/viewer.js', import.meta.url), 'utf8');
const classSource = source.slice(source.indexOf('function localURL(')).replace('export class LocalSceneView', 'class LocalSceneView');
const LocalSceneView = new Function(...Object.keys(contract), 'THREE', 'OrbitControls',
  classSource + '\nreturn LocalSceneView;')(...Object.values(contract), {}, class {});

function fixture(onRenderFrame) {
  let now = 1000;
  const view = Object.create(LocalSceneView.prototype);
  Object.assign(view, {disposed: false, document: {hidden: false}, sized: true, sceneError: null,
    controls: {update() {}}, renderer: {render() {now += 2;}}, camera: {
      position: {toArray: () => [1, 2, 3]}, quaternion: {toArray: () => [0, 0, 0, 1]}},
    now: () => now, _refreshStatus() {}, world: {visible: true}, hasPose: true, sequence: 4,
    stats: {}, statsAt: now, renderCount: 0, poseCount: 0, firstVisibleSubmittedAt: null,
    options: {onRenderFrame}, label: {dataset: {state: 'fresh'}}, advance:ms=>{now+=ms;}});
  return view;
}

test('frame observation follows CPU submission and retains first visible submission', () => {
  const calls = [], view = fixture(sample => calls.push(sample));
  view._frame(); view._invalidate(); view._frame();
  assert.equal(calls.length, 2);
  assert.equal(calls[0].render_submit_ms, 2);
  assert.equal(calls[0].submitted_at_ms, 1002);
  assert.equal(calls[1].first_visible_submitted_at_ms, 1002);
  assert.deepEqual(calls[0].camera_position, [1, 2, 3]);
  assert.ok(!Object.hasOwn(calls[0], 'paint_ms'));
  assert.ok(!Object.hasOwn(calls[0], 'gpu_ms'));
});

test('first-visible notification occurs after submission and only once per loaded epoch', () => {
  const view=fixture(undefined);let calls=0;
  view.options.onFirstVisibleFrame=()=>{calls++;assert.equal(view.firstVisibleSubmittedAt,1002);assert.equal(view.stats.render_submit_ms,2);};
  view._frame();view._frame();assert.equal(calls,1);
});

test('hidden documents and lost contexts do not report a submitted frame', () => {
  const calls = [], view = fixture(sample => calls.push(sample));
  view.document.hidden = true; view._frame();
  view.document.hidden = false; view.sceneError = 'lost'; view._frame();
  assert.equal(calls.length, 0);
});

test('optional observation does not require copying a camera when disabled', () => {
  const view = fixture(undefined);
  delete view.camera.position.toArray;
  view._frame();
  assert.equal(view.renderCount, 1);
});

test('a settled scene submits once, camera damping continues, and idle metrics become zero without a GPU call',()=>{
  const calls=[],metrics=[],view=fixture(sample=>calls.push(sample));view.options.onMetrics=sample=>metrics.push(sample);
  view._frame();view._frame();assert.equal(calls.length,1);
  view.controls.update=()=>true;view._frame();assert.equal(calls.length,2);
  view.controls.update=()=>false;view.advance(1000);view._frame();view.advance(1000);view._frame();
  assert.equal(calls.length,2);assert.equal(metrics.at(-1).render_activity,'idle');assert.equal(metrics.at(-1).rendering_fps,0);
  assert.equal(metrics.at(-1).render_policy,'on-demand');assert.equal(metrics.at(-1).render_submit_ms,null);
});

test('an independent metrics heartbeat reports hidden/error/zero-size states without rendering',()=>{
  const calls=[],metrics=[],view=fixture(sample=>calls.push(sample));view.options.onMetrics=sample=>metrics.push(sample);
  view.document.hidden=true;view.advance(1000);view._emitMetrics();assert.equal(metrics.at(-1).render_activity,'hidden');
  view.document.hidden=false;view.sceneError='lost';view.advance(1000);view._emitMetrics();assert.equal(metrics.at(-1).render_activity,'error');
  view.sceneError=null;view.sized=false;view.advance(1000);view._emitMetrics();assert.equal(metrics.at(-1).render_activity,'zero-size');
  assert.equal(calls.length,0);assert.ok(metrics.every(sample=>sample.rendering_fps===0));
});

test('renderer counters come from actual renderer.info and resource history remains bounded',()=>{
  const metrics=[],view=fixture(undefined);view.options.onMetrics=sample=>metrics.push(sample);
  view.renderer.info={render:{calls:7,triangles:123,lines:2,points:4},memory:{geometries:3,textures:1},programs:[{},{}]};
  view._frame();view.advance(1000);view._emitMetrics();
  assert.deepEqual(metrics.at(-1).last_draw,{draw_calls:7,triangles:123,lines:2,points:4,geometry_resources:3,texture_resources:1,shader_programs:2});
  for(let n=0;n<80;n++){view.advance(1000);view._emitMetrics();}
  assert.equal(view.resourceHistory.length,60);assert.equal(metrics.at(-1).resource_history.length,60);
  metrics.at(-1).resource_history[0].geometry_resources=999;assert.equal(view.resourceHistory[0].geometry_resources,3);
  assert.match(metrics.at(-1).note,/not VRAM/);assert.equal(Object.hasOwn(metrics.at(-1),'vram_bytes'),false);
});

test('CPU submission p95 uses a bounded recent sample set and expires after idle time',()=>{
  const metrics=[],view=fixture(undefined);view.options.onMetrics=sample=>metrics.push(sample);
  for(let n=0;n<300;n++){view._invalidate();view._frame();view.advance(5);}
  assert.equal(view.cpuSubmitSamples.length,240);view._emitMetrics(true);
  assert.equal(metrics.at(-1).render_submit_ms_p95,2);assert.equal(metrics.at(-1).render_submit_sample_count,240);
  view.advance(6000);view._emitMetrics();assert.equal(metrics.at(-1).render_submit_ms_p95,null);assert.equal(metrics.at(-1).render_submit_sample_count,0);
  assert.equal(metrics.at(-1).rendering_fps,0);assert.match(metrics.at(-1).note,/not GPU time or physical paint/);
});
