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
    options: {onRenderFrame}, label: {dataset: {state: 'fresh'}}});
  return view;
}

test('frame observation follows CPU submission and retains first visible submission', () => {
  const calls = [], view = fixture(sample => calls.push(sample));
  view._frame(); view._frame();
  assert.equal(calls.length, 2);
  assert.equal(calls[0].render_submit_ms, 2);
  assert.equal(calls[0].submitted_at_ms, 1002);
  assert.equal(calls[1].first_visible_submitted_at_ms, 1002);
  assert.deepEqual(calls[0].camera_position, [1, 2, 3]);
  assert.ok(!Object.hasOwn(calls[0], 'paint_ms'));
  assert.ok(!Object.hasOwn(calls[0], 'gpu_ms'));
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
