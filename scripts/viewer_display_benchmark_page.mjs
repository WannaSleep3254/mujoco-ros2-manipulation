/** Isolated display-only probe, served only by viewer_display_benchmark.py. */
import {LocalSceneView} from '/static/viewer.js';

performance.setResourceTimingBufferSize(20000);
const config = await (await fetch('/config')).json();
const stage = document.querySelector('#stage');
stage.style.width = `${config.width}px`;
stage.style.height = `${config.height}px`;
const state = {config, samples: [], metrics: [], inputSamples: [], errors: [], frames: 0,
  applied: 0, firstVisibleMs: null, lastSequence: 0, paused: false, polling: false, pendingInput: null};
globalThis.benchmark = state;
const initStarted = performance.now();
const afterAnimationFrame = () => new Promise(resolve => requestAnimationFrame(resolve));
let sourceStart = 0;
let view = null, canvas = null, zoom = 1, sequence = 0, inputGeneration = 0, currentBinding = config.binding;
let currentID = config.binding_id;

if (config.mode === 'web3d') {
  view = new LocalSceneView(stage, {binding_id: currentID, binding: currentBinding,
    assetURL: hash => '/assets/' + hash,
    onMetrics: sample => {state.metrics.push({...sample, at_ms: performance.now()});},
    onRenderFrame: sample => {
      if (!sample.displaying_pose) return;
      state.frames++;
      if (state.firstVisibleMs === null) state.firstVisibleMs = performance.now() - initStarted;
      if (state.pendingInput && sample.camera_position.some((v, i) => Math.abs(v - state.pendingInput.position[i]) > 1e-8)) {
        state.inputSamples.push({event_to_render_submit_ms: sample.submitted_at_ms - state.pendingInput.at,
          trusted: state.pendingInput.trusted, end: 'CPU render submission; not GPU completion or paint'});
        state.pendingInput = null;
      }
    }});
  await view.loadManifest('/manifest', {model_sha256: config.model_sha256, scene_sha256: config.scene_sha256});
  // Explicit fixed camera, also used by the MuJoCo JPEG renderer. Disable the
  // product's automatic fit, which intentionally favors robot visual bounds.
  const camera = config.camera;
  view.needsFit = false;
  view.camera.position.fromArray(camera.position);
  view.camera.up.fromArray(camera.up);
  view.camera.fov = camera.fov;
  view.camera.near = camera.near;
  view.camera.far = camera.far;
  view.controls.target.set(...camera.position.map((v, i) => v + camera.forward[i] * 2.6));
  view.camera.updateProjectionMatrix();
  view.controls.update();
  canvas = view.canvas;
  canvas.addEventListener('wheel', event => {
    state.pendingInput = {at: performance.now(), position: view.camera.position.toArray(), trusted: event.isTrusted};
  }, {capture: true, passive: true});
  const gl = view.renderer.getContext(), extension = gl.getExtension('WEBGL_debug_renderer_info');
  state.gl = {vendor: gl.getParameter(extension ? extension.UNMASKED_VENDOR_WEBGL : gl.VENDOR),
    renderer: gl.getParameter(extension ? extension.UNMASKED_RENDERER_WEBGL : gl.RENDERER), version: gl.getParameter(gl.VERSION)};
  state.view = view;
} else {
  canvas = document.createElement('canvas');
  canvas.width = config.width; canvas.height = config.height;
  stage.append(canvas);
  canvas.addEventListener('wheel', event => {
    event.preventDefault();
    zoom *= Math.pow(.95, -event.deltaY / 100);
    zoom = Math.min(10, Math.max(.1, zoom));
    state.pendingInput = {at: performance.now(), trusted: event.isTrusted, generation: ++inputGeneration};
  }, {passive: false});
}

async function poll() {
  if (state.paused || state.polling) return;
  state.polling = true;
  const start = performance.now();
  const elapsed = (start - sourceStart) / 1000 % config.source_period_sec;
  let index = 0;
  while (index + 1 < config.sample_count && config.sample_times_sec[index + 1] <= elapsed) index++;
  const requestInputGeneration = inputGeneration;
  const query = `?index=${index}&sequence=${++sequence}&zoom=${zoom}`;
  try {
    const response = await fetch((view ? '/pose' : '/frame') + query, {cache: 'no-store'});
    if (!response.ok) throw new Error(`Sample HTTP ${response.status}`);
    if (view) {
      const pose = await response.json();
      pose.binding = currentBinding; pose.binding_id = currentID;
      if (view.updatePose(pose, {elapsedRequestMs: performance.now() - start})) state.applied++;
    } else {
      const blob = await response.blob();
      const bitmap = await createImageBitmap(blob);
      canvas.getContext('2d').drawImage(bitmap, 0, 0);
      bitmap.close();
      await afterAnimationFrame();
      state.frames++; state.applied++;
      if (state.firstVisibleMs === null) state.firstVisibleMs = performance.now() - initStarted;
      if (state.pendingInput && state.pendingInput.generation === requestInputGeneration) {
        state.inputSamples.push({event_to_decode_next_raf_ms: performance.now() - state.pendingInput.at,
          trusted: state.pendingInput.trusted, end: 'JPEG decode/draw plus next rAF; not physical display paint'});
        state.pendingInput = null;
      }
    }
    state.lastSequence = sequence;
    state.samples.push({at_ms: performance.now(), sequence, record_index: index,
      source_time_sec: elapsed, response_ms: performance.now() - start});
  } catch (error) {state.errors.push({at_ms: performance.now(), message: String(error)});}
  finally {state.polling = false;}
}

sourceStart = performance.now();
await poll();
const timer = setInterval(poll, config.poll_ms);
state.ready = true;
state.snapshot = () => ({config, first_visible_ms: state.firstVisibleMs, frames: state.frames,
  applied: state.applied, last_sequence: state.lastSequence, samples: state.samples,
  metrics: state.metrics, input_samples: state.inputSamples, errors: state.errors,
  gl: state.gl ?? null, canvas_pixels: {width: canvas.width, height: canvas.height}, dpr: devicePixelRatio});

state.stability = async () => {
  if (!view) return null;
  const checks = [];
  const check = (name, pass) => {checks.push({name, pass: !!pass}); if (!pass) throw new Error(name);};
  const wait = ms => new Promise(resolve => setTimeout(resolve, ms));
  state.paused = true;
  while (state.polling) await wait(10);
  await wait(650);
  check('paused source becomes stale', view.label.dataset.state === 'stale' && view.hasPose);
  const initialCamera = view.camera.position.toArray();
  check('camera remains available while stale', view.zoomCamera('in'));
  check('stale camera zoom changes camera', initialCamera.some((q, i) => Math.abs(q - view.camera.position.toArray()[i]) > 1e-8));
  state.paused = false; await poll();
  check('fresh reconnect recovers', view.label.dataset.state !== 'stale');
  state.paused = true;
  const prior = await (await fetch('/pose?index=1&sequence=' + (++sequence))).json();
  prior.binding_id = currentID; prior.binding = currentBinding;
  const acceptedSequence = view.sequence;
  const malformed = structuredClone(prior); malformed.pose.geoms[0].position[0] = null;
  let rejected = false;
  try {view.updatePose(malformed);} catch (_) {rejected = true;}
  check('malformed pose is atomic', rejected && view.sequence === acceptedSequence);
  const old = {...prior, sequence: acceptedSequence};
  check('duplicate cannot refresh age', view.updatePose(old) === false);
  // Source rewind is explicitly a new epoch. The owner must create this epoch;
  // this test does not claim LocalSceneView detects arbitrary clock domains.
  currentID = 'c'.repeat(64); currentBinding = {...currentBinding, generation: currentBinding.generation + 1};
  view.setExpectedBinding(currentID, currentBinding);
  check('new source epoch hides previous model pose', !view.hasPose && !view.world.visible);
  check('late old epoch is rejected', view.updatePose(prior) === false);
  await view.loadManifest('/manifest', {model_sha256: config.model_sha256, scene_sha256: config.scene_sha256});
  view.needsFit = false;
  const rewind = await (await fetch('/pose?index=0&sequence=1')).json();
  rewind.binding_id = currentID; rewind.binding = currentBinding;
  check('rewind after explicit rebind accepts sequence one', view.updatePose(rewind));
  check('wrong model hash cannot overwrite new epoch', (() => {
    const wrong = structuredClone(rewind); wrong.sequence = 2; wrong.pose.model_sha256 = 'f'.repeat(64);
    try {view.updatePose(wrong); return false;} catch (_) {return view.sequence === 1;}
  })());
  sequence = 1;
  state.paused = false;
  return {checks, scope: 'Owned source pause/rebind/rewind and model mismatch; no robot command, physical controller or multi-day soak'};
};
state.stop = () => {clearInterval(timer); state.paused = true;};
