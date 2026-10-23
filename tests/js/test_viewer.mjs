/** Node-only boundary tests. Never constructs WebGLRenderer, DOM or ROS. */
import test from 'node:test';
import assert from 'node:assert/strict';
import {readFile} from 'node:fs/promises';
import {createHash, webcrypto} from 'node:crypto';
import * as contract from '../../src/mujoco_ros2_manipulation/operator_console/static/viewer_contract.mjs';

const hash = data => createHash('sha256').update(new Uint8Array(data)).digest('hex');
const BINDING_ID = 'b'.repeat(64);
const OTHER_BINDING_ID = 'c'.repeat(64);
const binding = (mode = 'live') => ({operator_session_id: 'a'.repeat(32), generation: 1, mode,
  source_id: 'manual', job_id: null, run_id: null, motion_session_id: null, request_id: mode === 'plan_preview' ? 'd'.repeat(32) : null});
const rotation = [1,0,0,0,1,0,0,0,1];
const geom = (id, type = 'box', group = 0) => ({id, name: 'geom-' + id, type, group,
  default_visible: group <= 2, size: [1, .5, .25], rgba: [.2, .5, .8, 1], asset_sha256: null});
const manifest = () => ({schema_version: 1, export_version: 'compiled-scene-v1',
  model_sha256: '1'.repeat(64), scene_sha256: '2'.repeat(64), compiled_model_sha256: '3'.repeat(64),
  units: {length: 'm', angle: 'rad', up_axis: 'Z', rotation: 'row-major'}, joint_names: [], home_rad: {},
  default_visible_groups: [0,1,2], geoms: [geom(0), geom(1, 'capsule', 3)], assets: []});
const pose = (mode = 'live') => ({schema_version: 1, binding_id: BINDING_ID, binding: binding(mode),
  sequence: 1, source_state_age_sec: mode === 'live' ? .05 : null, simulation_sec: mode === 'live' ? 5 : null,
  ...(mode === 'plan_preview' ? {preview_time_sec: .25} : {}),
  pose: {schema_version: 1, model_sha256: '1'.repeat(64), scene_sha256: '2'.repeat(64),
    geoms: [0,1].map(id => ({id, position: [id + 1, 2, 3], rotation: [...rotation]}))}});

function triangle() {
  const bytes = new ArrayBuffer(48);
  const view = new DataView(bytes);
  [2,3,4, 3,3,4, 2,4,4].forEach((value, index) => view.setFloat32(index * 4, value, true));
  [0,1,2].forEach((value, index) => view.setUint32(36 + index * 4, value, true));
  const descriptor = {sha256: hash(bytes), byte_length: 48, media_type: 'application/octet-stream',
    layout: {positions: {byte_offset: 0, byte_length: 36, count: 3, item_size: 3, component_type: 'float32'},
      indices: {byte_offset: 36, byte_length: 12, count: 3, item_size: 1, component_type: 'uint32'}}};
  return {bytes, descriptor};
}

class FakeGeometry {
  constructor(...args) {this.args = args; this.disposed = false;}
  rotateX(angle) {this.rotate = angle; return this;}
  setAttribute(name, value) {this[name] = value;}
  setIndex(value) {this.index = value;}
  computeVertexNormals() {}
  computeBoundingSphere() {}
  dispose() {this.disposed = true;}
}
class FakeGroup {
  constructor() {this.children = []; this.visible = true;}
  add(value) {this.children.push(value);}
}
class FakeMaterial {
  constructor(config) {this.config = config; this.disposed = false;}
  dispose() {this.disposed = true;}
}
class FakeMesh {
  constructor(geometry, material) {this.geometry = geometry; this.material = material;
    this.matrix = {set: (...values) => {this.matrixValues = values;}};}
}
const THREE = {Group: FakeGroup, Mesh: FakeMesh, BufferGeometry: FakeGeometry,
  BufferAttribute: class {constructor(array, itemSize) {this.array = array; this.itemSize = itemSize;}},
  BoxGeometry: FakeGeometry, SphereGeometry: FakeGeometry, CapsuleGeometry: FakeGeometry,
  CylinderGeometry: FakeGeometry, PlaneGeometry: FakeGeometry, MeshStandardMaterial: FakeMaterial,
  Color: class {setRGB(...values) {this.values = values; return this;}}, DoubleSide: 2};

// Compile the real class methods while deliberately skipping imports/constructor.
// Only THREE mocks are injected; no package download, GL context or GPU is used.
const source = await readFile(new URL('../../src/mujoco_ros2_manipulation/operator_console/static/viewer.js', import.meta.url), 'utf8');
const classSource = source.slice(source.indexOf('function localURL(')).replace('export class LocalSceneView', 'class LocalSceneView');
const {LocalSceneView, localURL} = new Function(...Object.keys(contract), 'THREE', 'OrbitControls',
  classSource + '\nreturn {LocalSceneView, localURL};')(...Object.values(contract), THREE, class {});

function fakeView() {
  let now = 1000;
  const value = Object.create(LocalSceneView.prototype);
  Object.assign(value, {disposed: false, bindingID: BINDING_ID, binding: binding(), manifest: manifest(),
    loading: false, loadGeneration: 0, loadController: null, poseController: null, sequence: -1,
    now: () => now, advance: ms => {now += ms;}, receivedAt: null, sourceAgeSec: null,
    world: new FakeGroup(), objects: new Map([0,1].map(id => [id, new FakeMesh()])),
    materials: new Set(), geometries: new Set(), sceneError: null, manualStaleReason: null,
    hasPose: false, needsFit: false, collisionVisible: false, poseCount: 0, renderCount: 0,
    stats: {}, statsAt: now, staleAfterMs: 500, canvas: {style: {}}, label: {dataset: {}, textContent: ''},
    camera: {position: {sentinel: 1}}, _fitCamera() {throw new Error('Unexpected camera mutation');},
    baseURL: 'http://127.0.0.1:8881/', crypto: webcrypto,
    assetURL: hash => '/api/view/assets/' + hash,
    scene: {add() {}, remove() {}}, planeExtentM: 10});
  return value;
}

test('bindings are exact, detached and reject missing identity components', () => {
  const original = binding();
  const checked = contract.validateBinding(BINDING_ID, original);
  assert.notEqual(original, checked);
  original.generation = 42;
  assert.equal(checked.generation, 1);
  for (const change of [b => delete b.motion_session_id, b => b.unknown = 'hidden', b => b.generation = true,
    b => b.operator_session_id = 'other', b => b.mode = 'commands', b => b.job_id = '../path',
    b => b.source_id = 'file:local', b => b.source_id = '/local/model', b => b.run_id = 'unpaired',
    b => b.request_id = 'c'.repeat(32)]) {
    const candidate = binding(); change(candidate);
    assert.throws(() => contract.validateBinding(BINDING_ID, candidate));
  }
});

test('manifest validates binary ranges, unique geoms, defaults and compiled identity', () => {
  const base = manifest();
  assert.equal(contract.validateManifest(base), base);
  const {descriptor} = triangle();
  base.assets = [descriptor];
  base.geoms[0].type = 'mesh'; base.geoms[0].asset_sha256 = descriptor.sha256;
  assert.equal(contract.validateManifest(base), base);
  const changes = [m => m.geoms.push(structuredClone(m.geoms[0])), m => m.geoms[0].asset_sha256 = 'f'.repeat(64),
    m => m.assets[0].layout.indices.byte_offset = 0, m => m.assets[0].byte_length++,
    m => m.geoms[1].default_visible = true, m => m.units.up_axis = 'Y', m => m.geoms[0].mesh_pos = [0,0,0],
    m => m.geoms[1].rgba[0] = NaN, m => m.geoms[0].size[0] = -1];
  for (const change of changes) {const bad = structuredClone(base); change(bad); assert.throws(() => contract.validateManifest(bad));}
  assert.throws(() => contract.validateManifest(base, {scene_sha256: 'f'.repeat(64)}), /mismatch/);
});

test('compiled little-endian vertices are unmodified and invalid floats/indices fail', () => {
  const {bytes, descriptor} = triangle();
  const mesh = contract.decodeMesh(bytes, descriptor);
  assert.deepEqual([...mesh.positions], [2,3,4, 3,3,4, 2,4,4]);
  assert.deepEqual([...mesh.indices], [0,1,2]);
  const nan = bytes.slice(0); new DataView(nan).setFloat32(0, NaN, true);
  assert.throws(() => contract.decodeMesh(nan, descriptor), /Nonfinite/);
  const badIndex = bytes.slice(0); new DataView(badIndex).setUint32(36, 3, true);
  assert.throws(() => contract.decodeMesh(badIndex, descriptor), /out of range/);
  assert.throws(() => contract.decodeMesh(bytes.slice(0, 44), descriptor), /byte length/);
});

test('pose verifies all bindings, whole catalog and proper world rotations before apply', () => {
  assert.ok(contract.validatePose(pose(), manifest(), BINDING_ID, binding()));
  const changes = [p => p.binding_id = OTHER_BINDING_ID, p => p.binding.generation++,
    p => p.pose.model_sha256 = 'f'.repeat(64), p => p.pose.scene_sha256 = 'f'.repeat(64),
    p => p.pose.geoms.pop(), p => p.pose.geoms[1].id = 0, p => p.sequence = 0,
    p => p.pose.geoms[1].position[0] = Infinity, p => p.pose.geoms[1].rotation[0] = -1,
    p => p.pose.geoms[1].rotation[1] = .1];
  for (const change of changes) {const bad = pose(); change(bad); assert.throws(() => contract.validatePose(bad, manifest(), BINDING_ID, binding()));}
  const zTurn = pose(); zTurn.pose.geoms[0].rotation = [0,-1,0,1,0,0,0,0,1];
  assert.ok(contract.validatePose(zTurn, manifest(), BINDING_ID, binding()));
});

test('live needs a finite age; read-only previews use null age and a plan time', () => {
  for (const age of [null, undefined, -1, Infinity]) {
    const p = pose(); p.source_state_age_sec = age;
    assert.throws(() => contract.validatePose(p, manifest(), BINDING_ID, binding()));
  }
  for (const mode of ['model_preview', 'plan_preview']) assert.ok(contract.validatePose(pose(mode), manifest(), BINDING_ID, binding(mode)));
  const bad = pose('plan_preview'); delete bad.preview_time_sec;
  assert.throws(() => contract.validatePose(bad, manifest(), BINDING_ID, binding('plan_preview')));
  const stamped = pose('model_preview'); stamped.source_state_age_sec = .01;
  assert.throws(() => contract.validatePose(stamped, manifest(), BINDING_ID, binding('model_preview')));
});

test('pose applies once, preserves the camera and never partly applies malformed poses', () => {
  const view = fakeView();
  const camera = structuredClone(view.camera);
  const input = pose();
  assert.equal(view.updatePose(input), true);
  assert.deepEqual(view.objects.get(0).matrixValues, [1,0,0,1,0,1,0,2,0,0,1,3,0,0,0,1]);
  assert.deepEqual(view.camera, camera);
  const received = view.receivedAt;
  view.advance(300);
  assert.equal(view.updatePose(input), false);
  assert.equal(view.receivedAt, received, 'Duplicate sequence cannot refresh freshness');
  const bad = pose(); bad.sequence = 2; bad.pose.geoms[0].position = [99,99,99]; bad.pose.geoms[1].rotation[0] = -1;
  assert.throws(() => view.updatePose(bad));
  assert.equal(view.sequence, 1);
  assert.equal(view.objects.get(0).matrixValues[3], 1, 'Validation is complete before matrix writes');
  assert.equal(view.updatePose({...input, binding_id: OTHER_BINDING_ID}), false);
  assert.equal(view.sequence, 1);
});

test('stale live poses dim with explicit last-pose text without moving the camera', () => {
  const view = fakeView(); view.updatePose(pose(), {elapsedRequestMs: 20});
  assert.equal(view.sourceAgeSec, .07);
  view.advance(431); view._refreshStatus();
  assert.equal(view.label.dataset.state, 'stale');
  assert.equal(view.canvas.style.opacity, '.55');
  assert.match(view.label.textContent, /마지막 수신 자세/);
  assert.deepEqual(view.camera.position, {sentinel: 1});
  assert.equal(contract.poseAgeMs(.1, 500, 700), 300);
  assert.equal(contract.poseAgeMs(.1, 700, 500), 100, 'Local clock anomaly cannot reduce the server age');
});

test('preview mode is labeled explicitly and does not invent a live freshness age', () => {
  const view = fakeView(); view.binding = binding('model_preview'); view.updatePose(pose('model_preview'));
  view.advance(100000); view._refreshStatus();
  assert.equal(view.label.dataset.state, 'preview');
  assert.equal(view.stats.source_age_ms, null);
  assert.match(view.label.textContent, /모델 미리보기.*읽기 전용/);
});

test('changing the expected binding aborts old requests and immediately hides the old scene', () => {
  const view = fakeView(); view.updatePose(pose());
  view.stats.manifest_fetch_ms = 123;
  view.stats.rendering_fps = 60;
  let aborted = 0;
  view.loadController = {abort() {aborted++;}}; view.poseController = {abort() {aborted++;}};
  assert.equal(view.setExpectedBinding(OTHER_BINDING_ID, {...binding(), generation: 2}), true);
  assert.equal(aborted, 2); assert.equal(view.sequence, -1); assert.equal(view.stats.sequence, null);
  assert.equal(view.stats.manifest_fetch_ms, null); assert.equal(view.stats.rendering_fps, 0);
  assert.equal(view.canvas.style.visibility, 'hidden');
  assert.equal(view.world.visible, false); assert.equal(view.hasPose, false);
  assert.equal(view.setExpectedBinding(OTHER_BINDING_ID, {...binding(), generation: 2}), false);
  assert.equal(view.updatePose(pose()), false);
});

test('resource loading uses only the current origin and verifies original byte SHA and length', async () => {
  assert.equal(localURL('/api/view/assets/abc', 'http://127.0.0.1:8881/'), 'http://127.0.0.1:8881/api/view/assets/abc');
  for (const value of ['https://other.invalid/a', 'file:///private/source.xml', 'data:text/plain,a', 'http://u:p@127.0.0.1:8881/a'])
    assert.throws(() => localURL(value, 'http://127.0.0.1:8881/'));
  const view = fakeView(), {bytes} = triangle();
  const requests = [];
  view.fetch = async (url, options) => {requests.push({url, options}); return new Response(bytes.slice(0));};
  const item = await view._bytes('/asset', {maximum: 48, sha256: hash(bytes), expectedBytes: 48});
  assert.equal(item.bytes, 48); assert.equal(item.sha256, hash(bytes));
  assert.equal(requests[0].options.redirect, 'error'); assert.equal(requests[0].options.credentials, 'same-origin');
  await assert.rejects(view._bytes('/asset', {maximum: 48, sha256: '0'.repeat(64)}), /SHA256 mismatch/);
  await assert.rejects(view._bytes('/asset', {maximum: 44}), /byte limit/);
  await assert.rejects(view._bytes('/asset', {maximum: 48, expectedBytes: 44}), /length mismatch/);
});

test('manifest loading deduplicates compiled assets, swaps only complete scenes and uses no mesh offset', async () => {
  const view = fakeView(), {bytes, descriptor} = triangle();
  const data = manifest(); data.assets = [descriptor];
  for (const g of data.geoms) {g.type = 'mesh'; g.asset_sha256 = descriptor.sha256;}
  const encoded = new TextEncoder().encode(JSON.stringify(data));
  const requests = [];
  view.fetch = async url => {requests.push(url); return new Response(url.endsWith('/manifest') ? encoded : bytes.slice(0));};
  const result = await view.loadManifest('/manifest', {sha256: hash(encoded)});
  assert.equal(result.scene_sha256, data.scene_sha256);
  assert.equal(requests.length, 2, 'Repeated geoms share one mesh download');
  assert.equal(view.objects.get(0).geometry, view.objects.get(1).geometry);
  assert.deepEqual([...view.objects.get(0).geometry.position.array], [2,3,4, 3,3,4, 2,4,4]);
  assert.equal(view.world.visible, false, 'Manifest loading cannot make a stale pose look live');
  assert.equal(view.geometries.size, 1);
  assert.equal(view.needsFit, false, 'Reloading an unchanged compiled scene preserves the camera');
});

test('bad asset SHA leaves the prior model hidden and cannot publish a partial new scene', async () => {
  const view = fakeView(), previous = view.world, {bytes, descriptor} = triangle();
  const data = manifest(); data.assets = [descriptor]; data.geoms[0].type = 'mesh'; data.geoms[0].asset_sha256 = descriptor.sha256;
  const encoded = new TextEncoder().encode(JSON.stringify(data));
  const changed = bytes.slice(0); new Uint8Array(changed)[0] ^= 1;
  view.fetch = async url => new Response(url.endsWith('/manifest') ? encoded : changed);
  await assert.rejects(view.loadManifest('/manifest'), /SHA256 mismatch/);
  assert.equal(view.world, previous); assert.equal(view.world.visible, false);
  assert.match(view.sceneError, /불러오지 못했습니다/);
});

test('superseded async manifest responses cannot replace the current binding', async () => {
  const view = fakeView();
  let respond;
  view.fetch = () => new Promise(resolve => {respond = resolve;});
  const loading = view.loadManifest('/manifest');
  view.setExpectedBinding(OTHER_BINDING_ID, {...binding(), generation: 2});
  respond(new Response(JSON.stringify(manifest())));
  await assert.rejects(loading, error => error.name === 'AbortError');
  assert.equal(view.bindingID, OTHER_BINDING_ID); assert.equal(view.world.visible, false);
  assert.equal(view.sequence, -1);
});

test('no-pose responses remain unavailable and never reset the last applied sequence', async () => {
  const view = fakeView(); view.updatePose(pose());
  view.fetch = async () => new Response(null, {status: 204});
  assert.equal(await view.pollPose('/pose'), false);
  assert.equal(view.sequence, 1);
  assert.equal(view.label.dataset.state, 'stale');
});

test('cylinders/capsules use MuJoCo half-height once and rotate the local Y axis to Z', () => {
  const view = fakeView();
  const capsule = view._geometry(geom(0, 'capsule'));
  assert.deepEqual(capsule.args.slice(0,2), [1,1]); assert.equal(capsule.rotate, Math.PI/2);
  const cylinder = view._geometry(geom(0, 'cylinder'));
  assert.deepEqual(cylinder.args.slice(0,3), [1,1,1]); assert.equal(cylinder.rotate, Math.PI/2);
  assert.deepEqual(view._geometry(geom(0, 'box')).args, [2,1,.5]);
  const plane = geom(0, 'plane'); plane.size = [0,0,0];
  assert.deepEqual(view._geometry(plane).args, [10,10]);
});

test('destroy removes only owned resources and cancels outstanding work exactly once', () => {
  const view = fakeView(); let disposals = 0, removals = 0, aborts = 0;
  const countDispose = {dispose() {disposals++;}};
  view.geometries.add(countDispose); view.materials.add(countDispose);
  view.loadController = view.poseController = {abort() {aborts++;}};
  view.renderer = {setAnimationLoop(value) {assert.equal(value, null);}, dispose() {disposals++;}, forceContextLoss() {disposals++;}};
  view.resizeObserver = {disconnect() {disposals++;}}; view.document = {removeEventListener() {removals++;}};
  view.controls = countDispose;
  view.canvas.removeEventListener = () => {removals++;};
  view.canvas.remove = view.label.remove = () => {removals++;};
  view.destroy(); view.destroy();
  assert.equal(view.disposed, true); assert.equal(aborts, 2); assert.equal(disposals, 6); assert.equal(removals, 4);
});
