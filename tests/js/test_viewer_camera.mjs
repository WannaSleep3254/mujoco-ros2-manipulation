/** Real camera methods with tiny math/control mocks; no DOM/Three/WebGL import. */
import test from 'node:test';
import assert from 'node:assert/strict';
import {readFile} from 'node:fs/promises';
import * as contract from '../../src/mujoco_ros2_manipulation/operator_console/static/viewer_contract.mjs';

class Vector3 {
  constructor(x = 0, y = 0, z = 0) {this.set(x, y, z);}
  set(x, y, z) {this.x = x; this.y = y; this.z = z; return this;}
  copy(other) {return this.set(other.x, other.y, other.z);}
  clone() {return new Vector3(this.x, this.y, this.z);}
  add(other) {this.x += other.x; this.y += other.y; this.z += other.z; return this;}
  sub(other) {this.x -= other.x; this.y -= other.y; this.z -= other.z; return this;}
  length() {return Math.hypot(this.x, this.y, this.z);}
  distanceTo(other) {return this.clone().sub(other).length();}
  normalize() {return this.multiplyScalar(1 / this.length());}
  multiplyScalar(value) {this.x *= value; this.y *= value; this.z *= value; return this;}
  toArray() {return [this.x, this.y, this.z];}
}

class Box3 {
  constructor() {this.min = [Infinity, Infinity, Infinity]; this.max = [-Infinity, -Infinity, -Infinity];}
  expandByObject(object) {
    for (let axis = 0; axis < 3; axis++) {
      this.min[axis] = Math.min(this.min[axis], object.bounds.min[axis]);
      this.max[axis] = Math.max(this.max[axis], object.bounds.max[axis]);
    }
    return this;
  }
  isEmpty() {return this.min.some((value, axis) => value > this.max[axis]);}
  getCenter(target) {return target.set(...this.min.map((value, axis) => (value + this.max[axis]) / 2));}
  getSize(target) {return target.set(...this.min.map((value, axis) => this.max[axis] - value));}
}

const THREE = {Vector3, Box3, MathUtils: {degToRad: value => value * Math.PI / 180}};
const source = await readFile(new URL('../../src/mujoco_ros2_manipulation/operator_console/static/viewer.js', import.meta.url), 'utf8');
const classSource = source.slice(source.indexOf('function localURL(')).replace('export class LocalSceneView', 'class LocalSceneView');
const LocalSceneView = new Function(...Object.keys(contract), 'THREE', 'OrbitControls',
  classSource + '\nreturn LocalSceneView;')(...Object.values(contract), THREE, class {});

function view() {
  const result = Object.create(LocalSceneView.prototype);
  const camera = {position: new Vector3(4, -3, 5), up: new Vector3(0, 0, 1),
    fov: 45, aspect: 1.5, near: .01, far: 1000, updateProjectionMatrix() {this.projections = (this.projections ?? 0) + 1;}};
  const controls = {target: new Vector3(1, 2, 3), enableDamping: true, updates: 0, saved: 0,
    update() {this.updates++;}, saveState() {this.saved++;}};
  Object.assign(result, {camera, controls, disposed: false, hasPose: true, manifest: {}, loading: false,
    needsFit: false, world: {updateMatrixWorld() {}}, objects: new Map([
      [0, {visible: true, userData: {geomType: 'box'}, bounds: {min: [1, 2, 3], max: [3, 6, 5]}}],
      [1, {visible: true, userData: {geomType: 'plane'}, bounds: {min: [-100, -100, 0], max: [100, 100, 0]}}],
      [2, {visible: false, userData: {geomType: 'box'}, bounds: {min: [-50, -50, 0], max: [50, 50, 50]}}],
    ])});
  return result;
}

function close(actual, expected, tolerance = 1e-10) {assert.ok(Math.abs(actual - expected) <= tolerance, `${actual} != ${expected}`);}
function direction(value) {return value.camera.position.clone().sub(value.controls.target).normalize();}

test('renderer restart camera snapshot restores only local camera after a fresh pose', () => {
  const prior=view(), snapshot=prior.cameraSnapshot(), restarted=view();
  restarted.camera.position.set(20,20,20);restarted.controls.target.set(0,0,0);
  restarted.fetch=()=>{throw new Error('Camera restore must not request');};
  assert.equal(restarted.restoreCamera(snapshot),true);
  for(const [index,value]of restarted.camera.position.toArray().entries())close(value,snapshot.position[index]);
  assert.deepEqual(restarted.controls.target.toArray(),snapshot.target);
  assert.equal(restarted.camera.near,snapshot.near);assert.equal(restarted.camera.far,snapshot.far);
  assert.equal(restarted.needsFit,false);
  restarted.hasPose=false;assert.equal(restarted.restoreCamera(snapshot),false);
  assert.equal(restarted.cameraSnapshot(),null);
});

test('invalid restart camera snapshots cannot change current placement', () => {
  const value=view(), before=value.camera.position.toArray();
  const bad=value.cameraSnapshot();bad.far=-1;
  assert.throws(()=>value.restoreCamera(bad),/projection/);
  assert.deepEqual(value.camera.position.toArray(),before);
});

test('presets preserve current target, distance, clipping and Z-up without network calls', () => {
  const value = view(), center = value.controls.target.toArray();
  const distance = value.camera.position.clone().sub(value.controls.target).length();
  value.fetch = () => {throw new Error('Camera must not fetch');};
  for (const preset of ['overview', 'front', 'top']) {
    assert.equal(value.cameraPreset(preset), true);
    assert.deepEqual(value.controls.target.toArray(), center);
    close(value.camera.position.clone().sub(value.controls.target).length(), distance);
    assert.deepEqual(value.camera.up.toArray(), [0, 0, 1]);
    assert.equal(value.camera.near, .01); assert.equal(value.camera.far, 1000);
    assert.equal(value.controls.enableDamping, true);
    const chosen = direction(value);
    if (preset === 'front') for (const [index, expected] of [0, -1, 0].entries()) close(chosen.toArray()[index], expected);
    if (preset === 'top') {assert.ok(chosen.z > .99999); assert.ok(chosen.y < 0);}
  }
  assert.equal(value.controls.saved, 0, 'Presets do not replace the saved fit/reset target');
  assert.equal(value.controls.updates, 6, 'Damping is drained before each exact placement');
});

test('explicit fit uses visible non-plane bounds and preserves user direction', () => {
  const value = view(), before = direction(value);
  assert.equal(value.fitCamera(), true);
  assert.deepEqual(value.controls.target.toArray(), [2, 4, 4]);
  const after = direction(value);
  for (const axis of ['x','y','z']) close(after[axis], before[axis]);
  assert.deepEqual(value.camera.up.toArray(), [0, 0, 1]);
  assert.equal(value.controls.saved, 1);
  assert.equal(value.camera.projections, 1);
  assert.ok(value.camera.far > value.camera.near);
  assert.equal(value.needsFit, false);
});

test('initial fit favors visible robot visuals over a huge group-0 installation without hiding it', () => {
  const value = view();
  value.objects.get(0).userData.geomGroup = 2;
  value.objects.get(1).userData.geomGroup = 1; // A visual plane is still excluded.
  value.objects.get(2).userData.geomGroup = 1; // Invisible visuals never affect framing.
  const installation = {visible: true, userData: {geomType: 'box', geomGroup: 0},
    bounds: {min: [-1000, -2000, -10], max: [1000, 2000, 10]}};
  value.objects.set(3, installation);
  assert.equal(value._fitCamera(), true);
  assert.deepEqual(value.controls.target.toArray(), [2, 4, 4]);
  const radius = Math.sqrt(2*2 + 4*4 + 2*2) / 2;
  const distance = radius / Math.sin(Math.PI / 8) * 1.2;
  close(value.camera.position.clone().sub(value.controls.target).length(), distance);
  assert.equal(installation.visible, true, 'Initial framing does not hide installation geometry');
  assert.equal(value.objects.size, 4);
  assert.equal(value.controls.saved, 1);
  assert.equal(value.fitCamera(), true, 'User-requested fit still includes the complete visible scene');
  assert.deepEqual(value.controls.target.toArray(), [0, 0, 0]);
  assert.ok(value.camera.position.clone().sub(value.controls.target).length() > 1000);
});

test('initial single-profile fit favors group-0 meshes over floor and table boxes', () => {
  const value = view();
  value.objects.get(0).userData = {geomType: 'mesh', geomGroup: 0};
  const floor = {visible: true, userData: {geomType: 'box', geomGroup: 0},
    bounds: {min: [-100, -100, -1], max: [100, 100, 0]}};
  const table = {visible: true, userData: {geomType: 'box', geomGroup: 0},
    bounds: {min: [-5, -5, 0], max: [5, 5, 1]}};
  value.objects.set(3, floor);value.objects.set(4, table);
  assert.equal(value._fitCamera(), true);
  assert.deepEqual(value.controls.target.toArray(), [2, 4, 4]);
  const radius = Math.sqrt(2*2 + 4*4 + 2*2) / 2;
  close(value.camera.position.clone().sub(value.controls.target).length(), radius / Math.sin(Math.PI / 8) * 1.2);
  assert.equal(floor.visible, true);assert.equal(table.visible, true);
  assert.equal(value.objects.get(0).userData.geomGroup, 0, 'Framing does not reclassify the mesh');
  assert.equal(value.fitCamera(), true);
  assert.deepEqual(value.controls.target.toArray(), [0, 0, 2]);
  assert.ok(value.camera.position.clone().sub(value.controls.target).length() > 100);
});

test('initial fit falls back to all visible non-plane geometry when no robot visuals exist', () => {
  const value = view();
  value.objects.get(0).userData.geomGroup = 0;
  assert.equal(value._fitCamera(), true);
  assert.deepEqual(value.controls.target.toArray(), [2, 4, 4]);
});

test('unavailable models do not change the camera; invalid presets and destroyed views reject', () => {
  const value = view(), before = value.camera.position.toArray();
  for (const key of ['hasPose', 'manifest']) {
    const original = value[key]; value[key] = false;
    assert.equal(value.fitCamera(), false); assert.equal(value.cameraPreset(), false);
    assert.deepEqual(value.camera.position.toArray(), before);
    value[key] = original;
  }
  value.loading = true;
  assert.equal(value.fitCamera(), false); assert.equal(value.cameraPreset('front'), false);
  assert.throws(() => value.cameraPreset('rear'), /Unknown camera preset/);
  assert.throws(() => value.cameraPreset('__proto__'), /Unknown camera preset/);
  value.loading = false; value.disposed = true;
  assert.throws(() => value.cameraPreset(), /destroyed/);
  assert.throws(() => value.fitCamera(), /destroyed/);
});

test('an empty/plane-only fit leaves target and scale untouched', () => {
  const value = view(), before = value.camera.position.toArray();
  value.objects.delete(0);
  assert.equal(value.fitCamera(), false);
  assert.deepEqual(value.camera.position.toArray(), before);
  assert.deepEqual(value.controls.target.toArray(), [1, 2, 3]);
});

test('camera placements flush pending damping and restore the user damping setting', () => {
  const value = view();
  value.controls.update = function () {
    assert.equal(this.enableDamping, false);
    if (this.pending) {value.camera.position.add(new Vector3(10, 10, 10)); this.pending = false;}
  };
  value.controls.pending = true;
  const before = value.camera.position.clone().sub(value.controls.target).length();
  value.cameraPreset('front');
  for (const [index, expected] of [0,-1,0].entries()) close(direction(value).toArray()[index], expected);
  close(value.camera.position.clone().sub(value.controls.target).length(), before);
  assert.equal(value.controls.enableDamping, true);
});

test('fresh pose updates preserve a user-selected preset and its target and scale', () => {
  const value = view();
  const binding = {operator_session_id:'a'.repeat(32),generation:1,mode:'live',source_id:'fr5',
    job_id:null,run_id:null,motion_session_id:null,request_id:null};
  const id = 'b'.repeat(64), model = 'c'.repeat(64), scene = 'd'.repeat(64);
  Object.assign(value, {bindingID:id,binding,manifest:{model_sha256:model,scene_sha256:scene,geoms:[{id:0}]},
    sequence:1,now:()=>100,stats:{},poseCount:0,_refreshStatus(){}});
  value.objects.get(0).matrix = {set(){}};
  value.cameraPreset('top');
  const position = value.camera.position.toArray(), target = value.controls.target.toArray();
  const updates = value.controls.updates;
  const packet = {schema_version:1,binding_id:id,binding,sequence:2,source_state_age_sec:.01,simulation_sec:42,
    pose:{schema_version:1,model_sha256:model,scene_sha256:scene,
      geoms:[{id:0,position:[2,3,4],rotation:[1,0,0,0,1,0,0,0,1]}]}};
  assert.equal(value.updatePose(packet), true);
  assert.deepEqual(value.camera.position.toArray(), position);
  assert.deepEqual(value.controls.target.toArray(), target);
  assert.equal(value.controls.updates, updates, 'An applied pose does not refit or reset the camera');
});

test('zoom buttons preserve direction and target and use relative sensitivity-scaled distance', () => {
  const value=view(),target=value.controls.target.toArray(),before=direction(value).toArray();
  const distance=value.camera.position.clone().sub(value.controls.target).length();
  value.fetch=()=>{throw new Error('Camera zoom must not fetch');};
  value.setControlSensitivity({rotate:.5,zoom:2,pan:1.5});
  assert.equal(value.zoomCamera('in'),true);
  close(value.camera.position.clone().sub(value.controls.target).length(),distance/(1.25**2));
  direction(value).toArray().forEach((v,i)=>close(v,before[i]));
  assert.deepEqual(value.controls.target.toArray(),target);
  assert.equal(value.zoomCamera('out'),true);
  close(value.camera.position.clone().sub(value.controls.target).length(),distance);
  assert.deepEqual(value.camera.up.toArray(),[0,0,1]);
  assert.equal(value.controls.enableDamping,true);
});

test('button zoom clamps at the same near/far orbit distances and cannot pass them', () => {
  const value=view();value.controls.minDistance=2;value.controls.maxDistance=7;
  for(let i=0;i<30;i++)value.zoomCamera('in');
  close(value.camera.position.clone().sub(value.controls.target).length(),2);
  assert.equal(value.zoomCamera('in'),false);
  for(let i=0;i<30;i++)value.zoomCamera('out');
  close(value.camera.position.clone().sub(value.controls.target).length(),7);
  assert.equal(value.zoomCamera('out'),false);
  const before=value.camera.position.toArray();value.loading=true;
  assert.equal(value.zoomCamera('in'),false);assert.deepEqual(value.camera.position.toArray(),before);
  assert.throws(()=>value.zoomCamera('sideways'),/Unknown zoom/);
  value.disposed=true;assert.throws(()=>value.zoomCamera('in'),/destroyed/);
});

test('sensitivity updates are separate, bounded and atomic and never move the camera', () => {
  const value=view(),before=value.camera.position.toArray();
  assert.deepEqual(value.setControlSensitivity({rotate:.25,zoom:3,pan:1.5}),{rotate:.25,zoom:3,pan:1.5});
  for(const invalid of [null,[],{rotate:1,zoom:1},{rotate:1,zoom:1,pan:1,extra:1},
      {rotate:NaN,zoom:1,pan:1},{rotate:1,zoom:Infinity,pan:1},{rotate:1,zoom:1,pan:0},
      {rotate:3.01,zoom:1,pan:1},{rotate:true,zoom:1,pan:1}]){
    assert.throws(()=>value.setControlSensitivity(invalid),/sensitivity/);
    assert.deepEqual(value.getControlSensitivity(),{rotate:.25,zoom:3,pan:1.5});
  }
  assert.deepEqual(value.camera.position.toArray(),before);
  const copy=value.getControlSensitivity();copy.zoom=1;
  assert.equal(value.getControlSensitivity().zoom,3);
});

test('default view re-fits the current model in overview without resetting sensitivity', () => {
  const value=view();value.setControlSensitivity({rotate:2,zoom:.5,pan:1.25});
  value.cameraPreset('top');value.zoomCamera('in');
  assert.equal(value.resetCamera(),true);
  assert.deepEqual(value.controls.target.toArray(),[2,4,4]);
  const expected=new Vector3(1,-1,.7).normalize().toArray();
  direction(value).toArray().forEach((v,i)=>close(v,expected[i]));
  assert.deepEqual(value.getControlSensitivity(),{rotate:2,zoom:.5,pan:1.25});
  value.manifest=null;const before=value.camera.position.toArray();
  assert.equal(value.resetCamera(),false);assert.deepEqual(value.camera.position.toArray(),before);
});

test('a new binding blanks the old pose without changing user camera sensitivities', () => {
  const value=view();value.setControlSensitivity({rotate:1.75,zoom:.75,pan:2.5});
  Object.assign(value,{now:()=>100,_refreshStatus(){},loadGeneration:0});
  const binding={operator_session_id:'a'.repeat(32),generation:2,mode:'model_preview',source_id:'profile:fr5',
    job_id:null,run_id:null,motion_session_id:null,request_id:null};
  value.setExpectedBinding('b'.repeat(64),binding);
  assert.equal(value.hasPose,false);
  assert.deepEqual(value.getControlSensitivity(),{rotate:1.75,zoom:.75,pan:2.5});
});
