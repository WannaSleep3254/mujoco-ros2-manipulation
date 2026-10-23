/** Null epochs reject late old responses without GL, DOM, or a fake identity. */
import test from 'node:test';
import assert from 'node:assert/strict';
import {readFile} from 'node:fs/promises';
import * as contract from '../../src/mujoco_ros2_manipulation/operator_console/static/viewer_contract.mjs';
const source=await readFile(new URL('../../src/mujoco_ros2_manipulation/operator_console/static/viewer.js',import.meta.url),'utf8');
const body=source.slice(source.indexOf('function localURL(')).replace('export class LocalSceneView','class LocalSceneView');
const LocalSceneView=new Function(...Object.keys(contract),'THREE','OrbitControls',body+'\nreturn LocalSceneView;')(...Object.values(contract),{},class{});
const binding={operator_session_id:'a'.repeat(32),generation:1,mode:'live',source_id:'fr5',job_id:null,run_id:null,motion_session_id:null,request_id:null};
const id='b'.repeat(64);
function view(){
  const value=Object.create(LocalSceneView.prototype);
  Object.assign(value,{disposed:false,bindingID:id,binding:{...binding},loadGeneration:1,loading:false,
    loadController:null,poseController:null,manifest:{},hasPose:true,world:{visible:true},sequence:12,
    sourceAgeSec:.01,receivedAt:1,now:()=>1,stats:{},statsAt:1,renderCount:1,poseCount:1,
    sceneError:null,manualStaleReason:null,staleAfterMs:500,canvas:{style:{}},label:{dataset:{},textContent:''},
    camera:{sentinel:'keep user camera'}});
  return value;
}
test('clearBinding aborts outstanding work, hides poses and keeps camera and geometry',()=>{
  const value=view();let aborted=0;
  value.loadController=value.poseController={abort(){aborted++;}};
  const camera=value.camera,world=value.world;
  value.clearBinding('새 시연 준비 중');
  assert.equal(aborted,2);assert.equal(value.bindingID,null);assert.equal(value.binding,null);
  assert.equal(value.loadController,null);assert.equal(value.poseController,null);
  assert.equal(value.sequence,-1);assert.equal(value.hasPose,false);assert.equal(value.world.visible,false);
  assert.equal(value.camera,camera);assert.equal(value.world,world);
  assert.equal(value.label.textContent,'새 시연 준비 중');assert.equal(value.stats.rendering_fps,0);
  assert.equal(value.updatePose({binding_id:id}),false);
});
test('late old pose polling cannot make an unavailable epoch fresh again',async()=>{
  const value=view();let resolve;
  value._json=()=>new Promise(done=>{resolve=done;});
  const pending=value.pollPose('/old-pose');
  value.clearBinding();
  resolve({data:{schema_version:1,binding_id:id,binding,sequence:13,source_state_age_sec:0}});
  assert.equal(await pending,false);
  assert.equal(value.bindingID,null);assert.equal(value.hasPose,false);assert.equal(value.world.visible,false);
  assert.equal(value.sequence,-1);assert.notEqual(value.label.dataset.state,'fresh');
});
test('late old manifest response cannot populate an unavailable epoch',async()=>{
  const value=view();let resolve;
  value._json=()=>new Promise(done=>{resolve=done;});
  const pending=value.loadManifest('/old-manifest');
  value.clearBinding();
  const manifest={schema_version:1,export_version:'compiled-scene-v1',model_sha256:'1'.repeat(64),scene_sha256:'2'.repeat(64),compiled_model_sha256:'3'.repeat(64),
    units:{length:'m',angle:'rad',up_axis:'Z',rotation:'row-major'},joint_names:[],home_rad:{},default_visible_groups:[0,1,2],
    geoms:[{id:0,name:'box',type:'box',group:0,default_visible:true,size:[1,1,1],rgba:[1,1,1,1],asset_sha256:null}],assets:[]};
  resolve({data:manifest,sha256:'4'.repeat(64),fetchMs:0,parseMs:0,verifyMs:0});
  await assert.rejects(pending,error=>error.name==='AbortError');
  assert.equal(value.bindingID,null);assert.equal(value.hasPose,false);assert.equal(value.world.visible,false);
});
