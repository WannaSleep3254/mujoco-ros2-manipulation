/** Local geometry inspection never changes source buffers, physics or commands. */
import test from 'node:test';
import assert from 'node:assert/strict';
import {readFile} from 'node:fs/promises';
import * as contract from '../../src/mujoco_ros2_manipulation/operator_console/static/viewer_contract.mjs';

class Vector3 {
 constructor(x=0,y=0,z=0){this.set(x,y,z);}
 set(x,y,z){Object.assign(this,{x,y,z});return this;}
 copy(v){return this.set(v.x,v.y,v.z);}
 clone(){return new Vector3(this.x,this.y,this.z);}
 add(v){return this.set(this.x+v.x,this.y+v.y,this.z+v.z);}
 sub(v){return this.set(this.x-v.x,this.y-v.y,this.z-v.z);}
 length(){return Math.hypot(this.x,this.y,this.z);}
 normalize(){return this.multiplyScalar(1/this.length());}
 multiplyScalar(v){return this.set(this.x*v,this.y*v,this.z*v);}
 toArray(){return [this.x,this.y,this.z];}
}
class Box3 {
 constructor(){this.min=[Infinity,Infinity,Infinity];this.max=[-Infinity,-Infinity,-Infinity];}
 expandByObject(object){for(let axis=0;axis<3;axis++){this.min[axis]=Math.min(this.min[axis],object.bounds.min[axis]);this.max[axis]=Math.max(this.max[axis],object.bounds.max[axis]);}return this;}
 isEmpty(){return this.min.some((v,i)=>v>this.max[i]);}
 getCenter(target){return target.set(...this.min.map((v,i)=>(v+this.max[i])/2));}
 getSize(target){return target.set(...this.min.map((v,i)=>this.max[i]-v));}
}
const THREE={Vector3,Vector2:class{constructor(x,y){Object.assign(this,{x,y});}},Box3,MathUtils:{degToRad:v=>v*Math.PI/180}};
const source=await readFile(new URL('../../src/mujoco_ros2_manipulation/operator_console/static/viewer.js',import.meta.url),'utf8');
const body=source.slice(source.indexOf('function localURL(')).replace('export class LocalSceneView','class LocalSceneView');
const LocalSceneView=new Function(...Object.keys(contract),'THREE','OrbitControls',body+'\nreturn LocalSceneView;')(...Object.values(contract),THREE,class{});
const id='b'.repeat(64),binding={operator_session_id:'a'.repeat(32),generation:1,mode:'model_preview',source_id:'manual',job_id:null,run_id:null,motion_session_id:null,request_id:null};

function fixture(){
 const sharedGeometry=Object.freeze({buffer:'shared immutable mesh'}),notices=[],view=Object.create(LocalSceneView.prototype);
 const objects=new Map();
 for(let n=0;n<5;n++){
  const object={name:n<2?'':`geom-${n}`,geometry:sharedGeometry,position:[n,0,0],visible:true,
   userData:{geomId:n,geomType:n===3?'plane':'box',geomGroup:n===2?3:0,alpha:n===0?.4:n===4?0:1},
   material:{opacity:1,transparent:false,depthWrite:true,emissive:{setRGB(...values){this.rgb=values;}}}};
  object.matrix={set:(...values)=>{object.position=[values[3],values[7],values[11]];}};objects.set(n,object);
 }
 let matrixUpdates=0;
 Object.assign(view,{disposed:false,options:{onGeometryChange:value=>notices.push(value)},bindingID:id,binding:{...binding},
  catalogBindingID:id,geometryEpoch:1,reviewMode:false,displayOverrides:new Map(),selectedGeometryID:null,objects,
  hasPose:true,loading:false,sceneError:null,contextLost:false,sized:true,document:{hidden:false},sequence:0,poseCount:0,sourceSample:null,
  manifest:{model_sha256:'1'.repeat(64),scene_sha256:'2'.repeat(64),geoms:[...objects].map(([id])=>({id})),joint_names:[]},
  now:()=>1000,stats:{},statsAt:1000,staleAfterMs:500,needsFit:false,canvas:{style:{},getBoundingClientRect:()=>({left:10,top:20,right:210,bottom:120,width:200,height:100})},label:{dataset:{},textContent:''},
  world:{visible:true,updateMatrixWorld(){matrixUpdates++;for(const object of objects.values())object.bounds={min:object.position.map(v=>v-.5),max:object.position.map(v=>v+.5)};}},
  camera:{position:new Vector3(4,-3,5),up:new Vector3(0,0,1),fov:45,aspect:2,updateProjectionMatrix(){},updateMatrixWorld(){}},
  controls:{target:new Vector3(0,0,0),enableDamping:true,update(){return false;},saveState(){}},
  fetch(){throw new Error('Inspection must not send a request');}});
 for(const object of objects.values())view._styleGeometry(object);
 return {view,objects,notices,sharedGeometry,get matrixUpdates(){return matrixUpdates;}};
}
function pose(view,sequence,position){return {schema_version:1,binding_id:view.bindingID,binding:{...view.binding},sequence,source_state_age_sec:null,simulation_sec:null,
 pose:{schema_version:1,model_sha256:'1'.repeat(64),scene_sha256:'2'.repeat(64),geoms:[...view.objects].map(([id,object])=>({id,position:id===0?position:object.position,rotation:[1,0,0,0,1,0,0,0,1]}))}};}
const pointer=(pointerId=1,x=50,y=50,extra={})=>({pointerId,clientX:x,clientY:y,button:0,isPrimary:true,...extra});

test('catalog snapshots distinguish unnamed instances sharing a buffer and selection highlights only one material',()=>{
 const {view,objects,sharedGeometry}=fixture(),catalog=view.geometryCatalog();
 assert.deepEqual(catalog.items.slice(0,2).map(item=>[item.id,item.name]),[[0,''],[1,'']]);
 catalog.items[0].visible=false;assert.equal(objects.get(0).visible,true);
 assert.equal(view.selectGeometry(0,catalog.scope_key),true);assert.deepEqual(objects.get(0).material.emissive.rgb,[.3,.12,.015]);
 assert.deepEqual(objects.get(1).material.emissive.rgb,[0,0,0]);assert.equal(objects.get(0).geometry,sharedGeometry);assert.equal(objects.get(1).geometry,sharedGeometry);
 assert.equal(view.selectGeometry(1,catalog.scope_key),true);assert.deepEqual(objects.get(0).material.emissive.rgb,[0,0,0]);
 assert.equal(view.geometryCatalog().selected_id,1);assert.equal(view.selectGeometry(null,catalog.scope_key),true);
});

test('visibility composes collision defaults, per-instance hiding and original alpha without revealing zero-alpha geometry',()=>{
 const {view,objects}=fixture(),scope=view.geometryCatalog().scope_key;
 assert.equal(view.setGeometryOpacity(0,.5,scope),true);assert.equal(objects.get(0).material.opacity,.2);assert.equal(objects.get(1).material.opacity,1);
 assert.equal(objects.get(0).material.transparent,true);assert.equal(objects.get(0).material.depthWrite,false);
 view.setGeometryVisible(0,false,scope);view.setCollisionVisible(true);assert.equal(objects.get(0).visible,false);assert.equal(objects.get(2).visible,true);
 view.setGeometryVisible(2,false,scope);view.setCollisionVisible(false);view.setCollisionVisible(true);assert.equal(objects.get(2).visible,false);
 view.setGeometryOpacity(4,1,scope);view.setGeometryVisible(4,true,scope);assert.equal(objects.get(4).visible,false);
 assert.throws(()=>view.setGeometryOpacity(0,NaN,scope));assert.throws(()=>view.setGeometryOpacity(0,1.1,scope));assert.throws(()=>view.setGeometryVisible(0,1,scope));
 assert.equal(view.resetDisplayOverrides(scope),true);assert.equal(objects.get(0).visible,true);assert.equal(objects.get(0).material.opacity,.4);assert.equal(objects.get(2).visible,true);
});

test('review entry restores display overrides and locks hiding/opacity while retaining read-only selection',()=>{
 const {view,objects}=fixture(),scope=view.geometryCatalog().scope_key;
 view.setGeometryVisible(0,false,scope);view.setGeometryOpacity(1,.2,scope);view.setReviewMode(true);
 assert.equal(objects.get(0).visible,true);assert.equal(objects.get(1).material.opacity,1);assert.equal(view.geometryCatalog().review_mode,true);
 assert.equal(view.setGeometryVisible(0,false,scope),false);assert.equal(view.setGeometryOpacity(0,0,scope),false);
 assert.equal(view.selectGeometry(0,scope),true);assert.equal(view.setReviewMode(true),false);assert.equal(view.geometryCatalog().selected_id,0);
 view.setReviewMode(false);view.binding.mode='plan_preview';assert.equal(view.setGeometryVisible(0,false,scope),false,'A caller cannot unlock an active plan preview');
});

test('a new or absent binding clears selection, overrides and gestures and rejects old callbacks even when geom IDs repeat',()=>{
 const {view}=fixture(),old=view.geometryCatalog().scope_key;view.selectGeometry(0,old);view.setGeometryVisible(0,false,old);view._pickDown(pointer());
 view.setExpectedBinding('c'.repeat(64),{...binding,generation:2});
 assert.equal(view.selectedGeometryID,null);assert.equal(view.displayOverrides.size,0);assert.equal(view.pickGesture,null);assert.equal(view.geometryCatalog().items.length,0);
 view.catalogBindingID=view.bindingID;
 assert.equal(view.setGeometryVisible(0,false,old),false);assert.equal(view.selectGeometry(0,old),false);assert.equal(view.fitSelected(old),false);
 view.clearBinding();assert.equal(view.geometryCatalog().scope_key,null);assert.equal(view.geometryCatalog().items.length,0);assert.equal(view.selectGeometry(0),false);
});

test('selected fit uses the latest world pose and ignores hidden or infinite-plane selections without moving the camera',()=>{
 const {view}=fixture(),scope=view.geometryCatalog().scope_key;view.selectGeometry(0,scope);
 view.updatePose(pose(view,1,[12,3,2]));assert.equal(view.fitSelected(scope),true);assert.deepEqual(view.controls.target.toArray(),[12,3,2]);
 const camera=view.camera.position.toArray();view.setGeometryVisible(0,false,scope);assert.equal(view.fitSelected(scope),false);assert.deepEqual(view.camera.position.toArray(),camera);
 view.selectGeometry(3,scope);assert.equal(view.fitSelected(scope),false);assert.deepEqual(view.camera.position.toArray(),camera);
});

test('picking requires a primary click with maximum excursion at most four CSS pixels and rejects drag-back and multitouch',()=>{
 const {view}=fixture(),calls=[];view._pickAt=(...args)=>calls.push(args);
 view._pickDown(pointer());view._pickMove(pointer(1,55,50));view._pickMove(pointer());view._pickUp(pointer());assert.equal(calls.length,0);
 view._pickDown(pointer());view._pickUp(pointer(1,54,50));assert.equal(calls.length,1);
 view._pickDown(pointer());view._pickDown(pointer(2,50,50,{isPrimary:false}));view._pickUp(pointer());assert.equal(calls.length,1);
 view._pickUp(pointer(2));view._pickDown(pointer(3,50,50,{button:2}));view._pickUp(pointer(3,50,50,{button:2}));assert.equal(calls.length,1);
 view._pickDown(pointer(4));view._cancelPicking();view._pickUp(pointer(4));assert.equal(calls.length,1);
});

test('ray picking updates world matrices and filters invisible geometry/planes while background clicks preserve selection',()=>{
 const h=fixture(),{view,objects}=h,scope=view.geometryCatalog().scope_key;let candidates;
 view.raycaster={setFromCamera(pointer){assert.equal(pointer.x,0);assert.equal(pointer.y,0);},intersectObjects(items){candidates=items;return [{object:objects.get(0)}];}};
 assert.equal(view._pickAt(110,70,scope),true);assert.ok(h.matrixUpdates>0);assert.deepEqual(candidates.map(item=>item.userData.geomId),[0,1]);
 view.raycaster.intersectObjects=()=>[];assert.equal(view._pickAt(110,70,scope),false);assert.equal(view.selectedGeometryID,0);
 view.document.hidden=true;assert.equal(view._pickAt(110,70,scope),false);
});
