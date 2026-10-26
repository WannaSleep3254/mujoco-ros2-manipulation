/** Late recording responses must not restore a pre-seek view. No DOM/GL/ROS. */
import test from 'node:test';
import assert from 'node:assert/strict';
import {readFile} from 'node:fs/promises';

const source=(await readFile(new URL('../../src/mujoco_ros2_manipulation/operator_console/static/recording.js',import.meta.url),'utf8'))
  .replace(/^import[^\n]*\n/,'').replace(/\nupdate\(\);\s*$/,'');
function deferred(){let resolve;const promise=new Promise(done=>{resolve=done;});return {resolve,promise};}
function view(character,generation){return {binding_id:character.repeat(64),binding:{generation},source:{label:'Fixture'},
  duration_sec:8,position_sec:0,manifest_descriptor:{sha256:'0'.repeat(64),model_sha256:'1'.repeat(64),scene_sha256:'2'.repeat(64)}};}
function response(value){return {ok:true,json:async()=>value};}
function harness(){
  const elements=new Map(),requests=[];
  const document={activeElement:null,getElementById(id){if(!elements.has(id))elements.set(id,{addEventListener(){}});return elements.get(id);}};
  class FakeView {
    constructor(){this.bindings=[];this.loads=[];this.polls=[];this.clears=0;}
    setExpectedBinding(id){this.binding=id;this.bindings.push(id);}
    clearBinding(){this.binding=null;this.clears++;}
    async loadManifest(url){this.loads.push(url);}
    async pollPose(url){this.polls.push(url);}
    markStale(reason){this.stale=reason;}
    destroy(){}
  }
  const api=new Function('LocalSceneView','document','window','fetch','setTimeout',source+'\nreturn {control,update,viewer};')(
    FakeView,document,{addEventListener(){}},(url,options)=>{const item={...deferred(),url,options};requests.push(item);return item.promise;},()=>{});
  return {...api,requests};
}
async function boot(h){const started=h.update();h.requests[0].resolve(response({view:view('a',1),token:'fixture-token'}));await started;}

test('seek acknowledgment switches binding immediately and rejects a late state response',async()=>{
  const h=harness();await boot(h);
  const late=h.update();const seek=h.control('seek',0);
  assert.equal(h.viewer.binding,null);
  assert.equal(h.viewer.clears,1);
  const command=h.requests.find(item=>item.options?.method==='POST');
  assert.equal(command.url,'/api/recording/control');assert.deepEqual(JSON.parse(command.options.body),{action:'seek',time_sec:0});
  command.resolve(response(view('b',2)));await seek;
  assert.equal(h.viewer.binding,'b'.repeat(64));
  h.requests[1].resolve(response({view:view('a',1),token:'fixture-token'}));await late;
  assert.equal(h.viewer.binding,'b'.repeat(64));
  assert.deepEqual(h.viewer.bindings,['a'.repeat(64),'b'.repeat(64)]);
});

test('pause retains the selected last pose and playback never calls a robot command endpoint',async()=>{
  const h=harness();await boot(h);
  const paused=h.control('pause');h.requests[1].resolve(response(view('a',1)));await paused;
  assert.equal(h.viewer.clears,0);assert.equal(h.viewer.binding,'a'.repeat(64));
  assert.deepEqual(h.requests.filter(item=>item.options?.method==='POST').map(item=>item.url),['/api/recording/control']);
});
