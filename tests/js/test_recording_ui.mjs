/** Recording scheduling and stale-response boundaries. No DOM, GL, ROS or network. */
import test from 'node:test';
import assert from 'node:assert/strict';
import {readFile} from 'node:fs/promises';

const source=(await readFile(new URL('../../src/mujoco_ros2_manipulation/operator_console/static/recording.js',import.meta.url),'utf8'))
  .replace(/^import[^\n]*\n/,'').replace(/\nupdate\(\);\s*$/,'');
function deferred(){let resolve,reject;const promise=new Promise((yes,no)=>{resolve=yes;reject=no;});return {resolve,reject,promise};}
function view(character='a',generation=1,extra={}){return {binding_id:character.repeat(64),binding:{generation},source:{label:'Fixture'},
  duration_sec:8,position_sec:0,playing:false,sequence:1,
  manifest_descriptor:{sha256:'0'.repeat(64),model_sha256:'1'.repeat(64),scene_sha256:'2'.repeat(64)},...extra};}
function response(value){return {ok:true,json:async()=>value};}
async function drain(){for(let i=0;i<30;i++)await Promise.resolve();}

function harness(){
  const elements=new Map(),requests=[],timers=new Map(),documentEvents={},windowEvents={};
  let timerID=0,activeGets=0,maximumGets=0;
  const document={activeElement:null,hidden:false,addEventListener:(name,callback)=>{documentEvents[name]=callback;},
    getElementById(id){if(!elements.has(id))elements.set(id,{textContent:'',value:'',disabled:false,addEventListener(){}});return elements.get(id);}};
  const window={addEventListener:(name,callback)=>{windowEvents[name]=callback;}};
  class FakeView {
    constructor(){this.bindings=[];this.loads=[];this.polls=[];this.clears=0;this.hasPose=false;this.sequence=-1;this.nextSequence=1;this.poseEpoch=0;this.cancelledReads=0;this.destroyed=0;}
    setExpectedBinding(id){this.binding=id;this.bindings.push(id);this.hasPose=false;this.sequence=-1;}
    clearBinding(){this.binding=null;this.clears++;this.hasPose=false;this.sequence=-1;}
    async loadManifest(url){this.loads.push(url);}
    async pollPose(url){
      const item={url,epoch:this.poseEpoch,binding:this.binding};this.polls.push(item);
      const cancellation=deferred();this.pendingPose=cancellation;
      const answer=await Promise.race([this.nextPose||Promise.resolve(true),cancellation.promise]);
      if(this.pendingPose===cancellation)this.pendingPose=null;
      this.nextPose=null;
      if(item.epoch!==this.poseEpoch||item.binding!==this.binding||this.destroyed)return false;
      if(answer){this.hasPose=true;this.sequence=this.nextSequence;}
      return answer;
    }
    cancelPoseRead(){this.cancelledReads++;this.poseEpoch++;this.pendingPose?.resolve(false);this.pendingPose=null;}
    markStale(reason){this.stale=reason;}
    destroy(){this.destroyed++;this.cancelPoseRead();}
  }
  const api=new Function('LocalSceneView','document','window','fetch','setTimeout','clearTimeout',source+'\nreturn {control,update,viewer};')(
    FakeView,document,window,(url,options={})=>{
      const item={...deferred(),url,options};requests.push(item);
      if(options.method!=='POST'){
        activeGets++;maximumGets=Math.max(maximumGets,activeGets);let finished=false;
        const done=()=>{if(!finished){finished=true;activeGets--;}};
        options.signal?.addEventListener('abort',done,{once:true});item.promise.then(done,done);
      }
      return item.promise;
    },(callback,delay)=>{const id=++timerID;timers.set(id,{callback,delay});return id;},id=>timers.delete(id));
  return {...api,requests,timers,document,element:id=>document.getElementById(id),
    get maximumGets(){return maximumGets;},
    runTimer(delay){const item=[...timers].find(([,value])=>value.delay===delay);assert.ok(item,'Expected timer '+delay+' ms');timers.delete(item[0]);item[1].callback();},
    visibility(hidden){document.hidden=hidden;documentEvents.visibilitychange();},
    pagehide(){windowEvents.pagehide();}};
}
async function boot(h,initial=view()){
  const started=h.update();h.requests.at(-1).resolve(response({view:initial,token:'fixture-token'}));await started;
}
async function tick(h,next=view(),delay=1000){h.runTimer(delay);h.requests.at(-1).resolve(response({view:next,token:'fixture-token'}));await drain();}
function stateReads(h){return h.requests.filter(item=>item.url==='/api/recording');}
function posts(h){return h.requests.filter(item=>item.options.method==='POST');}

test('visible playback polls at 20 Hz, while a paused recording reuses its last pose',async()=>{
  const h=harness();await boot(h);
  assert.equal(h.viewer.polls.length,1);assert.equal(h.timers.size,1);assert.equal([...h.timers.values()][0].delay,1000);
  await tick(h);await tick(h);
  assert.equal(h.viewer.polls.length,1,'Unchanged paused state must not fetch the same pose again');
  await tick(h,view('a',1,{playing:true}));
  assert.equal(h.viewer.polls.length,2);assert.equal([...h.timers.values()][0].delay,50);
  await tick(h,view('a',1,{playing:true}),50);
  assert.equal(h.viewer.polls.length,3);assert.equal(h.maximumGets,1);
  assert.equal(posts(h).length,0);
});

test('a new paused sequence or a missing displayed pose requires a single refresh',async()=>{
  const h=harness();await boot(h);h.viewer.nextSequence=2;
  await tick(h,view('a',1,{sequence:2,position_sec:1}));
  assert.equal(h.viewer.polls.length,2);assert.equal(h.viewer.sequence,2);
  await tick(h,view('a',1,{sequence:2,position_sec:1}));assert.equal(h.viewer.polls.length,2);
  h.viewer.hasPose=false;await tick(h,view('a',1,{sequence:2}));assert.equal(h.viewer.polls.length,3);
});

test('seek acknowledgement changes binding immediately and a late state body cannot restore the old view',async()=>{
  const h=harness();await boot(h);
  const body=deferred(),late=h.update();stateReads(h).at(-1).resolve({ok:true,json:()=>body.promise});await drain();
  const seek=h.control('seek',0);assert.equal(h.viewer.binding,null);assert.equal(h.viewer.clears,1);
  const command=posts(h)[0];assert.deepEqual(JSON.parse(command.options.body),{action:'seek',time_sec:0});
  command.resolve(response(view('b',2)));await seek;await late;
  body.resolve({view:view('a',1),token:'old-token'});await drain();
  assert.equal(h.viewer.binding,'b'.repeat(64));assert.deepEqual(h.viewer.bindings,['a'.repeat(64),'b'.repeat(64)]);
  h.runTimer(0);stateReads(h).at(-1).resolve(response({view:view('b',2),token:'fixture-token'}));await drain();
  assert.equal(h.viewer.loads.length,2);assert.equal(h.viewer.polls.length,2);assert.equal(h.maximumGets,1);
});

test('pause acknowledgement refreshes the final pose without clearing the last visible posture',async()=>{
  const h=harness();await boot(h,view('a',1,{playing:true}));
  const paused=h.control('pause');assert.equal(h.element('recording-pause').disabled,true);
  await h.control('play');assert.equal(posts(h).length,1,'Busy controls must not issue a second POST');
  posts(h)[0].resolve(response(view('a',1,{position_sec:1,sequence:2})));await paused;
  assert.equal(h.viewer.clears,0);assert.equal(h.viewer.binding,'a'.repeat(64));
  h.viewer.nextSequence=2;await tick(h,view('a',1,{position_sec:1,sequence:2}),0);
  assert.equal(h.viewer.polls.length,2);assert.equal(h.viewer.sequence,2);
  await tick(h,view('a',1,{position_sec:1,sequence:2}));assert.equal(h.viewer.polls.length,2);
  assert.deepEqual(posts(h).map(item=>item.url),['/api/recording/control']);
});

test('an in-flight old pose is cancelled by pause and cannot consume its fresh-pose requirement',async()=>{
  const h=harness();await boot(h,view('a',1,{playing:true}));
  const oldPose=deferred();h.viewer.nextPose=oldPose.promise;
  const oldUpdate=h.update();stateReads(h).at(-1).resolve(response({view:view('a',1,{playing:true}),token:'fixture-token'}));await drain();
  const paused=h.control('pause');posts(h)[0].resolve(response(view('a',1,{sequence:2,position_sec:1})));await paused;await oldUpdate;
  h.viewer.nextSequence=2;await tick(h,view('a',1,{sequence:2,position_sec:1}),0);
  const count=h.viewer.polls.length;oldPose.resolve(true);await drain();
  assert.equal(h.viewer.sequence,2);assert.equal(count,3);assert.equal(h.timers.size,1);
});

test('a paused control acknowledgement still requests one fresh GET even when its sequence is unchanged',async()=>{
  const h=harness();await boot(h);
  const paused=h.control('pause');posts(h)[0].resolve(response(view()));await paused;
  await tick(h,view(),0);assert.equal(h.viewer.polls.length,2);
  await tick(h);assert.equal(h.viewer.polls.length,2);
});

test('state deadline includes a stalled body and a later old body cannot change the recovered scene',async()=>{
  const h=harness();await boot(h);
  const body=deferred(),pending=h.update(),request=stateReads(h).at(-1);
  request.resolve({ok:true,json:()=>body.promise});await drain();h.runTimer(3000);await pending;
  assert.equal(request.options.signal.aborted,true);assert.match(h.element('recording-error').textContent,/초과/);
  await tick(h,view('b',2));assert.equal(h.viewer.binding,'b'.repeat(64));
  body.resolve({view:view('a',1),token:'old-token'});await drain();assert.equal(h.viewer.binding,'b'.repeat(64));
  assert.equal(h.maximumGets,1);assert.equal(h.timers.size,1);
});

test('a state request that never receives headers is aborted and does not block the next update',async()=>{
  const h=harness();const pending=h.update(),request=stateReads(h)[0];
  h.runTimer(3000);await pending;
  assert.ok(request.options.signal instanceof AbortSignal);
  assert.equal(request.options.signal.aborted,true);assert.equal(request.options.signal.reason.name,'TimeoutError');
  assert.match(h.element('recording-error').textContent,/초과/);
  await tick(h);assert.equal(h.viewer.polls.length,1);assert.equal(h.maximumGets,1);
  assert.equal(posts(h).length,0);
});

test('visibility transitions keep one update loop and hidden playback defers geometry and pose work',async()=>{
  const h=harness();h.document.hidden=true;await boot(h,view('a',1,{playing:true}));
  assert.equal(h.viewer.loads.length,0);assert.equal(h.viewer.polls.length,0);assert.equal([...h.timers.values()][0].delay,1000);
  await tick(h,view('a',1,{playing:true}));assert.equal(h.viewer.loads.length,0);
  const inFlight=h.update();
  h.visibility(false);h.visibility(true);h.visibility(false);
  assert.equal(stateReads(h).length,3);assert.equal(h.maximumGets,1);
  stateReads(h).at(-1).resolve(response({view:view('a',1,{playing:true}),token:'fixture-token'}));await inFlight;
  assert.equal(h.timers.size,1);assert.equal([...h.timers.values()][0].delay,0);
  await tick(h,view('a',1,{playing:true}),0);
  assert.equal(h.viewer.loads.length,1);assert.equal(h.viewer.polls.length,2);
  assert.equal([...h.timers.values()][0].delay,50);
  assert.equal(posts(h).length,0,'Hiding the tab never pauses the source automatically');
});

test('play at EOF accepts the new binding and loads its initial scene and pose',async()=>{
  const h=harness();await boot(h,view('a',1,{position_sec:8}));
  const playing=h.control('play');posts(h)[0].resolve(response(view('b',2,{playing:true})));await playing;
  assert.equal(h.viewer.binding,'b'.repeat(64));await tick(h,view('b',2,{playing:true}),0);
  assert.equal(h.viewer.loads.length,2);assert.equal(h.viewer.polls.length,2);
  assert.equal([...h.timers.values()][0].delay,50);
});

test('pagehide aborts a stalled read and late read or POST responses never touch the destroyed view',async()=>{
  for(const lateKind of ['read','success','failure']){
    const h=harness();await boot(h);let pending,late;
    if(lateKind==='read'){
      pending=h.update();late=stateReads(h).at(-1);
    }else{pending=h.control('seek',0);late=posts(h)[0];}
    h.pagehide();const before={binding:h.viewer.binding,loads:h.viewer.loads.length,error:h.element('recording-error').textContent};
    if(lateKind==='read')assert.equal(late.options.signal.aborted,true);
    if(lateKind==='failure')late.reject(new Error('Late transport failure'));
    else late.resolve(response(lateKind==='read'?{view:view('b',2),token:'late'}:view('b',2)));
    await pending;await drain();await h.update();await h.control('play');
    assert.equal(h.viewer.destroyed,1);assert.equal(h.viewer.binding,before.binding);assert.equal(h.viewer.loads.length,before.loads);
    assert.equal(h.element('recording-error').textContent,before.error);assert.equal(h.timers.size,0);
  }
});

test('a failed control is never retried and only a fresh state GET is scheduled',async()=>{
  const h=harness();await boot(h);const paused=h.control('pause');posts(h)[0].reject(new Error('Uncertain response'));await paused;
  assert.match(h.element('recording-error').textContent,/Uncertain/);await tick(h,view(),0);await tick(h);
  assert.equal(posts(h).length,1);assert.equal(posts(h)[0].url,'/api/recording/control');
});
