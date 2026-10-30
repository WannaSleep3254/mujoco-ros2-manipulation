/** Same-workspace task identity, uncertain delivery and separate native facts. */
import test from 'node:test';
import assert from 'node:assert/strict';
import {WorkspaceTaskControls} from '../../src/mujoco_ros2_manipulation/operator_console/static/task_controls.mjs';
import {graspEvidence,graspPresentation} from '../../src/mujoco_ros2_manipulation/operator_console/static/workspace_status.mjs';
const SEL='a'.repeat(32),NEXT='b'.repeat(32),MODEL='c'.repeat(64);
const snapshot=(extra={})=>({history:[],task:{selected:'grasp',selection_id:SEL,options:[{id:'motion',label:'관절·TCP',available:true},{id:'grasp',label:'집기·운반·놓기',available:true}],can_select:true,can_start:true,reason:'',grasp:{source:{id:'scene:ros_fr5_grasp',model_sha256:MODEL,manifest_sha256:'f'.repeat(64),domain:230,label:'FR5 grasp'},job:null,view:{mode:'model_preview',binding_id:'p'.repeat(64)}}},...extra});
const job=(extra={})=>({job_id:'d'.repeat(32),generation:1,model_sha256:MODEL,status:'running',finished:false,process_running:true,source_age_sec:.01,freshness:'fresh',sequence:1,native_guard:{phase:'ready',latched:false},cancel_requested:false,cancel_delivery:'not_requested',task:{stage:'carrying',physical_grasp:'confirmed',planning_attachment:'attached',events:[]},evidence:{finger_force_n:[1,1]},...extra});
const withJob=(extra={})=>{const value=snapshot();value.task.can_start=false;value.task.can_select=false;value.task.grasp.job=job(extra);value.task.grasp.view.mode='live';return value;};
function deferred(){let resolve,reject;const promise=new Promise((yes,no)=>{resolve=yes;reject=no;});return{resolve,reject,promise};}
class Element{
 constructor(){this.value='';this.textContent='';this.hidden=false;this.disabled=false;this.dataset={};this.children=[];this.listeners={};this.classList={toggle(){}};}
 addEventListener(name,listener){this.listeners[name]=listener;}
 append(...nodes){this.children.push(...nodes);}
 replaceChildren(...nodes){this.children=[...nodes];}
}
function harness(initial=snapshot()){
 const elements=new Map(),requests=[],timers=new Map(),notices=[];let now=0,nextTimer=0,nextState=initial,ready=true,before=0,after=0,refreshes=0,shown=0;
 const element=id=>{if(!elements.has(id))elements.set(id,new Element());return elements.get(id);};
 const cancels=[element('cancel'),element('cancel-secondary')],steps=Array.from({length:4},(_,index)=>{const node=element('step'+index);node.dataset.step=String(index);return node;});
 const document={getElementById:element,createElement:()=>new Element(),querySelectorAll(selector){return selector==='[data-cancel]'?cancels:selector==='[data-step]'?steps:[];}};
 const ui=new WorkspaceTaskControls({document,request:(path,body)=>{const pending={path,body,...deferred()};requests.push(pending);return pending.promise;},refresh:async()=>{refreshes++;ui.observe(nextState);},beforeCommand:()=>before++,afterCommand:()=>after++,notify:message=>notices.push(message),showOperate:()=>shown++,cameraReady:()=>ready,now:()=>now,schedule:fn=>{const id=++nextTimer;timers.set(id,fn);return id;},unschedule:id=>timers.delete(id)});
 ui.observe(initial);ui.render();
 return{ui,element,requests,notices,timers,setState(value){nextState=value;ui.observe(value);ui.render();},nextState(value){nextState=value;},advance(ms){now+=ms;},camera(value){ready=value;ui.render();},counts:()=>({before,after,refreshes,shown})};
}
test('choosing a task performs one selection POST only; a response cannot replace the authoritative GET',async()=>{
 const initial=snapshot();initial.task.selected='motion';const h=harness(initial),next=snapshot();next.task.selection_id=NEXT;
 const pending=h.ui.command('select','grasp');
 assert.equal(h.requests.length,1);assert.deepEqual(h.requests[0].body,{task:'grasp',selection_id:SEL});assert.equal(h.requests[0].path,'/api/task/select');
 assert.equal(h.element('task-select').disabled,true);assert.equal(await h.ui.command('start'),false);assert.equal(h.requests.length,1);
 h.nextState(next);h.requests[0].resolve({task:{selected:'motion',selection_id:SEL}});assert.equal(await pending,true);
 assert.equal(h.ui.state.task.selection_id,NEXT);assert.equal(h.element('grasp-task-panel').hidden,false);assert.equal(h.requests.length,1);assert.deepEqual(h.counts(),{before:1,after:1,refreshes:1,shown:1});
});
test('a fixed task start pins selection/model and repeated clicks cannot start another worker',async()=>{
 const h=harness(),pending=h.ui.command('start');assert.equal(h.requests.length,1);
 const request=h.requests[0];assert.equal(request.path,'/api/task/grasp/start');assert.equal(request.body.selection_id,SEL);assert.equal(request.body.model_sha256,MODEL);assert.match(request.body.request_id,/^[0-9a-f]{32}$/);
 assert.equal(await h.ui.command('start'),false);h.nextState(withJob());request.resolve({});await pending;
 assert.equal(h.element('grasp-start').disabled,true);assert.equal(h.element('grasp-cancel').disabled,false);assert.equal(h.element('cancel').disabled,false);assert.equal(h.requests.length,1);
});
test('cancel remains bound to the currently observed native job/generation, never the normal motion ID',async()=>{
 const h=harness(withJob()),pending=h.ui.command('cancel');
 assert.deepEqual(h.requests[0].body,{selection_id:SEL,job_id:'d'.repeat(32),generation:1,model_sha256:MODEL});assert.equal(h.requests[0].path,'/api/task/grasp/cancel');
 assert.equal(h.element('cancel').disabled,true);assert.equal(await h.ui.command('cancel'),false);
 h.nextState(withJob({cancel_requested:true,cancel_delivery:'signal_sent'}));h.requests[0].resolve({});await pending;
 assert.equal(h.element('cancel').disabled,true);assert.match(h.element('grasp-admission').textContent,/취소 전달.*제어기 정지.*프로세스/);
});
test('unconfirmed start is never retried or accepted from a late response; a new observed generation resolves it',async()=>{
 const h=harness(),pending=h.ui.command('start');[...h.timers.values()][0]();await pending;
 assert.equal(h.ui.unconfirmed.action,'start');assert.equal(h.element('grasp-start').disabled,true);
 assert.equal(await h.ui.command('start'),false);assert.equal(h.requests.length,1);assert.match(h.notices[0],/자동으로 다시 보내지 않습니다/);
 h.requests[0].resolve({task:{selected:'motion'}});await Promise.resolve();assert.equal(h.ui.state.task.selected,'grasp');assert.equal(h.requests.length,1);
 h.setState(withJob());assert.equal(h.ui.unconfirmed,null);assert.equal(h.element('cancel').disabled,false);
});
test('connection loss preserves the last task while blocking new commands, and camera failure never disables a known cancellation',()=>{
 const h=harness(withJob());h.camera(false);assert.equal(h.element('grasp-start').disabled,true);assert.equal(h.element('cancel').disabled,false);
 h.ui.lost();assert.equal(h.element('cancel').disabled,false);assert.equal(h.element('task-select').disabled,true);assert.match(h.element('grasp-admission').textContent,/状態|상태 조회/);
 const rows=graspEvidence(h.ui.state,{connected:false});assert.ok(rows.every(row=>row.tone!=='good'));assert.match(rows.at(-1).value,/정리 결과 미확인/);
});
test('stale, stopped, cancelled or ended confirmation remains historical; elapsed time cannot be reset by duplicate GETs',()=>{
 const h=harness(withJob());assert.match(graspPresentation(h.ui.state).physical,/현재 관측.*물리 집기 확인/);
 h.advance(600);h.setState(withJob());assert.match(h.element('grasp-physical').textContent,/마지막 기록/);assert.equal(h.ui.state.task.observation_age_sec,.61);
 h.setState(withJob({sequence:2,source_age_sec:.02}));assert.match(h.element('grasp-physical').textContent,/현재 관측/);
 for(const extra of [{finished:true,process_running:false,status:'succeeded'},{cancel_requested:true},{freshness:'stale'},{native_guard:{phase:'stopped',latched:true}}]){
  const value=withJob(extra);assert.match(graspPresentation(value).physical,/마지막 기록/);
 }
});
test('worker exit or request cancellation does not establish controller stop or successful cleanup',()=>{
 const ended=withJob({status:'cancelled',finished:true,process_running:false,owner_cleanup_clean:false,cleanup:{clean:false},task:{physical_grasp:'confirmed',planning_attachment:'attached',events:[]}});
 let rows=graspEvidence(ended);assert.doesNotMatch(rows.find(row=>row.label==='제어기 정지').value,/비활성 확인/);assert.match(rows.at(-1).value,/정리 결과 확인 필요/);assert.notEqual(rows.at(-1).tone,'good');
 ended.task.grasp.job.native_guard={phase:'stopped',latched:true,controllers:{left_controller:'inactive',right_controller:'inactive',gripper_controller:'inactive'}};
 ended.task.grasp.job.cleanup.clean=true;ended.task.grasp.job.owner_cleanup_clean=true;rows=graspEvidence(ended);
 assert.match(rows.find(row=>row.label==='제어기 정지').value,/3개 비활성 확인/);assert.equal(rows.at(-1).tone,'good');assert.match(rows[0].value,/마지막 기록/);
});
test('model or task identity mismatch cannot dispatch, and cleanup-pending jobs cannot switch or restart',async()=>{
 const value=withJob({model_sha256:'e'.repeat(64)}),h=harness(value);assert.equal(h.element('grasp-start').disabled,true);assert.equal(h.element('cancel').disabled,true);assert.equal(await h.ui.command('cancel'),false);assert.equal(h.requests.length,0);
 const cleanup=withJob({status:'failed',finished:true,process_running:false,owner_cleanup_clean:false});h.setState(cleanup);
 assert.equal(h.element('task-select').disabled,true);assert.equal(await h.ui.command('select','motion'),false);assert.equal(h.element('grasp-start').disabled,true);assert.equal(h.requests.length,0);
});
test('page teardown ignores late command observations and never retries or changes a different task',async()=>{
 const h=harness(),pending=h.ui.command('start');h.ui.destroy();h.requests[0].resolve({});assert.equal(await pending,false);assert.equal(h.requests.length,1);assert.equal(h.counts().refreshes,0);
});


test('a finished cancellation stays completed while signal delivery alone is never called worker acknowledgement',()=>{
 const cancelling=withJob({cancel_requested:true,cancel_delivery:'worker_signal_sent'});assert.match(graspEvidence(cancelling)[2].value,/취소 신호 전달.*수신 확인 중/);assert.doesNotMatch(graspEvidence(cancelling)[2].value,/요청 없음|수신 확인$/);
 const done=withJob({cancel_requested:true,status:'cancelled',finished:true,process_running:false,owner_cleanup_clean:true,cleanup:{clean:true}});assert.equal(graspPresentation(done).status,'요청 취소 완료');
 const h=harness(done);assert.equal(h.element('grasp-status').textContent,'요청 취소 완료');assert.match(h.element('grasp-admission').textContent,/작업이 종료/);assert.match(h.element('grasp-physical').textContent,/마지막 기록/);
});


test('lost GETs leave an exact last-known native cancellation reachable while new execution stays blocked',async()=>{
 const h=harness(withJob());h.ui.lost();assert.equal(h.element('grasp-start').disabled,true);assert.equal(h.element('task-select').disabled,true);assert.equal(h.element('cancel').disabled,false);assert.match(h.element('cancel').textContent,/마지막 확인/);assert.match(h.element('grasp-admission').textContent,/상태 조회 끊김.*마지막 확인 작업.*미확인/);
 const pending=h.ui.command('cancel');assert.equal(h.requests.length,1);assert.deepEqual(h.requests[0].body,{selection_id:SEL,job_id:'d'.repeat(32),generation:1,model_sha256:MODEL});
 h.nextState(withJob({cancel_requested:true,cancel_delivery:'worker_signal_sent'}));h.requests[0].resolve({});await pending;assert.equal(h.element('cancel').disabled,true);
});
test('an ambiguous cancel permits only a manual exact-owner retry after its deadline; late changed-owner replies are ignored',async()=>{
 const h=harness(withJob()),first=h.ui.command('cancel');[...h.timers.values()][0]();await first;
 assert.equal(h.requests.length,1);assert.equal(h.ui.unconfirmed.action,'cancel');assert.equal(h.element('cancel').disabled,false);assert.equal(h.element('grasp-start').disabled,true);
 const second=h.ui.command('cancel');assert.equal(h.requests.length,2);assert.deepEqual(h.requests[1].body,h.requests[0].body);assert.equal(await h.ui.command('cancel'),false);
 const other=snapshot();other.task.selected='motion';other.task.selection_id=NEXT;h.nextState(other);h.requests[1].resolve({});await second;
 h.requests[0].resolve({task:{selected:'grasp',selection_id:SEL}});await Promise.resolve();assert.equal(h.ui.state.task.selection_id,NEXT);assert.equal(h.ui.selected(),false);assert.equal(h.requests.length,2);
});
