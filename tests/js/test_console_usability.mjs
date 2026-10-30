/** Transport deadlines, explicit replan, and presentation-only task grouping. */
import test from 'node:test';
import assert from 'node:assert/strict';
import {readFile} from 'node:fs/promises';
import vm from 'node:vm';

const source=(await readFile(new URL('../../src/mujoco_ros2_manipulation/operator_console/static/app.js',import.meta.url),'utf8'))
 .replace(/boot\(\)\.catch\(error=>\{notice\('조작 화면을 불러오지 못했습니다: '\+error\.message\);\}\);\s*$/,'');
const PLAN='a'.repeat(32),EXECUTION='b'.repeat(32);

class Element{
 constructor(tag='div'){this.tagName=tag;this.children=[];this.dataset={};this.listeners={};this.hidden=false;this.disabled=false;this.value='';this.textContent='';this.className='';this.classList={toggle(){}};}
 append(...nodes){this.children.push(...nodes);}
 replaceChildren(...nodes){this.children=[...nodes];}
 addEventListener(name,callback){this.listeners[name]=callback;}
 setAttribute(name,value){this[name]=value;}
 removeAttribute(name){delete this[name];}
}
function state(overrides={}){return {positions_rad:{left:0,right:0},state_age_sec:0,clock_age_sec:0,simulation_sec:1,can_plan:true,can_submit:true,can_approve:false,server:{session_id:'session-a',status:'ready'},
 active_request_id:null,history:[],graph:{clock:1,joints:1,status:1,guards:1,commands:1},simulation_verified:true,
 lifecycle:{phase:'running',owned:{simulation:true,commands:true}},guard:{phase:'ready',latched:false},demo:{current:null,history:[],can_start:false},
 view:{enabled:true,mode:'live',source:{id:'workcell:pair',label:'FR5 + UR5e'},config_source_id:'workcell:pair',display_mode:'web3d'},...overrides};}
function harness(fetcher){
 const elements=new Map(),timers=new Map(),calls=[];let timerID=0;
 const element=id=>{if(!elements.has(id))elements.set(id,new Element());return elements.get(id);};element('strategy').value='coordinated';
 const document={getElementById:element,querySelectorAll:selector=>selector==='[data-cancel]'?[element('cancel'),element('cancel-secondary')]:[],createElement:tag=>new Element(tag)};
 const scene={cameraReady:()=>true,pause(){},update(){},isImage:()=>false,mount(){},select(){},invalidateTask(){this.taskInvalidations=(this.taskInvalidations||0)+1;}};
 const context=vm.createContext({document,fetch:(path,options)=>{calls.push({path,options});return fetcher(path,options);},AbortController,
  setTimeout:(callback,delay)=>{const id=++timerID;timers.set(id,{callback,delay});return id;},clearTimeout:id=>timers.delete(id),
  performance:{now:()=>5000},crypto:{randomUUID:()=> 'c'.repeat(32)},window:{}});
 vm.runInContext(source+`\nglobalThis.uiTest={request,beginTaskCommand,endTaskCommand,poll,readState,replan,groupRequests,summaryOf,renderReview,renderContext,history,detail,renderDetail,loadRequest,requestProgress,requestGuidance,guidanceText,guardPresentation,connectionPresentation,observationProblem,cancelCurrentRequest,render,refreshControls,demoStage,demoOutcomeStatus,
 set(data,positions={left:.2,right:.1},connected=true){state=data;targets=positions;config={label:'FR5 + UR5e',domain:160,joint_limits:{left:{lower:-1,upper:1},right:{lower:-1,upper:1}}};readConnected=connected;sceneControls=globalThis.scene;},
 get(){return {polling,readConnected,replanning,selected:state?.task?.selected};},taskController(value){taskControls=value;},cache(id,data,age=0){requestCache.set(id,{data,at:Date.now()-age});},age(id,age=2000){requestCache.get(id).at=Date.now()-age;},demos(data){demoCatalog=[{id:'contact_comparison',title:'접촉 비교'}];selectedDemo='contact_comparison';renderDemos(data);}};`,context);
 context.scene=scene;
 return {api:context.uiTest,element,scene,calls,timers,expire(){const entries=[...timers.values()];for(const entry of entries)entry.callback();},set(data,targets,connected){context.uiTest.set(data,targets,connected);}};
}
const reply=data=>Promise.resolve({ok:true,json:()=>Promise.resolve(data)});
const aborted=signal=>new Promise((_,reject)=>signal.addEventListener('abort',()=>reject(new Error('aborted')),{once:true}));
async function drain(){for(let i=0;i<10;i++)await Promise.resolve();}

for(const phase of ['headers','body'])test(`GET deadline covers stalled ${phase} and always releases its timer`,async()=>{
 const h=harness((path,options)=>phase==='headers'?aborted(options.signal):Promise.resolve({ok:true,json:()=>aborted(options.signal)}));
 const pending=h.api.request('/api/state');await drain();assert.equal(h.timers.size,1);assert.equal([...h.timers.values()][0].delay,3000);
 h.expire();await assert.rejects(pending,error=>error.code==='read_timeout');assert.equal(h.timers.size,0);assert.equal(h.calls.length,1);
});

test('a timed-out poll blocks actions and the next successful GET restores polling without replaying a POST',async()=>{
 let attempt=0;const h=harness((path,options)=>++attempt===1?aborted(options.signal):reply(state()));h.set(state());
 const first=h.api.poll();await drain();h.expire();await first;
 assert.equal(h.api.get().polling,false);assert.equal(h.api.get().readConnected,false);assert.equal(h.element('execute').disabled,true);assert.equal(h.element('review-approve').disabled,true);
 await h.api.poll();assert.equal(h.api.get().polling,false);assert.equal(h.api.get().readConnected,true);assert.equal(h.element('execute').disabled,false);
 assert.ok(h.calls.every(call=>call.options.method!=='POST'));assert.equal(h.calls.length,2);
});

test('POST failures are returned once and never armed for automatic retransmission',async()=>{
 const h=harness(()=>Promise.reject(new Error('connection interrupted')));
 await assert.rejects(h.api.request('/api/plan',{request_id:PLAN}),/connection interrupted/);
 assert.equal(h.calls.length,1);assert.equal(h.calls[0].options.method,'POST');assert.equal(h.calls[0].options.signal,undefined);assert.equal(h.timers.size,0);
});

test('explicit replan returns live, checks fresh state, and creates a new plan without approving it',async()=>{
 const h=harness((path,options)=>reply(path==='/api/state'?state():{}));h.set(state({view:{...state().view,mode:'plan_preview',review_plan:true}}));
 await h.api.replan();
 const mutations=h.calls.filter(call=>call.options.method==='POST');assert.deepEqual(mutations.map(call=>call.path),['/api/view/select','/api/plan']);
 const select=JSON.parse(mutations[0].options.body),plan=JSON.parse(mutations[1].options.body);
 assert.equal(select.mode,'live');assert.equal(plan.request_id,'c'.repeat(32));assert.equal(plan.target_rad.left,.2);
 assert.ok(!h.calls.some(call=>call.path==='/api/plan/execute'));assert.equal(h.api.get().replanning,false);
});

test('replan stops before creating a plan when the live transition fails or fresh admission is blocked',async()=>{
 for(const mode of ['transition','blocked']){
  const h=harness((path,options)=>path==='/api/view/select'&&mode==='transition'?Promise.reject(new Error('transition failed')):reply(path==='/api/state'?state({can_plan:false,reason:'state changed'}):{}));h.set(state());
  await h.api.replan();assert.ok(!h.calls.some(call=>call.path==='/api/plan'));assert.ok(!h.calls.some(call=>call.path==='/api/plan/execute'));
 }
});

test('display context separates a read-only robot model or plan from the live control target',()=>{
 const h=harness(()=>reply({}));
 h.set(state({view:{...state().view,mode:'model_preview',source:{label:'FR10'}}}));h.api.renderContext();
 assert.match(h.element('display-context').textContent,/모델 미리보기.*FR10/);assert.equal(h.element('control-context').textContent,'FR5 + UR5e');assert.match(h.element('context-note').textContent,/실행 대상은 바뀌지/);
 h.set(state({view:{...state().view,mode:'plan_preview',review_plan:true}}));h.api.renderContext();assert.match(h.element('display-context').textContent,/실행 전 계획/);assert.match(h.element('context-note').textContent,/관절표.*실제 수신 상태/);
 h.set(state({view:{...state().view,source:{kind:'recording',label:'기록 예제'}}}));h.api.renderContext();assert.equal(h.element('control-context').textContent,'실행 연결 없음');
});

test('history joins linked execution and plan while preserving unrelated and missing-plan records',()=>{
 const h=harness(()=>reply({}));
 const records=[{request_id:EXECUTION,mode:'approved_plan',approval:{plan_request_id:PLAN}},
  {request_id:PLAN,mode:'plan_only'},{request_id:'d'.repeat(32),mode:'automatic'},
  {request_id:'e'.repeat(32),mode:'approved_plan',plan_request_id:'f'.repeat(32)}];
 const groups=h.api.groupRequests(records);assert.equal(groups.length,3);assert.equal(groups[0].id,PLAN);assert.equal(groups[0].plan.request_id,PLAN);assert.equal(groups[0].executions[0].request_id,EXECUTION);
 assert.equal(groups[2].plan,null);assert.equal(groups[2].records.length,1);
});

test('review summary uses checked metadata and disconnected state keeps approval disabled',()=>{
 const h=harness(()=>reply({})),plan={request_id:PLAN,mode:'plan_only',command:{strategy:'coordinated',target_rad:{left:.2,right:.1}}};
 const pending={plan_request_id:PLAN,state:'awaiting_approval',remaining_sec:55};
 h.set(state({history:[plan],pending_plan:pending,can_approve:true,view:{...state().view,mode:'plan_preview',review_plan:true,binding:{request_id:PLAN}}}));
 h.api.cache(PLAN,{review_summary:{strategy:'coordinated',duration_sec:4.25,minimum:{distance_m:.041,time_sec:2},max_abs_target_delta_rad:.2}});
 h.api.renderReview();assert.equal(h.element('review-duration').textContent,'4.25 s');assert.equal(h.element('review-distance').textContent,'41.0 mm');assert.equal(h.element('review-approve').disabled,false);
 h.set(state({history:[plan],pending_plan:pending,can_approve:true,view:{...state().view,mode:'plan_preview',review_plan:true,binding:{request_id:PLAN}}}),undefined,false);
 h.api.renderReview();assert.equal(h.element('review-approve').disabled,true);assert.equal(h.element('review-replan').disabled,true);
});

test('an older failed state read cannot replace a newer successful state observation',async()=>{
 let rejectOld;const h=harness((path,options)=>h.calls.length===1?new Promise((resolve,reject)=>{rejectOld=reject;}):reply(state()));h.set(state());
 const old=h.api.poll();await drain();await h.api.readState();assert.equal(h.api.get().readConnected,true);
 rejectOld(new Error('old response failed'));await old;assert.equal(h.api.get().readConnected,true);assert.equal(h.element('execute').disabled,false);
});

test('an older successful state read is rejected after the latest read fails and cannot restore controls',async()=>{
 let releaseOld;const h=harness(()=>h.calls.length===1?new Promise(resolve=>{releaseOld=resolve;}):Promise.reject(new Error('latest state read failed')));h.set(state());
 const old=h.api.readState(),rejected=assert.rejects(old,error=>error.code==='state_read_superseded');await drain();
 await h.api.poll();assert.equal(h.api.get().readConnected,false);
 releaseOld({ok:true,json:()=>Promise.resolve(state())});await rejected;
 assert.equal(h.api.get().readConnected,false);assert.equal(h.element('execute').disabled,true);assert.equal(h.element('review-approve').disabled,true);
});

test('a state read is superseded as soon as a newer read starts, even before that newer read completes',async()=>{
 const releases=[];const h=harness(()=>new Promise(resolve=>releases.push(resolve)));h.set(state());
 const old=h.api.readState(),rejected=assert.rejects(old,error=>error.code==='state_read_superseded');await drain();
 const latest=h.api.readState();await drain();
 releases[0]({ok:true,json:()=>Promise.resolve(state({simulation_sec:1}))});await rejected;
 assert.equal(h.element('sim-time').textContent,'','The superseded state must not be rendered');
 releases[1]({ok:true,json:()=>Promise.resolve(state({simulation_sec:2}))});await latest;
 assert.equal(h.element('sim-time').textContent,'2.000 s');assert.equal(h.api.get().readConnected,true);
});

test('explicit replan cannot use an old successful read after a newer explicit read disconnects',async()=>{
 let releaseOld,reads=0;const h=harness(path=>{
  if(path==='/api/state')return ++reads===1?new Promise(resolve=>{releaseOld=resolve;}):Promise.reject(new Error('newer state read failed'));
  return reply({});
 });h.set(state());
 const pending=h.api.replan();await drain();await assert.rejects(h.api.readState(),/newer state read failed/);assert.equal(h.api.get().readConnected,false);
 releaseOld({ok:true,json:()=>Promise.resolve(state())});await pending;
 assert.ok(!h.calls.some(call=>['/api/plan','/api/plan/execute'].includes(call.path)));
 assert.equal(h.api.get().readConnected,false);assert.equal(h.api.get().replanning,false);assert.equal(h.element('execute').disabled,true);
});

test('periodic ticks cannot starve the explicit replan read while a valid response is delayed',async()=>{
 let releaseFresh,reads=0;
 const h=harness(path=>path==='/api/state'&&++reads===1?new Promise(resolve=>{releaseFresh=resolve;}):reply(path==='/api/state'?state():{}));h.set(state());
 const pending=h.api.replan();await drain();
 for(let tick=0;tick<4;tick++)await h.api.poll();
 assert.equal(h.calls.filter(call=>call.path==='/api/state').length,1,'Periodic ticks share the replan observation instead of issuing newer reads');
 assert.equal(h.calls.filter(call=>call.path==='/api/plan').length,0,'A plan cannot be created before its explicit fresh read completes');
 releaseFresh({ok:true,json:()=>Promise.resolve(state())});await pending;
 assert.equal(h.calls.filter(call=>call.path==='/api/plan').length,1);
 assert.ok(!h.calls.some(call=>call.path==='/api/plan/execute'));
 assert.equal(h.api.get().readConnected,true);assert.equal(h.api.get().replanning,false);
 assert.equal(h.calls.filter(call=>call.path==='/api/state').length,2,'A normal post-submit observation resumes after the owned read');
});

test('replan rechecks the current connection and latest issued read immediately before posting',async()=>{
 for(const change of ['disconnect','new-read']){
  let reads=0,injected=false,pendingRead,releaseNew;const fresh=state();
  const h=harness(path=>path==='/api/state'&&++reads===2&&change==='new-read'?new Promise(resolve=>{releaseNew=resolve;}):reply(path==='/api/state'?fresh:{}));h.set(state());
  h.scene.update=()=>{if(injected)return;injected=true;if(change==='disconnect')h.set(fresh,undefined,false);else pendingRead=h.api.readState();};
  await h.api.replan();
  assert.ok(!h.calls.some(call=>['/api/plan','/api/plan/execute'].includes(call.path)),change);
  if(pendingRead){const rejected=assert.rejects(pendingRead,error=>error.code==='state_read_superseded');releaseNew({ok:true,json:()=>Promise.resolve(state())});await rejected;}
  assert.equal(h.api.get().replanning,false);
 }
});

test('replan rejects a fresh response with active execution or demo even if can_plan is true',async()=>{
 for(const flags of [{active_request_id:EXECUTION},{demo:{current:{status:'running',scenario:'contact_comparison',job_id:PLAN},history:[],can_start:false}}]){
  const h=harness(path=>reply(path==='/api/state'?state(flags):{}));h.set(state());await h.api.replan();
  assert.ok(!h.calls.some(call=>['/api/plan','/api/plan/execute'].includes(call.path)));
  assert.equal(h.api.get().replanning,false);
 }
});

test('changing the selected demo after disconnection cannot re-enable execution controls',()=>{
 const h=harness(()=>reply({})),demo={current:null,can_start:true,history:[]};
 h.set(state());h.api.demos(demo);assert.equal(h.element('demo-run').disabled,false);
 h.set(state(),undefined,false);h.api.demos(demo);assert.equal(h.element('demo-run').disabled,true);
 h.api.demos({current:{status:'running',scenario:'contact_comparison',job_id:PLAN},can_start:false,history:[]});assert.equal(h.element('demo-stop').disabled,true);
});

test('task detail presents summary and linked identities with raw data kept separately',()=>{
 const h=harness(()=>reply({})),plan={request_id:PLAN,mode:'plan_only',status:'succeeded',command:{strategy:'coordinated'}},
 execution={request_id:EXECUTION,mode:'approved_plan',status:'succeeded',plan_request_id:PLAN,metrics:{received_to_terminal_wall_sec:8.5}};
 h.set(state({history:[execution,plan]}));h.api.cache(PLAN,{review_summary:{strategy:'coordinated',duration_sec:4,minimum:{distance_m:.04},max_abs_target_delta_rad:.2}});
 h.api.cache(EXECUTION,{...execution,events:[{name:'dispatch',wall_sec:1},{name:'terminal',wall_sec:8.5}]});
 const group=h.api.groupRequests([execution,plan])[0];h.api.renderDetail(group,EXECUTION);
 assert.equal(h.element('detail-title').textContent,'계획과 승인 실행');assert.equal(h.element('detail-summary').children.length,6);
 assert.equal(h.element('detail-summary').children[3].children[0].textContent,'계획 양팔 간격');
 assert.match(h.element('detail-links').children[0].textContent,new RegExp(PLAN));assert.match(h.element('detail-links').children[1].textContent,new RegExp(EXECUTION));
 assert.equal(h.element('detail-timeline').children.length,2);assert.match(h.element('detail-timeline').children[0].textContent,/실행 요청 전송/);
 assert.match(h.element('detail-content').textContent,/selected_request/);
});

test('running detail cache refreshes at plan-ready and completed summary is then reused',async()=>{
 const ready={request_id:PLAN,mode:'plan_only',status:'succeeded',stage:'plan_ready',command:{strategy:'coordinated',target_rad:{left:.2,right:.1}}},
  full={...ready,review_summary:{strategy:'coordinated',duration_sec:1.434,minimum:{distance_m:.1811,time_sec:.7},max_abs_target_delta_rad:Math.PI/60}};
 const h=harness(()=>reply(full));h.api.cache(PLAN,{request_id:PLAN,mode:'plan_only',status:'running',stage:'planning_both_arms',command:ready.command});
 h.set(state({history:[ready],pending_plan:{plan_request_id:PLAN,state:'awaiting_approval',remaining_sec:50}}));h.api.renderReview();
 await drain();await drain();
 assert.equal(h.calls.length,1);assert.equal(h.element('review-duration').textContent,'1.43 s');assert.equal(h.element('review-distance').textContent,'181.1 mm');assert.equal(h.element('review-delta').textContent,'3.0°');
 h.api.renderReview();await h.api.loadRequest(PLAN,{expected:ready});assert.equal(h.calls.length,1,'Complete terminal details are immutable and reusable');
});

test('incomplete terminal summary retries are bounded and complete data stops the retries',async()=>{
 const ready={request_id:PLAN,mode:'plan_only',status:'succeeded',stage:'plan_ready'};let count=0;
 const h=harness(()=>{count++;return reply(ready);});
 for(let attempt=0;attempt<6;attempt++){await h.api.loadRequest(PLAN,{expected:ready});h.api.age(PLAN);}
 assert.equal(count,3);
 const full={...ready,review_summary:{strategy:'coordinated',duration_sec:2,minimum:{distance_m:.04},max_abs_target_delta_rad:.2}};
 h.api.cache(PLAN,full,2000);await h.api.loadRequest(PLAN,{expected:ready});assert.equal(count,3);
});

test('failure guidance distinguishes budgets, candidates, endpoints, environment and within-arm clearance',()=>{
 const h=harness(()=>reply({}));
 const cases={budget_exhausted:['계산 시간 초과','계산 시간'],candidate_exhausted:['계획 후보 검사 종료','도달 불가능하다는 판정은 아닙니다'],
  invalid_start:['시작 자세 조건 위반','시작 자세'],invalid_goal:['목표 자세 조건 위반','목표 자세'],
  execution_environment_changed:['설치 환경 변경','기존 계획'],
  environment_clearance:['설비 간격 조건 위반','시뮬레이션 정책'],
  self_clearance:['팔 내부 간격 부족','같은 팔의 링크 사이 간격']};
 for(const [failure_kind,[title,description]] of Object.entries(cases)){
  const record={status:'failed',failure_kind,failure_stage:'safety_checks',error:{message:'original diagnostic'}};
  assert.equal(h.api.requestGuidance(record).title,title);assert.ok(h.api.guidanceText(record).includes(description));
  assert.match(h.api.guidanceText(record),/실패 단계: 전체 경로 검사/);assert.match(h.api.guidanceText(record),/원본 오류: original diagnostic/);
 }
 assert.equal(h.api.requestGuidance({status:'running',failure_kind:'budget_exhausted'}),null);
 assert.equal(h.calls.length,0,'Presenting failures never submits a retry');
});

test('state/review changes request a new plan while execution failure preserves stop/log guidance',()=>{
 const h=harness(()=>reply({}));
 for(const failure_kind of ['state_changed','review_invalidated'])assert.match(h.api.guidanceText({status:'failed',failure_kind}),/새 계획.*검토/);
 assert.match(h.api.guidanceText({status:'failed',failure_kind:'execution_failed'}),/정지 감시.*실행 로그/);
 assert.match(h.api.guidanceText({status:'failed',failure_kind:'execution_environment_changed'}),/재시작.*새 계획/);
 assert.match(h.api.guidanceText({status:'failed',failure_kind:'unrecognized',error:{message:'<raw>'}}),/원본 오류: <raw>/);
 assert.equal(h.calls.length,0);
});

test('guard stop confirmation requires the latch, stopped phase and a known reason',()=>{
 const h=harness(()=>reply({}));
 for(const reason of ['contact','clock_stall']){
  let result=h.api.guardPresentation({latched:true,phase:'stopping',reason});assert.equal(result.confirmed,false);assert.match(result.text,/정지 확인 중/);
  result=h.api.guardPresentation({latched:true,phase:'stopped',reason});assert.equal(result.confirmed,true);assert.ok(!result.text.includes('확인 중'));
 }
 for(const guard of [{latched:true,phase:'stopped',reason:''},{latched:true,phase:'stopped',reason:{}},
  {latched:false,phase:'stopped',reason:'contact'}])assert.equal(h.api.guardPresentation(guard).confirmed,false);
});

test('unlatched server block, stale observations and process errors never show ready or enable plan/approval',()=>{
 const h=harness(()=>reply({}));
 const plan={request_id:PLAN,mode:'plan_only',status:'succeeded',stage:'plan_ready',command:{strategy:'coordinated',target_rad:{left:.2,right:.1}}};
 const common={history:[plan],can_approve:true,pending_plan:{plan_request_id:PLAN,state:'awaiting_approval',remaining_sec:50},
  view:{...state().view,mode:'plan_preview',review_plan:true,binding:{request_id:PLAN}}};
 h.api.cache(PLAN,{...plan,review_summary:{strategy:'coordinated',duration_sec:1,minimum:{distance_m:.04},max_abs_target_delta_rad:.2}});
 for(const override of [{server:{session_id:'session-a',status:'blocked'}},{state_age_sec:.6},{clock_age_sec:.6},
  {state_age_sec:null},{lifecycle:{...state().lifecycle,error:'process exited'}}]){
  const current=state({...common,...override});h.set(current);const presentation=h.api.connectionPresentation(current);
  assert.equal(presentation.status,'blocked');assert.ok(!presentation.text.includes('실행 가능'));
  h.api.refreshControls();assert.equal(h.element('execute').disabled,true);assert.equal(h.element('review-approve').disabled,true);
 }
 const blocked=state({server:{session_id:'session-a',status:'blocked'}});
 assert.match(h.api.connectionPresentation(blocked).text,/재시작 필요/);assert.match(h.api.observationProblem(blocked),/실행 로그.*재시작/);
 assert.equal(h.api.connectionPresentation(state()).status,'ready');
});

test('request cancellation and failed cleanup never masquerade as confirmed physical stop',()=>{
 const h=harness(()=>reply({}));
 const cancelled=h.api.requestGuidance({status:'cancelled'});assert.equal(cancelled.title,'요청 취소 완료');assert.match(cancelled.text,/물리적 정지 확인과는 구분/);
 for(const cleanup_clean of [false,undefined]){
  const job={status:'cancelled',cleanup_clean};assert.equal(h.api.demoOutcomeStatus(job),'unknown');assert.match(h.api.demoStage(job),/정리 결과를 확인/);
 }
 const failed={status:'failed',cleanup_clean:false,error:'owned process remains'};assert.equal(h.api.demoOutcomeStatus(failed),'failed');assert.ok(!h.api.demoStage(failed).includes('마쳤습니다'));
 assert.match(h.api.demoStage({status:'cancel_requested'}),/정리될 때까지/);
 assert.match(h.api.demoStage({status:'cancelled',cleanup_clean:true}),/소유 실행의 정리를 마쳤습니다/);
});

test('cancel submission, server acknowledgement and terminal request cancellation stay distinct without duplicate POSTs',async()=>{
 let releaseCancel,current;
 const record={request_id:EXECUTION,session_id:'session-a',status:'running',stage:'first_motion',metrics:{}};
 current=state({active_request_id:EXECUTION,history:[record],can_plan:false,can_submit:false});
 const h=harness((path,options)=>path==='/api/cancel'?new Promise(resolve=>{releaseCancel=()=>resolve({ok:true,json:()=>Promise.resolve({request_id:EXECUTION,status:'cancel_requested'})});}):reply(path==='/api/state'?current:record));
 h.set(current);h.api.cache(EXECUTION,record);
 const pending=h.api.cancelCurrentRequest();await drain();assert.equal(h.element('cancel').disabled,true);assert.match(h.element('request-stage').textContent,/취소 요청 전달 중/);
 await h.api.cancelCurrentRequest();assert.equal(h.calls.filter(call=>call.path==='/api/cancel').length,1);
 releaseCancel();await pending;assert.match(h.element('request-stage').textContent,/서버 수신과 요청 종료/);
 const acknowledged={...record,stage:'cancel_received'};current=state({active_request_id:EXECUTION,history:[acknowledged],can_plan:false,can_submit:false});h.api.render(current);
 assert.match(h.element('cancel').textContent,/취소 수신.*종료 확인 중/);assert.match(h.element('request-stage').textContent,/서버가 취소를 수신/);
 const ended={...record,status:'cancelled',stage:'terminal'};current=state({active_request_id:null,history:[ended]});h.api.render(current);
 assert.equal(h.element('request-badge').textContent,'요청 취소 완료');assert.match(h.element('request-error').textContent,/물리적 정지 확인과는 구분/);
 assert.deepEqual(h.calls.filter(call=>call.options.method==='POST').map(call=>call.path),['/api/cancel']);
});

test('a lost cancel response stays unconfirmed and never schedules an automatic retransmission',async()=>{
 const record={request_id:EXECUTION,session_id:'session-a',status:'running',stage:'first_motion',metrics:{}},current=state({active_request_id:EXECUTION,history:[record],can_plan:false,can_submit:false});
 const h=harness((path,options)=>path==='/api/cancel'?Promise.reject(new Error('response lost')):reply(path==='/api/state'?current:record));h.set(current);h.api.cache(EXECUTION,record);
 await h.api.cancelCurrentRequest();assert.equal(h.element('cancel').disabled,true);assert.match(h.element('request-stage').textContent,/전달 결과 미확인/);
 assert.match(h.element('notice-text').textContent,/자동으로 다시 보내지 않습니다/);
 await h.api.cancelCurrentRequest();await h.api.poll();
 assert.equal(h.calls.filter(call=>call.path==='/api/cancel').length,1);assert.ok(!h.calls.some(call=>call.path==='/api/plan'||call.path==='/api/plan/execute'));
});

test('historical failure detail uses that request and retains raw diagnostic data',()=>{
 const h=harness(()=>reply({})),record={request_id:EXECUTION,status:'failed',failure_kind:'budget_exhausted',failure_stage:'safety_checks',error:{message:'original budget diagnostic'},metrics:{}};
 h.set(state({history:[record],guard:{latched:true,phase:'stopped',reason:'contact'}}));
 h.api.cache(EXECUTION,{...record,events:[]});h.api.renderDetail(h.api.groupRequests([record])[0],EXECUTION);
 assert.match(h.element('detail-error').textContent,/계산 시간 초과.*전체 경로 검사.*원본 오류: original budget diagnostic/);
 assert.ok(!h.element('detail-error').textContent.includes('접촉'));
 assert.match(h.element('detail-content').textContent,/budget_exhausted/);
});

test('actual fresh idle snapshot waits for its first telemetry and keeps only start available',()=>{
 // Observed GET /api/state from isolated port 8913, domain 207, before any engine start.
 const fresh=state({positions_rad:null,state_age_sec:null,clock_age_sec:null,simulation_sec:null,simulation_verified:false,
  active_request_id:null,can_submit:false,can_plan:false,can_approve:false,guard:null,server:null,
  graph:{clock:0,joints:0,status:0,commands:0,guards:0,plans:0,approvals:0},
  lifecycle:{phase:'idle',error:null,owned:{},generation:0},
  reason:'공유 시계·관절 상태·명령 노드가 각각 하나씩 연결되어야 합니다.'});
 const h=harness(()=>reply(fresh));h.set(fresh);h.api.render(fresh);
 assert.equal(h.api.observationProblem(fresh),null);assert.equal(h.api.connectionPresentation(fresh).status,'offline');
 assert.equal(h.element('connection').textContent,'시뮬레이션 연결 대기');assert.equal(h.element('start').disabled,false);
 assert.equal(h.element('execute').disabled,true);assert.equal(h.element('review-card').hidden,true);
 assert.ok(!h.element('connection').textContent.includes('수신 중단'));assert.equal(h.calls.length,0);
 const starting={...fresh,lifecycle:{...fresh.lifecycle,phase:'starting',generation:1,owned:{simulation:true,commands:true}},graph:{...fresh.graph,clock:1}};
 h.set(starting);h.api.render(starting);assert.equal(h.api.observationProblem(starting),null);
 assert.equal(h.api.connectionPresentation(starting).status,'starting');assert.match(h.element('connection').textContent,/준비 중.*상태 대기/);
 assert.equal(h.element('start').disabled,true);assert.equal(h.element('execute').disabled,true);
});

test('missing telemetry after expected or previously observed sources remains blocked, including during startup',()=>{
 const h=harness(()=>reply({}));
 const absent=state({positions_rad:null,state_age_sec:null,clock_age_sec:null,simulation_sec:null,simulation_verified:false,
  can_submit:false,can_plan:false,guard:null,server:null,active_request_id:null,
  graph:{clock:0,joints:0,status:0,commands:0,guards:0},lifecycle:{phase:'starting',owned:{},error:null}});
 for(const override of [{lifecycle:{...absent.lifecycle,phase:'running'}},{simulation_verified:true},{active_request_id:EXECUTION},
  {positions_rad:{left:0,right:0}},{simulation_sec:0},{state_age_sec:.6},{clock_age_sec:.6},
  {server:{session_id:'session-a',status:'blocked'}},{guard:{latched:true,phase:'stopping',reason:'contact'}},
  {lifecycle:{...absent.lifecycle,error:'process failed'}}]){
  const current={...absent,...override};assert.ok(h.api.observationProblem(current));assert.equal(h.api.connectionPresentation(current).status,'blocked');
 }
});


test('task change invalidates an older normal-state GET before it can overwrite the selected native task',async()=>{
 let resolveOld,reads=0;const selected=state({task:{selected:'grasp',selection_id:'d'.repeat(32),grasp:{source:{domain:230},job:null}}});
 const h=harness(()=>++reads===1?new Promise(resolve=>resolveOld=resolve):reply(selected));h.set(state());
 const old=h.api.readState(),rejected=assert.rejects(old,error=>error.code==='state_read_superseded');await drain();
 h.api.beginTaskCommand('select');assert.equal(h.scene.taskInvalidations,1);assert.equal(h.element('execute').disabled,true);
 await h.api.readState();h.api.endTaskCommand();resolveOld({ok:true,json:()=>Promise.resolve(state())});await rejected;
 assert.equal(h.api.get().selected,'grasp');assert.equal(h.element('execute').disabled,true);assert.equal(h.calls.length,2);assert.ok(h.calls.every(row=>row.options.method!=='POST'));
});
test('header cancellation follows the selected native owner and does not submit a normal-arm cancellation',async()=>{
 const h=harness(()=>{throw Error('normal cancellation must not be sent');});h.set(state({active_request_id:PLAN,task:{selected:'grasp'}}));let cancelled=0;
 h.api.taskController({command:async kind=>{assert.equal(kind,'cancel');cancelled++;}});await h.api.cancelCurrentRequest();
 assert.equal(cancelled,1);assert.equal(h.calls.length,0);
});
test('normal mutation headers retain the selection epoch captured at request construction',async()=>{
 let resolve;const h=harness(()=>new Promise(done=>resolve=done));h.set(state({task:{selected:'motion',selection_id:'d'.repeat(32)}}));
 const pending=h.api.request('/api/plan',{request_id:PLAN});h.set(state({task:{selected:'motion',selection_id:'e'.repeat(32)}}));
 assert.equal(h.calls[0].options.headers['X-Workspace-Selection'],'d'.repeat(32));resolve({ok:true,json:()=>Promise.resolve({})});await pending;
});
