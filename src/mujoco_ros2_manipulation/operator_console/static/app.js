'use strict';
const $=id=>document.getElementById(id);
let sceneControls=null,taskControls=null,taskCommandBusy=false;
function graspSelected(data=state){return data?.task?.selected==='grasp';}
function motionUnavailable(data=state){return graspSelected(data)||taskCommandBusy||Boolean(taskControls?.unconfirmed);}
let config,token,state,targets={},arm=0,unit='deg',polling=false,latestRequest=null,detailsAt=0,demoCatalog=[],selectedDemo=null,replanning=false,readConnected=false,detailSerial=0;
const requestCache=new Map(),requestLoads=new Map();
const pendingCancels=new Map();
const READ_TIMEOUT_MS=3000;
let stateReadSerial=0,lastObservedRead=0;
let replanReadPending=false;
let historySignature=null;
const labels={interrupted:'중단',starting:'준비 중',cancel_requested:'중단 요청 중',passed:'검증 통과',ready:'실행 가능',queued:'접수',submitting:'전송 확인 대기',running:'진행 중',succeeded:'성공',failed:'실패',cancelled:'요청 취소 완료',rejected:'거부',blocked:'재시작 필요',unknown:'결과 미확인'};
const terminal=new Set(['succeeded','failed','cancelled','rejected','unknown']);
function guardPresentation(guard){
 if(!guard?.latched)return {text:guard?.phase==='ready'?'감시 중':'감시 상태 확인 중',confirmed:false};
 const reason=({contact:'접촉',clock_stall:'시계 정지'})[guard.reason]||'원인 미확인';
 const confirmed=guard.phase==='stopped'&&['contact','clock_stall'].includes(guard.reason);
 return {text:`${reason} · ${confirmed?'정지 확인':'정지 확인 중'}`,confirmed};
}
function telemetryExpected(data,kind){
 // No first sample is different from losing a sample after connection. The
 // backend clears these observation fields when explicitly starting a new run.
 const expected=data.lifecycle?.phase==='running'||data.simulation_verified||Boolean(data.active_request_id)||Boolean(data.server);
 const observed=kind==='joints'?data.positions_rad!=null||Number.isFinite(data.joint_state_stamp_simulation_sec)||Number.isFinite(data.state_age_sec):Number.isFinite(data.simulation_sec)||Number.isFinite(data.clock_age_sec);
 return expected||observed;
}
function observationProblem(data){
 if(data.guard?.latched)return guardPresentation(data.guard).text+'. 정지 잠금은 해제되지 않습니다. 원인을 확인한 뒤 시뮬레이션을 재시작하세요.';
 if(data.lifecycle?.error)return '실행 프로세스에 오류가 있습니다. 실행 로그를 확인하고 종료 상태를 확인하세요.';
 if(data.server?.status==='blocked')return '명령 서버가 실행을 차단했습니다. 실행 로그와 종료 상태를 확인한 뒤 시뮬레이션을 재시작하세요.';
 if(data.state_age_sec===null&&telemetryExpected(data,'joints')||Number.isFinite(data.state_age_sec)&&data.state_age_sec>.5)return '관절 상태 수신이 멈췄습니다. 연결과 최신 상태가 복구된 뒤 새 계획을 생성하세요.';
 if(data.clock_age_sec===null&&telemetryExpected(data,'clock')||Number.isFinite(data.clock_age_sec)&&data.clock_age_sec>.5)return '시뮬레이션 시계가 갱신되지 않습니다. 연결과 감시 상태를 확인하세요.';
 return null;
}
function connectionPresentation(data){
 if(graspSelected(data)){
  const job=data.task.grasp?.job;
  if(job?.finished&&job.owner_cleanup_clean!==true)return {status:'blocked',text:'집기 정리 결과 확인 필요'};
  if(job?.finished)return {status:job.status==='failed'?'blocked':'ready',text:(({cancelled:'집기 취소 완료',succeeded:'집기 작업 완료',failed:'집기 작업 실패'})[job.status]||'집기 작업 종료')+' · 마지막 결과'};
  return {status:job?'running':'ready',text:job?.cancel_requested?'집기 취소 · 정리 확인 중':job?'집기 작업 진행 중':'ROS 집기 작업 연결'};
 }
 const problem=observationProblem(data);
 if(problem)return {status:'blocked',text:data.guard?.latched?guardPresentation(data.guard).text:data.server?.status==='blocked'?'실행 차단 · 재시작 필요':data.lifecycle?.error?'실행 오류':data.state_age_sec===null||data.state_age_sec>.5?'관절 상태 수신 중단':'시계 갱신 중단'};
 if(demoActive(data.demo?.current))return {status:'running',text:data.demo.current.status==='cancel_requested'?'시연 중단 · 정리 확인 중':'시연 진행 중'};
 if(data.can_submit&&data.simulation_verified&&data.guard?.phase==='ready'&&data.server?.status==='ready'&&Number.isFinite(data.state_age_sec)&&Number.isFinite(data.clock_age_sec))return {status:'ready',text:'실행 가능'};
 if(data.active_request_id)return {status:'running',text:'요청 처리 중'};
 return {status:data.lifecycle?.phase==='starting'?'starting':'offline',text:data.lifecycle?.phase==='starting'?'시뮬레이터 준비 중 · 관절 상태 대기':data.simulation_verified?'연결됨 · 준비 확인':'시뮬레이션 연결 대기'};
}
function failureStageLabel(stage){return ({pose_ik:'TCP 목표 IK 계산',pose_fk_check:'IK 해의 TCP 확인',pose_planned_endpoint_fk:'계획 종점 TCP 확인',pose_execution_fk:'관측 TCP 완료 확인',state_snapshot:'시작 상태 확인',endpoint_clearance:'시작·목표 간격 검사',endpoint_validity:'시작·목표 충돌 검사',planning_both_arms:'양팔 경로 계획',planning_left_arm:'왼팔 경로 계획',planning_right_arm:'오른팔 경로 계획',departure_search:'출발 시점 탐색',safety_checks:'전체 경로 검사',state_recheck:'시작 상태 재확인',final_state_recheck:'실행 직전 상태 확인',dispatch_to_action_results:'궤적 전송·실행 응답',settling:'관절 정착 확인',approved_plan_recheck:'승인 계획 재검사'})[stage]||'원본 기록에서 확인';}
function requestGuidance(record,data={}){
 const kind=record?.failure_kind??data.failure_kind,stage=record?.failure_stage??data.failure_stage;
 const messages={
  ik_no_solution:['IK 해 없음','이번 TCP 목표를 만족하는 관절 해를 찾지 못했습니다.','선택한 팔, world 위치와 방향을 확인하고 목표를 수정하세요.'],
  ik_timeout:['IK 계산·응답 시간 초과','제한 시간 안에 IK 결과를 확인하지 못했습니다.','현재 연결과 목표를 확인한 뒤 새 계획을 직접 요청하세요.'],
  ik_transport:['IK 연결 오류','MoveIt IK 서비스의 응답을 받지 못했습니다.','명령 노드와 IK 서비스 연결을 확인하세요. 자동으로 재전송하지 않습니다.'],
  ik_response_invalid:['IK 응답 거부','IK 응답의 관절 이름·한계·수치가 검증 조건과 다릅니다.','원본 오류와 모델 등록 정보를 확인하세요.'],
  ik_failed:['IK 계산 실패','MoveIt가 이번 TCP 목표의 IK 계산을 완료하지 못했습니다.','목표와 기준 링크, 원본 오류를 확인하세요.'],
  fk_goal_error:['TCP 목표 오차','IK 해를 FK로 다시 계산한 TCP가 목표 오차 조건을 만족하지 않습니다.','world 목표와 등록한 TCP 기준점을 확인하고 새 계획을 생성하세요.'],
  pose_execution_tolerance:['TCP 도착 오차','관절 동작은 끝났지만 관측한 TCP가 위치 5 mm·방향 0.03 rad의 완료 조건을 만족하지 않았습니다.','실제 자세와 실행 기록을 확인하세요. 완료로 간주해 다음 작업을 시작하지 마세요.'],
  fk_timeout:['FK 응답 시간 초과','제한 시간 안에 FK 검증을 마치지 못했습니다.','FK 서비스와 연결 상태를 확인하세요.'],
  fk_transport:['FK 연결 오류','IK 해의 FK 재검증 응답을 받지 못했습니다.','연결을 확인한 뒤 새 계획을 요청하세요.'],
  fk_response_invalid:['FK 응답 거부','기준 링크나 world 좌표의 FK 응답을 확인하지 못했습니다.','모델·기준 링크와 원본 오류를 확인하세요.'],
  budget_exhausted:['계산 시간 초과','정해진 계산 시간 안에 계획과 검사를 끝내지 못했습니다.','목표와 경로 생성 방식을 확인한 뒤 새 계획을 직접 요청하세요.'],
  candidate_exhausted:['계획 후보 검사 종료','검사한 후보 중 조건을 만족하는 경로를 찾지 못했습니다. 도달 불가능하다는 판정은 아닙니다.','목표 자세와 환경 간격을 확인한 뒤 새 계획을 요청하세요.'],
  departure_exhausted:['출발 시점 탐색 실패','현재 경로에서 조건을 만족하는 출발 시점을 찾지 못했습니다.','목표나 경로 생성 방식을 검토한 뒤 새 계획을 요청하세요.'],
  invalid_start:['시작 자세 조건 위반','현재 시작 자세가 간격 또는 충돌 검사 조건을 만족하지 않습니다.','시작 자세와 작업 환경을 확인하세요. 같은 목표의 반복 실행으로 해결하지 않습니다.'],
  invalid_goal:['목표 자세 조건 위반','목표 자세가 간격 또는 충돌 검사 조건을 만족하지 않습니다.','목표 자세를 수정하고 새 계획을 생성하세요.'],
  planner_failed:['경로 계획 실패','계획기가 이번 요청의 경로를 만들지 못했습니다.','목표와 계획 방식을 확인하세요. 실행을 자동으로 다시 요청하지 않습니다.'],
  state_changed:['시작 자세 변경','계획할 때와 실행 직전의 시작 자세가 달라졌습니다.','최신 상태를 확인한 뒤 새 계획을 생성하고 다시 검토하세요.'],
  review_invalidated:['검토 계획 사용 불가','계획 만료 또는 모델·환경·세션·상태 변경으로 기존 승인을 사용할 수 없습니다.','변경 원인을 확인한 뒤 새 계획을 생성하고 다시 검토하세요.'],
  execution_environment_changed:['설치 환경 변경','선택한 설치 환경의 파일 또는 실제 연결이 시작할 때와 달라졌습니다. 기존 계획을 실행할 수 없습니다.','환경을 다시 확인하고 같은 타깃으로 시뮬레이션과 명령 서버를 재시작한 뒤 새 계획을 검토하세요.'],
  execution_failed:['실행 실패','궤적 실행을 정상적으로 마치지 못했습니다.','정지 감시와 실행 로그를 확인하세요. 실행 차단 상태이면 재시작이 필요합니다.'],
  environment_clearance:['설비 간격 조건 위반','경로 또는 관측 상태의 설비 간격이 현재 시뮬레이션 정책을 만족하지 않습니다.','설비 배치와 목표, 간격 조건을 확인하고 새 계획을 검토하세요. 자동으로 우회하거나 재실행하지 않습니다.'],
  self_clearance:['팔 내부 간격 부족','경로 또는 관측 자세에서 같은 팔의 링크 사이 간격이 시뮬레이션 검사 조건을 만족하지 않습니다.','현재 자세와 목표를 확인하고 새 계획을 검토하세요.'],
  planning_error:['계획·검사 오류','계획 또는 실행 전 검사에서 오류가 발생했습니다.','실패 단계와 원본 오류를 확인한 뒤 새 계획 여부를 결정하세요.']};
 if(record?.status==='cancelled')return {title:'요청 취소 완료',text:'서버의 요청 처리가 취소로 끝났습니다. 물리적 정지 확인과는 구분합니다.',next:'최신 관절 상태와 접촉·시계 감시를 확인하세요.'};
 if(record?.status==='unknown')return {title:'종료 결과 미확인',text:'서버 교체나 연결 변경으로 이 요청의 최종 결과를 확인하지 못했습니다.',next:'현재 실행과 로그를 확인하세요. 이전 명령을 자동으로 재전송하지 않습니다.'};
 if(!['failed','rejected'].includes(record?.status))return null;
 const [title,text,next]=messages[kind]||['요청 실패','요청이 정상적으로 완료되지 않았습니다.','실패 단계와 원본 오류, 현재 연결 상태를 확인하세요.'];
 return {title,text,next,stage:stage?failureStageLabel(stage):null};
}
function guidanceText(record,data={}){
 const guidance=requestGuidance(record,data),raw=record?.error?.message||data.error?.message||'';
 return [...(guidance?[guidance.title+' · '+guidance.text,guidance.stage?'실패 단계: '+guidance.stage:null,guidance.next]:[]),raw?'원본 오류: '+raw:null].filter(Boolean).join(' ');
}
function publishCancellation(identifier=state?.active_request_id){
 const entry=identifier?pendingCancels.get(identifier):null;
 window.dispatchEvent?.(new CustomEvent('console-cancellation',{detail:entry?{request_id:identifier,phase:entry.phase}:null}));
}
function cancellationPhase(record){
 if(!record)return null;
 if(terminal.has(record.status)){pendingCancels.delete(record.request_id);return null;}
 if(record.stage==='cancel_received')return 'acknowledged';
 return pendingCancels.get(record.request_id)?.phase||null;
}
async function cancelCurrentRequest(){
 if(graspSelected()){await taskControls?.command('cancel');return;}
 const identifier=state?.active_request_id;if(!readConnected||!identifier||demoActive(state.demo?.current)||pendingCancels.has(identifier))return;
 const entry={phase:'submitting',session:state.server?.session_id||null};pendingCancels.set(identifier,entry);publishCancellation(identifier);refreshControls();requestProgress(state.history.find(item=>item.request_id===identifier));
 try{const result=await request('/api/cancel',{request_id:identifier});
  if(result.request_id!==identifier||result.status!=='cancel_requested')throw Error('취소 요청의 처리 결과가 일치하지 않습니다.');
  if(pendingCancels.get(identifier)===entry)entry.phase='submitted';
  notice('취소를 요청했습니다. 서버 수신과 요청 종료, 정지 감시 상태를 확인하세요.');
 }catch(error){if(pendingCancels.get(identifier)===entry)entry.phase='unconfirmed';notice('취소 요청의 전달 결과를 확인하지 못했습니다. 현재 상태와 로그를 확인하세요. 자동으로 다시 보내지 않습니다. '+error.message);}
 finally{publishCancellation(identifier);await poll();refreshControls();}
}
function notice(text){$('notice-text').textContent=text;$('notice').hidden=false;}
async function request(path,body){
 const read=body===undefined,controller=read?new AbortController():null;
 const selection=state?.task?.selection_id,selectionHeader=selection&&!path.startsWith('/api/task/')&&!/^\/api\/sync-recordings\/[^/]+\/control$/.test(path)?{'X-Workspace-Selection':selection}:{};
 const options=read?{signal:controller.signal,cache:'no-store'}:{method:'POST',headers:{'Content-Type':'application/json','X-Operator-Token':token,...selectionHeader},body:JSON.stringify(body)};
 const timer=read?setTimeout(()=>controller.abort(),READ_TIMEOUT_MS):null;
 try{const response=await fetch(path,options);const data=await response.json();
  if(!response.ok){const error=Error(data.error||`HTTP ${response.status}`);error.httpStatus=response.status;throw error;}return data;
 }catch(error){if(controller?.signal.aborted){const timeout=Error('상태 조회가 지연되었습니다. 연결을 다시 확인하고 있습니다.');timeout.code='read_timeout';throw timeout;}throw error;}
 finally{if(timer!==null)clearTimeout(timer);}
}
function goalKind(){return $('goal-kind')?.value==='pose'?'pose':'joint';}
function poseToolRow(){return config?.pose_tools?.find(row=>row.arm_id===$('pose-arm')?.value)||null;}
function poseGoalPayload(identifier){
 const row=poseToolRow();if(!row?.tool?.sha256)throw Error('팔과 등록한 TCP 기준점을 선택하세요.');
 const number=id=>{const raw=$(id)?.value;if(raw===undefined||raw.trim()===''||!Number.isFinite(Number(raw)))throw Error('TCP 위치와 방향을 모두 유한한 숫자로 입력하세요.');return Number(raw);};
 const position=['pose-x','pose-y','pose-z'].map(number),q=['pose-qx','pose-qy','pose-qz','pose-qw'].map(number),norm=Math.hypot(...q);
 if(!Number.isFinite(norm)||Math.abs(norm-1)>1e-6)throw Error('방향 quaternion xyzw의 길이는 1이어야 합니다. 도·rad 각도를 직접 입력하지 마세요.');
 return {schema_version:1,request_id:identifier,arm_id:row.arm_id,tool_id:row.tool.id,tool_sha256:row.tool.sha256,frame_id:'world',position_m:position,quaternion_xyzw:q.map(v=>v===0?0:v/norm)};
}
function goalPayload(identifier){return goalKind()==='pose'?poseGoalPayload(identifier):{schema_version:1,request_id:identifier,strategy:$('strategy').value,target_rad:{...targets}};}
function goalValidationReason(){
 if(goalKind()==='joint')return validTargets()?null:'모든 목표를 관절 한계 안의 숫자로 입력하세요.';
 try{poseGoalPayload('0'.repeat(32));return null;}catch(error){return error.message;}
}
function validGoal(){return goalValidationReason()===null;}
function canPlanGoal(data=state){if(motionUnavailable(data))return false;return goalKind()==='pose'?data?.can_pose_plan:data?.can_plan;}
function poseSignature(payload){return JSON.stringify({arm_id:payload.arm_id,tool_id:payload.tool_id,tool_sha256:payload.tool_sha256,frame_id:payload.frame_id,position_m:payload.position_m,quaternion_xyzw:payload.quaternion_xyzw});}
function goalMatchesPayload(payload){
 if(payload?.arm_id){if(goalKind()!=='pose')return false;try{return poseSignature(payload)===poseSignature(poseGoalPayload(payload.request_id));}catch{return false;}}
 return goalKind()==='joint'&&payload&&payload.strategy===$('strategy').value&&Object.keys(targets).every(name=>payload.target_rad?.[name]===targets[name]);
}
function renderPoseMetadata(){
 const row=poseToolRow(),node=$('pose-tool');if(node){node.textContent=row?`${row.tool.id} · 기준 링크 ${row.tool.parent_tip} · SHA ${row.tool.sha256.slice(0,12)}…`:'이 실행 대상에는 등록한 TCP 기준점이 없습니다.';node.title=row?.tool?.sha256||'';}
 if($('pose-support'))$('pose-support').textContent=state?.can_pose_plan?'Pose 계획 지원과 TCP 등록 정보가 일치합니다.':state?.pose_reason||state?.reason||'명령 노드의 Pose 계획 지원과 등록 정보를 확인하고 있습니다.';
}
function configurePoseInputs(){
 const select=$('pose-arm');if(!select)return;select.replaceChildren();
 for(const row of config.pose_tools||[]){const option=textNode('option',`${row.label||row.arm_id} · ${row.tool.id}`);option.value=row.arm_id;select.append(option);}
 if(!select.options.length){const option=textNode('option','등록된 TCP 기준점 없음');option.value='';select.append(option);}
 renderPoseMetadata();
}
function setGoalKind(){
 if($('joint-goal-panel'))$('joint-goal-panel').hidden=goalKind()!=='joint';
 if($('pose-goal-panel'))$('pose-goal-panel').hidden=goalKind()!=='pose';
 if($('execute'))$('execute').textContent=goalKind()==='pose'?'① IK 확인·계획 생성':'① 계획 생성';
 renderPoseMetadata();refreshControls();
}
function strategyLabel(strategy){return ({coordinated:'양팔 공동 계획',departure_timing:'팔별 경로와 출발 시점 조정',tcp_pose:'TCP Pose · 선택 팔 계획'})[strategy]||'계획 방식 확인 중';}
function summaryOf(record,data){
 const summary=data?.review_summary||record?.review_summary||{},command=data?.command||record?.command,initial=data?.initial_state_rad;
 let maximum=summary.max_abs_target_delta_rad;
 if(!Number.isFinite(maximum)&&initial&&command?.target_rad){const changes=Object.keys(command.target_rad).map(name=>Number.isFinite(initial[name])?Math.abs(command.target_rad[name]-initial[name]):NaN);if(changes.length&&changes.every(Number.isFinite))maximum=Math.max(...changes);}
 return {strategy:(data?.pose_goal||record?.pose_goal)?'tcp_pose':summary.strategy||command?.strategy,duration:summary.duration_sec??data?.planned_clearance?.duration_sec,
  minimum:summary.minimum||data?.planned_clearance?.minimum,maximum};
}
function needsPlanSummary(record){return record?.mode==='plan_only'&&record.status==='succeeded'&&record.stage!=='plan_not_required';}
function completePlanSummary(data){const summary=summaryOf(data,data);return Number.isFinite(summary.duration)&&Number.isFinite(summary.minimum?.distance_m)&&Number.isFinite(summary.maximum);}
function shouldLoadRequest(cached,expected,refresh=false){
 if(!cached)return true;
 if(expected&&(cached.data.status!==expected.status||cached.data.stage!==expected.stage))return true;
 if(needsPlanSummary(expected)&&!completePlanSummary(cached.data))return (cached.incompleteReads||0)<3&&Date.now()-cached.at>=1500;
 return refresh&&Date.now()-cached.at>=1500;
}
async function loadRequest(id,{refresh=false,expected=null,force=false}={}){
 if(requestLoads.has(id))return requestLoads.get(id);
 const cached=requestCache.get(id);if(!force&&!shouldLoadRequest(cached,expected,refresh))return cached.data;
 const pending=request('/api/requests/'+id).then(data=>{const incomplete=needsPlanSummary(expected)&&data.status===expected.status&&data.stage===expected.stage&&!completePlanSummary(data);
  requestCache.delete(id);requestCache.set(id,{at:Date.now(),data,incompleteReads:incomplete?(cached?.incompleteReads||0)+1:0});while(requestCache.size>40)requestCache.delete(requestCache.keys().next().value);return data;}).finally(()=>{if(requestLoads.get(id)===pending)requestLoads.delete(id);});
 requestLoads.set(id,pending);return pending;
}
function textNode(tag,text,cls){const el=document.createElement(tag);el.textContent=text;if(cls)el.className=cls;return el;}
function value(v,decimals=3,suffix=' s'){return typeof v==='number'&&Number.isFinite(v)?v.toFixed(decimals)+suffix:'—';}
function displayed(v){return unit==='deg'?v*180/Math.PI:v;}
function radians(v){return unit==='deg'?v*Math.PI/180:v;}
function badge(el,status,text){el.className='badge'+(status==='succeeded'||status==='passed'||status==='ready'?' good':status==='failed'||status==='blocked'||status==='rejected'?' bad':status==='running'||status==='starting'||status==='cancel_requested'||status==='queued'||status==='submitting'?' busy':'');el.textContent=text||labels[status]||status;}
function show(view,{focusTask=false}={}){
 const section=view==='review'?'operate':view;
 document.querySelectorAll('.view').forEach(el=>el.hidden=el.id!==section);
 document.querySelectorAll('[data-view]').forEach(el=>{el.classList.toggle('active',el.dataset.view===view);if(el.dataset.view===view)el.setAttribute('aria-current','page');else el.removeAttribute('aria-current');});
 if(section==='operate')$('operate').dataset.workspaceMode=view;
 $('view-title').textContent={operate:'로봇 조작',review:'계획 검토',demos:'동작 시연',history:'실행 기록',reports:'평가 리포트',analysis:'기록 분석'}[view];
 window.dispatchEvent?.(new CustomEvent('console-navigation',{detail:{view}}));
 if(state)updateLiveFrames(state);sceneControls?.mount();
 if(focusTask)$('task-title')?.focus?.({preventScroll:true});
}
function fields(){
 const container=$('joint-fields');container.replaceChildren();$('unit-label').textContent=unit;
 config.instances[arm].joints.forEach((name,index)=>{
  const row=textNode('div','','joint-row'),label=textNode('label','','joint-name');label.htmlFor='target-'+name;
  label.append(textNode('span',`J${index+1} · ${name.replace(/^(left_|right_)/,'')}`));
  const limit=config.joint_limits[name];label.append(textNode('small',`${displayed(limit.lower).toFixed(1)} ~ ${displayed(limit.upper).toFixed(1)} ${unit}`));
  label.title=name;
  const current=textNode('span','—','joint-current');current.dataset.current=name;
  const input=document.createElement('input');input.type='number';input.id='target-'+name;input.dataset.joint=name;
  input.setAttribute('aria-label',name+' 목표 '+unit);input.step='any';input.inputMode='decimal';
  input.min=displayed(limit.lower);input.max=displayed(limit.upper);input.required=true;input.value=displayed(targets[name]).toFixed(unit==='deg'?2:4);
  input.addEventListener('input',()=>{targets[name]=input.value===''?NaN:radians(Number(input.value));refreshControls();});
  row.append(label,current,input);container.append(row);
 });
 $('arm-left').setAttribute('aria-selected',arm===0);$('arm-right').setAttribute('aria-selected',arm===1);updateCurrent();
}
function updateCurrent(){document.querySelectorAll('[data-current]').forEach(el=>{const v=state?.positions_rad?.[el.dataset.current];el.textContent=typeof v==='number'?displayed(v).toFixed(unit==='deg'?2:4):'—';});}
function renderContext(){
 if(graspSelected()){const source=state.task.grasp?.source,job=state.task.grasp?.job;$('display-context').textContent=(state.view?.mode==='model_preview'?'실행 전 모델':'Native 실제 관측')+' · '+(source?.label||'ROS 집기 모델');$('control-context').textContent=(source?.label||'ROS 집기')+' · ROS DOMAIN '+source?.domain;$('context-note').textContent=!readConnected?'상태 조회 중단 · 마지막 관측과 현재 실행을 구분합니다.':job?.finished?'종료한 집기 작업의 마지막 관측입니다. 새 실행의 현재 상태가 아닙니다.':job?.cancel_requested?'취소 전달·제어기 정지·프로세스 정리를 각각 확인합니다.':'고정 모델의 집기·운반·놓기를 이 작업실에서 실행합니다. 일반 관절·TCP 조작과 동시에 실행하지 않습니다.';return;}
 const view=state?.view,source=view?.source,mode=view?.mode,recorded=source?.kind==='recording',demo=demoActive(state?.demo?.current);
 const label=source?.label||view?.catalog?.find(item=>item.id===source?.id)?.label||config?.label||'표시 대상 확인 중';
 const kind=recorded?'기록 재생':mode==='model_preview'?'모델 미리보기':mode==='plan_preview'?(view.review_plan?'실행 전 계획':'저장 궤적'):demo?'실시간 시연':'실제 관절 상태';
 $('display-context').textContent=kind+' · '+label;
 $('display-context').dataset.mode=mode||'live';
 $('control-context').textContent=recorded?'실행 연결 없음':(config?.execution_target?config.label+' · '+config.execution_target.environment_id:config?.label)||'연결 확인 중';
 $('context-note').textContent=!readConnected?'상태 조회가 끊겼습니다. 마지막 표시를 유지하며 새 실행을 차단합니다.':recorded?'기록을 표시하는 읽기 전용 화면입니다.':mode==='model_preview'?'표시 모델을 바꿔도 실행 대상은 바뀌지 않습니다.':mode==='plan_preview'?'3D 화면은 계획 자세입니다. 관절표의 현재 값과 감시 상태는 실제 수신 상태입니다.':demo?'시연 중에는 별도 관절 목표를 보낼 수 없습니다.':'시뮬레이션 전용 · 실행은 계획 검토와 승인 후 시작합니다.';
 $('joint-current-label').textContent='실제 현재';
}
function validTargets(){return config&&Object.entries(config.joint_limits).every(([n,l])=>Number.isFinite(targets[n])&&targets[n]>=l.lower&&targets[n]<=l.upper);}
function refreshControls(){
 if(!state)return;
 const taskBusy=motionUnavailable();
 const owned=state.lifecycle.owned,phase=state.lifecycle.phase,transition=['starting','stopping'].includes(phase),demoBusy=demoActive(state.demo?.current);
 $('start').disabled=taskBusy||!readConnected||Boolean(state.view?.preview_active)||demoBusy||transition||owned.simulation||owned.commands||Object.values(state.graph).some(v=>v>0)||performance.now()<2000;
 $('stop').disabled=taskBusy||!readConnected||demoBusy||transition||!(owned.simulation||owned.commands);
 $('attach').hidden=!(state.simulation_verified&&state.graph.status===0&&!owned.commands);$('attach').disabled=taskBusy||!readConnected||Boolean(state.view?.preview_active)||demoBusy||transition;
 $('execute').disabled=replanning||!readConnected||demoBusy||Boolean(observationProblem(state))||!canPlanGoal()||!validGoal()||!sceneControls;const cancelRecord=state.history.find(item=>item.request_id===state.active_request_id),cancelPhase=cancellationPhase(cancelRecord);document.querySelectorAll('[data-cancel]').forEach(el=>{el.disabled=!readConnected||demoBusy||!state.active_request_id||Boolean(cancelPhase);el.textContent=cancelPhase==='acknowledged'?'취소 수신 · 종료 확인 중':cancelPhase==='unconfirmed'?'취소 전달 결과 미확인':cancelPhase?'취소 요청 전달 중':'■ 양팔 동작 취소';});
 $('use-current').disabled=taskBusy||!readConnected||!state.positions_rad||state.state_age_sec>0.5;
 $('planning-case').disabled=demoBusy||transition||owned.simulation||state.graph.clock>0;$('native-gui').disabled=$('planning-case').disabled;
 $('admission').textContent=demoBusy?'동작 시연 중입니다. 시연을 마치거나 중단한 뒤 목표를 전송하세요.':goalValidationReason()||(!sceneControls?'계획 검토에는 로컬 3D 표시가 필요합니다.':observationProblem(state)||(canPlanGoal()?'계획을 생성합니다. 경로를 확인하고 승인해야 로봇이 움직입니다.':(goalKind()==='pose'?state.pose_reason||state.reason:state.reason)||'계획 검토 명령 노드의 연결을 확인하세요.'));
 renderPoseMetadata();
 renderReview();
 if(taskBusy){for(const id of ['execute','review-preview','review-approve','review-discard','review-replan','demo-run','demo-stop'])$(id).disabled=true;}
 taskControls?.render();
}
function reviewMatchesTargets(review){const record=state?.history.find(item=>item.request_id===review?.plan_request_id),original=record?.pose_goal||record?.command;return goalMatchesPayload(original);}
function renderReview(){
 if(graspSelected()){ $('review-card').hidden=true;return;}
 const review=state?.pending_plan;$('review-card').hidden=!review;if(!review)return;
 const waiting=review.state==='awaiting_approval'&&review.remaining_sec>0,busy=replanning||Boolean(state.active_request_id)||demoActive(state.demo?.current),matches=reviewMatchesTargets(review);
 const preview=state.view?.review_plan===true&&state.view?.binding?.request_id===review.plan_request_id&&state.view?.mode==='plan_preview';
 const visible=preview&&sceneControls?.cameraReady();
 badge($('review-badge'),waiting?'ready':'blocked',({awaiting_approval:'승인 대기',expired:'만료',invalidated:'무효화',consumed:'사용 완료',approval_queued:'승인 전송 중'})[review.state]||'다시 계획 필요');
 $('review-id').textContent=review.plan_request_id;
 $('review-summary').textContent=waiting?`계획과 검사가 끝났습니다. 실행 승인까지 ${Math.ceil(review.remaining_sec)}초 남았습니다.`:'이 계획은 실행할 수 없습니다. 실시간 보기로 돌아가 새 계획을 생성하세요.';
 $('review-preview').disabled=!readConnected||!waiting||busy||!sceneControls||state.view?.display_mode==='image';
 $('review-discard').disabled=!readConnected||!waiting||busy;
 $('review-approve').disabled=!readConnected||Boolean(observationProblem(state))||!waiting||!state.can_approve||!visible||!matches||busy;
 $('review-replan').disabled=!readConnected||busy||!validGoal()||!sceneControls||['starting','stopping'].includes(state.lifecycle?.phase);
 const record=state.history.find(item=>item.request_id===review.plan_request_id),data=requestCache.get(review.plan_request_id)?.data,summary=summaryOf(record,data);
 if(record&&!requestLoads.has(review.plan_request_id)&&shouldLoadRequest(requestCache.get(review.plan_request_id),record))loadRequest(review.plan_request_id,{expected:record}).then(()=>{if(state?.pending_plan?.plan_request_id===review.plan_request_id)renderReview();}).catch(()=>{if(state?.pending_plan?.plan_request_id===review.plan_request_id)$('review-detail').textContent='계획 수치를 불러오지 못했습니다. 계획 상세에서 다시 확인하세요.';});
 const pose=data?.pose_goal||record?.pose_goal,poseContext=$('review-pose-context');if(poseContext){poseContext.hidden=!pose;poseContext.textContent=pose?`TCP 목표: ${pose.arm_id} · ${pose.tool_id} · world · 위치 ${pose.position_m.map(v=>Number(v).toPrecision(5)).join(', ')} m · quaternion xyzw ${pose.quaternion_xyzw.map(v=>Number(v).toPrecision(5)).join(', ')}. IK 결과를 포함한 검토 경로만 승인합니다.`:'';}
 $('review-strategy').textContent=strategyLabel(summary.strategy);$('review-duration').textContent=value(summary.duration,2,' s');
 $('review-distance').textContent=value(Number.isFinite(summary.minimum?.distance_m)?summary.minimum.distance_m*1000:undefined,1,' mm');
 $('review-delta').textContent=value(Number.isFinite(summary.maximum)?summary.maximum*180/Math.PI:undefined,1,'°');
 $('review-detail').textContent=data&&completePlanSummary(data)?'궤적 시간은 시뮬레이션 초입니다. 관절 변화는 계획 생성 당시의 시작→목표 차이입니다.':(requestCache.get(review.plan_request_id)?.incompleteReads||0)>=3?'계획 수치를 읽지 못했습니다. 계획 상세 보기에서 다시 확인하세요.':'계획 수치를 불러오고 있습니다.';
 $('review-admission').textContent=!waiting?'승인은 한 번만 사용할 수 있습니다.':observationProblem(state)||(!matches?'목표나 계획 방식을 바꿨습니다. 실시간 보기로 돌아가 새 계획을 생성하세요.':!preview?'경로 미리보기를 열고 확인하세요.':!visible?'검토할 경로를 로컬 3D 화면에 표시하고 있습니다.':!state.can_approve?'현재 상태에서 실행을 승인할 수 없습니다. 연결과 감시 상태를 확인하세요.':'미리보기 재생은 로봇을 움직이지 않습니다. 실행하려면 승인·실행 버튼을 누르세요.');
}
async function replan(){
 if(motionUnavailable()||replanning||!readConnected||!validGoal()||!sceneControls||state?.active_request_id||demoActive(state?.demo?.current))return;
 replanning=true;refreshControls();
 const payload=goalPayload(crypto.randomUUID().replaceAll('-','')),poseRequest=Boolean(payload.arm_id);
 try{
  sceneControls.pause();
  await request('/api/view/select',{mode:'live',source_id:state.view.config_source_id});
  const expectedRead=stateReadSerial+1;
  let current;
  replanReadPending=true;
  try{current=await readState();}finally{replanReadPending=false;}
  if(!(poseRequest?current.can_pose_plan:current.can_plan))throw Error((poseRequest?current.pose_reason||current.reason:current.reason)||'현재 상태에서 새 계획을 생성할 수 없습니다.');
  if(!goalMatchesPayload(payload))throw Error('목표 종류나 값을 바꿨습니다. 새 목표로 계획 생성을 다시 눌러주세요.');
  if(!readConnected||expectedRead!==stateReadSerial||expectedRead!==lastObservedRead||current!==state||!(poseRequest?state.can_pose_plan:state.can_plan)||state.active_request_id||demoActive(state.demo?.current))throw Error('상태 확인이 바뀌었습니다. 최신 연결과 실행 상태를 확인한 뒤 다시 계획하세요.');
  // The explicit click creates a new plan only. Approval is a separate action.
  await request(poseRequest?'/api/pose/plan':'/api/plan',payload);latestRequest=null;await poll();
 }catch(error){notice(error.message);await poll();}
 finally{replanning=false;refreshControls();}
}
function requestProgress(record){
 if(!record)return;
 badge($('request-badge'),record.status,record.stage==='plan_not_required'?'이동 불필요':record.mode==='plan_only'&&record.status==='succeeded'?'계획 생성 완료':undefined);$('request-id').textContent=record.request_id.slice(0,12)+'…';
 const stages={submitted:'명령 노드 수신 확인 대기',received:'목표 수신',state_snapshot:'시작 자세 확인',computation_started:'경로 계산 시작',pose_ik:'TCP 목표 IK 계산',planning_both_arms:'양팔 공동 경로 계획',planning_left_arm:'FR5 경로 계획',planning_right_arm:'UR5e 경로 계획',departure_search:'출발 타이밍 탐색',safety_checks:'거리·충돌 검사',computation_done:'계획·검사 완료',dispatch:'동작 예약·명령 전송',first_motion:'관절 이동 확인',first_motion_left:'FR5 이동 확인',first_motion_right:'UR5e 이동 확인',actions_done:'양팔 Action 완료',settled:'관절 수렴 확인',terminal:labels[record.status],cancel_received:'양팔 취소 요청 수신'};
 $('request-stage').textContent=({plan_ready:'계획·검사 완료 · 실행 승인 대기',plan_not_required:'이미 목표 자세에 있습니다. 실행할 이동이 없습니다.',plan_revoked:'미승인 계획 폐기',approval_validating:'승인 계획 재검사'})[record.stage]||stages[record.stage]||record.stage||labels[record.status];
 const cancelPhase=cancellationPhase(record);if(cancelPhase)$('request-stage').textContent=cancelPhase==='acknowledged'?'서버가 취소를 수신했습니다. 요청 종료를 확인하고 있습니다.':cancelPhase==='unconfirmed'?'취소 전달 결과 미확인 · 현재 실행 상태를 확인하세요.':'취소 요청 전달 중 · 서버 수신과 요청 종료를 확인하세요.';
 const stage=record.stage||'';let progress=0;
 if(/planning|computation|search|safety/.test(stage))progress=1;
 if(/dispatch|action|motion|settled|cancel/.test(stage))progress=2;
 if(record.status==='succeeded')progress=record.mode==='plan_only'?1:3;
 document.querySelectorAll('[data-step]').forEach(el=>el.classList.toggle('reached',Number(el.dataset.step)<=progress));
 const detail=requestCache.get(record.request_id)?.data,message=guidanceText(record,detail);$('request-error').hidden=!message;$('request-error').className=['failed','rejected','unknown'].includes(record.status)?'error-text':'help';$('request-error').textContent=message;
 $('first-motion').textContent=value(record.metrics?.received_to_first_motion_wall_sec);
 $('terminal-time').textContent=value(record.metrics?.received_to_terminal_wall_sec);
 if(latestRequest!==record.request_id){latestRequest=record.request_id;detailsAt=0;$('clearance').textContent='—';}
 if(Date.now()-detailsAt>1500){detailsAt=Date.now();loadRequest(record.request_id,{refresh:!terminal.has(record.status),expected:record}).then(data=>{
  if(latestRequest===data.request_id){$('clearance').textContent=value(data.planned_clearance?.minimum?.distance_m*1000,1,' mm');const current=state?.history.find(item=>item.request_id===data.request_id)||record;$('request-error').textContent=guidanceText(current,data);$('request-error').hidden=!$('request-error').textContent;renderReview();}
 }).catch(()=>{});}
}
function history(records){
 const groups=groupRequests(records),signature=JSON.stringify({records,task:state?.task?.selected,active:state?.active_request_id,demoBusy:demoActive(state?.demo?.current),local3d:Boolean(sceneControls),pending:state?.pending_plan?.state,
  summaries:groups.map(group=>{const record=group.plan||group.records[0];return summaryOf(record,requestCache.get(record.request_id)?.data);})});
 if(signature===historySignature)return;historySignature=signature;
 $('history-count').textContent=groups.length;$('history-empty').hidden=groups.length>0;
 const body=$('history-body');body.replaceChildren();
 for(const group of groups){
  const record=group.executions[0]||group.plan||group.records[0],summary=summaryOf(group.plan||record,requestCache.get((group.plan||record).request_id)?.data);
  const row=document.createElement('tr');row.dataset.requestId=record.request_id;row.dataset.planRequestId=group.id;
  const title=document.createElement('td');title.append(textNode('strong',group.executions.length?'승인 실행 요청':group.plan?'계획 검토':'직접 실행 기록'),textNode('small',strategyLabel(summary.strategy)),textNode('code',group.id.slice(0,10)+'…'));row.append(title);
  const result=document.createElement('td'),tag=document.createElement('span'),pending=state?.pending_plan?.plan_request_id===group.id?state.pending_plan:null;
  const planStatus=!group.executions.length&&pending?({awaiting_approval:'승인 대기',expired:'계획 만료',invalidated:'계획 무효화',approval_queued:'승인 전송 중'})[pending.state]:null;
  badge(tag,record.status,planStatus||(!group.executions.length&&group.plan&&record.status==='succeeded'?'계획 생성':undefined));result.append(tag,textNode('small',group.executions.length?`계획 1건 · 실행 요청 ${group.executions.length}건`:group.plan?'실행하지 않은 계획':record.stage||'상태 확인 중'));row.append(result);
  row.append(textNode('td',value(group.executions[0]?.metrics?.received_to_first_motion_wall_sec)),textNode('td',value((group.executions[0]||record).metrics?.received_to_terminal_wall_sec)),textNode('td',value(Number.isFinite(summary.minimum?.distance_m)?summary.minimum.distance_m*1000:undefined,1,' mm')));
  const actions=document.createElement('td'),button=textNode('button','작업 상세');button.addEventListener('click',()=>detail(record.request_id));actions.append(button);
  if(sceneControls&&record.status==='succeeded'&&record.trace_path&&record.mode!=='plan_only'){const preview=textNode('button','저장 궤적');preview.disabled=motionUnavailable()||Boolean(state?.active_request_id)||demoActive(state?.demo?.current);preview.addEventListener('click',()=>sceneControls.previewRequest(record.request_id));actions.append(preview);}row.append(actions);body.append(row);
 }
}
function groupRequests(records){
 const groups=new Map();
 for(const record of records){const id=record.plan_request_id||record.approval?.plan_request_id||record.request_id;if(!groups.has(id))groups.set(id,{id,plan:null,executions:[],records:[]});const group=groups.get(id);group.records.push(record);if(record.mode==='plan_only')group.plan=record;else if(id!==record.request_id)group.executions.push(record);}
 return [...groups.values()];
}
function detailField(label,text){const item=textNode('div','','detail-field');item.append(textNode('dt',label),textNode('dd',text));return item;}
function renderDetail(group,id){
 const base=group.records.find(item=>item.request_id===id)||group.records[0],data=requestCache.get(id)?.data||base,record={...data,...base},plan=group.plan,
  planData=plan?requestCache.get(plan.request_id)?.data:null,summary=summaryOf(plan||record,planData||data);
 if(terminal.has(data.status)&&!terminal.has(base.status)){record.status=data.status;record.error=data.error;record.metrics=data.metrics;}
 $('history-detail').hidden=false;$('detail-title').textContent=group.executions.length?'계획과 승인 실행':group.plan?'실행 전 계획 기록':'실행 기록';
 $('detail-error').textContent=guidanceText(record,data);
 const fields=$('detail-summary');fields.replaceChildren();fields.append(detailField('결과',labels[record.status]||record.status),detailField('계획 방식',strategyLabel(summary.strategy)),detailField('계획 궤적 시간',value(summary.duration,2,' s')),detailField('계획 양팔 간격',value(Number.isFinite(summary.minimum?.distance_m)?summary.minimum.distance_m*1000:undefined,1,' mm')),detailField('시작→목표 최대 변화',value(Number.isFinite(summary.maximum)?summary.maximum*180/Math.PI:undefined,1,'°')),detailField('요청 처리 시간',value(record.metrics?.received_to_terminal_wall_sec)));
 const completion=data?.pose_completion||record?.pose_completion;
 if(completion?.errors){fields.append(detailField('관측 TCP 위치 오차',value(completion.errors.position_error_m*1000,3,' mm')),detailField('관측 TCP 방향 오차',value(completion.errors.orientation_error_rad,5,' rad')),detailField('TCP 완료 기준',`${value(completion.position_tolerance_m*1000,1,' mm')} / ${value(completion.orientation_tolerance_rad,3,' rad')}`));}
 const planned=(planData||data)?.pose_kinematics?.planned_endpoint_errors;
 if(planned){fields.append(detailField('계획 TCP 위치 오차',value(planned.position_error_m*1000,3,' mm')),detailField('계획 TCP 방향 오차',value(planned.orientation_error_rad,5,' rad')));}
 const links=$('detail-links');links.replaceChildren();if(plan||group.executions.length)links.append(textNode('p','계획 ID · '+group.id));for(const execution of group.executions)links.append(textNode('p','승인 실행 ID · '+execution.request_id+' · '+(labels[execution.status]||execution.status)));if(!plan&&group.executions.length)links.append(textNode('small','이 화면에 원래 계획의 이력이 남아 있지 않습니다.'));
 const timeline=$('detail-timeline');timeline.replaceChildren();const names={received:'목표 수신',plan_ready:'계획 검토 준비',approved_plan_recheck:'승인 계획 재검사',dispatch:'실행 요청 전송',first_motion:'관절 이동 확인',actions_done:'양팔 실행 응답 완료',settled:'목표 자세 확인',cancel_received:'취소 요청 수신',terminal:'요청 종료'};
 for(const event of (data.events||[]).filter(item=>names[item.name]).slice(-12))timeline.append(textNode('li',`${value(event.wall_sec,3,' s')} · ${names[event.name]}`));
 $('detail-timeline-empty').hidden=timeline.children.length>0;
 $('detail-content').textContent=JSON.stringify({plan:planData||plan,selected_request:data},null,2);
}
async function detail(id){const serial=++detailSerial,group=groupRequests(state?.history||[]).find(item=>item.records.some(record=>record.request_id===id));if(!group)return;renderDetail(group,id);try{await Promise.all([loadRequest(id,{force:true}),...(group.plan&&group.plan.request_id!==id?[loadRequest(group.plan.request_id,{force:true})]:[])]);if(serial===detailSerial)renderDetail(groupRequests(state?.history||[]).find(item=>item.id===group.id)||group,id);}catch(error){if(serial===detailSerial)$('detail-error').textContent='기록을 불러오지 못했습니다. '+error.message;}}
function render(data){
 data=taskControls?.observe(data)||data;state=data;window.dispatchEvent?.(new CustomEvent('console-observation',{detail:data}));const ready=data.can_submit,active=data.active_request_id,latched=data.guard?.latched;
 for(const [identifier,pending] of pendingCancels){const record=data.history.find(item=>item.request_id===identifier);if(record&&terminal.has(record.status)||pending.session&&data.server?.session_id&&pending.session!==data.server.session_id)pendingCancels.delete(identifier);}
 publishCancellation();
 const connection=connectionPresentation(data);badge($('connection'),connection.status,connection.text);
 $('sim-time').textContent=value(data.simulation_sec,3,' s');$('state-age').textContent=value(data.state_age_sec*1000,0,' ms');
 if(data.state_age_sec===null)$('state-age').textContent='—';
 $('guard-state').textContent=guardPresentation(data.guard).text;$('guard-state').className=latched?'bad':data.guard?.phase==='ready'?'good':'';
 $('connection-detail').textContent=JSON.stringify({domain:config.domain,simulation_verified:data.simulation_verified,topic_counts:data.graph,guard:data.guard,lifecycle:data.lifecycle},null,2);
 if(data.lifecycle.error)notice(data.lifecycle.error);
 renderDemos(data.demo);updateLiveFrames(data);renderContext();
 updateCurrent();refreshControls();history(data.history);
 const record=data.history.find(r=>r.request_id===active)||data.history[0];if(!graspSelected(data))requestProgress(record);taskControls?.render();
 $('domain').textContent='ROS DOMAIN '+(graspSelected(data)?data.task.grasp?.source?.domain:config.domain);$('cell-label').textContent=graspSelected(data)?data.task.grasp?.source?.label||'ROS 집기 작업':config.label;if($('cell-summary'))$('cell-summary').textContent=graspSelected(data)?'팔 12관절 · 손가락 2관절 · 자유 블록':'하나의 시뮬레이션 환경 · 12관절 · 양팔 제어';
 if(graspSelected(data)){const job=data.task.grasp?.job;$('connection-detail').textContent=JSON.stringify({selected_task:'grasp',selection_id:data.task.selection_id,source:data.task.grasp?.source,job},null,2);$('sim-time').textContent=value(job?.simulation_sec);$('state-age').textContent=value(data.task.observation_age_sec*1000,0,' ms');if(data.task.observation_age_sec==null)$('state-age').textContent='—';$('guard-state').textContent=job?.native_guard?.latched?'집기 정지 감시 잠김':job?.native_guard?.phase==='ready'?'집기 정지 감시 중':'집기 감시 관측 대기';}
}
function disconnected(){
 taskControls?.lost();
 window.dispatchEvent?.(new CustomEvent('console-observation-lost'));readConnected=false;badge($('connection'),'blocked','상태 조회 중단');$('execute').disabled=$('review-preview').disabled=$('review-approve').disabled=$('review-discard').disabled=$('review-replan').disabled=true;
 for(const id of ['start','stop','attach','use-current','demo-run','demo-stop'])$(id).disabled=true;
 document.querySelectorAll('[data-cancel]').forEach(el=>el.disabled=true);
 $('admission').textContent='상태 조회가 끊겼습니다. 명령을 자동으로 재전송하지 않습니다.';
 $('demo-admission').textContent='상태 연결을 다시 확인하고 있습니다. 시연을 자동으로 다시 실행하지 않습니다.';
 renderContext();taskControls?.render();
}
async function readState(){
 const serial=++stateReadSerial;
 try{const data=await request('/api/state');
  if(serial!==stateReadSerial||serial<lastObservedRead){const stale=Error('더 최신 상태 조회가 진행 중입니다. 상태를 확인한 뒤 다시 계획하세요.');stale.code='state_read_superseded';throw stale;}
  lastObservedRead=serial;readConnected=true;render(data);return data;
 }
 catch(error){if(serial===stateReadSerial&&serial>=lastObservedRead){lastObservedRead=serial;disconnected();}throw error;}
}
async function poll(){
 // A periodic tick must not supersede the explicit observation that owns a
 // replan. Other explicit observations still retain their serial safety gates.
 if(polling||replanReadPending||taskCommandBusy)return;polling=true;
 try{await readState();window.operatorReady=true;}
 catch(error){}
 finally{polling=false;}
}
async function action(path,body){if(motionUnavailable()){notice('선택한 작업의 실행 상태를 확인하세요. 일반 관절·TCP 명령은 집기 작업과 동시에 보내지 않습니다.');return null;}try{const result=await request(path,body);await poll();return result;}catch(error){notice(error.message);return null;}}
function demoActive(job){return Boolean(job&&['starting','running','cancel_requested'].includes(job.status));}
function demoOutcomeStatus(job){return job?.status==='cancelled'&&job.cleanup_clean!==true?'unknown':job?.status;}
function demoDefinition(id){return demoCatalog.find(item=>item.id===id);}
function demoTitle(id){return demoDefinition(id)?.title||({contact_comparison:'접촉 자동 정지 비교',timing_comparison:'경로와 출발 시점 비교'}[id])||'동작 시연';}
function demoVariant(value){
 if(value&&typeof value==='object')return {id:value.id||value.variant,label:value.label||value.title||demoVariant(value.id||value.variant).label};
 return {id:value,label:({contact:'장애물 있음',obstacle:'장애물 있음',control:'장애물 없음',baseline:'공동 계획',timing:'출발 타이밍 조정',prepare:'공통 준비'}[value])||String(value||'준비')};
}
function demoVariants(id){const defined=demoDefinition(id)?.variants;return (Array.isArray(defined)&&defined.length?defined:(id==='contact_comparison'?['contact','control']:['baseline','timing'])).map(demoVariant).filter(item=>item.id!=='prepare');}
function mountDemos(){
 const container=$('demo-scenarios');container.replaceChildren();selectedDemo=demoCatalog[0]?.id||null;
 for(const demo of demoCatalog){
  const button=document.createElement('button');button.type='button';button.className='demo-scenario';button.dataset.demo=demo.id;button.setAttribute('aria-pressed',String(demo.id===selectedDemo));
  button.append(textNode('strong',demo.title),textNode('p',demo.description||''),textNode('small',demoVariants(demo.id).map(item=>item.label).join(' → ')));
  button.addEventListener('click',()=>{selectedDemo=demo.id;renderDemos(state?.demo);});container.append(button);
 }
 if(!demoCatalog.length)container.append(textNode('p','현재 사용할 수 있는 동작 시연이 없습니다.','demo-empty-catalog'));
}
function demoStage(job){
 if(!job)return '실행 대기';
 if(job.status==='cancel_requested')return '중단 요청을 처리하고 있습니다. 현재 실행이 정리될 때까지 기다리세요.';
 if(job.status==='cancelled')return job.cleanup_clean===true?'시연 중단과 소유 실행의 정리를 마쳤습니다.':'시연 요청이 종료됐습니다. 실행 정리 결과를 확인해야 합니다.';
 if(job.status==='succeeded')return '두 조건의 실행과 결과 확인을 마쳤습니다.';
 if(job.status==='failed')return '시연을 완료하지 못했습니다. 아래 오류와 조건별 결과를 확인하세요.';
 const stage=String(job.stage||''),variant=demoVariant(job.variant).label;
 if(stage==='timing_observe_tail')return '동작을 마쳤습니다. 두 조건에 같은 관찰 시간을 적용하고 있습니다.';
 if(stage==='searching_departure_delay')return '같은 공간 경로의 출발 시점을 계산하고 있습니다.';
 if(/compare|comparison/.test(stage))return '두 조건의 결과를 대조하고 있습니다.';
 if(/cleanup|shutdown|stopping/.test(stage))return '현재 실행을 종료하고 다음 단계를 준비하고 있습니다.';
 if(/prepar|planning|freeze|bundle|discover/.test(stage))return job.scenario==='timing_comparison'?'새 경로를 계획하고 비교할 동작을 준비하고 있습니다.':'동일한 동작을 비교할 작업 환경을 준비하고 있습니다.';
 if(job.variant)return variant+' 조건을 실행하고 있습니다.';
 return '시뮬레이션과 실행 상태를 준비하고 있습니다.';
}
function renderDemoResults(job){
 const results=Array.isArray(job?.results)?job.results.filter(result=>result.variant!=='prepare'):[],timing=job?.scenario==='timing_comparison',hasContactTime=!timing&&results.some(result=>Number.isFinite(result.contact_motion_elapsed_sec));
 $('demo-result-table').hidden=results.length===0;$('demo-result-empty').hidden=results.length>0;
 $('demo-result-context').textContent=job?demoTitle(job.scenario)+' · '+job.job_id.slice(0,8):'같은 시연 안의 두 조건을 비교합니다.';
 const head=document.createElement('tr'),columns=timing?['조건','검증 결과','계획 간격','관측 간격','관절 이동량','궤적 시간']:['조건','검증 결과','접촉','정지',...(hasContactTime?['접촉 시점']:[]),'궤적 시간'];for(const label of columns)head.append(textNode('th',label));
 $('demo-result-head').replaceChildren(head);$('demo-result-body').replaceChildren();
 for(const result of results){
  const row=document.createElement('tr');row.append(textNode('td',result.label||demoVariant(result.variant).label));const status=document.createElement('td'),tag=document.createElement('span');badge(tag,result.status);status.append(tag);row.append(status);
  if(timing)row.append(textNode('td',value(Number.isFinite(result.planned_min_distance_m)?result.planned_min_distance_m*1000:undefined,1,' mm')),textNode('td',value(Number.isFinite(result.observed_min_distance_m)?result.observed_min_distance_m*1000:undefined,1,' mm')),textNode('td',value(result.joint_travel_rad,3,' rad')));
  else row.append(textNode('td',typeof result.contact_count==='number'?String(result.contact_count):'—'),textNode('td',result.stopped===true?'자동 정지':result.stopped===false?'정지 없음':'—'));
  if(hasContactTime)row.append(textNode('td',value(result.contact_motion_elapsed_sec,3,' s')));row.append(textNode('td',value(result.duration_sec,3,' s')));$('demo-result-body').append(row);
 }
 const matched=job?.status==='succeeded'&&(job.comparison?.matched||job.comparison?.identical_commands);
 $('demo-result-note').textContent=timing?(matched?'같은 입력과 계획 자료로 비교했습니다. ':'')+'간격과 이동량은 기록된 표본 기준이며, 최적 경로를 보장하지 않습니다. 궤적 시간은 명령의 길이이며, 준비·계산 시간과 별도로 표시합니다.':(matched?'같은 명령으로 장애물 유무를 비교했습니다. ':'')+'접촉 조건의 자동 정지와 장애물 없는 조건의 완료를 확인합니다. 궤적 시간은 전체 명령 길이를 나타냅니다. 실패·중단한 시연은 완료된 비교로 취급하지 않습니다.';
}
function renderDemos(demo){
 const current=demo?.current,busy=demoActive(current);if(busy&&demoDefinition(current.scenario))selectedDemo=current.scenario;
 const selected=demoDefinition(selectedDemo),display=current&&(busy||current.scenario===selectedDemo)?current:null;
 document.querySelectorAll('[data-demo]').forEach(button=>{button.setAttribute('aria-pressed',String(button.dataset.demo===selectedDemo));button.disabled=busy;});
 $('demo-current-title').textContent=selected?selected.title:'시연을 선택하세요';$('demo-description').textContent=selected?.description||'두 조건을 순서대로 실행하고 결과를 확인합니다.';
 badge($('demo-status'),demoOutcomeStatus(display)||'idle',display?undefined:'대기');$('demo-stage').textContent=demoStage(display);
 const steps=['준비',...demoVariants(selectedDemo).map(item=>item.label),'결과 확인'];let reached=0;
 if(display){const variant=demoVariants(display.scenario).findIndex(item=>item.id===display.variant);reached=display.status==='succeeded'?4:/compare|comparison/.test(String(display.stage||''))?3:variant>=0?variant+1:0;}
 const stages=$('demo-stages');stages.replaceChildren();steps.forEach((label,index)=>{const item=textNode('li',label);item.classList.toggle('reached',index<reached);item.classList.toggle('current',Boolean(display)&&index===reached);stages.append(item);});
 const error=display?.error;let message=typeof error==='string'?error:error?.message||'';$('demo-error').hidden=!message;$('demo-error').textContent=message;
 $('demo-run').disabled=!readConnected||!selected||!demo?.can_start||busy;$('demo-stop').disabled=!readConnected||!busy||current.status==='cancel_requested';
 $('demo-admission').textContent=busy?'현재 시연이 두 조건을 순서대로 실행하고 있습니다.':demo?.can_start?'준비 완료 · 새 시연을 실행할 수 있습니다.':demo?.reason||'시연 실행 상태를 확인하고 있습니다.';
 renderDemoResults(current);
 const history=Array.isArray(demo?.history)?demo.history:[];$('demo-history-empty').hidden=history.length>0;$('demo-history').replaceChildren();
 for(const job of history.slice(0,6)){const row=document.createElement('li'),identity=document.createElement('span');identity.append(textNode('strong',demoTitle(job.scenario)),textNode('code',String(job.job_id||'').slice(0,12)+'…'));const status=document.createElement('span');badge(status,demoOutcomeStatus(job));row.append(identity,status);$('demo-history').append(row);}
}
function updateLiveFrames(data){
 if(sceneControls&&!data.view){sceneControls.destroy();sceneControls=null;document.querySelectorAll('.scene-toolbar,.viewer-measurements,.view-mode-line,.scene-slot').forEach(el=>el.hidden=true);$('view-method').textContent='ROS 관절 상태 → 서버 이미지';}
 if(sceneControls){sceneControls.update(data);if(!sceneControls.isImage())return;}
 const frame=data.frame||{},job=data.demo?.current,sequence=frame.sequence,hasSequence=typeof sequence==='number'&&Number.isFinite(sequence)&&sequence>=0;
 const matchesDemo=Boolean(job&&frame.job_id===job.job_id&&frame.run_id&&frame.run_id===job.run_id&&frame.model_sha256&&frame.model_sha256===(job.active_launch?.model_sha256||job.model_sha256));
 const hasFrame=hasSequence&&!frame.error&&(!demoActive(job)||matchesDemo);
 const stale=hasFrame&&(frame.age_sec>1||frame.source_state_age_sec>0.5||data.state_age_sec===null||data.state_age_sec>0.5||data.graph?.clock!==1||data.graph?.joints!==1||data.clock_age_sec===null||data.clock_age_sec>.5);
 const token=[frame.job_id||'manual',frame.run_id||data.lifecycle?.session_id||'',frame.model_sha256||'',frame.generation??data.lifecycle?.generation??'',sequence].join(':');
 function image(id,empty,visible,section){const element=$(id);if(!visible){element.hidden=true;element.removeAttribute('src');delete element.dataset.frameToken;$(empty).hidden=false;return;}element.hidden=false;$(empty).hidden=true;if(!$(section).hidden&&element.dataset.frameToken!==token){element.dataset.frameToken=token;const parameters=new URLSearchParams({sequence:String(sequence),job_id:frame.job_id||'',run_id:frame.run_id||'',model_sha256:frame.model_sha256||'',generation:String(frame.generation??data.lifecycle?.generation??''),operator_session_id:frame.operator_session_id||data.lifecycle?.session_id||''});element.src='/api/frame?'+parameters.toString();}}
 image('frame','frame-empty',hasFrame,'operate');image('demo-frame','demo-frame-empty',hasFrame&&matchesDemo,'demos');
 $('view-dot').classList.toggle('live',hasFrame&&!stale);$('stale-frame').hidden=!stale;
 $('frame-label').textContent=frame.error?'렌더링 오류':hasFrame?(stale?'갱신 대기':'현재 관절 자세'):config.render_enabled?'관절 상태 대기':'렌더링 비활성화';$('frame-age').textContent=hasFrame?value(frame.age_sec*1000,0,' ms 전 갱신'):'—';
 $('demo-view-dot').classList.toggle('live',hasFrame&&matchesDemo&&!stale&&demoActive(job));$('demo-stale-frame').hidden=!(stale&&matchesDemo);$('demo-stale-frame').textContent=job&&!demoActive(job)?'시연 종료 · 마지막 수신 화면':'화면 갱신 대기 · 마지막 수신 화면';
 $('demo-frame-label').textContent=frame.error?'렌더링 오류':hasFrame&&matchesDemo?(!demoActive(job)?'마지막 실행 화면':stale?'갱신 대기':demoVariant(job.variant).label):demoActive(job)?'현재 조건의 화면 준비 중':'시연 대기';
 $('demo-frame-age').textContent=hasFrame&&matchesDemo?value(frame.age_sec*1000,0,' ms 전 갱신'):'—';$('demo-live-variant').textContent=job?demoTitle(job.scenario)+' · '+demoVariant(job.variant).label:'실제 관절 상태를 반영한 모델 뷰';
 if(frame.error){$('frame-empty').dataset.renderError='true';$('demo-frame-empty').dataset.renderError='true';$('frame-empty').replaceChildren(textNode('p',frame.error));$('demo-frame-empty').replaceChildren(textNode('p',frame.error));}else{if($('frame-empty').dataset.renderError){delete $('frame-empty').dataset.renderError;$('frame-empty').replaceChildren(textNode('div','⬡','scene-symbol'),textNode('h2','시뮬레이션을 시작하세요'),textNode('p','시뮬레이션을 시작하면 현재 자세가 표시됩니다.'));}if($('demo-frame-empty').dataset.renderError){delete $('demo-frame-empty').dataset.renderError;$('demo-frame-empty').replaceChildren(textNode('div','▶','scene-symbol'),textNode('h3','시연을 선택하고 실행하세요'),textNode('p','현재 시연의 로봇과 작업 환경이 표시됩니다.'));}}
}

function beginTaskCommand(kind){
 taskCommandBusy=true;++stateReadSerial;
 if(kind==='select'||kind==='start')sceneControls?.invalidateTask('작업 전환 · 새 관측 대기');
 else sceneControls?.viewer?.markStale('취소 전달 · 정지 상태 확인 중');
 refreshControls();
}
function endTaskCommand(){taskCommandBusy=false;refreshControls();}

async function boot(){
 const bootstrap=await request('/api/bootstrap');config=bootstrap.config;token=bootstrap.token;targets={...config.home_rad};demoCatalog=(bootstrap.demos||[]).filter(item=>['contact_comparison','timing_comparison'].includes(item.id));mountDemos();
 $('cell-label').textContent=config.label;$('domain').textContent='ROS DOMAIN '+config.domain;
 if(config.ros_grasp_viewer_url){
  try{const destination=new URL(config.ros_grasp_viewer_url);const link=$('ros-grasp-link'),card=$('ros-grasp-navigation');
   if(link&&card&&destination.protocol==='http:'&&destination.hostname==='127.0.0.1'&&Number(destination.port)>=1024&&Number(destination.port)<=65535&&!destination.username&&!destination.password&&destination.pathname==='/'&&!destination.search&&!destination.hash){link.href=destination.href;card.hidden=false;}
  }catch(_error){/* An unavailable navigation target never changes control admission. */}
 }
 fields();
 if(config.render_enabled&&config.scene_view_version===1){try{const {SceneControls}=await import('./scene_app.js');sceneControls=new SceneControls({request,notify:notice,showOperate:()=>show('review',{focusTask:true}),onAvailabilityChange:()=>{if(state)refreshControls();}});}catch(error){notice('로컬 3D 화면을 시작하지 못했습니다: '+error.message);}}else{document.querySelectorAll('.scene-toolbar,.viewer-measurements,.view-mode-line,.scene-slot').forEach(el=>el.hidden=true);$('view-method').textContent=config.render_enabled?'ROS 관절 상태 → 서버 이미지':'3D·이미지 표시 비활성화';}
 const {WorkspaceTaskControls}=await import('./task_controls.mjs');taskControls=new WorkspaceTaskControls({document,request,refresh:readState,notify:notice,showOperate:()=>show('operate',{focusTask:true}),cameraReady:()=>Boolean(sceneControls?.cameraReady()),beforeCommand:beginTaskCommand,afterCommand:endTaskCommand});
 window.addEventListener('pagehide',()=>taskControls?.destroy());
 document.querySelectorAll('[data-view]').forEach(el=>el.addEventListener('click',()=>show(el.dataset.view)));
 window.addEventListener('workspace-record',()=>{const record=state?.history.find(row=>row.request_id===state.active_request_id)||state?.history[0];if(record){show('history');detail(record.request_id);}});
 $('dismiss').addEventListener('click',()=>$('notice').hidden=true);
 $('unit').addEventListener('change',()=>{unit=$('unit').value;fields();refreshControls();});
 $('arm-left').addEventListener('click',()=>{arm=0;fields();});$('arm-right').addEventListener('click',()=>{arm=1;fields();});
 $('use-home').addEventListener('click',()=>{targets={...config.home_rad};fields();refreshControls();});
 $('use-sample').addEventListener('click',()=>{targets={...config.targets.sample};fields();refreshControls();});
 $('use-current').addEventListener('click',()=>{if(state?.positions_rad){targets={...state.positions_rad};fields();refreshControls();}});
 configurePoseInputs();setGoalKind();
 $('goal-kind').addEventListener('change',setGoalKind);$('pose-arm').addEventListener('change',()=>{renderPoseMetadata();refreshControls();});
 for(const id of ['pose-x','pose-y','pose-z','pose-qx','pose-qy','pose-qz','pose-qw'])$(id).addEventListener('input',refreshControls);
 $('strategy').addEventListener('change',()=>{$('strategy-help').textContent=$('strategy').value==='coordinated'?'양팔을 함께 고려해 관절 공간 경로를 계획합니다.':'팔별 경로는 유지하고, 양팔의 출발 간격을 탐색합니다.';refreshControls();});
 $('demo-run').addEventListener('click',async()=>{if(!readConnected||!selectedDemo||!state?.demo?.can_start)return;$('demo-run').disabled=true;await action('/api/demos/start',{scenario:selectedDemo,native_gui:false});});
 $('demo-stop').addEventListener('click',async()=>{const job=state?.demo?.current;if(!readConnected||!job||!demoActive(job))return;$('demo-stop').disabled=true;await action('/api/demos/stop',{job_id:job.job_id});});
 $('start').addEventListener('click',()=>action('/api/start',{native_gui:$('native-gui').checked,planning_case:$('planning-case').value}));
 $('attach').addEventListener('click',()=>action('/api/attach',{}));$('stop').addEventListener('click',()=>graspSelected()?taskControls?.command('cancel'):action('/api/stop',{}));
 $('execute').addEventListener('click',async()=>{
  if(!readConnected||replanning||!canPlanGoal()||!validGoal())return;
  $('execute').disabled=true;
  const payload=goalPayload(crypto.randomUUID().replaceAll('-','')),poseRequest=Boolean(payload.arm_id);
  const result=await action(poseRequest?'/api/pose/plan':'/api/plan',payload);if(result){latestRequest=null;show('review',{focusTask:true});}
 });
 $('review-preview').addEventListener('click',async()=>{const review=state?.pending_plan;if(!readConnected||!review||review.state!=='awaiting_approval')return;await sceneControls?.previewReview(review.plan_request_id);await poll();});
 $('review-replan').addEventListener('click',replan);
 $('review-details').addEventListener('click',()=>{const review=state?.pending_plan;if(review){show('history');detail(review.plan_request_id);}});
 $('review-approve').addEventListener('click',async()=>{const review=state?.pending_plan;if($('review-approve').disabled||!review)return;$('review-approve').disabled=true;sceneControls?.pause();const result=await action('/api/plan/execute',{schema_version:1,request_id:crypto.randomUUID().replaceAll('-',''),plan_request_id:review.plan_request_id,plan_sha256:review.plan_sha256,session_id:review.session_id});if(result)latestRequest=null;});
 $('review-discard').addEventListener('click',async()=>{const review=state?.pending_plan;if(!review)return;$('review-discard').disabled=true;const result=await action('/api/plan/discard',{request_id:review.plan_request_id});if(result)await sceneControls?.select('live',state.view?.config_source_id);await poll();});
 document.querySelectorAll('[data-cancel]').forEach(el=>el.addEventListener('click',cancelCurrentRequest));
 document.querySelectorAll('[data-camera]').forEach(el=>el.addEventListener('click',async()=>{if(sceneControls&&!sceneControls.isImage()){sceneControls.camera(el.dataset.camera);document.querySelectorAll('[data-camera]').forEach(b=>b.classList.toggle('selected',b.dataset.camera===el.dataset.camera));}else if(await action('/api/camera',{view:el.dataset.camera}))document.querySelectorAll('[data-camera]').forEach(b=>b.classList.toggle('selected',b.dataset.camera===el.dataset.camera));}));
 $('import-target').addEventListener('click',()=>$('target-file').click());
 $('target-file').addEventListener('change',async()=>{
  const file=$('target-file').files[0];if(!file)return;
  if(file.size>60000){notice('목표 파일은 60KB 이하여야 합니다.');return;}
  const result=await action('/api/target',{text:await file.text()});
  if(result){targets=result.target_rad;fields();refreshControls();} $('target-file').value='';
 });
 $('export-target').addEventListener('click',()=>{if(!validTargets()){notice('목표값을 확인하세요.');return;}const link=document.createElement('a');link.href=URL.createObjectURL(new Blob([JSON.stringify(targets,null,2)+'\n'],{type:'application/json'}));link.download=config.workcell+'-target.json';link.click();setTimeout(()=>URL.revokeObjectURL(link.href),1000);});
 $('load-logs').addEventListener('click',async()=>{try{$('logs').textContent=(await request('/api/logs')).logs.map(l=>'['+l.name+']\n'+l.text).join('\n\n');}catch(error){notice(error.message);}});
 for(const report of bootstrap.reports){
  const card=textNode('article','','report-card');card.append(textNode('small',report.published_date+' / '+report.models.join(' · ')),textNode('h3',report.title),textNode('p',report.overview));
  const link=textNode('a','리포트 열기 ↗','button');link.href='/reports/'+report.url;link.target='_blank';link.rel='noopener';card.append(link);$('report-cards').append(card);
 }
 await poll();setInterval(poll,250);
}
boot().catch(error=>{notice('조작 화면을 불러오지 못했습니다: '+error.message);});
