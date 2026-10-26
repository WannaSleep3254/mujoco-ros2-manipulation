'use strict';
const $=id=>document.getElementById(id);
let sceneControls=null;
let config,token,state,targets={},arm=0,unit='deg',polling=false,latestRequest=null,detailsAt=0,demoCatalog=[],selectedDemo=null;
const labels={interrupted:'중단',starting:'준비 중',cancel_requested:'중단 요청 중',passed:'검증 통과',ready:'실행 가능',queued:'접수',submitting:'전송 확인 대기',running:'진행 중',succeeded:'성공',failed:'실패',cancelled:'취소 완료',rejected:'거부',blocked:'재시작 필요',unknown:'결과 미확인'};
const terminal=new Set(['succeeded','failed','cancelled','rejected','unknown']);
function notice(text){$('notice-text').textContent=text;$('notice').hidden=false;}
async function request(path,body){
 const options=body===undefined?{}:{method:'POST',headers:{'Content-Type':'application/json','X-Operator-Token':token},body:JSON.stringify(body)};
 const response=await fetch(path,options);const data=await response.json();
 if(!response.ok)throw Error(data.error||`HTTP ${response.status}`);return data;
}
function textNode(tag,text,cls){const el=document.createElement(tag);el.textContent=text;if(cls)el.className=cls;return el;}
function value(v,decimals=3,suffix=' s'){return typeof v==='number'&&Number.isFinite(v)?v.toFixed(decimals)+suffix:'—';}
function displayed(v){return unit==='deg'?v*180/Math.PI:v;}
function radians(v){return unit==='deg'?v*Math.PI/180:v;}
function badge(el,status,text){el.className='badge'+(status==='succeeded'||status==='passed'||status==='ready'?' good':status==='failed'||status==='blocked'||status==='rejected'?' bad':status==='running'||status==='starting'||status==='cancel_requested'||status==='queued'||status==='submitting'?' busy':'');el.textContent=text||labels[status]||status;}
function show(view){document.querySelectorAll('.view').forEach(el=>el.hidden=el.id!==view);document.querySelectorAll('[data-view]').forEach(el=>el.classList.toggle('active',el.dataset.view===view));$('view-title').textContent={operate:'로봇 조작',demos:'동작 시연',history:'실행 기록',reports:'평가 리포트'}[view];if(state)updateLiveFrames(state);sceneControls?.mount();}
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
function validTargets(){return config&&Object.entries(config.joint_limits).every(([n,l])=>Number.isFinite(targets[n])&&targets[n]>=l.lower&&targets[n]<=l.upper);}
function refreshControls(){
 if(!state)return;
 const owned=state.lifecycle.owned,phase=state.lifecycle.phase,transition=['starting','stopping'].includes(phase),demoBusy=demoActive(state.demo?.current);
 $('start').disabled=Boolean(state.view?.preview_active)||demoBusy||transition||owned.simulation||owned.commands||Object.values(state.graph).some(v=>v>0)||performance.now()<2000;
 $('stop').disabled=demoBusy||transition||!(owned.simulation||owned.commands);
 $('attach').hidden=!(state.simulation_verified&&state.graph.status===0&&!owned.commands);$('attach').disabled=Boolean(state.view?.preview_active)||demoBusy||transition;
 $('execute').disabled=demoBusy||!state.can_plan||!validTargets()||!sceneControls;document.querySelectorAll('[data-cancel]').forEach(el=>el.disabled=demoBusy||!state.active_request_id);
 $('use-current').disabled=!state.positions_rad||state.state_age_sec>0.5;
 $('planning-case').disabled=demoBusy||transition||owned.simulation||state.graph.clock>0;$('native-gui').disabled=$('planning-case').disabled;
 $('admission').textContent=demoBusy?'동작 시연 중입니다. 시연을 마치거나 중단한 뒤 목표를 전송하세요.':!validTargets()?'모든 목표를 관절 한계 안의 숫자로 입력하세요.':!sceneControls?'계획 검토에는 로컬 3D 표시가 필요합니다.':state.can_plan?'계획을 생성합니다. 경로를 확인하고 승인해야 로봇이 움직입니다.':state.reason||'계획 검토 명령 노드의 연결을 확인하세요.';
 renderReview();
}
function reviewMatchesTargets(review){const record=state?.history.find(item=>item.request_id===review?.plan_request_id),original=record?.command;return original&&original.strategy===$('strategy').value&&Object.keys(targets).every(name=>original.target_rad?.[name]===targets[name]);}
function renderReview(){
 const review=state?.pending_plan;$('review-card').hidden=!review;if(!review)return;
 const waiting=review.state==='awaiting_approval'&&review.remaining_sec>0,busy=Boolean(state.active_request_id)||demoActive(state.demo?.current),matches=reviewMatchesTargets(review);
 const preview=state.view?.review_plan===true&&state.view?.binding?.request_id===review.plan_request_id&&state.view?.mode==='plan_preview';
 const visible=preview&&sceneControls?.cameraReady();
 badge($('review-badge'),waiting?'ready':'blocked',({awaiting_approval:'승인 대기',expired:'만료',invalidated:'무효화',consumed:'사용 완료',approval_queued:'승인 전송 중'})[review.state]||'다시 계획 필요');
 $('review-id').textContent=review.plan_request_id;
 $('review-summary').textContent=waiting?`계획과 검사가 끝났습니다. 실행 승인까지 ${Math.ceil(review.remaining_sec)}초 남았습니다.`:'이 계획은 실행할 수 없습니다. 실시간 보기로 돌아가 새 계획을 생성하세요.';
 $('review-preview').disabled=!waiting||busy||!sceneControls||state.view?.display_mode==='image';
 $('review-discard').disabled=!waiting||busy;
 $('review-approve').disabled=!waiting||!state.can_approve||!visible||!matches||busy;
 $('review-admission').textContent=!waiting?'승인은 한 번만 사용할 수 있습니다.':!matches?'목표나 계획 방식을 바꿨습니다. 실시간 보기로 돌아가 새 계획을 생성하세요.':!preview?'경로 미리보기를 열고 확인하세요.':!visible?'검토할 경로를 로컬 3D 화면에 표시하고 있습니다.':!state.can_approve?'현재 상태에서 실행을 승인할 수 없습니다. 연결과 감시 상태를 확인하세요.':'미리보기 재생은 로봇을 움직이지 않습니다. 실행하려면 승인·실행 버튼을 누르세요.';
}
function requestProgress(record){
 if(!record)return;
 badge($('request-badge'),record.status,record.stage==='plan_not_required'?'이동 불필요':record.mode==='plan_only'&&record.status==='succeeded'?'계획 생성 완료':undefined);$('request-id').textContent=record.request_id.slice(0,12)+'…';
 const stages={submitted:'명령 노드 수신 확인 대기',received:'목표 수신',state_snapshot:'시작 자세 확인',computation_started:'경로 계산 시작',planning_both_arms:'양팔 공동 경로 계획',planning_left_arm:'FR5 경로 계획',planning_right_arm:'UR5e 경로 계획',departure_search:'출발 타이밍 탐색',safety_checks:'거리·충돌 검사',computation_done:'계획·검사 완료',dispatch:'동작 예약·명령 전송',first_motion:'관절 이동 확인',first_motion_left:'FR5 이동 확인',first_motion_right:'UR5e 이동 확인',actions_done:'양팔 Action 완료',settled:'관절 수렴 확인',terminal:labels[record.status],cancel_received:'양팔 취소 요청 수신'};
 $('request-stage').textContent=({plan_ready:'계획·검사 완료 · 실행 승인 대기',plan_not_required:'이미 목표 자세에 있습니다. 실행할 이동이 없습니다.',plan_revoked:'미승인 계획 폐기',approval_validating:'승인 계획 재검사'})[record.stage]||stages[record.stage]||record.stage||labels[record.status];
 const stage=record.stage||'';let progress=0;
 if(/planning|computation|search|safety/.test(stage))progress=1;
 if(/dispatch|action|motion|settled|cancel/.test(stage))progress=2;
 if(record.status==='succeeded')progress=record.mode==='plan_only'?1:3;
 document.querySelectorAll('[data-step]').forEach(el=>el.classList.toggle('reached',Number(el.dataset.step)<=progress));
 $('request-error').hidden=!record.error;$('request-error').textContent=record.error?.message||'';
 $('first-motion').textContent=value(record.metrics?.received_to_first_motion_wall_sec);
 $('terminal-time').textContent=value(record.metrics?.received_to_terminal_wall_sec);
 if(latestRequest!==record.request_id){latestRequest=record.request_id;detailsAt=0;$('clearance').textContent='—';}
 if(Date.now()-detailsAt>1500){detailsAt=Date.now();request('/api/requests/'+record.request_id).then(data=>{
  if(latestRequest===data.request_id)$('clearance').textContent=value(data.planned_clearance?.minimum?.distance_m*1000,1,' mm');
 }).catch(()=>{});}
}
function history(records){
 $('history-count').textContent=records.length;$('history-empty').hidden=records.length>0;
 const body=$('history-body');body.replaceChildren();
 for(const record of records){
  const row=document.createElement('tr');row.dataset.requestId=record.request_id;
  const id=document.createElement('td');id.append(textNode('code',record.request_id.slice(0,12)+'…'));row.append(id);
  const result=document.createElement('td'),tag=document.createElement('span');badge(tag,record.status,record.mode==='plan_only'&&record.status==='succeeded'?'계획 생성':undefined);result.append(tag);row.append(result);
  row.append(textNode('td',record.stage||'—'),textNode('td',value(record.metrics?.received_to_first_motion_wall_sec)),textNode('td',value(record.metrics?.received_to_terminal_wall_sec)));
  const action=document.createElement('td'),button=textNode('button','상세');button.addEventListener('click',()=>detail(record.request_id));action.append(button);if(sceneControls&&record.status==='succeeded'&&record.trace_path&&record.mode!=='plan_only'){const preview=textNode('button','저장 궤적 보기');preview.disabled=Boolean(state?.active_request_id)||demoActive(state?.demo?.current);preview.addEventListener('click',()=>sceneControls.previewRequest(record.request_id));action.append(preview);}row.append(action);body.append(row);
 }
}
async function detail(id){try{const data=await request('/api/requests/'+id);$('history-detail').hidden=false;$('detail-error').textContent=data.error?.message||'';$('detail-content').textContent=JSON.stringify(data,null,2);}catch(error){notice(error.message);}}
function render(data){
 state=data;const ready=data.can_submit,active=data.active_request_id,latched=data.guard?.latched;
 if(demoActive(data.demo?.current))badge($('connection'),'running',data.demo.current.status==='cancel_requested'?'시연 중단 중':'시연 진행 중');
 else badge($('connection'),latched?'blocked':ready?'ready':active?'running':'offline',latched?'정지 잠금':ready?'실행 가능':active?'요청 처리 중':data.lifecycle.phase==='starting'?'시뮬레이터 준비 중':data.simulation_verified?'연결됨 · 준비 확인':'연결 대기');
 $('sim-time').textContent=value(data.simulation_sec,3,' s');$('state-age').textContent=value(data.state_age_sec*1000,0,' ms');
 if(data.state_age_sec===null)$('state-age').textContent='—';
 $('guard-state').textContent=latched?'정지 잠금':data.guard?.phase==='ready'?'감시 중':'연결 대기';$('guard-state').className=latched?'bad':data.guard?.phase==='ready'?'good':'';
 $('connection-detail').textContent=JSON.stringify({domain:config.domain,simulation_verified:data.simulation_verified,topic_counts:data.graph,guard:data.guard,lifecycle:data.lifecycle},null,2);
 if(data.lifecycle.error)notice(data.lifecycle.error);
 renderDemos(data.demo);updateLiveFrames(data);
 updateCurrent();refreshControls();history(data.history);
 const record=data.history.find(r=>r.request_id===active)||data.history[0];requestProgress(record);
}
async function poll(){
 if(polling)return;polling=true;
 try{render(await request('/api/state'));window.operatorReady=true;}
 catch(error){badge($('connection'),'blocked','연결 끊김');$('execute').disabled=$('review-preview').disabled=$('review-approve').disabled=$('review-discard').disabled=true;document.querySelectorAll('[data-cancel]').forEach(el=>el.disabled=true);$('admission').textContent='화면 서버와 연결되지 않았습니다. 자동으로 명령을 재전송하지 않습니다.';$('demo-run').disabled=true;$('demo-stop').disabled=true;$('demo-admission').textContent='화면 서버와 연결되지 않았습니다. 실행 상태를 다시 확인하고 있습니다.';}
 finally{polling=false;}
}
async function action(path,body){try{const result=await request(path,body);await poll();return result;}catch(error){notice(error.message);return null;}}
function demoActive(job){return Boolean(job&&['starting','running','cancel_requested'].includes(job.status));}
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
 if(job.status==='cancelled')return '시연 중단과 실행 정리를 마쳤습니다.';
 if(job.status==='succeeded')return '두 조건의 실행과 결과 확인을 마쳤습니다.';
 if(job.status==='failed')return '시연을 완료하지 못했습니다. 아래 오류와 조건별 결과를 확인하세요.';
 const stage=String(job.stage||''),variant=demoVariant(job.variant).label;
 if(stage==='timing_observe_tail')return '동작을 마쳤습니다. 두 조건에 같은 관찰 시간을 적용하고 있습니다.';
 if(stage==='searching_departure_delay')return '같은 공간 경로의 출발 시점을 계산하고 있습니다.';
 if(/compare|comparison/.test(stage))return '두 조건의 결과를 대조하고 있습니다.';
 if(/cleanup|shutdown|stopping/.test(stage))return '현재 실행을 종료하고 다음 단계를 준비하고 있습니다.';
 if(/prepar|planning|freeze|bundle|discover/.test(stage))return job.scenario==='timing_comparison'?'새 경로를 계획하고 비교할 동작을 준비하고 있습니다.':'동일한 동작으로 비교할 장면을 준비하고 있습니다.';
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
 badge($('demo-status'),display?.status||'idle',display?undefined:'대기');$('demo-stage').textContent=demoStage(display);
 const steps=['준비',...demoVariants(selectedDemo).map(item=>item.label),'결과 확인'];let reached=0;
 if(display){const variant=demoVariants(display.scenario).findIndex(item=>item.id===display.variant);reached=display.status==='succeeded'?4:/compare|comparison/.test(String(display.stage||''))?3:variant>=0?variant+1:0;}
 const stages=$('demo-stages');stages.replaceChildren();steps.forEach((label,index)=>{const item=textNode('li',label);item.classList.toggle('reached',index<reached);item.classList.toggle('current',Boolean(display)&&index===reached);stages.append(item);});
 const error=display?.error;let message=typeof error==='string'?error:error?.message||'';$('demo-error').hidden=!message;$('demo-error').textContent=message;
 $('demo-run').disabled=!selected||!demo?.can_start||busy;$('demo-stop').disabled=!busy||current.status==='cancel_requested';
 $('demo-admission').textContent=busy?'현재 시연이 두 조건을 순서대로 실행하고 있습니다.':demo?.can_start?'준비 완료 · 새 시연을 실행할 수 있습니다.':demo?.reason||'시연 실행 상태를 확인하고 있습니다.';
 renderDemoResults(current);
 const history=Array.isArray(demo?.history)?demo.history:[];$('demo-history-empty').hidden=history.length>0;$('demo-history').replaceChildren();
 for(const job of history.slice(0,6)){const row=document.createElement('li'),identity=document.createElement('span');identity.append(textNode('strong',demoTitle(job.scenario)),textNode('code',String(job.job_id||'').slice(0,12)+'…'));const status=document.createElement('span');badge(status,job.status);row.append(identity,status);$('demo-history').append(row);}
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
 if(frame.error){$('frame-empty').dataset.renderError='true';$('demo-frame-empty').dataset.renderError='true';$('frame-empty').replaceChildren(textNode('p',frame.error));$('demo-frame-empty').replaceChildren(textNode('p',frame.error));}else{if($('frame-empty').dataset.renderError){delete $('frame-empty').dataset.renderError;$('frame-empty').replaceChildren(textNode('div','⬡','scene-symbol'),textNode('h2','작업장면을 준비하세요'),textNode('p','시뮬레이션을 시작하면 현재 자세가 표시됩니다.'));}if($('demo-frame-empty').dataset.renderError){delete $('demo-frame-empty').dataset.renderError;$('demo-frame-empty').replaceChildren(textNode('div','▶','scene-symbol'),textNode('h3','시연을 선택하고 실행하세요'),textNode('p','현재 실행 중인 조건의 장면이 표시됩니다.'));}}
}

async function boot(){
 const bootstrap=await request('/api/bootstrap');config=bootstrap.config;token=bootstrap.token;targets={...config.home_rad};demoCatalog=(bootstrap.demos||[]).filter(item=>['contact_comparison','timing_comparison'].includes(item.id));mountDemos();
 $('cell-label').textContent=config.label;$('domain').textContent='ROS DOMAIN '+config.domain;
 fields();
 if(config.render_enabled&&config.scene_view_version===1){try{const {SceneControls}=await import('./scene_app.js');sceneControls=new SceneControls({request,notify:notice,showOperate:()=>show('operate')});}catch(error){notice('로컬 3D 화면을 시작하지 못했습니다: '+error.message);}}else{document.querySelectorAll('.scene-toolbar,.viewer-measurements,.view-mode-line,.scene-slot').forEach(el=>el.hidden=true);$('view-method').textContent=config.render_enabled?'ROS 관절 상태 → 서버 이미지':'장면 표시 비활성화';}
 document.querySelectorAll('[data-view]').forEach(el=>el.addEventListener('click',()=>show(el.dataset.view)));
 $('dismiss').addEventListener('click',()=>$('notice').hidden=true);
 $('unit').addEventListener('change',()=>{unit=$('unit').value;fields();refreshControls();});
 $('arm-left').addEventListener('click',()=>{arm=0;fields();});$('arm-right').addEventListener('click',()=>{arm=1;fields();});
 $('use-home').addEventListener('click',()=>{targets={...config.home_rad};fields();refreshControls();});
 $('use-sample').addEventListener('click',()=>{targets={...config.targets.sample};fields();refreshControls();});
 $('use-current').addEventListener('click',()=>{if(state?.positions_rad){targets={...state.positions_rad};fields();refreshControls();}});
 $('strategy').addEventListener('change',()=>{$('strategy-help').textContent=$('strategy').value==='coordinated'?'양팔을 함께 고려해 관절 공간 경로를 계획합니다.':'팔별 경로는 유지하고, 양팔의 출발 간격을 탐색합니다.';refreshControls();});
 $('demo-run').addEventListener('click',async()=>{if(!selectedDemo||!state?.demo?.can_start)return;$('demo-run').disabled=true;await action('/api/demos/start',{scenario:selectedDemo,native_gui:false});});
 $('demo-stop').addEventListener('click',async()=>{const job=state?.demo?.current;if(!job||!demoActive(job))return;$('demo-stop').disabled=true;await action('/api/demos/stop',{job_id:job.job_id});});
 $('start').addEventListener('click',()=>action('/api/start',{native_gui:$('native-gui').checked,planning_case:$('planning-case').value}));
 $('attach').addEventListener('click',()=>action('/api/attach',{}));$('stop').addEventListener('click',()=>action('/api/stop',{}));
 $('execute').addEventListener('click',async()=>{
  if(!validTargets())return;
  $('execute').disabled=true;
  const payload={schema_version:1,request_id:crypto.randomUUID().replaceAll('-',''),strategy:$('strategy').value,target_rad:{...targets}};
  const result=await action('/api/plan',payload);if(result)latestRequest=null;
 });
 $('review-preview').addEventListener('click',async()=>{const review=state?.pending_plan;if(!review||review.state!=='awaiting_approval')return;await sceneControls?.previewReview(review.plan_request_id);await poll();});
 $('review-approve').addEventListener('click',async()=>{const review=state?.pending_plan;if($('review-approve').disabled||!review)return;$('review-approve').disabled=true;sceneControls?.pause();const result=await action('/api/plan/execute',{schema_version:1,request_id:crypto.randomUUID().replaceAll('-',''),plan_request_id:review.plan_request_id,plan_sha256:review.plan_sha256,session_id:review.session_id});if(result)latestRequest=null;});
 $('review-discard').addEventListener('click',async()=>{const review=state?.pending_plan;if(!review)return;$('review-discard').disabled=true;const result=await action('/api/plan/discard',{request_id:review.plan_request_id});if(result)await sceneControls?.select('live',state.view?.config_source_id);await poll();});
 document.querySelectorAll('[data-cancel]').forEach(el=>el.addEventListener('click',()=>{if(state?.active_request_id)action('/api/cancel',{request_id:state.active_request_id}).then(r=>{if(r)notice('양팔 취소를 요청했습니다. 취소 완료 상태를 확인하세요.');});}));
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
