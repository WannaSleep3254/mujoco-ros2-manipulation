'use strict';
const $=id=>document.getElementById(id);
let config,token,state,targets={},arm=0,unit='deg',polling=false,lastFrame=-1,latestRequest=null,detailsAt=0;
const labels={ready:'실행 가능',queued:'접수',submitting:'전송 확인 대기',running:'진행 중',succeeded:'성공',failed:'실패',cancelled:'취소 완료',rejected:'거부',blocked:'재시작 필요',unknown:'결과 미확인'};
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
function badge(el,status,text){el.className='badge'+(status==='succeeded'||status==='ready'?' good':status==='failed'||status==='blocked'||status==='rejected'?' bad':status==='running'||status==='queued'||status==='submitting'?' busy':'');el.textContent=text||labels[status]||status;}
function show(view){document.querySelectorAll('.view').forEach(el=>el.hidden=el.id!==view);document.querySelectorAll('[data-view]').forEach(el=>el.classList.toggle('active',el.dataset.view===view));$('view-title').textContent={operate:'로봇 조작',history:'실행 기록',reports:'평가 리포트'}[view];}
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
 const owned=state.lifecycle.owned,phase=state.lifecycle.phase,transition=['starting','stopping'].includes(phase);
 $('start').disabled=transition||owned.simulation||owned.commands||Object.values(state.graph).some(v=>v>0)||performance.now()<2000;
 $('stop').disabled=transition||!(owned.simulation||owned.commands);
 $('attach').hidden=!(state.simulation_verified&&state.graph.status===0&&!owned.commands);$('attach').disabled=transition;
 $('execute').disabled=!state.can_submit||!validTargets();document.querySelectorAll('[data-cancel]').forEach(el=>el.disabled=!state.active_request_id);
 $('use-current').disabled=!state.positions_rad||state.state_age_sec>0.5;
 $('planning-case').disabled=transition||owned.simulation||state.graph.clock>0;$('native-gui').disabled=$('planning-case').disabled;
 $('admission').textContent=!validTargets()?'모든 목표를 관절 한계 안의 숫자로 입력하세요.':state.can_submit?'준비 완료 · 양팔 목표를 전송할 수 있습니다.':state.reason||'연결 확인 중';
}
function requestProgress(record){
 if(!record)return;
 badge($('request-badge'),record.status);$('request-id').textContent=record.request_id.slice(0,12)+'…';
 const stages={submitted:'명령 노드 수신 확인 대기',received:'목표 수신',state_snapshot:'시작 자세 확인',computation_started:'경로 계산 시작',planning_both_arms:'양팔 공동 경로 계획',planning_left_arm:'FR5 경로 계획',planning_right_arm:'UR5e 경로 계획',departure_search:'출발 타이밍 탐색',safety_checks:'거리·충돌 검사',computation_done:'계획·검사 완료',dispatch:'동작 예약·명령 전송',first_motion:'관절 이동 확인',first_motion_left:'FR5 이동 확인',first_motion_right:'UR5e 이동 확인',actions_done:'양팔 Action 완료',settled:'관절 수렴 확인',terminal:labels[record.status],cancel_received:'양팔 취소 요청 수신'};
 $('request-stage').textContent=stages[record.stage]||record.stage||labels[record.status];
 const stage=record.stage||'';let progress=0;
 if(/planning|computation|search|safety/.test(stage))progress=1;
 if(/dispatch|action|motion|settled|cancel/.test(stage))progress=2;
 if(record.status==='succeeded')progress=3;
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
  const result=document.createElement('td'),tag=document.createElement('span');badge(tag,record.status);result.append(tag);row.append(result);
  row.append(textNode('td',record.stage||'—'),textNode('td',value(record.metrics?.received_to_first_motion_wall_sec)),textNode('td',value(record.metrics?.received_to_terminal_wall_sec)));
  const action=document.createElement('td'),button=textNode('button','상세');button.addEventListener('click',()=>detail(record.request_id));action.append(button);row.append(action);body.append(row);
 }
}
async function detail(id){try{const data=await request('/api/requests/'+id);$('history-detail').hidden=false;$('detail-error').textContent=data.error?.message||'';$('detail-content').textContent=JSON.stringify(data,null,2);}catch(error){notice(error.message);}}
function render(data){
 state=data;const ready=data.can_submit,active=data.active_request_id,latched=data.guard?.latched;
 badge($('connection'),latched?'blocked':ready?'ready':active?'running':'offline',latched?'정지 잠금':ready?'실행 가능':active?'요청 처리 중':data.lifecycle.phase==='starting'?'시뮬레이터 준비 중':data.simulation_verified?'연결됨 · 준비 확인':'연결 대기');
 $('sim-time').textContent=value(data.simulation_sec,3,' s');$('state-age').textContent=value(data.state_age_sec*1000,0,' ms');
 if(data.state_age_sec===null)$('state-age').textContent='—';
 $('guard-state').textContent=latched?'정지 잠금':data.guard?.phase==='ready'?'감시 중':'연결 대기';$('guard-state').className=latched?'bad':data.guard?.phase==='ready'?'good':'';
 $('connection-detail').textContent=JSON.stringify({domain:config.domain,simulation_verified:data.simulation_verified,topic_counts:data.graph,guard:data.guard,lifecycle:data.lifecycle},null,2);
 if(data.lifecycle.error)notice(data.lifecycle.error);
 const frame=data.frame,hasFrame=frame.sequence!==undefined,stale=(hasFrame&&(frame.age_sec>1||data.state_age_sec===null||data.state_age_sec>0.5))||Boolean(frame.error&&lastFrame>=0);
 $('view-dot').classList.toggle('live',hasFrame&&!stale);$('stale-frame').hidden=!stale;
 $('frame-label').textContent=hasFrame?(stale?'갱신 대기':'현재 관절 자세'):config.render_enabled?'관절 상태 대기':'렌더링 비활성화';
 $('frame-age').textContent=hasFrame?value(frame.age_sec*1000,0,' ms 전 갱신'):'—';
 if(hasFrame&&frame.sequence!==lastFrame){lastFrame=frame.sequence;$('frame').src='/api/frame?sequence='+frame.sequence;$('frame').hidden=false;$('frame-empty').hidden=true;}
 if(frame.error){$('frame-label').textContent='렌더링 오류';$('frame-empty').replaceChildren(textNode('p',frame.error));}
 updateCurrent();refreshControls();history(data.history);
 const record=data.history.find(r=>r.request_id===active)||data.history[0];requestProgress(record);
}
async function poll(){
 if(polling)return;polling=true;
 try{render(await request('/api/state'));window.operatorReady=true;}
 catch(error){badge($('connection'),'blocked','연결 끊김');$('execute').disabled=true;document.querySelectorAll('[data-cancel]').forEach(el=>el.disabled=true);$('admission').textContent='화면 서버와 연결되지 않았습니다. 자동으로 명령을 재전송하지 않습니다.';}
 finally{polling=false;}
}
async function action(path,body){try{const result=await request(path,body);await poll();return result;}catch(error){notice(error.message);return null;}}
async function boot(){
 const bootstrap=await request('/api/bootstrap');config=bootstrap.config;token=bootstrap.token;targets={...config.home_rad};
 $('cell-label').textContent=config.label;$('domain').textContent='ROS DOMAIN '+config.domain;
 fields();
 document.querySelectorAll('[data-view]').forEach(el=>el.addEventListener('click',()=>show(el.dataset.view)));
 $('dismiss').addEventListener('click',()=>$('notice').hidden=true);
 $('unit').addEventListener('change',()=>{unit=$('unit').value;fields();refreshControls();});
 $('arm-left').addEventListener('click',()=>{arm=0;fields();});$('arm-right').addEventListener('click',()=>{arm=1;fields();});
 $('use-home').addEventListener('click',()=>{targets={...config.home_rad};fields();refreshControls();});
 $('use-sample').addEventListener('click',()=>{targets={...config.targets.sample};fields();refreshControls();});
 $('use-current').addEventListener('click',()=>{if(state?.positions_rad){targets={...state.positions_rad};fields();refreshControls();}});
 $('strategy').addEventListener('change',()=>{$('strategy-help').textContent=$('strategy').value==='coordinated'?'양팔을 함께 고려해 관절 공간 경로를 계획합니다.':'팔별 경로는 유지하고, 양팔의 출발 간격을 탐색합니다.';});
 $('start').addEventListener('click',()=>action('/api/start',{native_gui:$('native-gui').checked,planning_case:$('planning-case').value}));
 $('attach').addEventListener('click',()=>action('/api/attach',{}));$('stop').addEventListener('click',()=>action('/api/stop',{}));
 $('execute').addEventListener('click',async()=>{
  if(!validTargets())return;
  $('execute').disabled=true;
  const payload={schema_version:1,request_id:crypto.randomUUID().replaceAll('-',''),strategy:$('strategy').value,target_rad:{...targets}};
  const result=await action('/api/command',payload);if(result)latestRequest=null;
 });
 document.querySelectorAll('[data-cancel]').forEach(el=>el.addEventListener('click',()=>{if(state?.active_request_id)action('/api/cancel',{request_id:state.active_request_id}).then(r=>{if(r)notice('양팔 취소를 요청했습니다. 취소 완료 상태를 확인하세요.');});}));
 document.querySelectorAll('[data-camera]').forEach(el=>el.addEventListener('click',async()=>{if(await action('/api/camera',{view:el.dataset.camera}))document.querySelectorAll('[data-camera]').forEach(b=>b.classList.toggle('selected',b===el));}));
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
