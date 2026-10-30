/** Native task UI owns no ROS logic. The startup-pinned server remains authority. */
import {graspIdentity,graspJobActive,graspPresentation,graspStageLabels,graspPhaseLabels} from './workspace_status.mjs';
const finite=value=>typeof value==='number'&&Number.isFinite(value);
const format=(value,digits=3,suffix='')=>finite(value)?value.toFixed(digits)+suffix:'—';
const TIMEOUT_MS=3000;
export class WorkspaceTaskControls {
 constructor({document,request,refresh,beforeCommand=()=>{},afterCommand=()=>{},notify=()=>{},showOperate=()=>{},cameraReady=()=>true,now=()=>performance.now(),schedule=setTimeout,unschedule=clearTimeout}){
  Object.assign(this,{document,request,refresh,beforeCommand,afterCommand,notify,showOperate,cameraReady,now,schedule,unschedule});
  this.state=null;this.connected=false;this.pending=null;this.sample=null;this.eventsKey=null;this.needsFresh=true;this.epoch=0;this.closed=false;this.unconfirmed=null;
  this.get('task-select').addEventListener('change',()=>this.command('select',this.get('task-select').value));
  this.get('grasp-start').addEventListener('click',()=>this.command('start'));
  this.get('grasp-cancel').addEventListener('click',()=>this.command('cancel'));
 }
 get(id){return this.document.getElementById(id);}
 text(id,value){this.get(id).textContent=String(value??'—');}
 selected(){return this.state?.task?.selected==='grasp';}
 observe(state){
  this.connected=true;this.needsFresh=false;
  if(this.unconfirmed){const pending=this.unconfirmed,task=state?.task,job=task?.grasp?.job;const changed=task?.selection_id!==pending.selection_id;const started=pending.action==='start'&&job&&(job.job_id!==pending.job_id||job.generation>pending.generation);const cancelled=pending.action==='cancel'&&job?.job_id===pending.job_id&&(job.cancel_requested||job.finished);if(changed||started||cancelled)this.unconfirmed=null;}
  const task=state?.task,job=task?.grasp?.job;
  if(task?.selected==='grasp'&&job){
   const key=JSON.stringify([task.selection_id,task.grasp.source?.model_sha256,job.job_id,job.generation,task.grasp.view?.binding_id]),sequence=job.sequence;
   const reported=finite(job.source_age_sec)&&job.source_age_sec>=0?job.source_age_sec:null,now=this.now();
   const newer=!this.sample||key!==this.sample.key||Number.isSafeInteger(sequence)&&sequence>this.sample.sequence;
   const elapsed=this.sample&&finite(this.sample.age)?this.sample.age+Math.max(0,now-this.sample.at)/1000:null;
   const age=newer?reported:elapsed===null?reported:reported===null?elapsed:Math.max(elapsed,reported);
   this.sample={key,sequence,age,at:now};state={...state,task:{...task,observation_age_sec:age}};
  }else this.sample=null;
  this.state=state;return state;
 }
 lost(){this.connected=false;this.needsFresh=true;this.render();}
 render(){
  const task=this.state?.task,selected=this.selected(),ready=this.connected&&!this.needsFresh&&!this.pending&&!this.closed&&!this.unconfirmed;
  this.get('task-selection').hidden=!task;
  this.get('operate').dataset.task=selected?'grasp':'motion';
  if(!task)return;
  const select=this.get('task-select'),signature=JSON.stringify(task.options||[]);
  if(signature!==this.optionsKey){select.replaceChildren();for(const option of task.options||[]){const item=this.document.createElement('option');item.value=option.id;item.textContent=option.label;item.disabled=!option.available;select.append(item);}this.optionsKey=signature;}
  select.value=task.selected;select.disabled=!ready||!task.can_select;
  this.text('task-selection-reason',this.pending==='select'?'작업과 표시 모델을 전환하고 있습니다.':!this.connected?'상태 조회 중단 · 현재 작업을 확인하기 전에는 전환하지 않습니다.':this.unconfirmed?'이전 명령 결과 미확인 · 실행 기록과 상태를 확인하세요. 자동 재전송하지 않습니다.':task.reason||'작업을 선택해도 실행하지 않습니다. 실행 버튼으로 직접 시작하세요.');
  this.get('grasp-task-panel').hidden=!selected;
  this.get('motion-goal-editor').hidden=selected;
  this.get('task-footnote').hidden=selected;
  for(const id of ['start','attach','planning-case','native-gui'])this.get(id).hidden=selected||id==='attach'&&this.get(id).hidden;
  this.get('stop').hidden=selected;
  for(const node of this.document.querySelectorAll('[data-view="review"], [data-view="demos"]'))node.disabled=selected;
  if(!selected){for(const node of this.document.querySelectorAll('[data-step]'))node.textContent=['01 목표 수신','02 계획·검사','03 양팔 동작','04 완료 확인'][Number(node.dataset.step)];return;}
  const p=graspPresentation(this.state,{connected:this.connected}),job=p.job;
  const canCancel=!this.closed&&!this.pending&&p.matched&&graspJobActive(job)&&!job.cancel_requested;
  this.get('grasp-start').disabled=!ready||!p.matched||!task.can_start||!this.cameraReady();
  this.get('grasp-cancel').disabled=!canCancel;this.text('grasp-cancel',!this.connected?'마지막 확인 집기 작업 취소':'집기 작업 취소');
  for(const button of this.document.querySelectorAll('[data-cancel]')){button.disabled=!canCancel;button.textContent=job?.cancel_requested&&!job.finished?'집기 취소 처리 · 정리 확인 중':!this.connected?'■ 마지막 확인 집기 작업 취소':'■ 집기 작업 취소';}
  this.text('grasp-source',p.source?`${p.source.label} · ROS DOMAIN ${p.source.domain}`:'등록한 실행 대상 확인 중');
  this.text('grasp-model',p.source?.model_sha256||'—');
  this.text('grasp-stage',this.pending==='start'?'실행 응답 대기':this.pending==='cancel'?'취소 전달 결과 확인 중':p.stage);
  this.text('grasp-status',this.pending==='start'?'실행 요청 중':this.pending==='cancel'?'취소 요청 중':p.status);
  this.get('grasp-status').className='badge'+(job?.status==='failed'?' bad':p.active?' busy':'');
  this.text('grasp-admission',this.pending?'명령의 응답을 기다립니다. 자동으로 다시 보내지 않습니다.':!this.connected?'상태 조회 끊김 · 마지막 확인 작업에 취소를 요청할 수 있습니다. 현재 실행과 정지 상태는 미확인입니다.':!p.matched?'선택한 실행 대상과 모델·세대가 일치하지 않습니다.':job?.finished?(job.owner_cleanup_clean===true?'작업이 종료됐습니다. 마지막 관측과 정리 기록을 확인하세요.':'소유 프로세스의 정리를 확인하지 못했습니다. 새 실행과 작업 전환을 차단합니다.'):job?.cancel_requested?'취소 전달·제어기 정지·소유 프로세스 정리를 각각 확인합니다.':!this.cameraReady()?'3D 모델과 자세 표시를 준비하고 있습니다.':task.reason||'검증용 그리퍼와 블록으로 집기 → 운반 → 놓기를 순서대로 실행합니다.');
  this.text('grasp-job-id',job?`${job.job_id} · 실행 세대 ${job.generation}`:'새 실행 전');
  this.text('grasp-physical',p.physical);this.text('grasp-attachment',p.attachment);
  this.text('grasp-source-age',format(p.age,2,' s'));
  this.text('grasp-relative-error',format(job?.evidence?.relative_position_error_m,5,' m'));
  this.text('grasp-finger-force',(job?.evidence?.finger_force_n||[]).map(value=>format(value,3,' N')).join(' / ')||'—');
  this.text('grasp-error',job?.error?JSON.stringify(job.error):'');this.get('grasp-error').hidden=!job?.error;
  const events=job?.task?.events||[],key=JSON.stringify([job?.job_id,job?.generation,events]);
  if(key!==this.eventsKey){this.eventsKey=key;this.get('grasp-events').replaceChildren();for(const event of events.slice(-20)){const li=this.document.createElement('li');li.textContent=(graspStageLabels[event.stage]||graspPhaseLabels[event.stage]||event.stage||'작업')+' · '+(event.event||'상태 변경');this.get('grasp-events').append(li);}}
  this.text('request-stage',p.stage);this.text('request-id',job?.job_id||'—');this.text('request-badge',p.status);
  this.get('request-badge').className=this.get('grasp-status').className;
  this.text('first-motion','—');this.text('terminal-time','—');this.text('clearance','—');
  this.text('request-error',job?.error?JSON.stringify(job.error):'');this.get('request-error').hidden=!job?.error;
  for(const node of this.document.querySelectorAll('[data-step]')){const index=Number(node.dataset.step);node.textContent=['01 접근·집기','02 집기 확인','03 운반·놓기','04 결과·정리'][index];node.classList.toggle('reached',job?.task?.events?.some(event=>index===0||index===1&&event.physical_grasp==='confirmed'||index===2&&event.stage==='carrying'||index===3&&job.finished&&job.owner_cleanup_clean===true));}
 }
 async command(action,choice){
  if(this.closed||this.pending)return false;
  if(action!=='cancel'&&(this.unconfirmed||!this.connected||this.needsFresh))return false;
  const task=this.state?.task,job=task?.grasp?.job;let path,body;
  if(action==='select'){
   if(!task?.can_select||choice===task.selected||!task.options?.some(row=>row.id===choice&&row.available))return false;
   path='/api/task/select';body={task:choice,selection_id:task.selection_id};
  }else if(action==='start'){
   if(!this.selected()||!graspIdentity(this.state)||!task.can_start||!this.cameraReady())return false;
   path='/api/task/grasp/start';body={selection_id:task.selection_id,request_id:crypto.randomUUID().replaceAll('-',''),model_sha256:task.grasp.source.model_sha256};
  }else if(action==='cancel'){
   if(!this.selected()||!graspIdentity(this.state)||!graspJobActive(job)||job.cancel_requested)return false;
   path='/api/task/grasp/cancel';body={selection_id:task.selection_id,job_id:job.job_id,generation:job.generation,model_sha256:job.model_sha256};
  }else return false;
  if(action==='cancel')this.unconfirmed=null; // An explicit manual retry targets the same exact cached owner only.
  const epoch=++this.epoch;this.pending=action;this.needsFresh=true;this.beforeCommand(action);this.render();
  let timer,success=false;
  try{
   // Timeout limits observation only. A late response cannot dispatch or retry.
   await Promise.race([this.request(path,body),new Promise((_,reject)=>{timer=this.schedule(()=>reject(Error('명령 응답 제한시간을 넘었습니다.')),TIMEOUT_MS);})]);
   if(this.closed||this.epoch!==epoch)return false;
   success=true;this.showOperate();
  }catch(error){if(!this.closed&&this.epoch===epoch){if(!Number.isInteger(error.httpStatus))this.unconfirmed={action,selection_id:task.selection_id,job_id:job?.job_id,generation:job?.generation??0};this.notify('명령 처리 결과를 확인하지 못했습니다. 자동으로 다시 보내지 않습니다. 최신 상태와 실행 기록을 확인하세요. '+error.message);}}
  finally{
   if(timer!==undefined)this.unschedule(timer);
   if(!this.closed&&this.epoch===epoch){try{await this.refresh();}catch(_error){this.connected=false;this.needsFresh=true;}this.pending=null;this.afterCommand();this.render();}
  }
  return success;
 }
 destroy(){this.closed=true;this.epoch++;this.pending=null;this.needsFresh=true;}
}
