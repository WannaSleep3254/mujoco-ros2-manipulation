import {workspaceEvidence} from './workspace_status.mjs';
// Presentation state only. Existing app.js handlers own every command and approval.
const get=id=>document.getElementById(id),operate=get('operate');
let observation=null,connected=false,view='operate',cancellation=null;
const narrow=matchMedia('(max-width:1199px)');
let objectsChosen=false;
function setObjects(collapsed){
 operate.classList.toggle('objects-collapsed',collapsed);
 get('toggle-objects').setAttribute('aria-expanded',String(!collapsed));
 get('toggle-objects').textContent=collapsed?'객체 목록 펼치기':'객체 목록 접기';
}
function setTask(collapsed){
 operate.classList.toggle('task-collapsed',collapsed);
 get('toggle-task').setAttribute('aria-expanded',String(!collapsed));
 get('toggle-task').textContent=collapsed?'작업 패널 펼치기':'작업 패널 접기';
}
function render(){
 const reviewing=view==='review',plan=observation?.pending_plan,saved=reviewing&&observation?.view?.mode==='plan_preview'&&observation.view.review_plan!==true;
 operate.classList.toggle('saved-preview',saved);
 const grasp=observation?.task?.selected==='grasp',job=observation?.task?.grasp?.job;
 get('task-title').textContent=grasp?'집기·운반·놓기':reviewing?'계획 검토':'목표 설정';
 get('task-kicker').textContent=grasp?'ROS GRASP TASK':reviewing?'REVIEW & EXECUTE':'TARGET INPUT';
 get('review-empty').hidden=!reviewing||Boolean(plan)&&!saved;
 if(reviewing&&!plan){const active=observation?.history?.find(row=>row.request_id===observation.active_request_id);get('review-empty').querySelector('h3').textContent=observation?.active_request_id?(active?.mode==='plan_only'?'계획과 검사 결과를 기다립니다':active?'현재 요청을 실행하고 있습니다':'현재 요청 상태를 확인하고 있습니다'):'검토할 계획이 없습니다';}
 if(saved)get('review-empty').querySelector('h3').textContent='저장한 궤적을 확인하고 있습니다';
 get('review-empty').querySelector('p').textContent=saved?'기록된 경로의 읽기 전용 재생입니다. 이 궤적을 바로 실행할 수 없습니다. 새 동작은 조작 화면에서 계획하고 승인하세요.':'조작 화면에서 목표를 정하고 계획을 생성하세요. 저장된 계획은 실행 기록에서 다시 볼 수 있습니다.';
 const count=get('review-count');count.hidden=plan?.state!=='awaiting_approval';count.textContent='1';
 const current=observation?.history?.find(row=>row.request_id===observation.active_request_id)||(!observation?.active_request_id?observation?.history?.[0]:null);
 get('activity-summary').textContent=!connected?'상태 조회 대기 · 마지막 자료':current?`${current.request_id.slice(0,8)} · ${({queued:'접수',running:'진행 중',succeeded:'요청 성공',cancelled:'요청 취소 완료',failed:'실패',rejected:'거부',unknown:'결과 미확인'})[current.status]||'처리 중'}`:'목표 설정 → 계획 검토 → 승인·실행';
 if(grasp)get('activity-summary').textContent=!connected?'집기 상태 조회 대기 · 마지막 자료':job?`${job.job_id.slice(0,8)} · ${({starting:'시작 중',running:'진행 중',cancelling:'취소·정리 중',succeeded:'작업 완료',failed:'작업 실패',cancelled:'취소 완료'})[job.status]||'상태 확인 중'}`:'집기 → 운반 → 놓기 · 실행 전';
 get('activity-record').disabled=grasp||!current;
 const rows=workspaceEvidence(observation,{connected,cancellation});
 get('execution-evidence').replaceChildren(...rows.map(row=>{const div=document.createElement('div'),label=document.createElement('dt'),value=document.createElement('dd');label.textContent=row.label;value.textContent=row.value;value.className=row.tone;div.append(label,value);return div;}));
}
get('toggle-objects').addEventListener('click',()=>{objectsChosen=true;setObjects(!operate.classList.contains('objects-collapsed'));});
get('toggle-task').addEventListener('click',()=>setTask(!operate.classList.contains('task-collapsed')));
get('activity-record').addEventListener('click',()=>window.dispatchEvent(new CustomEvent('workspace-record')));
window.addEventListener('console-navigation',event=>{view=event.detail.view;if(view==='review'||view==='operate')setTask(false);render();});
window.addEventListener('console-cancellation',event=>{cancellation=event.detail;render();});
window.addEventListener('console-observation',event=>{observation=event.detail;connected=true;render();});
window.addEventListener('console-observation-lost',()=>{connected=false;render();});
narrow.addEventListener('change',()=>{if(!objectsChosen)setObjects(narrow.matches);});
setObjects(narrow.matches);render();
