/** Presentation of existing observations only; never admission or stop logic.
 * Request rows refer to the active request (otherwise the latest record).
 * Guard/lifecycle rows explicitly describe the current environment, not that
 * request's historical outcome. Compact status omits most Action events.
 */
const row=(label,value,tone='neutral')=>({label,value,tone});
const finite=value=>typeof value==='number'&&Number.isFinite(value);
const requestName=record=>'요청 '+String(record.request_id).slice(0,8);
const terminalStatuses=new Set(['succeeded','failed','cancelled','rejected','unknown']);

function selectedRequest(state){
 const history=Array.isArray(state?.history)?state.history:[];
 // An active ID missing from this snapshot must not inherit an older outcome.
 return state?.active_request_id?history.find(item=>item?.request_id===state.active_request_id):history.find(item=>item?.request_id);
}

function operationFor(record){
 const operation=record?.operation;
 return operation?.kind==='operation-observation-v1'&&operation.schema_version===1&&
  operation.request_id===record.request_id&&(!record.session_id||operation.session_id===record.session_id)?operation:null;
}

function delivery(record,operation,localCancellation){
 if(!record)return ['요청 기록 미확인','neutral'];
 if(record.stage==='console_stopped_before_publish')return ['명령 노드 전송 전에 취소됨','neutral'];
 const cancellation=operation?.cancellation?.state;
 if(record.stage==='cancel_received'||operation?.cancellation?.server_acknowledged===true)
  return ['취소 수신 확인 · 종료 결과는 별도 확인',terminalStatuses.has(record.status)?'neutral':'busy'];
 // Local transport state is narrower than a server acknowledgement. It applies
 // only to this still-active request, never an earlier/terminal record.
 if(localCancellation?.request_id===record.request_id&&!terminalStatuses.has(record.status)){
  if(localCancellation.phase==='unconfirmed')return ['취소 전달 결과 미확인 · 자동 재전송 없음','warn'];
  if(localCancellation.phase==='submitting')return ['취소 요청 전달 중 · 노드 수신 미확인','busy'];
  if(localCancellation.phase==='submitted')return ['콘솔이 취소 요청을 접수 · 명령 노드 수신 미확인','busy'];
 }
 if(cancellation==='submission_unconfirmed'||record.stage==='cancel_submission_unconfirmed')return ['취소 전달 결과 미확인','warn'];
 if(cancellation==='submission_in_progress'||record.stage==='cancel_submitting')return ['취소 요청 전달 중','busy'];
 if(cancellation==='submitted'||record.stage==='cancel_requested')return ['취소 요청 전달 · 노드 수신 미확인','busy'];
 if(record.status==='cancelled')return ['요청은 취소로 종료 · 취소 수신 시점 미확인','neutral'];
 if(record.status==='submitting'||record.stage==='submitted')return ['명령 노드 수신 확인 대기','busy'];
 if(record.status==='unknown')return ['전달·종료 결과 미확인','warn'];
 return ['취소 전달 여부는 기록에서 확인','neutral'];
}

function actionResult(record,operation){
 if(!record)return ['Action 결과 미확인','neutral'];
 if(record.mode==='plan_only')return ['계획 요청 · Action 실행 없음','neutral'];
 if(record.stage==='console_stopped_before_publish')return ['명령 노드 전송 전 취소 · Action 실행 없음','neutral'];
 const events=Array.isArray(operation?.events?.items)?operation.events.items:[];
 const result=[...events].reverse().find(event=>event?.name==='goal_result');
 if(result){
  const status=({4:'성공',5:'취소',6:'중단'})[result.status]||'상태 미확인';
  const arm=typeof result.arm==='string'?result.arm+' · ':'';
  const error=Number.isInteger(result.error_code)?' / code '+result.error_code:'';
  return [arm+status+' 응답'+error+' · 개별 관측, 전체 결과는 상세 기록',result.status===6||Number.isInteger(result.error_code)&&result.error_code!==0?'warn':'neutral'];
 }
 const responseTime=record.metrics?.received_to_actions_done_wall_sec;
 if(record.stage==='actions_done'||events.some(event=>event?.name==='actions_done')||finite(responseTime)&&responseTime>=0)
  return ['양팔 Action 응답 완료 · 개별 결과는 상세 기록','neutral'];
 return ['요약에 개별 Action 결과 없음 · 상세 기록에서 확인','neutral'];
}

function terminalResult(record){
 if(!record)return ['요청 종료 미확인','neutral'];
 if(record.mode==='plan_only'&&record.status==='succeeded')return [record.stage==='plan_not_required'?'이동 불필요 · 계획 요청 종료':'계획 생성 완료 · 아직 실행하지 않음','good'];
 const values={succeeded:['요청 성공','good'],failed:['요청 실패','bad'],cancelled:['요청 취소 완료 · 물리 정지와 구분','neutral'],
  rejected:['요청 거부','warn'],unknown:['종료 결과 미확인','warn'],queued:['접수 · 처리 중','busy'],running:['처리 중','busy'],submitting:['전송 확인 대기','busy']};
 return values[record.status]||['요청 종료 미확인','neutral'];
}

function currentStop(state,connected){
 if(!connected)return row('물리 정지 관측','조회 중단 · 현재 정지 상태 미확인','warn');
 const guard=state?.guard,reason=({contact:'접촉',clock_stall:'시계 정지'})[guard?.reason];
 if(guard?.latched===true&&reason){
  // This is a guard report, not a new velocity measurement or per-request proof.
  return row('물리 정지 관측','현재 감시 · '+reason+' / '+(guard.phase==='stopped'?'정지 보고':'정지 확인 중')+' · 요청별 판정 아님',guard.phase==='stopped'?'neutral':'warn');
 }
 if(guard?.latched===true)return row('물리 정지 관측','현재 감시 · 정지 원인·확인 결과 미확인','warn');
 if(guard?.latched===false&&guard.phase==='ready'){
  const fresh=finite(state.state_age_sec)&&state.state_age_sec>=0&&state.state_age_sec<=.5&&finite(state.clock_age_sec)&&state.clock_age_sec>=0&&state.clock_age_sec<=.5;
  return row('물리 정지 관측',fresh?'현재 감시 중 · 정지 판정 아님':'현재 상태 수신 지연·미확인 · 정지 판정 없음',fresh?'neutral':'warn');
 }
 return row('물리 정지 관측','현재 감시 결과 미확인','neutral');
}

function currentCleanup(state,connected){
 if(!connected)return row('소유 프로세스 정리','조회 중단 · 현재 정리 결과 미확인','warn');
 const lifecycle=state?.lifecycle;
 if(lifecycle?.error||lifecycle?.phase==='failed')return row('소유 프로세스 정리','현재 실행 환경 오류 · 종료 결과는 로그 확인','bad');
 if(lifecycle?.phase==='stopping')return row('소유 프로세스 정리','현재 환경 종료·자원 정리 중 · 완료 미확인','busy');
 const owned=lifecycle?.owned,entries=owned&&typeof owned==='object'?Object.values(owned):null;
 const running=entries?.filter(value=>value===true).length||0;
 if(running)return row('소유 프로세스 정리','현재 소유 프로세스 '+running+'개 실행 중 · 요청별 정리 판정 아님','neutral');
 if(entries&&entries.every(value=>typeof value==='boolean'))return row('소유 프로세스 정리','현재 실행 중인 소유 프로세스 없음 · 정리 성공 여부는 종료 기록 확인','neutral');
 return row('소유 프로세스 정리','현재 소유 프로세스·정리 결과 미확인','neutral');
}

export function workspaceEvidence(state,{connected=true,cancellation=null}={}){
 if(state?.task?.selected==='grasp')return graspEvidence(state,{connected});
 const record=selectedRequest(state),operation=operationFor(record);
 const prefix=(connected?'':'마지막 수신 · ')+(record?requestName(record)+' · ':state?.active_request_id?'현재 요청 기록 없음 · ':'');
 const requestRow=(label,result)=>row(label,prefix+result[0],connected?result[1]:'warn');
 return [requestRow('요청·취소 전달',delivery(record,operation,cancellation)),requestRow('Action 응답',actionResult(record,operation)),
  currentStop(state,connected),requestRow('요청 종료',terminalResult(record)),currentCleanup(state,connected)];
}


// These rows describe the selected native task. Historical physical evidence is
// never promoted to a current grasp after observation loss, stop or completion.
export const graspStageLabels={approaching:'접근',closing:'그리퍼 닫기',extracting:'들어 올리기',holding:'집기 확인',carrying:'운반',releasing:'놓기',settling:'안정화',succeeded:'배치 완료',failed:'실패',cancelled:'취소됨'};
export const graspPhaseLabels={preflight:'실행 대상 확인',unused_domain:'ROS 환경 확인',launch:'시뮬레이션 시작',initial_settle:'초기 안정화',approach:'접근',close:'그리퍼 닫기',extract:'들어 올리기',verification_lift:'집기 확인',transport:'운반',carry:'운반',place:'내려놓기',lower:'내려놓기',open:'그리퍼 열기',retract:'물러나기',settle:'배치 안정화',final:'최종 관측',cleanup:'실행 정리',complete:'작업 종료'};
export function graspJobActive(job){return Boolean(job&&(!job.finished||job.process_running===true));}
export function graspIdentity(state){
 const task=state?.task,snapshot=task?.grasp,source=snapshot?.source,job=snapshot?.job;
 return Boolean(task?.selected==='grasp'&&/^[0-9a-f]{32}$/.test(task.selection_id||'')&&source?.id==='scene:ros_fr5_grasp'&&/^[0-9a-f]{64}$/.test(source.model_sha256||'')&&/^[0-9a-f]{64}$/.test(source.manifest_sha256||'')&&(!job||job.model_sha256===source.model_sha256&&/^[0-9a-f]{32}$/.test(job.job_id||'')&&Number.isInteger(job.generation)&&job.generation>0));
}
export function graspPresentation(state,{connected=true}={}){
 const task=state?.task,snapshot=task?.grasp,job=snapshot?.job;
 const matched=graspIdentity(state),active=matched&&graspJobActive(job);
 const age=finite(task?.observation_age_sec)?task.observation_age_sec:job?.source_age_sec;
 const fresh=Boolean(matched&&connected&&active&&!job?.cancel_requested&&!job?.native_guard?.latched&&job?.native_guard?.phase==='ready'&&job?.freshness==='fresh'&&finite(age)&&age>=0&&age<=.5&&snapshot?.view?.mode==='live');
 const status=!connected?'상태 조회 중단':!matched?'집기 대상 식별 확인 필요':job?.cancel_requested&&!job.finished?'취소 처리 중':job?({starting:'시작 중',running:'실행 중',cancelling:'취소 처리 중',succeeded:'작업 완료',failed:'작업 실패',cancelled:'요청 취소 완료'})[job.status]||'결과 확인 중':'실행 전';
 return {job,source:snapshot?.source,matched,active,fresh,age,status,
  stage:graspStageLabels[job?.task?.stage]||graspPhaseLabels[job?.phase]||'단계 관측 대기',
  physical:!job?'물리 관측 없음':(fresh?'현재 관측 · ':'마지막 기록 · ')+({confirmed:'물리 집기 확인',released:'놓기 관측',slipped:'미끄러짐 관측',unconfirmed:'집기 미확인'})[job?.task?.physical_grasp||'unconfirmed'],
  attachment:({attached:'MoveIt 부착 확인',world:'MoveIt 작업공간 객체',uninitialized:'장면 연결 대기',unknown:'부착 상태 미확인'})[job?.task?.planning_attachment]||'부착 관측 대기'};
}
export function graspEvidence(state,{connected=true}={}){
 const p=graspPresentation(state,{connected}),job=p.job,prefix=connected?'':'마지막 수신 · ',guard=job?.native_guard;
 if(!p.matched)return [row('집기 대상','모델·실행 세대 식별 미확인','warn')];
 const stopped=guard?.latched===true&&guard?.phase==='stopped'&&['left_controller','right_controller','gripper_controller'].every(name=>guard.controllers?.[name]==='inactive');
 const clean=job?.cleanup?.clean===true&&job.owner_cleanup_clean===true;
 const action=job?.action_terminal;
 return [row('집기·놓기',p.physical,p.fresh?'neutral':'warn'),row('MoveIt 객체',prefix+p.attachment,'neutral'),
  row('취소 전달',prefix+(({not_requested:'요청 없음',signal_sent:'취소 신호 전달 · 수신 확인 중',worker_signal_sent:'취소 신호 전달 · 작업자 수신 확인 중',waiting_for_worker:'작업자 준비 후 취소 전달 대기',worker_observed:'작업자 취소 수신 확인',process_exited:'프로세스 종료 관측'})[job?.cancel_delivery]||'요청 없음'),'neutral'),
  row('개별 Action',prefix+(action?`${graspPhaseLabels[action.stage]||action.stage||'동작'} · ${({4:'성공',5:'취소',6:'중단'})[action.status]||'응답 확인 중'} · 전체 작업 결과와 구분`:'개별 종료 응답 대기'),'neutral'),
  row('제어기 정지',!connected?'조회 중단 · 정지 상태 미확인':stopped?'제어기 3개 비활성 확인':guard?.latched?'제어기 정지 처리 중':guard?.phase==='ready'?'정지 감시 중 · 정지 판정 아님':'정지 관측 대기','neutral'),
  row('작업 결과',prefix+p.status,connected&&job?.status==='failed'?'bad':'neutral'),
  row('소유 실행 정리',!connected?'조회 중단 · 정리 결과 미확인':!job?'실행 전':clean?'Native 종료·소유 프로세스 정리 확인':job.finished?'정리 결과 확인 필요':'실행 중 · 종료 후 확인',connected&&clean?'good':'warn')];
}
