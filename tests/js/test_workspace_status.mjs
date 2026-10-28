import test from 'node:test';
import assert from 'node:assert/strict';
import {workspaceEvidence} from '../../src/mujoco_ros2_manipulation/operator_console/static/workspace_status.mjs';

const ID='a'.repeat(32),OTHER='b'.repeat(32);
const record=extra=>({request_id:ID,session_id:'session-a',mode:'approved_plan',status:'running',stage:'first_motion',...extra});
const operation=extra=>({kind:'operation-observation-v1',schema_version:1,request_id:ID,session_id:'session-a',events:{items:[]},...extra});
const state=extra=>({history:[record()],active_request_id:ID,guard:{latched:false,phase:'ready',reason:''},state_age_sec:.1,clock_age_sec:.1,lifecycle:{phase:'running',owned:{simulation:true,commands:true}},...extra});

test('the helper is pure, returns five scoped rows, and cancelled never establishes Action or physical stop',()=>{
 const input=state({active_request_id:null,history:[record({status:'cancelled',stage:'terminal'})]});const original=structuredClone(input),rows=workspaceEvidence(input);
 assert.equal(rows.length,5);assert.deepEqual(input,original);
 assert.match(rows[0].value,/요청 aaaaaaaa.*취소 수신 시점 미확인/);
 assert.match(rows[1].value,/개별 Action 결과 없음/);assert.match(rows[2].value,/현재 감시 중.*정지 판정 아님/);
 assert.match(rows[3].value,/요청 취소 완료.*물리 정지와 구분/);assert.match(rows[4].value,/현재 소유 프로세스 2개 실행 중/);
});

test('active request missing from the snapshot cannot inherit a completed older request',()=>{
 const rows=workspaceEvidence(state({active_request_id:OTHER,history:[record({status:'succeeded'})]}));
 assert.match(rows[3].value,/현재 요청 기록 없음.*요청 종료 미확인/);assert.notEqual(rows[3].tone,'good');
});

test('request cancellation acknowledgement survives compact event omission but mismatched operation IDs are ignored',()=>{
 const observed=record({operation:operation({cancellation:{state:'server_acknowledged',server_acknowledged:true},events:{items:[{name:'terminal'}],omitted_prefix_count:14}})});
 assert.match(workspaceEvidence(state({history:[observed]}))[0].value,/취소 수신 확인/);
 for(const mismatch of [{request_id:OTHER},{session_id:'other-session'}]){
  const changed={...observed,operation:{...observed.operation,...mismatch}};assert.doesNotMatch(workspaceEvidence(state({history:[changed]}))[0].value,/취소 수신 확인/);
 }
});

test('last individual Action event is presented as a partial observation, never as all-arms success',()=>{
 const rows=workspaceEvidence(state({history:[record({operation:operation({events:{items:[{name:'goal_result',arm:'left',status:5,error_code:0}],source_count:20,retained_count:1,omitted_prefix_count:19}})})]}));
 assert.match(rows[1].value,/left.*취소 응답.*개별 관측.*전체 결과는 상세 기록/);assert.notEqual(rows[1].tone,'good');
});

test('plan-only success means plan complete without an Action execution',()=>{
 const rows=workspaceEvidence(state({active_request_id:null,history:[record({mode:'plan_only',status:'succeeded',stage:'plan_ready'})]}));
 assert.match(rows[1].value,/계획 요청.*Action 실행 없음/);assert.match(rows[3].value,/계획 생성 완료.*아직 실행하지 않음/);
});

test('the compact actions_done timestamp confirms response completion without inventing individual results',()=>{
 for(const time of [0,2.4]){
  const rows=workspaceEvidence(state({history:[record({status:'succeeded',stage:'terminal',metrics:{received_to_actions_done_wall_sec:time}})]}));
  assert.match(rows[1].value,/양팔 Action 응답 완료.*개별 결과는 상세 기록/);assert.notEqual(rows[1].tone,'good');
 }
 for(const time of [null,NaN,-1]){
  assert.doesNotMatch(workspaceEvidence(state({history:[record({metrics:{received_to_actions_done_wall_sec:time}})]}))[1].value,/응답 완료/);
 }
});

test('current guard latch is explicit about its scope and requires a known reason for a stop report',()=>{
 for(const reason of ['contact','clock_stall']){
  const rows=workspaceEvidence(state({guard:{latched:true,phase:'stopped',reason}}));
  assert.match(rows[2].value,/현재 감시.*정지 보고.*요청별 판정 아님/);assert.notEqual(rows[2].tone,'good');
 }
 for(const guard of [{latched:true,phase:'stopped',reason:'unknown'},{latched:false,phase:'stopped',reason:'contact'},null]){
  assert.doesNotMatch(workspaceEvidence(state({guard}))[2].value,/정지 보고/);
 }
 assert.match(workspaceEvidence(state({guard:{latched:true,phase:'stopping',reason:'contact'}}))[2].value,/정지 확인 중/);
});

test('disconnection preserves historical scope while current guard and cleanup stay unknown',()=>{
 const rows=workspaceEvidence(state({history:[record({status:'succeeded'})],guard:{latched:true,phase:'stopped',reason:'contact'},lifecycle:{phase:'idle',owned:{}}}),{connected:false});
 assert.match(rows[3].value,/마지막 수신.*요청 성공/);assert.equal(rows[3].tone,'warn');
 assert.match(rows[2].value,/조회 중단.*정지 상태 미확인/);assert.match(rows[4].value,/조회 중단.*정리 결과 미확인/);
 assert.ok(rows.every(row=>row.tone!=='good'));
});

test('idle or no owned processes does not prove successful cleanup of the latest request',()=>{
 for(const lifecycle of [{phase:'idle',owned:{}},{phase:'idle',owned:{simulation:false,commands:false}},{phase:'running',owned:{}}]){
  const last=workspaceEvidence(state({history:[record({status:'cancelled'})],lifecycle}))[4];
  assert.match(last.value,/현재 실행 중인 소유 프로세스 없음.*정리 성공 여부는 종료 기록 확인/);assert.notEqual(last.tone,'good');
 }
 assert.match(workspaceEvidence(state({lifecycle:{phase:'stopping',owned:{}}}))[4].value,/정리 중.*완료 미확인/);
 assert.equal(workspaceEvidence(state({lifecycle:{phase:'failed',owned:{},error:'abnormal shutdown'}}))[4].tone,'bad');
 assert.match(workspaceEvidence(null)[4].value,/미확인/);
});

test('missing or stale joint/clock ages do not become fresh observation',()=>{
 for(const ages of [{state_age_sec:null},{state_age_sec:1},{clock_age_sec:NaN},{clock_age_sec:-1}]){
  assert.match(workspaceEvidence(state(ages))[2].value,/수신 지연·미확인/);
 }
});

test('cancelled before publish never says the command node received a cancellation',()=>{
 const rows=workspaceEvidence(state({history:[record({status:'cancelled',stage:'console_stopped_before_publish'})]}));
 assert.match(rows[0].value,/전송 전에 취소됨/);assert.doesNotMatch(rows[0].value,/취소 수신 확인/);assert.match(rows[1].value,/Action 실행 없음/);
});

test('local cancellation phases describe delivery without assuming server receipt or stop',()=>{
 for(const [phase,pattern] of [['submitting',/취소 요청 전달 중.*노드 수신 미확인/],['submitted',/콘솔이 취소 요청을 접수.*명령 노드 수신 미확인/],['unconfirmed',/취소 전달 결과 미확인.*자동 재전송 없음/]]){
  const cancellation={request_id:ID,phase},before=structuredClone(cancellation);
  const rows=workspaceEvidence(state(),{cancellation});
  assert.match(rows[0].value,pattern);assert.doesNotMatch(rows[0].value,/취소 수신 확인/);
  assert.match(rows[2].value,/정지 판정 아님/);assert.match(rows[3].value,/처리 중/);assert.deepEqual(cancellation,before);
 }
});

test('local cancellation applies only to the matching nonterminal request and server acknowledgement wins',()=>{
 const cancellation={request_id:ID,phase:'unconfirmed'};
 assert.doesNotMatch(workspaceEvidence(state(),{cancellation:{...cancellation,request_id:OTHER}})[0].value,/취소 전달 결과 미확인/);
 for(const status of ['succeeded','failed','cancelled','rejected','unknown']){
  assert.doesNotMatch(workspaceEvidence(state({active_request_id:null,history:[record({status,stage:'terminal'})]}),{cancellation})[0].value,/취소 전달 결과 미확인/);
 }
 for(const ack of [{stage:'cancel_received'},{operation:operation({cancellation:{server_acknowledged:true,state:'server_acknowledged'}})}]){
  const rows=workspaceEvidence(state({history:[record(ack)]}),{cancellation});
  assert.match(rows[0].value,/취소 수신 확인/);assert.doesNotMatch(rows[0].value,/취소 전달 결과 미확인/);
 }
 const disconnected=workspaceEvidence(state(),{connected:false,cancellation})[0];
 assert.match(disconnected.value,/마지막 수신.*취소 전달 결과 미확인/);assert.equal(disconnected.tone,'warn');
});
