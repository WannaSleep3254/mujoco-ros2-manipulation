/** Approval stays unavailable until the exact current plan is visible and valid. */
import test from 'node:test';
import assert from 'node:assert/strict';
import {readFile} from 'node:fs/promises';
import vm from 'node:vm';

const source=(await readFile(new URL('../../src/mujoco_ros2_manipulation/operator_console/static/app.js',import.meta.url),'utf8'))
  .replace(/boot\(\)\.catch\(error=>\{notice\('조작 화면을 불러오지 못했습니다: '\+error\.message\);\}\);\s*$/,'');

function harness(){
 const elements=new Map();
 const element=id=>{if(!elements.has(id))elements.set(id,{hidden:false,disabled:true,textContent:'',className:'',value:''});return elements.get(id);};
 element('strategy').value='coordinated';
 const context=vm.createContext({document:{getElementById:element}});
 vm.runInContext(source+`\nglobalThis.reviewTest={renderReview,set(value,positions,visible){state=value;targets=positions;sceneControls={cameraReady:()=>visible};}};`,context);
 const plan='b'.repeat(32),pending={plan_request_id:plan,plan_sha256:'c'.repeat(64),session_id:'a'.repeat(32),state:'awaiting_approval',remaining_sec:55};
 const state={pending_plan:pending,active_request_id:null,can_approve:true,demo:{current:null},
  history:[{request_id:plan,command:{strategy:'coordinated',target_rad:{left:.1,right:.2}}}],
  view:{mode:'plan_preview',review_plan:true,binding:{request_id:plan},display_mode:'web3d'}};
 return {element,state,render(visible=true,positions={left:.1,right:.2}){context.reviewTest.set(state,positions,visible);context.reviewTest.renderReview();}};
}

test('a generated plan never enables approval before its own preview has rendered',()=>{
 const h=harness();h.state.view.review_plan=false;h.render();
 assert.equal(h.element('review-preview').disabled,false);assert.equal(h.element('review-approve').disabled,true);
 h.state.view.review_plan=true;h.render(false);assert.equal(h.element('review-approve').disabled,true);
 h.render(true);assert.equal(h.element('review-approve').disabled,false);
 h.state.view.binding.request_id='e'.repeat(32);h.render();assert.equal(h.element('review-approve').disabled,true);
});

test('editing targets or strategy cannot approve the old goal from the command panel',()=>{
 const h=harness();h.render();assert.equal(h.element('review-approve').disabled,false);
 h.render(true,{left:.3,right:.2});assert.equal(h.element('review-approve').disabled,true);
 assert.match(h.element('review-admission').textContent,/목표나 계획 방식을 바꿨/);
 h.element('strategy').value='departure_timing';h.render();assert.equal(h.element('review-approve').disabled,true);
});

test('expiry, consumed approval, disconnect and another request keep execution disabled',()=>{
 for(const condition of ['expired','consumed','approval_queued','not_ready','active']){
  const h=harness();
  if(condition==='not_ready')h.state.can_approve=false;
  else if(condition==='active')h.state.active_request_id='d'.repeat(32);
  else h.state.pending_plan.state=condition;
  h.render();assert.equal(h.element('review-approve').disabled,true,condition);
 }
});
