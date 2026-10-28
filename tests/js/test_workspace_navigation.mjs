/** Workspace navigation must remain presentation-only and preserve edited goals. */
import test from 'node:test';
import assert from 'node:assert/strict';
import {readFile} from 'node:fs/promises';
import vm from 'node:vm';
import {workspaceEvidence} from '../../src/mujoco_ros2_manipulation/operator_console/static/workspace_status.mjs';

const directory=new URL('../../src/mujoco_ros2_manipulation/operator_console/static/',import.meta.url);
const app=(await readFile(new URL('app.js',directory),'utf8'))
 .replace(/boot\(\)\.catch\(error=>\{notice\('조작 화면을 불러오지 못했습니다: '\+error\.message\);\}\);\s*$/,'');
const workspace=(await readFile(new URL('workspace.js',directory),'utf8')).replace(/^import .*?;\n/,'');
const html=await readFile(new URL('index.html',directory),'utf8');
const REQUEST='a'.repeat(32),OTHER='b'.repeat(32);

class Events {
 constructor(){this.listeners=new Map();}
 addEventListener(type,listener){const list=this.listeners.get(type)||[];list.push(listener);this.listeners.set(type,list);}
 dispatchEvent(event){for(const listener of this.listeners.get(event.type)||[])listener(event);return true;}
}
class Element extends Events {
 constructor(id=''){
  super();this.id=id;this.dataset={};this.hidden=false;this.disabled=false;this.value='';this.textContent='';this.children=[];this.attributes={};
  const classes=new Set();this.classList={contains:name=>classes.has(name),toggle(name,force){const enabled=force??!classes.has(name);if(enabled)classes.add(name);else classes.delete(name);return enabled;}};
 }
 setAttribute(name,value){this.attributes[name]=String(value);}
 removeAttribute(name){delete this.attributes[name];}
 getAttribute(name){return this.attributes[name]??null;}
 append(...nodes){this.children.push(...nodes);}
 replaceChildren(...nodes){this.children=[...nodes];}
 querySelector(selector){if(selector==='h3'){this.heading??=new Element();return this.heading;}if(selector==='p'){this.paragraph??=new Element();return this.paragraph;}return null;}
 click(){if(!this.disabled)this.dispatchEvent({type:'click'});}
}
class CustomEvent {
 constructor(type,options={}){this.type=type;this.detail=options.detail;}
}
function harness({narrow=false}={}){
 const elements=new Map(),window=new Events(),media=new Events(),network=[];
 media.matches=narrow;
 const element=id=>{if(!elements.has(id))elements.set(id,new Element(id));return elements.get(id);};
 const views=['operate','history','analysis','reports','demos'].map(element);
 const navigation=['operate','review','analysis','history','demos','reports'].map(view=>{const node=new Element('nav-'+view);node.dataset.view=view;return node;});
 const scene=element('scene-slot'),canvas=new Element('canvas');scene.append(canvas);
 element('target-left').value='12.75';element('pose-x').value='0.385';element('strategy').value='departure_timing';
 element('review-approve').disabled=true;element('cancel').disabled=false;
 const document={getElementById:element,createElement:()=>new Element(),querySelectorAll(selector){if(selector==='.view')return views;if(selector==='[data-view]')return navigation;return [];}};
 let mounts=0;
 const context=vm.createContext({document,window,CustomEvent,matchMedia:()=>media,workspaceEvidence,
  fetch:(...args)=>{network.push(args);throw Error('Navigation attempted network I/O');},
  scene:{mount(){mounts++;assert.equal(element('scene-slot'),scene);assert.equal(scene.children[0],canvas);}}});
 // Only boot is omitted. The same app navigation implementation and workspace
 // event handlers run together; renderer internals have their own SceneControls tests.
 vm.runInContext(app+`\nsceneControls=globalThis.scene;targets={left:.12,right:.34};globalThis.navigationTest={show,targets:()=>({...targets})};`,context);
 vm.runInContext(workspace,context);
 return {element,window,media,views,navigation,network,canvas,scene,api:context.navigationTest,mounts:()=>mounts,
  observe:state=>window.dispatchEvent(new CustomEvent('console-observation',{detail:state})),
  lost:()=>window.dispatchEvent(new CustomEvent('console-observation-lost'))};
}
const snapshot=overrides=>({active_request_id:null,history:[],pending_plan:null,lifecycle:{phase:'idle',owned:{}},...overrides});

test('operation and review share the existing canvas and preserve edited joint/Pose goals',()=>{
 const h=harness(),targets=JSON.stringify(h.api.targets()),joint=h.element('target-left'),pose=h.element('pose-x');
 for(const mode of ['review','operate','review','analysis','operate']){
  h.api.show(mode);
  assert.equal(h.element('target-left'),joint);assert.equal(joint.value,'12.75');
  assert.equal(h.element('pose-x'),pose);assert.equal(pose.value,'0.385');
  assert.equal(h.element('strategy').value,'departure_timing');
  assert.equal(JSON.stringify(h.api.targets()),targets);
  assert.equal(h.scene.children[0],h.canvas);
  assert.equal(h.element('operate').hidden,mode==='analysis');
  assert.equal(h.navigation.filter(node=>node.getAttribute('aria-current')==='page').length,1);
  assert.equal(h.navigation.find(node=>node.dataset.view===mode).getAttribute('aria-current'),'page');
 }
 assert.equal(h.mounts(),5);assert.deepEqual(h.network,[]);
 assert.equal(h.element('operate').dataset.workspaceMode,'operate');
});

test('panel collapse, mode changes and responsive changes never approve, cancel or send a request',()=>{
 const h=harness();h.observe(snapshot({pending_plan:{state:'awaiting_approval',remaining_sec:50}}));
 for(const id of ['toggle-objects','toggle-task','toggle-objects','toggle-task'])h.element(id).click();
 for(const mode of ['review','history','analysis','demos','reports','operate'])h.api.show(mode);
 h.media.matches=true;h.media.dispatchEvent({type:'change'});
 assert.deepEqual(h.network,[]);
 assert.equal(h.element('review-approve').disabled,true,'presentation cannot override the approval gate');
 assert.equal(h.element('cancel').disabled,false,'presentation cannot change ownership or cancellation admission');
});

test('responsive object collapse follows width until the user makes a choice',()=>{
 const h=harness({narrow:true}),toggle=h.element('toggle-objects'),panel=h.element('operate');
 assert.equal(panel.classList.contains('objects-collapsed'),true);assert.equal(toggle.getAttribute('aria-expanded'),'false');
 h.media.matches=false;h.media.dispatchEvent({type:'change'});
 assert.equal(panel.classList.contains('objects-collapsed'),false);assert.equal(toggle.getAttribute('aria-expanded'),'true');
 toggle.click();assert.equal(panel.classList.contains('objects-collapsed'),true);
 h.media.matches=false;h.media.dispatchEvent({type:'change'});h.api.show('review');
 assert.equal(panel.classList.contains('objects-collapsed'),true,'mode/width changes preserve the explicit user choice');
 h.element('toggle-task').click();assert.equal(h.element('toggle-task').getAttribute('aria-expanded'),'false');
 h.api.show('operate');assert.equal(h.element('toggle-task').getAttribute('aria-expanded'),'true','entering a task brings its actions back');
});

test('empty review distinguishes planning, running execution and missing active-request evidence',()=>{
 const h=harness();h.api.show('review');
 const heading=()=>h.element('review-empty').querySelector('h3').textContent;
 h.observe(snapshot());assert.equal(h.element('review-empty').hidden,false);assert.match(heading(),/검토할 계획이 없습니다/);
 h.observe(snapshot({active_request_id:REQUEST,history:[{request_id:REQUEST,status:'running',mode:'plan_only'}]}));
 assert.match(heading(),/계획과 검사 결과/);
 h.observe(snapshot({active_request_id:REQUEST,history:[{request_id:REQUEST,status:'running',mode:'approved_plan'}]}));
 assert.match(heading(),/현재 요청을 실행/);assert.doesNotMatch(heading(),/계획과 검사 결과/);
 h.observe(snapshot({active_request_id:OTHER,history:[{request_id:REQUEST,status:'succeeded',mode:'plan_only'}]}));
 assert.match(heading(),/현재 요청 상태를 확인/);assert.equal(h.element('activity-record').disabled,true);
});

test('pending-plan state controls the review count without enabling execution or replacing expired-plan details',()=>{
 const h=harness();h.api.show('review');
 for(const state of ['awaiting_approval','expired','invalidated','consumed']){
  h.observe(snapshot({pending_plan:{state,remaining_sec:state==='awaiting_approval'?50:0}}));
  assert.equal(h.element('review-empty').hidden,true,'the review panel owns existing-plan expiry/invalidity details');
  assert.equal(h.element('review-count').hidden,state!=='awaiting_approval');
  assert.equal(h.element('review-approve').disabled,true);
 }
 h.observe(snapshot());assert.equal(h.element('review-empty').hidden,false);assert.equal(h.element('review-count').hidden,true);
 assert.deepEqual(h.network,[]);
});

test('saved trajectory preview is read-only even while another pending review remains in the snapshot',()=>{
 for(const pending_plan of [null,{state:'awaiting_approval',remaining_sec:35,plan_request_id:OTHER},{state:'expired',remaining_sec:0,plan_request_id:OTHER}]){
  const h=harness();h.api.show('review');
  h.observe(snapshot({pending_plan,view:{mode:'plan_preview',review_plan:false,binding:{request_id:REQUEST}}}));
  const panel=h.element('review-empty');
  assert.equal(panel.hidden,false);
  assert.equal(h.element('operate').classList.contains('saved-preview'),true);
  assert.match(panel.querySelector('h3').textContent,/저장한 궤적/);
  assert.match(panel.querySelector('p').textContent,/읽기 전용.*바로 실행할 수 없습니다/);
  assert.equal(h.element('review-approve').disabled,true);
  assert.equal(h.scene.children[0],h.canvas);
  h.api.show('operate');assert.equal(h.element('operate').classList.contains('saved-preview'),false);
  assert.equal(panel.hidden,true);
  h.api.show('review');assert.equal(panel.hidden,false);
  h.observe(snapshot({pending_plan:{state:'awaiting_approval',remaining_sec:25,plan_request_id:OTHER},view:{mode:'plan_preview',review_plan:true,binding:{request_id:OTHER}}}));
  assert.equal(h.element('operate').classList.contains('saved-preview'),false);
  assert.equal(panel.hidden,true,'the exact execution review takes over only when it is the displayed preview');
  assert.deepEqual(h.network,[]);
 }
});

test('failed and disconnected observations remain visibly distinct, and records are navigation-only',()=>{
 const h=harness();let opened=0;h.window.addEventListener('workspace-record',()=>opened++);
 h.observe(snapshot({history:[{request_id:REQUEST,mode:'approved_plan',status:'failed'}]}));
 assert.match(h.element('activity-summary').textContent,/aaaaaaaa.*실패/);
 assert.equal(h.element('activity-record').disabled,false);h.element('activity-record').click();assert.equal(opened,1);
 h.lost();assert.match(h.element('activity-summary').textContent,/조회 대기.*마지막 자료/);
 const evidence=h.element('execution-evidence').children.map(row=>row.children[1].textContent).join('\n');
 assert.match(evidence,/마지막 수신.*실패/);assert.match(evidence,/정지 상태 미확인/);assert.match(evidence,/정리 결과 미확인/);
 assert.deepEqual(h.network,[]);
});

test('the delivered shell has one owner for each goal, approval and scene control ID',()=>{
 const ids=[...html.matchAll(/\bid="([^"]+)"/g)].map(match=>match[1]);
 const duplicates=ids.filter((id,index)=>ids.indexOf(id)!==index);
 assert.deepEqual(duplicates,[],'duplicate IDs silently wire a new panel to the wrong command');
 for(const id of ['scene-slot','goal-kind','joint-fields','pose-arm','execute','review-card','review-approve','review-discard','review-preview']){
  assert.equal(ids.filter(value=>value===id).length,1,id);
 }
 assert.equal((html.match(/id="operate"/g)||[]).length,1);
 assert.equal((html.match(/id="review"/g)||[]).length,0,'review is a presentation of the existing operate workspace');
});
