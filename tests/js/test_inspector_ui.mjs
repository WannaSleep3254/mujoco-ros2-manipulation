/** Real read-only inspector code with delayed browser fetches and view epochs. */
import test from 'node:test';
import assert from 'node:assert/strict';
import {readFile} from 'node:fs/promises';
import vm from 'node:vm';

const source=await readFile(new URL('../../src/mujoco_ros2_manipulation/operator_console/static/inspector.js',import.meta.url),'utf8');
const appSource=(await readFile(new URL('../../src/mujoco_ros2_manipulation/operator_console/static/app.js',import.meta.url),'utf8'))
 .replace(/boot\(\)\.catch\(error=>\{notice\('조작 화면을 불러오지 못했습니다: '\+error\.message\);\}\);\s*$/,'');
const A='a'.repeat(32),B='b'.repeat(32),BINDING_A='a'.repeat(64),BINDING_B='b'.repeat(64),SCENE='c'.repeat(64);
const deferred=()=>{let resolve,reject;const promise=new Promise((yes,no)=>{resolve=yes;reject=no;});return {promise,resolve,reject};};
async function drain(){for(let index=0;index<20;index++)await Promise.resolve();}

class Element{
 constructor(tag='div'){this.tagName=tag.toUpperCase();this.children=[];this.listeners=new Map();this.hidden=false;this.disabled=false;this.textContent='';this._value='';this.dataset={};this.classList={toggle(){}};}
 get value(){return this._value;}
 set value(value){const text=String(value);this._value=this.tagName==='SELECT'&&!this.children.some(child=>child.value===text)?'':text;}
 append(...children){this.children.push(...children);if(this.tagName==='SELECT'&&!this.children.some(child=>child.value===this._value))this._value=this.children[0]?.value??'';}
 replaceChildren(...children){this.children=[...children];if(this.tagName==='SELECT')this._value=this.children[0]?.value??'';}
 addEventListener(name,callback){if(!this.listeners.has(name))this.listeners.set(name,[]);this.listeners.get(name).push(callback);}
 dispatchEvent(event){for(const callback of this.listeners.get(event.type)||[])callback(event);}
 removeAttribute(name){delete this[name];}
 setAttribute(name,value){this[name]=String(value);}
}
const content=element=>[element.textContent,...element.children.map(content)].join(' ');

function harness({hidden=false,panelHidden=false,withApp=false}={}){
 let now=10_000,timerID=0,urlID=0;
 const elements=new Map(),requests=[],timers=new Map(),created=[],revoked=[],listeners=new Map(),documentListeners=new Map();
 const element=id=>{if(!elements.has(id))elements.set(id,new Element(['sensor-frame-select','analysis-request','object-select'].includes(id)?'select':'div'));return elements.get(id);};
 const makeOption=(value,label)=>{const node=new Element('option');node.value=value;node.textContent=label;return node;};
 element('analysis').hidden=panelHidden;
 element('sensor-image').hidden=true;
 element('sensor-frame-select').append(makeOption('live','latest'));
 element('analysis-request').append(makeOption('','select'));
 element('object-select').append(makeOption('','select'));
 const nav=new Element('button');nav.dataset.view='analysis';
 const dispatch=(registry,event)=>{for(const callback of registry.get(event.type)||[])callback(event);};
 const document={hidden,getElementById:element,createElement:tag=>new Element(tag),
  querySelectorAll:selector=>selector==='[data-view="analysis"]'?[nav]:[],
  addEventListener(name,callback){if(!documentListeners.has(name))documentListeners.set(name,[]);documentListeners.get(name).push(callback);}};
 class Event{constructor(type,options={}){this.type=type;this.detail=options.detail;}}
 const window={addEventListener(name,callback){if(!listeners.has(name))listeners.set(name,[]);listeners.get(name).push(callback);},dispatchEvent(event){dispatch(listeners,event);}};
 const context=vm.createContext({document,window,CustomEvent:Event,AbortController,
  Date:{now:()=>now},setTimeout(callback,delay){const id=++timerID;timers.set(id,{callback,at:now+delay,delay});return id;},clearTimeout:id=>timers.delete(id),
  URL:{createObjectURL(blob){const url='blob:inspector-'+(++urlID);created.push({url,blob});return url;},revokeObjectURL:url=>revoked.push(url)},
  fetch(path,options={}){const pending=deferred();requests.push({path,options,...pending});return pending.promise;}});
 if(withApp){
  vm.runInContext(appSource,context);
  // Keep production render()/disconnected() and their real event delivery. The
  // unrelated command panels are outside this small inspector DOM fixture.
  vm.runInContext(`config={domain:160};
   for(const name of ['renderDemos','updateLiveFrames','renderContext','updateCurrent','refreshControls','history','requestProgress'])globalThis[name]=()=>{};
   globalThis.productionInspectorEvents={render,disconnected};`,context);
 }
 vm.runInContext(source,context);
 return {context,document,element,requests,created,revoked,timers,
  emit(value){window.dispatchEvent(new Event('console-observation',{detail:value}));},
  production(value){context.productionInspectorEvents.render(value);},
  lost(){window.dispatchEvent(new Event('console-observation-lost'));},
  visibility(value){document.hidden=value;dispatch(documentListeners,new Event('visibilitychange'));},
  select(id,value){element(id).value=value;element(id).dispatchEvent(new Event('change'));},
  advance(milliseconds){now+=milliseconds;},
  reads(prefix){return requests.filter(request=>request.path.startsWith(prefix));},
  reply(request,value={},status=200){request.resolve({status,ok:status>=200&&status<300,json:async()=>value,blob:async()=>value});},
  fail(request,message='synthetic read failure'){request.reject(new Error(message));},
  assertReadOnly(){assert.ok(requests.every(request=>!request.options.method||request.options.method==='GET'));
   assert.ok(requests.every(request=>!/^\/api\/(?:command|plan|cancel|demos|start|stop|target)/.test(request.path)));}
 };
}

function image(generation=A,sequence=1,historySequences=[sequence]){
 const frame=value=>({generation,sequence:value,image_available:true,stamp_sec:value+.25,age_sec:.1});
 return {generation,source_id:'camera0',status:'ready',clock_domain:'unknown',age_sec:.1,
  latest:frame(sequence),latest_received:frame(sequence),history:historySequences.map(frame)};
}
function observation({sensor=null,binding=null,history=[]}={}){
 return {image_source:sensor,history,view:{binding_id:binding,hierarchy_available:Boolean(binding),manifest_descriptor:{scene_sha256:SCENE}},
  lifecycle:{phase:'running'},guard:{phase:'ready',latched:false},demo:{current:null,history:[]},
  state_age_sec:.1,clock_age_sec:.1,simulation_sec:3,can_submit:false};
}
const hierarchy=(name,id)=>({scene_sha256:SCENE,bodies:[{id,body_id:1,parent_body_id:0,name,robot_instance_id:'left',tool_role:'unknown'}],geoms:[{geom_id:7,body_id:1}]});
const analysis=name=>({phase:'planning',status:'running',timeline:{items:[{name,wall_sec:1,simulation_sec:40}]},sources:{joints:{stamp_sec:40,age_sec:.1,clock_domain:'ros_simulation'}}});

test('inspector receives production app observation/lost events and only issues GETs',async()=>{
 const h=harness({withApp:true});
 h.production(observation({sensor:image(),binding:BINDING_A,history:[{request_id:A,status:'running',stage:'planning'}]}));
 assert.equal(h.reads('/api/image-source/frame').length,1);
 assert.equal(h.reads('/api/view/hierarchy').length,1);
 h.select('analysis-request',A);assert.equal(h.reads('/api/analysis/').length,1);
 h.reply(h.reads('/api/analysis/')[0],analysis('planning'));await drain();
 assert.match(content(h.element('analysis-events')),/planning/);
 vm.runInContext('productionInspectorEvents.disconnected()',h.context);
 assert.match(h.element('sensor-state').textContent,/상태 조회 중단/);
 h.assertReadOnly();
});

test('a delayed image from an older acquisition generation never replaces the current image',async()=>{
 const h=harness();h.emit(observation({sensor:image(A)}));const old=h.reads('/api/image-source/frame')[0];
 h.emit(observation({sensor:image(B)}));const current=h.reads('/api/image-source/frame')[1];
 h.reply(current,{frame:'current-generation'});await drain();const url=h.element('sensor-image').src;
 h.reply(old,{frame:'old-generation'});await drain();
 assert.equal(h.element('sensor-image').src,url);assert.equal(h.element('sensor-image').hidden,false);
 assert.equal(h.created.length,1);assert.equal(h.created[0].blob.frame,'current-generation');h.assertReadOnly();
});

test('an optional image source can be absent without stopping the request and hierarchy inspector',()=>{
 const h=harness();assert.doesNotThrow(()=>h.emit(observation({sensor:null,binding:BINDING_A})));
 assert.equal(h.reads('/api/image-source').length,0);assert.equal(h.reads('/api/view/hierarchy').length,1);
 assert.match(h.element('sensor-state').textContent,/등록한 이미지 공급원이 없습니다/);h.assertReadOnly();
});

test('old hierarchy response is rejected by binding even when both scenes have the same digest',async()=>{
 const h=harness();h.emit(observation({binding:BINDING_A}));const old=h.reads('/api/view/hierarchy')[0];
 h.emit(observation({binding:BINDING_B}));const current=h.reads('/api/view/hierarchy')[1];
 h.reply(current,hierarchy('current-body','current'));await drain();h.select('object-select','current');
 const details=content(h.element('object-details'));
 h.reply(old,hierarchy('old-body','old'));await drain();
 assert.match(details,/current-body/);assert.equal(content(h.element('object-details')),details);
 assert.equal(h.element('object-select').children.some(child=>child.value==='old'),false);h.assertReadOnly();
});

test('an evicted image selection remains visibly unavailable and never falls back to the live frame',async()=>{
 const h=harness();h.emit(observation({sensor:image(A,2,[1,2])}));
 h.reply(h.reads('/api/image-source/frame')[0],{frame:2});await drain();
 h.select('sensor-frame-select','1');h.reply(h.reads('/api/image-source/frame')[1],{frame:1});await drain();
 h.emit(observation({sensor:image(A,3,[3])}));await drain();
 assert.equal(h.element('sensor-frame-select').value,'1');assert.equal(h.element('sensor-image').hidden,true);
 assert.equal(h.element('sensor-image').src,undefined);assert.match(h.element('sensor-frame-note').textContent,/보존 범위 밖/);
 assert.equal(h.reads('/api/image-source/frame').length,2,'Eviction must not request current frame 3');h.assertReadOnly();
});

test('a hidden document or hidden analysis panel generates no image or hierarchy reads',async()=>{
 for(const hiddenKind of ['document','panel']){
  const h=harness({hidden:hiddenKind==='document',panelHidden:hiddenKind==='panel'});
  for(let index=0;index<8;index++){h.advance(2000);h.emit(observation({sensor:image(A,index+1),binding:index%2?BINDING_A:BINDING_B}));}
  await drain();assert.equal(h.reads('/api/image-source').length,0,hiddenKind);
  assert.equal(h.reads('/api/view/hierarchy').length,0,hiddenKind);h.assertReadOnly();
 }
});

test('analysis reads are single-flight and stable observations are reread at most once per 1.5 seconds',async()=>{
 const h=harness(),state=observation({history:[{request_id:A,status:'running',stage:'planning'}]});
 h.emit(state);h.select('analysis-request',A);const first=h.reads('/api/analysis/')[0];
 for(let index=0;index<20;index++)h.emit(state);
 assert.equal(h.reads('/api/analysis/').length,1,'Pending fetch must remain single-flight');
 h.reply(first,analysis('first'));await drain();
 h.advance(1499);for(let index=0;index<20;index++)h.emit(state);
 assert.equal(h.reads('/api/analysis/').length,1);
 h.advance(1);h.emit(state);assert.equal(h.reads('/api/analysis/').length,2);
 h.advance(5000);for(let index=0;index<20;index++)h.emit(state);
 assert.equal(h.reads('/api/analysis/').length,2,'Slow responses cannot start a fetch chain');
 h.reply(h.reads('/api/analysis/')[1],analysis('second'));await drain();h.assertReadOnly();
});

test('a delayed selected-request result cannot overwrite a newly selected request timeline',async()=>{
 const h=harness(),state=observation({history:[{request_id:A,status:'running'},{request_id:B,status:'running'}]});
 h.emit(state);h.select('analysis-request',A);const old=h.reads('/api/analysis/')[0];
 h.select('analysis-request',B);h.reply(old,analysis('old-request'));await drain();
 assert.doesNotMatch(content(h.element('analysis-events')),/old-request/);
 h.emit(state);h.reply(h.reads('/api/analysis/')[1],analysis('current-request'));await drain();
 assert.match(content(h.element('analysis-events')),/current-request/);h.assertReadOnly();
});

test('an image discarded while hidden is fetched again on return even without a new source sample',async()=>{
 const h=harness(),state=observation({sensor:image()});h.emit(state);
 const first=h.reads('/api/image-source/frame')[0];h.visibility(true);
 h.reply(first,{frame:'discarded-hidden'});await drain();assert.equal(h.created.length,0);
 h.visibility(false);await drain();
 assert.equal(h.reads('/api/image-source/frame').length,2,'Returning must retry the same selected frame after its old response was discarded');
 h.reply(h.reads('/api/image-source/frame')[1],{frame:'restored'});await drain();
 assert.equal(h.element('sensor-image').hidden,false);assert.equal(h.created[0].blob.frame,'restored');h.assertReadOnly();
});

test('reconnecting after an in-flight image was discarded can restore the same selected image',async()=>{
 const h=harness(),state=observation({sensor:image()});h.emit(state);
 const first=h.reads('/api/image-source/frame')[0];h.lost();
 h.reply(first,{frame:'discarded-disconnect'});await drain();assert.equal(h.created.length,0);
 h.emit(state);await drain();
 assert.equal(h.reads('/api/image-source/frame').length,2,'Same sequence must be retryable after connection invalidated its in-flight read');
 h.reply(h.reads('/api/image-source/frame')[1],{frame:'restored'});await drain();
 assert.equal(h.element('sensor-image').hidden,false);h.assertReadOnly();
});

test('changing or clearing the binding immediately removes old object selection and details',async()=>{
 for(const nextBinding of [BINDING_B,null]){
  const h=harness();h.emit(observation({binding:BINDING_A}));h.reply(h.reads('/api/view/hierarchy')[0],hierarchy('old-body','old'));
  await drain();h.select('object-select','old');assert.match(content(h.element('object-details')),/old-body/);
  h.emit(observation({binding:nextBinding}));
  assert.equal(h.element('object-select').children.some(child=>child.value==='old'),false,'Old options must be cleared before another response');
  assert.equal(h.element('object-details').children.length,0,'Old object details must not describe a new or unavailable binding');h.assertReadOnly();
 }
});
