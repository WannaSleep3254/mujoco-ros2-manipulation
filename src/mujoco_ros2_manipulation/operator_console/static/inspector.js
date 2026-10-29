'use strict';
// Local observation and recording. No command/approval/cancel endpoint is called here.
function sensorFrameKey(snapshot, selection='live') {
 const frame=selection==='live'?snapshot?.latest:snapshot?.history?.find(item=>String(item.sequence)===selection);
 if(!frame||frame.generation!==snapshot?.generation||frame.image_available===false||!Number.isSafeInteger(frame.sequence)||frame.sequence<1||!/^[0-9a-f]{32}$/.test(snapshot?.generation||''))return null;
 return {generation:snapshot.generation,sequence:frame.sequence,metadata:frame};
}
function observationTime(value, suffix=' s') {return typeof value==='number'&&Number.isFinite(value)?value.toFixed(3)+suffix:'미확인';}
(()=>{
 const get=id=>document.getElementById(id),panel=get('analysis');if(!panel)return;
 let imageDetails=null,imageDetailsAt=0,imageDetailsSerial=0,imageDetailsPending=false;
 let observed=false,selectedGeom=null,geometrySelection=null,treeButtons=[],treeBodies=new Map(),treeGeoms=new Map();
 let state=null,imageGeneration=null,frameURL=null,frameToken=null,imageSerial=0,analysisSerial=0,hierarchySerial=0,hierarchyBinding=null,hierarchy=null,requestKey=null,analysisReadAt=0,analysisPending=false,hierarchyPendingBinding=null;
 const requests=get('analysis-request'),frames=get('sensor-frame-select'),objects=get('object-select');
 const text=(tag,value)=>{const node=document.createElement(tag);node.textContent=value;return node;};
 function field(parent,label,value){const item=text('div','');item.className='detail-field';item.append(text('dt',label),text('dd',value));parent.append(item);}
 function clearImage(){++imageSerial;frameToken=null;get('sensor-image').hidden=true;get('sensor-image').removeAttribute('src');if(frameURL){URL.revokeObjectURL(frameURL);frameURL=null;}}
 async function read(path){const controller=new AbortController(),timer=setTimeout(()=>controller.abort(),3000);try{const reply=await fetch(path,{signal:controller.signal});if(reply.status===204)return null;if(!reply.ok)throw Error(reply.status===410?'이미지 보존 범위 밖입니다.':'자료 조회 실패 ('+reply.status+')');return await reply.json();}finally{clearTimeout(timer);}}
 function options(select,items,value){select.replaceChildren(...items.map(([key,label])=>{const node=text('option',label);node.value=key;return node;}));select.value=items.some(item=>item[0]===value)?value:items[0][0];}
 function renderSensor(){
  const compact=state?.image_source;
  const source=imageDetails&&compact&&imageDetails.generation===compact.generation?{...imageDetails,...compact,history:imageDetails.history}:compact;
  if(source&&!panel.hidden&&!document.hidden&&!imageDetailsPending&&Date.now()-imageDetailsAt>1000){
   imageDetailsPending=true;const serial=++imageDetailsSerial,expected=source.generation;
   read('/api/image-source').then(value=>{if(serial===imageDetailsSerial&&state?.image_source?.generation===expected&&value?.generation===expected){imageDetails=value;imageDetailsAt=Date.now();renderSensor();}}).catch(()=>{}).finally(()=>{imageDetailsPending=false;});
  }
  if(!source){get('sensor-state').textContent='등록한 이미지 공급원이 없습니다.';clearImage();return;}
  if(source.generation!==imageGeneration){imageGeneration=source.generation;frames.value='live';clearImage();}
  const names={waiting_source:'공급원 연결 대기',waiting_frame:'첫 이미지 대기',encoding:'이미지 변환 중',invalid:'시계·입력 확인 필요',ready:'수신 중',live:'수신 중',fresh:'수신 중',waiting:'첫 이미지 대기',disconnected:'공급원 연결 없음',ambiguous:'공급원이 둘 이상입니다',stale:'수신 지연',clock_reversed:'시계 역행',closed:'수신 종료',error:'이미지 변환 오류'};
  get('sensor-state').textContent=(names[source.status]||source.status||'상태 미확인')+' · '+(source.topic||source.source_id||'');
  const selected=frames.value||'live',history=(source.history||[]).filter(item=>item.image_available&&item.generation===source.generation).slice(-64).reverse();
  // Preserve an evicted selection visibly; never substitute today's live frame.
  const items=[['live','최신 이미지'],...history.map(item=>[String(item.sequence),'수신 '+item.sequence+' · 원천 '+observationTime(item.stamp_sec??item.source_stamp_sec)])];
  if(selected!=='live'&&!items.some(item=>item[0]===selected))items.push([selected,'수신 '+selected+' · 보존 범위 밖']);
  options(frames,items,selected);
  const key=sensorFrameKey(source,frames.value),meta=get('sensor-metadata');meta.replaceChildren();
  field(meta,'수신 세대',source.generation||'미확인');
  field(meta,'수신 나이',observationTime(source.latest_received?.age_sec??source.age_sec??source.source_age_sec));
  field(meta,'시계 영역',source.clock_domain||'unknown');
  if(key){field(meta,'수신 순번',String(key.sequence));field(meta,'원천 이미지 시각',observationTime(key.metadata.stamp_sec??key.metadata.source_stamp_sec));}
  get('sensor-frame-note').textContent=frames.value==='live'?'마지막 수신 이미지를 표시합니다. 상태가 지연·단절이면 실시간 이미지가 아닙니다.':'보존한 이미지입니다. 이 시점을 선택해도 로봇이나 3D 자세를 재생하지 않습니다.';
  if(!key){clearImage();if(frames.value!=='live')get('sensor-frame-note').textContent='이미지 보존 범위 밖입니다.';return;}
  if(panel.hidden||document.hidden){++imageSerial;frameToken=null;return;}
  const token=key.generation+':'+key.sequence;if(token===frameToken)return;
  frameToken=token;const serial=++imageSerial,controller=new AbortController(),timer=setTimeout(()=>controller.abort(),3000);
  fetch('/api/image-source/frame?generation='+key.generation+'&sequence='+key.sequence,{signal:controller.signal}).then(async reply=>{if(!reply.ok)throw Error('이미지 보존 범위 밖이거나 공급원이 바뀌었습니다.');return reply.blob();}).then(blob=>{
   if(serial!==imageSerial||state?.image_source?.generation!==key.generation||document.hidden||panel.hidden)return;
   const url=URL.createObjectURL(blob);if(frameURL)URL.revokeObjectURL(frameURL);frameURL=url;get('sensor-image').src=url;get('sensor-image').hidden=false;
  }).catch(error=>{if(serial===imageSerial){clearImage();get('sensor-frame-note').textContent=error.message;}}).finally(()=>clearTimeout(timer));
 }
 async function renderRequest(force=false){
  const identifier=requests.value;if(!identifier){get('analysis-events').replaceChildren();return;}
  const record=(state?.history||[]).find(item=>item.request_id===identifier),key=identifier+':'+(record?.status||'')+':'+(record?.stage||'');
  if(analysisPending||!force&&key===requestKey&&Date.now()-analysisReadAt<1500)return;requestKey=key;analysisReadAt=Date.now();analysisPending=true;const serial=++analysisSerial;
  try{const data=await read('/api/analysis/'+identifier);if(serial!==analysisSerial||requests.value!==identifier)return;
   const events=get('analysis-events');events.replaceChildren();
   for(const item of data.timeline?.items||[])events.append(text('li',observationTime(item.wall_sec)+' 경과 · ROS '+observationTime(item.simulation_sec)+' · '+item.name+(item.arm?' · '+item.arm:'')));
   get('analysis-state').textContent=(data.phase||'unknown')+' · '+(data.status||'unknown')+' · 경과 시간과 ROS 시각은 서로 빼서 지연으로 해석하지 않습니다.';
   const sources=get('analysis-sources');sources.replaceChildren();
   for(const [name,value] of Object.entries(data.sources||{}))if(value)field(sources,name,(value.clock_domain||'unknown')+' · 원천 '+observationTime(value.stamp_sec)+' · 수신 나이 '+observationTime(value.age_sec));
  }catch(error){if(serial===analysisSerial)get('analysis-state').textContent=error.message;}finally{analysisPending=false;}
 }
 function bodyKey(item){return String(item.id??item.body_id??item.stable_id);}
 function bodyList(){return hierarchy?.bodies||hierarchy?.nodes||[];}
 function bodyName(item){return item?(item.name||item.body_name||('body '+item.body_id)):'';}
 function markObjectSelection(){
  for(const [key,row] of treeBodies)row.button.setAttribute('aria-pressed',String(selectedGeom===null&&key===objects.value));
  for(const [key,row] of treeGeoms)row.button.setAttribute('aria-pressed',String(key===selectedGeom));
 }
 function renderObject(){
  const list=bodyList(),item=list.find(value=>bodyKey(value)===objects.value),detail=get('object-details');detail.replaceChildren();markObjectSelection();if(!item)return;
  const geom=(hierarchy?.geoms||[]).find(value=>value.geom_id===selectedGeom);
  const parent=list.find(value=>value.id===item.parent_id||value.body_id===item.parent_body_id);
  const fields=[['선택',geom?'geom '+geom.geom_id+' · '+(geom.name||'이름 없음'):'body '+item.body_id],
   ['body',bodyName(item)],['부모',item.parent_id==null&&item.parent_body_id==null?'없음 (world)':bodyName(parent)||'body '+item.parent_body_id],
   ['등록된 로봇',item.instance_id??item.robot_instance_id??'미분류'],
   ['연결 형상',(hierarchy?.geoms||[]).filter(value=>value.body_id===item.body_id).map(value=>'#'+value.geom_id).join(', ')||'없음'],
   ['공구 역할',item.tool_role==='unknown'||!item.tool_role?'미지정':item.tool_role]];
  for(const [label,value] of fields)field(detail,label,String(value));
 }
 function treeAvailable(){return observed&&hierarchy&&hierarchyBinding===state?.view?.binding_id;}
 function selectObject(item,geom=null,fromViewer=false){
  if(!treeAvailable())return;
  objects.value=bodyKey(item);selectedGeom=geom?.geom_id??null;renderObject();
  let current=treeBodies.get(bodyKey(item));const visited=new Set();
  while(current&&!visited.has(current)){visited.add(current);if(current.details)current.details.open=true;current=treeBodies.get(current.parent);}
  if(!fromViewer)publishGeometrySelection();
 }
 function publishGeometrySelection(){
  geometrySelection={binding_id:hierarchyBinding,scene_sha256:hierarchy.scene_sha256,geom_id:selectedGeom};
  window.dispatchEvent(new CustomEvent('console-geometry-select',{detail:geometrySelection}));
 }
 function applyGeometrySelection(selection){
  if(!treeAvailable()||selection?.binding_id!==hierarchyBinding||selection.scene_sha256!==hierarchy.scene_sha256)return;
  if(selection.geom_id===null){selectedGeom=null;renderObject();return;}
  const geom=(hierarchy.geoms||[]).find(item=>item.geom_id===selection.geom_id),body=geom&&bodyList().find(item=>item.body_id===geom.body_id);if(body)selectObject(body,geom,true);
 }
 function renderObjectTree(){
  const root=get('object-tree');treeButtons=[];treeBodies=new Map();treeGeoms=new Map();if(!root)return;root.replaceChildren();
  const list=bodyList(),byId=new Map(list.map(item=>[bodyKey(item),item])),children=new Map(),roots=[];
  for(const item of list){const parent=item.parent_id!=null?String(item.parent_id):list.find(value=>value.body_id===item.parent_body_id)?.id;
   if(parent!=null&&byId.has(String(parent))&&String(parent)!==bodyKey(item)){const key=String(parent);if(!children.has(key))children.set(key,[]);children.get(key).push(item);}else roots.push(item);}
  const geomsByBody=new Map();for(const geom of hierarchy?.geoms||[]){if(!geomsByBody.has(geom.body_id))geomsByBody.set(geom.body_id,[]);geomsByBody.get(geom.body_id).push(geom);}
  const visited=new Set(),binding=hierarchyBinding;
  const pick=(label,callback,cls='')=>{const button=text('button',label);button.type='button';button.className='object-pick '+cls;button.disabled=!observed;button.setAttribute('aria-pressed','false');button.addEventListener('click',()=>{if(treeAvailable()&&binding===hierarchyBinding)callback();});treeButtons.push(button);return button;};
  function branch(items,depth=0){const ul=text('ul','');ul.className='object-tree-list';
   for(const item of items){const key=bodyKey(item);if(visited.has(key))continue;visited.add(key);
    const li=text('li','');li.className='object-body';const descendants=children.get(key)||[],geoms=geomsByBody.get(item.body_id)||[];
    const button=pick(bodyName(item),()=>selectObject(item));button.title='body '+item.body_id+' 속성 보기';let details=null,container=li;
    if(descendants.length||geoms.length){details=text('details','');details.open=depth<2;const summary=text('summary',bodyName(item));summary.title='body '+item.body_id+' 펼치기·접기';details.append(summary);li.append(details);container=details;button.textContent='body '+item.body_id+' 속성';}
    container.append(button);treeBodies.set(key,{button,details,parent:item.parent_id==null?null:String(item.parent_id)});
    if(geoms.length){const geomList=text('ul','');geomList.className='object-tree-list object-geoms';for(const geom of geoms){const row=text('li','');const control=pick((geom.name||'형상')+' · #'+geom.geom_id,()=>selectObject(item,geom),'object-geom');control.title='3D 형상 #'+geom.geom_id+' 선택';row.append(control);geomList.append(row);treeGeoms.set(geom.geom_id,{button:control,body:item});}container.append(geomList);}
    if(descendants.length)container.append(branch(descendants,depth+1));ul.append(li);
   }return ul;}
  root.append(branch(roots));
  // A malformed optional hierarchy must not recurse forever or hide a cycle.
  const disconnected=list.filter(item=>!visited.has(bodyKey(item)));if(disconnected.length)root.append(branch(disconnected));
  markObjectSelection();
 }
 async function renderHierarchy(){
  const binding=state?.view?.binding_id;
  if(binding!==hierarchyBinding){if(geometrySelection?.binding_id!==binding)geometrySelection=null;hierarchyBinding=binding;hierarchy=null;selectedGeom=null;++hierarchySerial;objects.replaceChildren();get('object-tree')?.replaceChildren();treeButtons=[];treeBodies.clear();treeGeoms.clear();get('object-details').replaceChildren();get('object-state').textContent=binding?'현재 모델의 객체 정보를 확인하고 있습니다.':'현재 표시 모델이 없습니다.';}
  const visible=!panel.hidden||get('operate')?.hidden===false;
  objects.disabled=!observed||!hierarchy;for(const button of treeButtons)button.disabled=!observed;
  if(!binding||!visible||document.hidden||!observed||hierarchy||hierarchyPendingBinding===binding)return;
  if(state.view?.hierarchy_available!==true){get('object-state').textContent='현재 표시 공급원이 객체 계층을 준비하지 않았습니다.';return;}
  hierarchyPendingBinding=binding;const serial=++hierarchySerial;
  try{const data=await read('/api/view/hierarchy?binding_id='+encodeURIComponent(binding));if(serial!==hierarchySerial||!observed||state?.view?.binding_id!==binding)return;
   const expectedScene=state.view?.manifest_descriptor?.scene_sha256;
   if(data&&expectedScene&&data.scene_sha256!==expectedScene)throw Error('표시 모델과 객체 계층의 버전이 다릅니다. 다시 확인합니다.');
   hierarchy=data;const list=bodyList();options(objects,[['','객체 선택'],...list.map(item=>[bodyKey(item),(item.robot_instance_id?item.robot_instance_id+' · ':'')+bodyName(item)])],'');objects.disabled=!hierarchy;
   get('object-state').textContent=data?'body를 펼쳐 속성을 확인하거나 형상을 선택하세요.':'현재 표시 공급원은 객체 계층을 제공하지 않습니다.';renderObjectTree();renderObject();applyGeometrySelection(geometrySelection);
  }catch(error){if(serial===hierarchySerial){hierarchyBinding=null;get('object-state').textContent=error.message;}}finally{if(hierarchyPendingBinding===binding)hierarchyPendingBinding=null;}
 }
 function render(){if(!state)return;renderSensor();const selected=requests.value;options(requests,[['','요청을 선택하세요'],...(state.history||[]).map(item=>[item.request_id,item.request_id.slice(0,8)+' · '+item.status+' · '+(item.mode||'automatic')])],selected);if(!panel.hidden&&!document.hidden)renderRequest();renderHierarchy();}
 frames.addEventListener('change',()=>{clearImage();renderSensor();});requests.addEventListener('change',()=>renderRequest(true));objects.addEventListener('change',()=>{if(!treeAvailable())return;const item=bodyList().find(value=>bodyKey(value)===objects.value);if(item)selectObject(item);else{selectedGeom=null;renderObject();publishGeometrySelection();}});
 window.addEventListener('console-observation',event=>{const returning=!observed;observed=true;state=event.detail;if(returning&&hierarchy)get('object-state').textContent='body를 펼쳐 속성을 확인하거나 형상을 선택하세요.';render();});
 window.addEventListener('console-geometry-selection',event=>{
  if(!observed||event.detail?.binding_id!==state?.view?.binding_id)return;geometrySelection=event.detail;applyGeometrySelection(geometrySelection);
 });
 window.addEventListener('console-observation-lost',()=>{observed=false;geometrySelection=null;++hierarchySerial;objects.disabled=true;for(const button of treeButtons)button.disabled=true;get('object-state').textContent='상태 조회 중단 · 마지막 객체 정보';get('sensor-state').textContent='상태 조회 중단 · 마지막 수신 자료';++imageSerial;frameToken=null;get('analysis-state').textContent='상태 조회 중단 · 마지막 조회 자료';});
 document.querySelectorAll('[data-view="analysis"], [data-view="operate"]').forEach(button=>button.addEventListener('click',()=>setTimeout(render,0)));
 document.addEventListener('visibilitychange',()=>{if(document.hidden){++imageSerial;frameToken=null;}else render();});
})();


// Capture controls preserve observations only. Playback links open immutable
// recordings explicitly; selecting a sensor frame above remains independent.
(()=>{
 const get=id=>document.getElementById(id),panel=get('analysis'),start=get('sync-capture-start'),stop=get('sync-capture-stop');
 if(!panel||!start||!stop)return;
 let token=null,capture=null,pending=false,changing=false,closed=false,readAt=0,serial=0;
 const identifier=value=>typeof value==='string'&&/^[0-9a-f]{32}$/.test(value);
 const reason=value=>({user_stop:'요청에 따라 종료',source_changed:'실행 또는 센서 공급원 변경',source_unavailable:'관측 연결 종료',clock_reversed:'원천 시계 역행',duration_limit:'기록 시간 상한 도달',byte_or_sample_limit:'기록 용량 또는 표본 상한 도달',insufficient_or_invalid_capture:'필수 표본이나 기록 식별을 확인하지 못했습니다'}[value]||value);
 const allowed=()=>!closed&&!document.hidden&&!panel.hidden;
 function enable(){const active=capture?.status==='recording',busy=changing||['preparing','saving','stopping'].includes(capture?.status);
  start.disabled=closed||busy||!token||active||capture?.can_start===false;
  stop.disabled=closed||busy||!token||!active||!identifier(capture?.id);
 }
 function render(data){
  capture=data.capture;token=data.token;
  const labels={idle:'기록 대기',recording:'동기 기록 중',preparing:'기록 준비 중',saving:'기록 저장 중',stopping:'기록 종료 중',completed:'기록 저장 완료',failed:'기록을 저장하지 못했습니다',unavailable:'기록 입력을 확인하세요'};
  const counts=Number.isSafeInteger(capture?.pose_count)&&Number.isSafeInteger(capture?.image_count)?' · 3D '+capture.pose_count+'개 · 이미지 '+capture.image_count+'개':'';
  get('sync-capture-state').textContent=(labels[capture?.status]||'기록 상태 미확인')+counts+(capture?.reason?' · '+reason(capture.reason):'')+(capture?.availability_reason?' · '+capture.availability_reason:'');
  const list=get('sync-capture-list');list.replaceChildren();
  for(const item of (data.recordings||[]).slice(-8).reverse()){
   if(!identifier(item.id))continue;
   const row=document.createElement('li'),link=document.createElement('a'),note=document.createElement('p');
   link.href='/synchronized?id='+item.id;link.target='_blank';link.rel='noopener';link.textContent=item.label||'동기 기록';
   note.className='help';note.textContent=(Number.isFinite(item.duration_sec)?item.duration_sec.toFixed(2)+' s':'길이 미확인')+(item.ended_reason?' · '+reason(item.ended_reason):'');
   row.append(link,note);list.append(row);
  }
  get('sync-capture-empty').hidden=list.childElementCount>0;enable();
 }
 async function refresh(force=false){
  if(!allowed()||pending||!force&&Date.now()-readAt<1000)return;
  pending=true;readAt=Date.now();const epoch=++serial,controller=new AbortController(),timer=setTimeout(()=>controller.abort(),4000);
  try{const reply=await fetch('/api/sync-recordings',{cache:'no-store',signal:controller.signal});if(!reply.ok)throw Error('동기 기록 상태를 확인하지 못했습니다.');
   const data=await reply.json();if(!closed&&epoch===serial)render(data);
  }catch(error){if(!closed&&epoch===serial){token=null;get('sync-capture-state').textContent='동기 기록 연결을 확인할 수 없습니다.';enable();}}
  finally{clearTimeout(timer);pending=false;}
 }
 async function change(action){
  if(closed||changing||!token)return;
  if(action==='stop'&&!identifier(capture?.id))return;
  changing=true;enable();get('sync-capture-error').textContent='';
  const controller=new AbortController(),timer=setTimeout(()=>controller.abort(),8000);
  try{const payload=action==='start'?{}:{id:capture.id};
   const reply=await fetch('/api/sync-recordings/'+action,{method:'POST',headers:{'Content-Type':'application/json','X-Operator-Token':token},body:JSON.stringify(payload),signal:controller.signal});
   const data=await reply.json();if(!reply.ok)throw Error(data.error||'기록 조작을 적용하지 못했습니다.');
  }catch(error){if(!closed)get('sync-capture-error').textContent=error.name==='AbortError'?'기록 조작 응답이 지연되었습니다. 상태를 확인한 후 다시 조작하세요.':error.message;}
  finally{clearTimeout(timer);changing=false;enable();void refresh(true);}
 }
 start.addEventListener('click',()=>void change('start'));stop.addEventListener('click',()=>void change('stop'));
 window.addEventListener('console-observation',()=>void refresh());
 document.querySelectorAll('[data-view="analysis"]').forEach(button=>button.addEventListener('click',()=>setTimeout(()=>void refresh(true),0)));
 document.addEventListener('visibilitychange',()=>{if(!document.hidden)void refresh(true);});
 window.addEventListener('pagehide',()=>{closed=true;++serial;token=null;enable();});
 void refresh(true);
})();
