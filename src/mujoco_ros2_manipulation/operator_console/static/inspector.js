'use strict';
// Read-only inspection. No command/approval/cancel endpoint is called here.
function sensorFrameKey(snapshot, selection='live') {
 const frame=selection==='live'?snapshot?.latest:snapshot?.history?.find(item=>String(item.sequence)===selection);
 if(!frame||frame.generation!==snapshot?.generation||frame.image_available===false||!Number.isSafeInteger(frame.sequence)||frame.sequence<1||!/^[0-9a-f]{32}$/.test(snapshot?.generation||''))return null;
 return {generation:snapshot.generation,sequence:frame.sequence,metadata:frame};
}
function observationTime(value, suffix=' s') {return typeof value==='number'&&Number.isFinite(value)?value.toFixed(3)+suffix:'미확인';}
(()=>{
 const get=id=>document.getElementById(id),panel=get('analysis');if(!panel)return;
 let imageDetails=null,imageDetailsAt=0,imageDetailsSerial=0,imageDetailsPending=false;
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
 function renderObject(){
  const list=hierarchy?.bodies||hierarchy?.nodes||[],item=list.find(value=>String(value.id??value.body_id??value.stable_id)===objects.value),detail=get('object-details');detail.replaceChildren();if(!item)return;
  for(const [label,value] of [['body',item.name||item.body_name||'이름 없음'],['부모',item.parent_id==null?'없음 (world)':(list.find(parent=>parent.id===item.parent_id)?.name||'body '+item.parent_body_id)],['로봇',item.instance_id??item.robot_instance_id??'환경/미분류'],['형상',JSON.stringify((hierarchy?.geoms||[]).filter(geom=>geom.body_id===item.body_id).map(geom=>geom.geom_id))],['공구',item.tool_role??'unknown']])field(detail,label,String(value));
 }
 async function renderHierarchy(){
  const binding=state?.view?.binding_id;
  if(binding!==hierarchyBinding){hierarchyBinding=binding;hierarchy=null;++hierarchySerial;objects.replaceChildren();get('object-details').replaceChildren();get('object-state').textContent=binding?'현재 모델의 객체 정보를 확인하고 있습니다.':'현재 표시 모델이 없습니다.';}
  if(!binding||panel.hidden||document.hidden||hierarchy||hierarchyPendingBinding===binding)return;
  if(state.view?.hierarchy_available!==true){get('object-state').textContent='현재 표시 공급원이 객체 계층을 준비하지 않았습니다.';return;}
  hierarchyPendingBinding=binding;const serial=++hierarchySerial;
  try{const data=await read('/api/view/hierarchy?binding_id='+encodeURIComponent(binding));if(serial!==hierarchySerial||state?.view?.binding_id!==binding)return;hierarchy=data;
   const list=data?.bodies||data?.nodes||[];options(objects,[['','객체 선택'],...list.map(item=>[String(item.id??item.body_id??item.stable_id),(item.robot_instance_id?item.robot_instance_id+' · ':'')+(item.name||item.body_name||'body '+item.id)])],'');
   get('object-state').textContent=data?'현재 표시 모델의 body·geom 연결입니다.':'현재 표시 공급원은 객체 계층을 제공하지 않습니다.';renderObject();
  }catch(error){if(serial===hierarchySerial){hierarchyBinding=null;get('object-state').textContent=error.message;}}finally{if(hierarchyPendingBinding===binding)hierarchyPendingBinding=null;}
 }
 function render(){if(!state)return;renderSensor();const selected=requests.value;options(requests,[['','요청을 선택하세요'],...(state.history||[]).map(item=>[item.request_id,item.request_id.slice(0,8)+' · '+item.status+' · '+(item.mode||'automatic')])],selected);if(!panel.hidden&&!document.hidden){renderRequest();renderHierarchy();}}
 frames.addEventListener('change',()=>{clearImage();renderSensor();});requests.addEventListener('change',()=>renderRequest(true));objects.addEventListener('change',renderObject);
 window.addEventListener('console-observation',event=>{state=event.detail;render();});
 window.addEventListener('console-observation-lost',()=>{get('sensor-state').textContent='상태 조회 중단 · 마지막 수신 자료';++imageSerial;frameToken=null;get('analysis-state').textContent='상태 조회 중단 · 마지막 조회 자료';});
 document.querySelectorAll('[data-view="analysis"]').forEach(button=>button.addEventListener('click',()=>setTimeout(render,0)));
 document.addEventListener('visibilitychange',()=>{if(document.hidden){++imageSerial;frameToken=null;}else render();});
})();
