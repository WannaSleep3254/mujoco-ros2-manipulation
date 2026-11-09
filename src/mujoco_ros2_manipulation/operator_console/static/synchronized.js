import {LocalSceneView} from './viewer.js';

const READ_TIMEOUT_MS=4000,MAX_JPEG_BYTES=512*1024,MAX_TASK_EVENTS=128;
const CONTROL_IDS=['sync-play','sync-pause','sync-restart','sync-previous','sync-next','sync-seek','sync-speed'];
const finite=value=>typeof value==='number'&&Number.isFinite(value)&&value>=0;
const stamp=value=>finite(value)?value.toFixed(3)+' s':'—';
const count=value=>Number.isSafeInteger(value)&&value>=0?String(value):'미확인';
const ns=value=>typeof value==='string'&&/^\d{1,20}$/.test(value);
const eventStage=value=>({starting:'시작 준비',preflight:'사전 확인',unused_domain:'통신 영역 확인',
  initial_settle:'초기 자세 안정화',approach:'접근',close:'그리퍼 닫기',grasp:'집기',verification_lift:'들어 올려 집기 확인',
  transport:'운반',place:'놓을 위치로 이동',open:'그리퍼 열기',settle:'놓은 뒤 안정화',complete:'작업 종료',release:'놓기',
  retreat:'후퇴',running:'진행 중',succeeded:'성공',completed:'완료',cancelled:'취소',failed:'실패',cleanup:'종료 처리'}[value]||value||'작업');
const associationReason=value=>({before_native:'첫 3D 관측 전',pre_native:'첫 3D 관측 전',native_unavailable:'3D 관측 없음',
  native_not_live:'3D 공급원 종료 후',native_cleanup_or_hold:'정지·종료 처리 중',
  native_not_fresh_and_progressing:'신선하게 진행하는 3D 관측 없음',
  insufficient_paired_native_and_image_observations:'함께 재생할 3D 자세와 이미지가 충분히 수집되지 않음',
  native_stale:'3D 관측 지연',native_held:'3D 시계 정지',after_exit:'3D 공급원 종료 후',
  outside_media:'보존한 재생 구간 밖',outside_retained_media:'보존한 재생 구간 밖',pose_not_retained:'해당 3D 표본이 보존되지 않음'}[value]||value||'연결 시각 없음');

/** A descriptor selects both tracks once. No later "latest" lookup may replace either. */
export class SynchronizedPlayer {
  constructor({id,viewer,element,fetcher=(...args)=>fetch(...args),imageLoader=loadImage,hidden=()=>document.hidden,
               timers={set:(...args)=>setTimeout(...args),clear:id=>clearTimeout(id)},createElement=tag=>document.createElement(tag)}){
    this.id=id;this.prefix='/api/sync-recordings/'+id;this.viewer=viewer;this.element=element;
    this.fetcher=fetcher;this.imageLoader=imageLoader;this.hidden=hidden;this.timers=timers;this.createElement=createElement;
    this.token=null;this.view=null;this.binding=null;this.loaded=null;this.revision=0;
    this.inflight=false;this.busy=false;this.closed=false;this.timer=null;this.pending=false;
    this.readController=null;this.mediaController=null;this.image=null;this.displayKey=null;this.controlError=null;
    this.scrubbing=false;this.eventRecord=null;this.eventJSON=null;this.eventRows=new Map();
    this.enable();
  }
  node(id){return this.element(id);}
  error(message=''){this.node('sync-error').textContent=message;this.node('sync-error').hidden=!message;}
  mediaReady(){return this.view?.media_state!=='unavailable';}
  enable(){
    const disabled=this.closed||this.busy||!this.token||!this.mediaReady();
    for(const id of CONTROL_IDS)this.node(id).disabled=disabled;
    for(const {button} of this.eventRows.values())if(button)button.disabled=disabled;
  }
  resetEvents(){
    this.eventRecord=null;this.eventJSON=null;this.eventRows.clear();
    this.node('sync-events').replaceChildren();this.node('sync-untimed-events').replaceChildren();
    this.node('sync-events-panel').hidden=true;
  }
  clearEventSelection(message='선택 시각을 확인하고 있습니다.'){
    for(const {row} of this.eventRows.values()){row.dataset.syncEventCurrent='false';row.setAttribute('aria-current','false');}
    this.node('sync-events-state').textContent=message;
  }
  acceptEvents(view){
    if(view.recording_version!==2){if(this.eventRecord!==null)this.resetEvents();this.node('sync-events-panel').hidden=true;return;}
    if(!Array.isArray(view.events)||view.events.length>MAX_TASK_EVENTS||!/^[0-9a-f]{64}$/.test(view.recording_sha256||''))throw Error('작업 이벤트의 기록 식별자를 확인할 수 없습니다.');
    const serialized=JSON.stringify(view.events);
    if(this.eventRecord===view.recording_sha256){
      if(this.eventJSON!==serialized)throw Error('보존한 작업 이벤트가 기록 안에서 바뀌었습니다.');
    }else{
      this.resetEvents();let previous=0;const ids=new Set();
      for(const event of view.events){
        if(!event||typeof event.event_id!=='string'||!event.event_id||event.event_id.length>160||ids.has(event.event_id)||
           !Number.isSafeInteger(event.event_sequence)||event.event_sequence<=previous||!ns(event.emitted_monotonic_ns)||
           typeof event.native_association?.available!=='boolean'||
           (event.time_sec!==null&&(!finite(event.time_sec)||!finite(view.duration_sec)||event.time_sec>view.duration_sec))||
           (event.native_association.available?!ns(event.native_association.stamp_ns):event.time_sec!==null))throw Error('작업 이벤트의 순서나 연결 시각을 확인할 수 없습니다.');
        ids.add(event.event_id);previous=event.event_sequence;
      }
      this.eventRecord=view.recording_sha256;this.eventJSON=serialized;
      const node=(tag,text,parent)=>{const value=this.createElement(tag);value.textContent=text;if(parent)parent.append(value);return value;};
      for(const event of view.events){
        const timed=finite(event.time_sec),list=this.node(timed?'sync-events':'sync-untimed-events');
        const row=node('li','',list);row.dataset.eventId=event.event_id;row.dataset.syncEventCurrent='false';row.setAttribute('aria-current','false');
        const label='#'+event.event_sequence+' · '+eventStage(event.stage)+(event.status?' · '+eventStage(event.status):'');
        let button=null;
        if(timed){
          button=node('button',stamp(event.time_sec)+' · '+label,row);button.type='button';
          button.addEventListener('click',()=>{if(!this.closed&&!this.hidden()&&this.eventRecord===view.recording_sha256)void this.control('seek',event.time_sec);});
        }else node('strong',label,row);
        const association=event.native_association;
        const native=association.available?'3D 표본 #'+association.sequence+' · 원천 '+association.stamp_ns+' ns':
          '3D 시각 연결 없음 · '+associationReason(association.reason);
        node('small','이벤트 발생 단조 시계 '+event.emitted_monotonic_ns+' ns\n수신 단조 시계 '+event.parent_received_monotonic_ns+' ns\n'+native,row).className='sync-event-source';
        if(!timed&&association.available)node('small',associationReason(event.media_seek_reason),row).className='sync-event-source';
        if(event.details&&Object.keys(event.details).length)node('p',JSON.stringify(event.details),row).className='sync-event-detail';
        this.eventRows.set(event.event_id,{row,button});
      }
      this.node('sync-untimed-panel').hidden=!view.events.some(event=>event.time_sec===null);
      this.node('sync-untimed-panel').open=view.media_state==='unavailable';
      this.clearEventSelection();
    }
    if(!Array.isArray(view.selection.event_ids)||view.selection.event_ids.some(id=>!this.eventRows.get(id)?.button))throw Error('선택 시각의 작업 이벤트를 확인할 수 없습니다.');
    this.node('sync-events-panel').hidden=false;
  }
  commitEvents(view){
    if(view.recording_version!==2)return;
    const selected=new Set(view.selection.event_ids);
    for(const [id,{row}] of this.eventRows){const current=selected.has(id);row.dataset.syncEventCurrent=String(current);row.setAttribute('aria-current',String(current));}
    this.node('sync-events-state').textContent=this.mediaReady()?
      '표시한 영상·3D 시각의 연결 이벤트 '+selected.size+'개':
      '작업 이벤트 '+view.events.length+'개 · 재생 시각 연결 없음';
  }
  schedule(delay=this.hidden()?1000:this.view?.playing?50:1000){
    if(this.closed)return;
    if(this.timer!==null){this.timers.clear(this.timer);this.timer=null;}
    if(this.inflight||this.busy){if(delay===0)this.pending=true;return;}
    this.timer=this.timers.set(()=>{this.timer=null;void this.update();},delay);
  }
  clearDisplays(reason='선택 시각의 기록을 준비하고 있습니다.'){
    this.image?.destroy();this.image=null;this.node('sync-image-frame').replaceChildren();
    for(const kind of ['pose','image']){this.node('sync-'+kind+'-empty').hidden=false;this.node('sync-'+kind+'-empty').textContent=reason;}
    this.node('sync-status').textContent=reason;this.node('sync-status').dataset.state='waiting';this.displayKey=null;this.clearEventSelection(reason);
  }
  acceptView(view){
    const version=view?.recording_version??1,unavailable=version===2&&view.media_state==='unavailable';
    const timingValid=unavailable?
      view.position_sec===null&&view.duration_sec===null&&view.selection?.cursor_ns===null&&view.selection?.time_sec===null&&
        view.selection?.pose===null&&view.selection?.image===null&&view.playing===false:
      finite(view?.position_sec)&&finite(view?.duration_sec)&&view.position_sec<=view.duration_sec&&/^\d{1,19}$/.test(view.selection?.cursor_ns||'');
    if(!view||![1,2].includes(version)||!view.selection||!/^[0-9a-f]{64}$/.test(view.binding_id||'')||!timingValid||
       version===2&&(!['ready','unavailable'].includes(view.media_state)||view.recording_kind!=='synchronized-grasp-recording-v2')){
      throw Error('기록 시각과 식별자를 확인할 수 없습니다.');
    }
    this.acceptEvents(view);this.view=view;
    if(this.binding!==view.binding_id){
      this.binding=view.binding_id;this.loaded=null;this.clearDisplays('선택 시각 전환 중');
      this.viewer.setExpectedBinding(this.binding,view.binding);
    }
    this.node('sync-source').textContent=view.source.label+' · 보존한 관측 기록';
    this.node('sync-seek').max=unavailable?'0':String(view.duration_sec);
    if(!this.scrubbing)this.node('sync-seek').value=unavailable?'0':String(view.position_sec);
    this.node('sync-time').textContent=unavailable?'재생할 영상·3D 기록 없음':stamp(view.position_sec)+' / '+stamp(view.duration_sec);
    this.node('sync-speed').value=String(view.speed);
    this.node('sync-playback-state').textContent=unavailable?'이벤트 기록':view.playing?'재생 중':view.position_sec>=view.duration_sec?'재생 완료':'일시 정지';
    const original=view.acquisition_identity||{},counters=view.observation_counters;
    const sourceDetails=version===2&&counters?[
      '3D 수집 방식 '+(view.pose_sample_semantics==='parent_accepted_latest_ipc'?'부모가 수용한 최신 IPC 관측':view.pose_sample_semantics||'미확인'),
      '부모 자세 큐 제외 '+count(counters.poses_dropped)+'개 · 원천 자세 순번 건너뜀 '+count(counters.poses_source_gaps)+'개',
      '이벤트 큐 제외 '+count(counters.events_dropped)+'개 · 영상 전송 누락 '+count(counters.image_transport_gaps)+'개',
      '이미지 수신 집계 '+JSON.stringify(counters.image_counters||{})]:[];
    this.node('sync-identity').textContent=['기록 '+view.recording_sha256,
      unavailable?'영상·3D 시각 없음 · '+associationReason(view.media_reason):version===2?
        '기록 공통 시각 '+view.selection.cursor_ns+' ns · 같은 실행의 소프트웨어 시각 연결':
        '원천 시계 '+(view.acquisition_clock_domain||'미확인')+' · 원천 시각 '+view.selection.cursor_ns+' ns',
      version===2?'집기 작업 '+(original.job_id||'미확인')+' · 작업 세대 '+(original.job_generation??'미확인'):
        '실행 '+(original.motion_session_id||'미확인')+' · 이미지 수신 세대 '+(original.image_generation||'미확인'),
      version===2?'3D 시각은 반올림한 MuJoCo 시간이며 영상은 원래 ROS 헤더의 정수 ns입니다.':'선택 규칙은 0.5초 이내 이전 표본이며 자세를 보간하지 않습니다.',
      ...sourceDetails,...(view.task_status?['원래 작업 상태 '+eventStage(view.task_status)]:[])].join('\n');
  }
  async readState(){
    const controller=new AbortController();this.readController=controller;
    const timer=this.timers.set(()=>controller.abort(new Error('기록 조회 시간이 초과되었습니다.')),READ_TIMEOUT_MS);
    const aborted=new Promise((_,reject)=>controller.signal.addEventListener('abort',()=>reject(controller.signal.reason||new Error('기록 조회가 종료되었습니다.')),{once:true}));
    try{return await Promise.race([aborted,(async()=>{
      const reply=await this.fetcher(this.prefix,{cache:'no-store',signal:controller.signal});
      if(!reply.ok)throw Error('기록을 불러올 수 없습니다. ('+reply.status+')');
      const value=await reply.json();if(controller.signal.aborted)throw controller.signal.reason;return value;
    })()]);}finally{this.timers.clear(timer);if(this.readController===controller)this.readController=null;}
  }
  async applySelection(view,epoch){
    const valid=()=>!this.closed&&!this.busy&&epoch===this.revision&&view.binding_id===this.binding&&!this.hidden();
    if(!valid())return;
    if(!this.mediaReady()){
      this.clearDisplays('영상·3D 기록이 없습니다. '+associationReason(view.media_reason));
      this.node('sync-pose-status').textContent='기록 없음';this.node('sync-image-status').textContent='기록 없음';
      this.node('sync-pose-time').textContent='3D 원천 시각 없음';this.node('sync-image-time').textContent='영상 원천 시각 없음';
      this.node('sync-displayed-time').textContent='재생 시각 없음';this.node('sync-status').dataset.state='partial';
      this.commitEvents(view);return;
    }
    const descriptor=view.manifest_descriptor,id=view.binding_id;
    if(this.loaded!==id){
      await this.viewer.loadManifest(this.prefix+'/manifest?binding_id='+id,{binding_id:id,binding:view.binding,
        sha256:descriptor.sha256,model_sha256:descriptor.model_sha256,scene_sha256:descriptor.scene_sha256});
      if(!valid())return;this.loaded=id;
    }
    const selected=view.selection,key=id+':'+selected.cursor_ns;
    if(key===this.displayKey)return;
    const hasPose=selected.pose_status==='ready'&&selected.pose!==null,hasImage=selected.image_status==='ready'&&selected.image!==null;
    if(hasPose&&selected.pose.binding_id!==id)throw Error('3D 기록의 선택 식별자가 다릅니다.');
    this.node('sync-status').textContent='선택한 두 기록을 준비하고 있습니다.';this.node('sync-status').dataset.state='waiting';
    const controller=new AbortController();this.mediaController=controller;
    let image=null;
    try{
      if(hasImage)image=await this.imageLoader(this.prefix+'/images/'+selected.image.jpeg_sha256+'.jpg',selected.image,{signal:controller.signal,fetcher:this.fetcher,timers:this.timers});
      if(!valid()||controller.signal.aborted)return;
      // Apply and reveal both panels in one synchronous turn after image decode.
      // A repeated pose is valid while a camera track advances at another rate.
      if(hasPose){
        this.viewer.updatePose(selected.pose);
        if(!this.viewer.hasPose||this.viewer.sequence<selected.pose.sequence)throw Error('선택한 3D 자세를 표시하지 못했습니다.');
      }
      this.image?.destroy();this.image=image;image=null;
      this.node('sync-image-frame').replaceChildren(...(this.image?[this.image.element]:[]));
      for(const [kind,ready] of [['pose',hasPose],['image',hasImage]]){
        this.node('sync-'+kind+'-empty').hidden=ready;
        this.node('sync-'+kind+'-empty').textContent='이 시각의 '+(kind==='pose'?'3D 자세':'이미지')+' 기록이 없습니다.';
        this.node('sync-'+kind+'-status').textContent=ready?'선택 기록 표시':'기록 없음';
        this.node('sync-'+kind+'-time').textContent=(ready?'표시 시각 ':'마지막 표본 시각 ')+stamp(selected[kind+'_time_sec'])+' · 선택 시각과 차이 '+stamp(selected[kind+'_delta_sec']);
      }
      this.node('sync-status').dataset.state=hasPose&&hasImage?'ready':'partial';
      this.node('sync-status').textContent=hasPose&&hasImage?'같은 선택 시각의 두 관측을 표시했습니다.':'일부 기록이 없습니다. 빠진 프레임을 보간하지 않습니다.';
      this.node('sync-displayed-time').textContent='표시된 선택 시각 '+stamp(selected.time_sec);
      this.displayKey=key;this.commitEvents(view);if(!this.controlError)this.error();
    }finally{image?.destroy();if(this.mediaController===controller)this.mediaController=null;}
  }
  async update(){
    if(this.closed)return;
    if(this.timer!==null){this.timers.clear(this.timer);this.timer=null;}
    if(this.inflight||this.busy){this.pending=true;return;}
    this.inflight=true;const epoch=this.revision;
    try{
      const data=await this.readState();if(this.closed||epoch!==this.revision||this.busy)return;
      this.token=data.token;this.acceptView(data.view);this.enable();
      const back=this.node('sync-back');if(back)back.hidden=data.console_available===false;
      if(!this.hidden())await this.applySelection(data.view,epoch);
    }catch(error){if(!this.closed&&epoch===this.revision){this.clearDisplays('기록 표시를 확인할 수 없습니다.');this.node('sync-status').dataset.state='error';this.error(error.message||'기록 연결을 확인하세요.');}}
    finally{this.inflight=false;if(!this.closed&&!this.busy){const immediate=this.pending&&!this.hidden();this.pending=false;this.schedule(immediate?0:undefined);}}
  }
  async control(action,value){
    if(this.closed||this.busy||!this.token||!this.mediaReady())return;
    this.busy=true;const epoch=++this.revision;
    this.scrubbing=false;
    if(this.timer!==null){this.timers.clear(this.timer);this.timer=null;}
    this.readController?.abort();this.mediaController?.abort();this.viewer.cancelPoseRead?.();
    this.clearDisplays('재생 위치를 적용하고 있습니다.');this.enable();
    const payload={action};if(action==='seek')payload.time_sec=value;if(action==='speed')payload.speed=value;
    const controller=new AbortController(),timer=this.timers.set(()=>controller.abort(),READ_TIMEOUT_MS);
    try{
      const reply=await this.fetcher(this.prefix+'/control',{method:'POST',headers:{'Content-Type':'application/json','X-Operator-Token':this.token},body:JSON.stringify(payload),signal:controller.signal});
      const view=await reply.json();if(this.closed||epoch!==this.revision)return;
      if(!reply.ok)throw Error(view.error||'재생 조작을 적용하지 못했습니다.');this.acceptView(view);this.controlError=null;this.error();
    }catch(error){if(!this.closed&&epoch===this.revision){this.controlError=error.name==='AbortError'?'재생 조작 응답이 지연되었습니다. 현재 위치를 다시 조회합니다.':error.message;this.error(this.controlError);}}
    finally{this.timers.clear(timer);this.busy=false;if(!this.closed){this.enable();this.schedule(this.hidden()?1000:0);}}
  }
  visibility(){
    ++this.revision;this.readController?.abort();this.mediaController?.abort();
    if(this.hidden()){this.pending=false;this.clearDisplays('숨겨진 탭의 표시 조회를 쉬고 있습니다.');}
    this.schedule(this.hidden()?1000:0);
  }
  destroy(){
    this.closed=true;++this.revision;this.readController?.abort();this.mediaController?.abort();
    if(this.timer!==null)this.timers.clear(this.timer);this.timer=null;
    this.image?.destroy();this.image=null;this.resetEvents();this.viewer.destroy();this.enable();
  }
}

/** Fetch, hash and decode an immutable JPEG before making it visible. */
export async function loadImage(url,descriptor,{signal,fetcher=(...args)=>fetch(...args),timers={set:(...args)=>setTimeout(...args),clear:id=>clearTimeout(id)}}){
  if(!/^[0-9a-f]{64}$/.test(descriptor?.jpeg_sha256||'')||!Number.isSafeInteger(descriptor.byte_length)||
     descriptor.byte_length<1||descriptor.byte_length>MAX_JPEG_BYTES||!Number.isSafeInteger(descriptor.width)||!Number.isSafeInteger(descriptor.height)||
     descriptor.width<1||descriptor.height<1||descriptor.width>640||descriptor.height>480)throw Error('이미지 기록의 크기와 식별자를 확인할 수 없습니다.');
  if(signal?.aborted)throw Error('이미지 기록 조회가 종료되었습니다.');
  const controller=new AbortController();let rejectAbort;
  const aborted=new Promise((_,reject)=>{rejectAbort=reject;});
  const cancel=()=>{controller.abort();rejectAbort(new Error('이미지 기록 조회가 종료되거나 지연되었습니다.'));};
  signal?.addEventListener('abort',cancel,{once:true});
  const timer=timers.set(cancel,READ_TIMEOUT_MS);let objectURL=null;
  try{
    if(signal?.aborted)cancel();
    return await Promise.race([aborted,(async()=>{
      const reply=await fetcher(url,{cache:'no-store',signal:controller.signal});
      if(!reply.ok||!(reply.headers.get('Content-Type')||'').startsWith('image/jpeg'))throw Error('선택한 이미지 기록을 가져오지 못했습니다.');
      const declared=Number(reply.headers.get('Content-Length'));
      if(!Number.isSafeInteger(declared)||declared!==descriptor.byte_length)throw Error('이미지 기록의 바이트 길이가 다릅니다.');
      const raw=await reply.arrayBuffer();if(raw.byteLength!==descriptor.byte_length)throw Error('이미지 기록이 잘렸습니다.');
      const hash=[...new Uint8Array(await crypto.subtle.digest('SHA-256',raw))].map(value=>value.toString(16).padStart(2,'0')).join('');
      if(hash!==descriptor.jpeg_sha256)throw Error('이미지 기록의 해시가 다릅니다.');
      if(controller.signal.aborted)throw Error('이미지 조회가 종료되었습니다.');
      objectURL=URL.createObjectURL(new Blob([raw],{type:'image/jpeg'}));
      const image=new Image();image.alt='선택한 기록 시각의 센서 영상';image.src=objectURL;
      await image.decode();
      if(controller.signal.aborted)throw Error('이미지 표시가 종료되었습니다.');
      if(image.naturalWidth!==descriptor.width||image.naturalHeight!==descriptor.height)throw Error('이미지 기록의 해상도가 다릅니다.');
      const readyURL=objectURL;objectURL=null;
      return {element:image,destroy:()=>URL.revokeObjectURL(readyURL)};
    })()]);
  }finally{timers.clear(timer);signal?.removeEventListener('abort',cancel);if(objectURL)URL.revokeObjectURL(objectURL);}
}

function bootstrap(){
  const id=new URLSearchParams(location.search).get('id'),element=id=>document.getElementById(id);
  if(!/^[0-9a-f]{32}$/.test(id||'')){element('sync-error').textContent='기록 목록에서 재생할 항목을 선택하세요.';element('sync-error').hidden=false;return;}
  const prefix='/api/sync-recordings/'+id;
  const viewer=new LocalSceneView(element('sync-scene'),{recordedPlayback:true,assetURL:hash=>prefix+'/assets/'+hash,
    onMetrics:value=>{element('sync-metrics').textContent=JSON.stringify(value,null,2);}});
  const player=new SynchronizedPlayer({id,viewer,element});
  for(const [button,action] of [['sync-play','play'],['sync-pause','pause'],['sync-restart','seek'],['sync-previous','previous'],['sync-next','next']])element(button).addEventListener('click',()=>player.control(action,action==='seek'?0:undefined));
  element('sync-seek').addEventListener('input',()=>{player.scrubbing=true;});
  element('sync-seek').addEventListener('change',()=>player.control('seek',Number(element('sync-seek').value)));
  element('sync-seek').addEventListener('pointercancel',()=>{player.scrubbing=false;player.schedule(0);});
  element('sync-speed').addEventListener('change',()=>player.control('speed',Number(element('sync-speed').value)));
  document.addEventListener('visibilitychange',()=>player.visibility());window.addEventListener('pagehide',()=>player.destroy());
  void player.update();
}
// The player is also embedded in the workspace analysis panel. Importing it
// there must never bootstrap a second player from the page URL.
if(typeof document!=='undefined'&&typeof location!=='undefined'&&location.pathname==='/synchronized')bootstrap();
