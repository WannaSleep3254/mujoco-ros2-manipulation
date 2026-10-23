/** Local scene controls: read-only view/selection and saved-plan playback. */
import {LocalSceneView} from './viewer.js';
const $=id=>document.getElementById(id);
export class SceneControls {
  constructor({request,notify,showOperate}) {
    this.request=request;this.notify=notify;this.showOperate=showOperate;
    this.seekInFlight=false;this.pendingSeek=null;this.seekSerial=0;this.seekEpoch=0;this.selecting=false;this.viewer=null;this.expected=null;this.loading=null;this.loadedBinding=null;this.data=null;this.playing=false;this.playEpoch=0;this.playStart=null;this.playTimer=null;this.manifestFailures=0;this.manifestRetryAt=0;this.notifiedManifest=null;
    this.element=document.createElement('div');this.element.className='local-scene';
    this.metrics=$('scene-metrics');
    try {this.viewer=new LocalSceneView(this.element,{onMetrics:metrics=>{this.metrics.textContent=JSON.stringify(metrics,null,2);if(!this.isImage()&&metrics.binding_id===this.expected){const age=metrics.mode==='live'?(Number.isFinite(metrics.source_age_ms)?Math.round(metrics.source_age_ms)+' ms 전 상태':'상태 수신 대기'):metrics.mode?'읽기 전용 보기':'장면 준비 대기';$('frame-age').textContent=$('demo-frame-age').textContent=age;const status={fresh:'현재 관절 자세',stale:'마지막 수신 자세',preview:'읽기 전용 미리보기',loading:'장면 준비 중',waiting:'관절 상태 대기',error:'장면 표시 오류'}[metrics.view_state]||'장면 준비 중';$('frame-label').textContent=$('demo-frame-label').textContent=status;}}});}
    catch(error){this.element.textContent='로컬 3D를 시작하지 못했습니다. 서버 이미지 표시를 선택할 수 있습니다.';this.notify(error.message);}
    $('scene-model').addEventListener('click',()=>this.select('model_preview',$('scene-source').value));
    $('scene-live').addEventListener('click',()=>this.select('live',this.data?.config_source_id));
    $('display-mode').addEventListener('change',()=>this.action('/api/view/display',{mode:$('display-mode').value}));
    $('scene-collision').addEventListener('change',()=>this.viewer?.setCollisionVisible($('scene-collision').checked));
    $('plan-seek').addEventListener('input',()=>{this.pause();this.seek(Number($('plan-seek').value));});
    $('plan-play').addEventListener('click',()=>this.playing?this.pause():this.play());
  }
  isImage(){return this.data?.display_mode==='image';}
  async action(path,payload){try{return await this.request(path,payload);}catch(error){this.notify(error.message);return null;}}
  async select(mode,source_id){if(this.selecting||!source_id)return;this.selecting=true;this.pause();try{await this.action('/api/view/select',{mode,source_id});}finally{this.selecting=false;}}
  async previewRequest(request_id){if(this.selecting)return;this.selecting=true;this.pause();try{const result=await this.action('/api/view/plan',{request_id});if(result)this.showOperate();}finally{this.selecting=false;}}
  canSeek(expected=this.data?.binding_id){return Boolean(expected&&this.data?.mode==='plan_preview'&&this.data.binding_id===expected&&this.data.status==='ready'&&!this.data.error&&Number.isFinite(this.data.duration_sec)&&this.data.duration_sec>0);}
  seek(time,expected=this.data?.binding_id){
    if(!this.canSeek(expected))return Promise.resolve(false);
    return new Promise(resolve=>{const entry={time,expected,serial:++this.seekSerial,epoch:this.seekEpoch,resolve};if(this.seekInFlight){this.pendingSeek?.resolve(false);this.pendingSeek=entry;}else this._sendSeek(entry);});
  }
  async _sendSeek(entry){
    this.seekInFlight=true;let ok=false;
    try {const result=await this.request('/api/view/seek',{time_sec:entry.time,binding_id:entry.expected});ok=Boolean(result)&&this.canSeek(entry.expected)&&entry.epoch===this.seekEpoch;if(ok&&entry.serial===this.seekSerial)$('plan-seek').value=String(entry.time);}
    catch(error){if(this.canSeek(entry.expected)&&entry.epoch===this.seekEpoch&&entry.serial===this.seekSerial)this.notify(error.message);}
    finally {entry.resolve(ok);const next=this.pendingSeek;this.pendingSeek=null;this.seekInFlight=false;if(next){if(this.canSeek(next.expected)&&next.epoch===this.seekEpoch)this._sendSeek(next);else next.resolve(false);}}
  }
  _clearPendingSeek(){this.pendingSeek?.resolve(false);this.pendingSeek=null;this.seekSerial++;this.seekEpoch++;}
  pause(){this.playEpoch++;this.playing=false;clearTimeout(this.playTimer);$('plan-play').textContent='▶ 계획 재생';}
  play(){
    if(!this.canSeek())return;
    this.pause();const epoch=this.playEpoch,binding=this.data.binding_id,duration=this.data.duration_sec;
    this.playing=true;const start={wall:performance.now(),time:Number($('plan-seek').value)};
    if(start.time>=duration)start.time=0;
    $('plan-play').textContent='Ⅱ 재생 멈춤';
    const frame=async()=>{if(!this.playing||this.playEpoch!==epoch||this.data?.binding_id!==binding)return;const time=Math.min(duration,start.time+(performance.now()-start.wall)/1000);const ok=await this.seek(time,binding);if(this.playEpoch!==epoch||this.data?.binding_id!==binding)return;if(!ok||time>=duration)this.pause();else if(this.playing)this.playTimer=setTimeout(frame,100);};frame();
  }
  camera(view){this.viewer?.cameraPreset(view);}
  mount(){
    const slot=$('demos').hidden?$('scene-slot'):$('demo-scene-slot');
    if(this.element.parentElement!==slot){slot.append(this.element);this.viewer?.resize();}
  }
  update(state){
    const data=state.view;if(data?.binding&&this.data?.binding&&data.binding.operator_session_id===this.data.binding.operator_session_id&&data.binding.generation<this.data.binding.generation)return;
    const previousError=Boolean(this.data?.status==='error'||this.data?.error),failed=Boolean(data?.status==='error'||data?.error);
    if(this.data?.binding_id!==data?.binding_id||failed||!data){this.pause();this._clearPendingSeek();}this.data=data;
    if(!data){$('plan-play').disabled=$('plan-seek').disabled=true;if(this.viewer&&(this.expected!==null||previousError))this.viewer.clearBinding('장면 준비 대기');this.expected=null;this.loadedBinding=null;this.loading=null;return;}
    this.mount();const image=this.isImage();this.element.hidden=image;
    $('scene-slot').hidden=image;$('demo-scene-slot').hidden=image;
    if(!image){$('frame').hidden=$('demo-frame').hidden=true;$('frame-empty').hidden=$('demo-frame-empty').hidden=true;$('stale-frame').hidden=$('demo-stale-frame').hidden=true;}
    const selected=data.source?.id||data.config_source_id||null,catalog=data.catalog||[],select=$('scene-source');
    const signature=JSON.stringify(catalog),rebuild=signature!==this.catalogSignature,draft=select.value;
    if(rebuild){this.catalogSignature=signature;select.replaceChildren();for(const item of catalog){const option=document.createElement('option');option.value=item.id;option.textContent=item.label+(item.available?'':' · 준비 필요');option.disabled=!item.available;select.append(option);}}
    if(selected!==this.appliedSource)select.value=selected||'';
    else if(rebuild)select.value=catalog.some(item=>item.id===draft&&item.available)?draft:selected||'';
    this.appliedSource=selected;
    const modelOnly=data.mode==='model_preview';$('view-arm-labels').hidden=modelOnly;$('view-source-label').hidden=!modelOnly;$('view-source-label').textContent=modelOnly?(data.source?.label||catalog.find(item=>item.id===selected)?.label||'모델 미리보기'):'';
    $('view-method').textContent=image?'ROS 관절 상태 → 서버 이미지':'관절 상태 → 로컬 3D 표시';$('display-mode').value=data.display_mode||'web3d';$('display-mode').disabled=data.preview_active;
    const busy=Boolean(state.active_request_id)||['starting','stopping','demo'].includes(state.lifecycle.phase)||['starting','running','cancel_requested'].includes(state.demo?.current?.status);
    $('scene-model').disabled=$('scene-live').disabled=busy||this.selecting||image||!data.enabled;
    $('scene-source').disabled=busy||this.selecting||image||!data.enabled;
    $('scene-mode').textContent=data.mode==='plan_preview'?'저장된 계획 · 읽기 전용':data.mode==='model_preview'?'모델 미리보기 · 실행 연결 없음':'실시간 관절 상태';
    $('plan-preview').hidden=data.mode!=='plan_preview';
    const planReady=this.canSeek();$('plan-play').disabled=$('plan-seek').disabled=!planReady;
    if(planReady){$('plan-seek').max=String(data.duration_sec);if(!this.playing&&document.activeElement!==$('plan-seek'))$('plan-seek').value=String(data.preview_time_sec||0);$('plan-time').textContent=(data.preview_time_sec||0).toFixed(2)+' / '+data.duration_sec.toFixed(2)+' s';}
    else {this.pause();$('plan-time').textContent=data.mode==='plan_preview'?'계획 표시 대기':'';}
    if(!this.viewer)return;
    if(failed){if(!previousError||this.expected!==null)this.viewer.clearBinding('장면 표시 오류 · 실시간/모델 보기를 다시 선택하세요.');this.expected=null;this.loadedBinding=null;this.loading=null;return;}
    const id=data.binding_id;
    if(!id){if(this.expected!==null||previousError){this.viewer.clearBinding('장면 준비 대기');this.expected=null;this.loadedBinding=null;this.loading=null;}return;}
    if(this.expected!==id){$('frame-age').textContent=$('demo-frame-age').textContent=data.mode==='live'?'상태 수신 대기':'읽기 전용 보기';this.expected=id;this.viewer.setExpectedBinding(id,data.binding);this.loading=null;this.loadedBinding=null;this.manifestFailures=0;this.manifestRetryAt=0;this.notifiedManifest=null;}
    if(image)return;
    const descriptor=data.manifest_descriptor;
    if(!descriptor){this.viewer.markStale(data.status==='error'?'장면을 준비하지 못했습니다':'장면 준비 대기');return;}
    if(this.loadedBinding!==id&&!this.loading&&this.manifestFailures<5&&performance.now()>=this.manifestRetryAt){
      const expected=id;
      const pending=this.viewer.loadManifest('/api/view/manifest?binding_id='+id,{binding_id:id,binding:data.binding,sha256:descriptor.sha256,model_sha256:descriptor.model_sha256,scene_sha256:descriptor.scene_sha256})
      .then(()=>{if(expected===this.expected&&this.loading===pending){this.loadedBinding=expected;this.manifestFailures=0;this.manifestRetryAt=0;}})
      .catch(error=>{if(expected===this.expected&&this.loading===pending){this.manifestFailures++;this.manifestRetryAt=performance.now()+Math.min(30000,1000*2**(this.manifestFailures-1));if(this.notifiedManifest!==expected){this.notifiedManifest=expected;this.notify('3D 장면 로드 실패: '+error.message);}}})
      .finally(()=>{if(expected===this.expected&&this.loading===pending)this.loading=null;});
      this.loading=pending;
    }else if(this.loadedBinding===id&&!this.loading)this.viewer.pollPose('/api/view/pose?binding_id='+id).catch(()=>{});
  }
  destroy(){this.pause();this._clearPendingSeek();this.viewer?.destroy();}
}
