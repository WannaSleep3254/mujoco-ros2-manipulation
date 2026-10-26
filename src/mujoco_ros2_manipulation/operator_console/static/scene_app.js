/** Local scene controls: read-only view/selection and saved-plan playback. */
import {LocalSceneView} from './viewer.js';
const $=id=>document.getElementById(id);
const cameraElements=selector=>document.querySelectorAll?.(selector)??[];
// Keep preferences across model/binding switches and a controller recreation in
// this page. A page reload starts at 1x; no robot/server setting is persisted.
let retainedCameraSensitivity={rotate:1,zoom:1,pan:1};
export class SceneControls {
  constructor({request,notify,showOperate,onAvailabilityChange=()=>{}}) {
    this.request=request;this.notify=notify;this.showOperate=showOperate;
    this.onAvailabilityChange=onAvailabilityChange;this.destroyed=false;this.recovering=false;this.recoveryFailed=false;this.recoveryEpoch=0;this.pendingCamera=null;this.playbackSpeed=1;
    this.cameraListeners=[];
    this.seekInFlight=false;this.pendingSeek=null;this.seekSerial=0;this.seekEpoch=0;this.selecting=false;this.viewer=null;this.expected=null;this.loading=null;this.loadedBinding=null;this.data=null;this.playing=false;this.playEpoch=0;this.playStart=null;this.playTimer=null;this.manifestFailures=0;this.manifestRetryAt=0;this.notifiedManifest=null;
    this.element=document.createElement('div');this.element.className='local-scene';
    this.cameraSensitivity={...retainedCameraSensitivity};
    this.metrics=$('scene-metrics');
    this._createViewer();
    this.listenCamera($('scene-model'),'click',()=>this.select('model_preview',$('scene-source').value));
    this.listenCamera($('scene-live'),'click',()=>this.select('live',this.data?.config_source_id));
    this.listenCamera($('display-mode'),'change',()=>this.action('/api/view/display',{mode:$('display-mode').value}));
    this.listenCamera($('scene-collision'),'change',()=>this.viewer?.setCollisionVisible($('scene-collision').checked));
    this.listenCamera($('plan-seek'),'input',()=>{this.pause();this.seek(Number($('plan-seek').value));});
    this.listenCamera($('plan-play'),'click',()=>this.playing?this.pause():this.play());
    if($('plan-speed'))this.listenCamera($('plan-speed'),'change',()=>this.setPlaybackSpeed(Number($('plan-speed').value)));
    if($('plan-clearance-seek'))this.listenCamera($('plan-clearance-seek'),'click',()=>this.seekMinimumClearance());
    if($('scene-restart'))this.listenCamera($('scene-restart'),'click',()=>this.restartScene());
    if($('scene-fallback'))this.listenCamera($('scene-fallback'),'click',()=>this.useServerImage());
    for(const button of cameraElements('[data-scene-zoom]'))this.listenCamera(button,'click',()=>this.zoom(button.dataset.sceneZoom));
    for(const button of cameraElements('[data-scene-reset]'))this.listenCamera(button,'click',()=>this.resetCamera());
    for(const input of cameraElements('[data-scene-speed]'))this.listenCamera(input,'input',()=>this.setSensitivity(input.dataset.sceneSpeed,Number(input.value)));
    this.updateCameraControls();
  }
  _createViewer(){
    try{this.element.textContent='';this.viewer=new LocalSceneView(this.element,{controlSensitivity:this.cameraSensitivity,
      onContextLost:()=>{this.pause();this.updateRecoveryControls();this.onAvailabilityChange();},
      onFirstVisibleFrame:()=>{this.recoveryFailed=false;this.updateCameraControls();this.onAvailabilityChange();},
      onMetrics:metrics=>{this.metrics.textContent=JSON.stringify(metrics,null,2);if(!this.isImage()&&metrics.binding_id===this.expected){const age=metrics.mode==='live'?(Number.isFinite(metrics.source_age_ms)?Math.round(metrics.source_age_ms)+' ms 전 상태':'상태 수신 대기'):metrics.mode?'읽기 전용 보기':'장면 준비 대기';$('frame-age').textContent=$('demo-frame-age').textContent=age;const status={fresh:'현재 관절 자세',stale:'마지막 수신 자세',preview:'읽기 전용 미리보기',loading:'장면 준비 중',waiting:'관절 상태 대기',error:'장면 표시 오류'}[metrics.view_state]||'장면 준비 중';$('frame-label').textContent=$('demo-frame-label').textContent=status;}}});return true;}
    catch(error){this.viewer=null;this.element.textContent='로컬 3D를 시작하지 못했습니다. 3D 재시작 또는 서버 이미지를 선택하세요.';this.notify(error.message);return false;}
  }
  updateRecoveryControls(){
    const failed=!this.isImage()&&Boolean(this.data?.enabled)&&(!this.viewer||Boolean(this.viewer.sceneError)||this.viewer.contextLost||this.recoveryFailed||this.recovering);
    if($('scene-recovery'))$('scene-recovery').hidden=!failed;
    for(const id of ['scene-restart','scene-fallback'])if($(id)){$(id).hidden=!failed;$(id).disabled=this.destroyed||this.recovering||!this.data?.enabled;}
    if($('scene-recovery-status'))$('scene-recovery-status').textContent=this.recovering?'3D를 다시 준비하고 현재 자세를 확인하고 있습니다. 실행 승인은 대기합니다.':'표시만 다시 시작합니다. 서버 이미지 선택 시 실시간 보기로 돌아오며 로봇 명령은 보내지 않습니다.';
  }
  async restartScene(){
    if(this.destroyed||this.recovering||!this.data?.enabled||this.isImage())return false;
    this.pause();this._clearPendingSeek();this.recovering=true;const epoch=++this.recoveryEpoch;
    const data=this.data,id=data.binding_id,descriptor=data.manifest_descriptor;
    const previous=this.viewer;
    const camera=previous?.manifest?.scene_sha256===descriptor?.scene_sha256?previous.cameraSnapshot?.():null;
    previous?.destroy();this.viewer=null;this.expected=id;this.loadedBinding=null;this.loading=null;
    this.updateRecoveryControls();this.onAvailabilityChange();
    try{
      if(!id||!descriptor||!this._createViewer())return false;
      const viewer=this.viewer;viewer.setExpectedBinding(id,data.binding);
      this.pendingCamera=camera?{id,value:camera}:null;
      const pending=viewer.loadManifest('/api/view/manifest?binding_id='+id,{binding_id:id,binding:data.binding,sha256:descriptor.sha256,model_sha256:descriptor.model_sha256,scene_sha256:descriptor.scene_sha256});
      this.loading=pending;await pending;
      if(this.destroyed||epoch!==this.recoveryEpoch||this.viewer!==viewer||this.data?.binding_id!==id)return false;
      this.loading=null;this.loadedBinding=id;this.manifestFailures=0;this.notifiedManifest=null;
      const applied=await viewer.pollPose('/api/view/pose?binding_id='+id);
      if(this.destroyed||epoch!==this.recoveryEpoch||this.data?.binding_id!==id)return false;
      if(applied)this._restoreCamera(id);
      return applied;
    }catch(error){if(epoch===this.recoveryEpoch&&!this.destroyed){this.recoveryFailed=true;this.notify('3D 재시작 실패: '+error.message);}return false;}
    finally{if(epoch===this.recoveryEpoch){this.loading=null;this.recovering=false;this.updateRecoveryControls();this.onAvailabilityChange();}}
  }
  _restoreCamera(id){if(this.pendingCamera?.id===id){this.viewer?.restoreCamera?.(this.pendingCamera.value);this.pendingCamera=null;}}
  async useServerImage(){
    if(this.destroyed||this.recovering||!this.data?.enabled)return false;
    this.pause();this._clearPendingSeek();
    if(this.data.preview_active){if(!this.data.config_source_id||!await this.action('/api/view/select',{mode:'live',source_id:this.data.config_source_id}))return false;}
    return Boolean(await this.action('/api/view/display',{mode:'image'}));
  }
  isImage(){return this.data?.display_mode==='image';}
  async action(path,payload){if(this.destroyed)return null;try{return await this.request(path,payload);}catch(error){if(!this.destroyed)this.notify(error.message);return null;}}
  async select(mode,source_id){if(this.selecting||!source_id)return;this.selecting=true;this.pause();try{await this.action('/api/view/select',{mode,source_id});}finally{this.selecting=false;}}
  async previewRequest(request_id){if(this.selecting)return;this.selecting=true;this.pause();try{const result=await this.action('/api/view/plan',{request_id});if(result)this.showOperate();}finally{this.selecting=false;}}
  async previewReview(request_id){if(this.selecting)return;this.selecting=true;this.pause();try{const result=await this.action('/api/view/review',{request_id});if(result)this.showOperate();}finally{this.selecting=false;}}
  canSeek(expected=this.data?.binding_id){return Boolean(!this.destroyed&&!this.recovering&&this.viewer&&!this.viewer.disposed&&!this.viewer.sceneError&&expected&&this.data?.mode==='plan_preview'&&this.data.binding_id===expected&&this.data.status==='ready'&&!this.data.error&&Number.isFinite(this.data.duration_sec)&&this.data.duration_sec>0);}
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
  setPlaybackSpeed(value){
    if(![.25,.5,1].includes(value))return false;
    const resume=this.playing;
    if(resume&&this.playStart)$('plan-seek').value=String(Math.min(this.data.duration_sec,this.playStart.time+(performance.now()-this.playStart.wall)/1000*this.playStart.speed));
    this.pause();this.playbackSpeed=value;if($('plan-speed'))$('plan-speed').value=String(value);if(resume)this.play();return true;
  }
  seekMinimumClearance(){const stamp=this.data?.clearance_minimum_time_sec;if(!this.canSeek()||!Number.isFinite(stamp)||stamp<0||stamp>this.data.duration_sec)return Promise.resolve(false);this.pause();return this.seek(stamp);}
  play(){
    if(!this.canSeek())return;
    this.pause();const epoch=this.playEpoch,binding=this.data.binding_id,duration=this.data.duration_sec;
    this.playing=true;const start=this.playStart={wall:performance.now(),time:Number($('plan-seek').value),speed:this.playbackSpeed};
    if(start.time>=duration)start.time=0;
    $('plan-play').textContent='Ⅱ 재생 멈춤';
    const frame=async()=>{if(!this.playing||this.playEpoch!==epoch||this.data?.binding_id!==binding)return;const time=Math.min(duration,start.time+(performance.now()-start.wall)/1000*start.speed);const ok=await this.seek(time,binding);if(this.playEpoch!==epoch||this.data?.binding_id!==binding)return;if(!ok||time>=duration)this.pause();else if(this.playing)this.playTimer=setTimeout(frame,100);};frame();
  }
  camera(view){this.viewer?.cameraPreset(view);}
  listenCamera(element,event,callback){element.addEventListener(event,callback);this.cameraListeners.push({element,event,callback});}
  cameraReady(){return Boolean(!this.recovering&&this.viewer&&!this.viewer.disposed&&!this.viewer.sceneError&&!this.viewer.contextLost&&this.viewer.sized&&
    !this.viewer.document?.hidden&&Number.isFinite(this.viewer.firstVisibleSubmittedAt)&&this.data?.enabled&&!this.isImage()&&!this.data.error&&this.data.status!=='error'&&
    this.data.binding_id&&this.viewer.bindingID===this.data.binding_id&&this.viewer.hasPose&&this.viewer.manifest&&!this.viewer.loading);}
  zoom(direction){return this.cameraReady()?this.viewer.zoomCamera(direction):false;}
  resetCamera(){
    if(!this.cameraReady())return false;
    const changed=this.viewer.resetCamera();
    if(changed)for(const button of cameraElements('[data-camera]'))button.classList?.toggle('selected',button.dataset.camera==='overview');
    return changed;
  }
  setSensitivity(kind,value){
    if(this.recovering||!this.viewer||this.viewer.disposed||this.viewer.sceneError||!this.data?.enabled||this.isImage()||this.data.error||this.data.status==='error'||!['rotate','zoom','pan'].includes(kind))return false;
    try {const applied=this.viewer.setControlSensitivity({...this.cameraSensitivity,[kind]:value});
      this.cameraSensitivity={...applied};retainedCameraSensitivity={...applied};this.updateCameraControls();return true;}
    catch(error){this.notify(error.message);this.updateCameraControls();return false;}
  }
  updateCameraControls(){
    this.updateRecoveryControls();
    const local=Boolean(!this.recovering&&this.viewer&&!this.viewer.disposed&&!this.viewer.sceneError&&this.data?.enabled&&!this.isImage()&&!this.data.error&&this.data.status!=='error');
    for(const panel of cameraElements('[data-scene-camera-controls]'))panel.hidden=!this.viewer||!this.data?.enabled;
    for(const button of cameraElements('[data-scene-zoom], [data-scene-reset]'))button.disabled=!this.cameraReady();
    for(const input of cameraElements('[data-scene-speed]')){const value=this.cameraSensitivity[input.dataset.sceneSpeed];
      input.disabled=!local;input.value=String(value);input.setAttribute('aria-valuetext',value.toFixed(2)+'배');}
    for(const output of cameraElements('[data-scene-speed-value]'))output.textContent=this.cameraSensitivity[output.dataset.sceneSpeedValue].toFixed(2)+'×';
    for(const help of cameraElements('[data-scene-camera-help]'))help.textContent=this.isImage()?
      '서버 이미지는 확대·축소와 감도 조절을 지원하지 않습니다.':'드래그 회전 · 휠/핀치 확대·축소 · 오른쪽 드래그 이동';
  }
  mount(){
    const slot=$('demos').hidden?$('scene-slot'):$('demo-scene-slot');
    if(this.element.parentElement!==slot){slot.append(this.element);this.viewer?.resize();}
  }
  update(state){
    if(this.destroyed)return;
    const data=state.view;if(data?.binding&&this.data?.binding&&data.binding.operator_session_id===this.data.binding.operator_session_id&&data.binding.generation<this.data.binding.generation)return;
    if(this.recovering&&data?.binding_id!==this.expected){this.recoveryEpoch++;this.recovering=false;this.viewer?.clearBinding('표시 장면이 바뀌었습니다');this.pendingCamera=null;this.expected=this.loadedBinding=this.loading=null;}
    const previousError=Boolean(this.data?.status==='error'||this.data?.error),failed=Boolean(data?.status==='error'||data?.error);
    if(this.data?.binding_id!==data?.binding_id||failed||!data){this.pause();this._clearPendingSeek();}this.data=data;this.updateCameraControls();
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
    $('scene-mode').textContent=data.mode==='plan_preview'?(data.review_plan?'실행 전 계획 검토 · 아직 실행하지 않음':'저장된 계획 · 읽기 전용'):data.mode==='model_preview'?'모델 미리보기 · 실행 연결 없음':'실시간 관절 상태';
    $('plan-preview').hidden=data.mode!=='plan_preview';
    $('plan-preview-title').textContent=data.review_plan?'실행 전 계획 · 읽기 전용 미리보기':'저장된 계획 · 읽기 전용';
    const planReady=this.canSeek();$('plan-play').disabled=$('plan-seek').disabled=!planReady;
    if($('plan-speed')){$('plan-speed').disabled=!planReady;$('plan-speed').value=String(this.playbackSpeed);}
    if($('plan-clearance-seek'))$('plan-clearance-seek').disabled=!planReady||!Number.isFinite(data.clearance_minimum_time_sec)||data.clearance_minimum_time_sec<0||data.clearance_minimum_time_sec>data.duration_sec;
    if(planReady){$('plan-seek').max=String(data.duration_sec);if(!this.playing&&document.activeElement!==$('plan-seek'))$('plan-seek').value=String(data.preview_time_sec||0);$('plan-time').textContent=(data.preview_time_sec||0).toFixed(2)+' / '+data.duration_sec.toFixed(2)+' s';}
    else {this.pause();$('plan-time').textContent=data.mode==='plan_preview'?'계획 표시 대기':'';}
    if(!this.viewer||this.recovering)return;
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
    }else if(this.loadedBinding===id&&!this.loading)this.viewer.pollPose('/api/view/pose?binding_id='+id).then(applied=>{if(applied&&this.expected===id)this._restoreCamera(id);}).catch(()=>{});
  }
  destroy(){if(this.destroyed)return;this.destroyed=true;this.recoveryEpoch++;this.recovering=false;this.pause();this._clearPendingSeek();for(const {element,event,callback} of this.cameraListeners)element.removeEventListener(event,callback);this.cameraListeners=[];
    for(const panel of cameraElements('[data-scene-camera-controls]'))panel.hidden=true;
    for(const input of cameraElements('[data-scene-zoom], [data-scene-reset], [data-scene-speed]'))input.disabled=true;
    this.viewer?.destroy();}
}
