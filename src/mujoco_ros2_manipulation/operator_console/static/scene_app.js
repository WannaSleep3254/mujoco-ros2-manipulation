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
    this.geometrySnapshot=null;this.geometrySignature=null;
    this.poseTimer=null;this.poseTimerAt=null;this.poseRead=null;this.poseNextAt=0;this.poseEpoch=0;this.poseTarget=null;this.poseVisible=null;
    this.submittedPoseViewer=null;this.submittedPoseBinding=null;this.submittedPoseSequence=null;
    this.seekInFlight=false;this.pendingSeek=null;this.seekSerial=0;this.seekEpoch=0;this.selecting=false;this.viewer=null;this.expected=null;this.loading=null;this.loadedBinding=null;this.data=null;this.playing=false;this.playEpoch=0;this.playStart=null;this.playTimer=null;this.manifestFailures=0;this.manifestRetryAt=0;this.notifiedManifest=null;
    this.element=document.createElement('div');this.element.className='local-scene';
    this.cameraSensitivity={...retainedCameraSensitivity};
    this.metrics=$('scene-metrics');
    this._createViewer();
    this.listenCamera($('scene-model'),'click',()=>this.select('model_preview',$('scene-source').value));
    this.listenCamera($('scene-live'),'click',()=>this.select('live',this.data?.config_source_id));
    this.listenCamera($('display-mode'),'change',()=>this.action('/api/view/display',{mode:$('display-mode').value}));
    this.listenCamera($('scene-collision'),'change',()=>this.viewer?.setCollisionVisible(Boolean($('scene-collision').checked)));
    this.listenCamera($('plan-seek'),'input',()=>{this.pause();this.seek(Number($('plan-seek').value));});
    this.listenCamera($('plan-play'),'click',()=>this.playing?this.pause():this.play());
    if($('plan-speed'))this.listenCamera($('plan-speed'),'change',()=>this.setPlaybackSpeed(Number($('plan-speed').value)));
    if($('plan-clearance-seek'))this.listenCamera($('plan-clearance-seek'),'click',()=>this.seekMinimumClearance());
    if($('scene-restart'))this.listenCamera($('scene-restart'),'click',()=>this.restartScene());
    if($('scene-fallback'))this.listenCamera($('scene-fallback'),'click',()=>this.useServerImage());
    for(const button of cameraElements('[data-scene-zoom]'))this.listenCamera(button,'click',()=>this.zoom(button.dataset.sceneZoom));
    for(const button of cameraElements('[data-scene-reset]'))this.listenCamera(button,'click',()=>this.resetCamera());
    for(const button of cameraElements('[data-scene-fit-all]'))this.listenCamera(button,'click',()=>this.fitAll());
    for(const button of cameraElements('[data-scene-fit-selected]'))this.listenCamera(button,'click',()=>this.geometryAction('fit',null,button.dataset.geometryScope));
    for(const select of cameraElements('[data-scene-geometry]'))this.listenCamera(select,'change',()=>this.geometryAction('select',select.value===''?null:Number(select.value),select.dataset.geometryScope));
    for(const input of cameraElements('[data-scene-geometry-hidden]'))this.listenCamera(input,'change',()=>this.geometryAction('visible',!input.checked,input.dataset.geometryScope));
    for(const input of cameraElements('[data-scene-geometry-opacity]'))this.listenCamera(input,'input',()=>this.geometryAction('opacity',Number(input.value),input.dataset.geometryScope));
    for(const button of cameraElements('[data-scene-geometry-reset]'))this.listenCamera(button,'click',()=>this.geometryAction('reset',null,button.dataset.geometryScope));
    for(const input of cameraElements('[data-scene-speed]'))this.listenCamera(input,'input',()=>this.setSensitivity(input.dataset.sceneSpeed,Number(input.value)));
    this.listenCamera(document,'visibilitychange',()=>{this.syncPosePolling(true);this.updateCameraControls();this.onAvailabilityChange();});
    this.updateCameraControls();
  }
  _createViewer(){
    let created;
    try{this.element.textContent='';this.viewer=created=new LocalSceneView(this.element,{controlSensitivity:this.cameraSensitivity,
      onContextLost:()=>{this.pause();this.updateRecoveryControls();this.onAvailabilityChange();},
      onFirstVisibleFrame:()=>{this.recoveryFailed=false;this.updateCameraControls();this.onAvailabilityChange();},
      onGeometryChange:snapshot=>this.updateGeometryControls(snapshot),
      onRenderFrame:frame=>this.sceneFrameSubmitted(frame,created),
      onMetrics:metrics=>{this.metrics.textContent=JSON.stringify(metrics,null,2);if(!this.isImage()&&metrics.binding_id===this.expected){const age=metrics.mode==='live'?(Number.isFinite(metrics.source_age_ms)?Math.round(metrics.source_age_ms)+' ms 전 상태':'상태 수신 대기'):metrics.mode?'읽기 전용 보기':'장면 준비 대기';$('frame-age').textContent=$('demo-frame-age').textContent=age;const status={fresh:'현재 관절 자세',stale:'마지막 수신 자세',preview:'읽기 전용 미리보기',loading:'장면 준비 중',waiting:'관절 상태 대기',error:'장면 표시 오류'}[metrics.view_state]||'장면 준비 중';$('frame-label').textContent=$('demo-frame-label').textContent=status;}}});
      this.viewer.setReviewMode(this.data?.mode==='plan_preview');
      this.viewer.setCollisionVisible(Boolean($('scene-collision')?.checked));this.updateGeometryControls(this.viewer.geometryCatalog());return true;}
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
    this.pause();this._clearPendingSeek();this.resetPosePolling();this.recovering=true;const epoch=++this.recoveryEpoch;
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
    finally{if(epoch===this.recoveryEpoch){this.loading=null;this.recovering=false;this.updateRecoveryControls();this.syncPosePolling();this.onAvailabilityChange();}}
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
    return new Promise(resolve=>{const entry={time,expected,serial:++this.seekSerial,epoch:this.seekEpoch,resolve};this.updateCameraControls();this.onAvailabilityChange();if(this.seekInFlight){this.pendingSeek?.resolve(false);this.pendingSeek=entry;}else this._sendSeek(entry);});
  }
  async _sendSeek(entry){
    this.seekInFlight=true;this.activeSeek=entry;let ok=false;
    try {const result=await this.request('/api/view/seek',{time_sec:entry.time,binding_id:entry.expected});ok=Boolean(result)&&this.canSeek(entry.expected)&&entry.epoch===this.seekEpoch;if(ok&&entry.serial===this.seekSerial){$('plan-seek').value=String(entry.time);this.poseTarget={id:entry.expected,time:entry.time,serial:entry.serial,epoch:entry.epoch,started:performance.now(),seek:true,stateConfirmed:false};this.syncPosePolling(true);this.renderPlanTime();}}
    catch(error){if(this.canSeek(entry.expected)&&entry.epoch===this.seekEpoch&&entry.serial===this.seekSerial)this.notify(error.message);}
    finally {entry.resolve(ok);const next=this.pendingSeek;this.pendingSeek=null;this.seekInFlight=false;if(this.activeSeek===entry)this.activeSeek=null;if(next){if(this.canSeek(next.expected)&&next.epoch===this.seekEpoch)this._sendSeek(next);else next.resolve(false);}}
  }
  _clearPendingSeek(){this.pendingSeek?.resolve(false);this.pendingSeek=null;this.seekSerial++;this.seekEpoch++;}
  pause(){this.playEpoch++;this.playing=false;clearTimeout(this.playTimer);this.playTimer=null;$('plan-play').textContent='▶ 계획 재생';}
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
    this.data.binding_id&&this.viewer.bindingID===this.data.binding_id&&this.viewer.hasPose&&this.viewer.manifest&&!this.viewer.loading&&
    (this.data.mode!=='plan_preview'||(this.previewPoseMatches()&&!this.poseTarget.exhausted&&this.submittedPoseViewer===this.viewer&&this.submittedPoseBinding===this.expected&&this.submittedPoseSequence===this.viewer.sequence)));}
  zoom(direction){return this.cameraReady()?this.viewer.zoomCamera(direction):false;}
  fitAll(){return this.cameraReady()?this.viewer.fitCamera():false;}
  geometryAction(action,value,scope){
    const snapshot=this.geometrySnapshot;
    if(!this.cameraReady()||!snapshot||snapshot.scope_key!==scope||snapshot.binding_id!==this.data?.binding_id)return false;
    if(['visible','opacity','reset'].includes(action)&&(this.data?.mode==='plan_preview'||snapshot.review_mode))return false;
    const id=snapshot.selected_id;
    let changed=false;
    if(action==='select')changed=this.viewer.selectGeometry(value,scope);
    else if(action==='fit')changed=this.viewer.fitSelected(scope);
    else if(action==='visible'&&id!==null)changed=this.viewer.setGeometryVisible(id,value,scope);
    else if(action==='opacity'&&id!==null)changed=this.viewer.setGeometryOpacity(id,value,scope);
    else if(action==='reset')changed=this.viewer.resetDisplayOverrides(scope);
    this.updateGeometryControls(this.viewer.geometryCatalog());return changed;
  }
  updateGeometryControls(snapshot=this.geometrySnapshot){
    const changed=snapshot!==this.geometrySnapshot;
    if(snapshot)this.geometrySnapshot=snapshot;
    const current=this.geometrySnapshot,items=current?.items||[],scope=current?.scope_key||'';
    const valid=Boolean(current&&current.binding_id===this.data?.binding_id);
    const ready=valid&&this.cameraReady(),review=this.data?.mode==='plan_preview'||Boolean(current?.review_mode);
    const selected=valid?items.find(item=>item.id===current.selected_id):null;
    const signature=changed||this.geometrySignature===null?JSON.stringify([scope,items.map(item=>[item.id,item.name,item.type])]):this.geometrySignature;
    for(const select of cameraElements('[data-scene-geometry]')){
      if(signature!==this.geometrySignature){
        select.replaceChildren();const empty=document.createElement('option');empty.value='';empty.textContent='형상 선택';select.append(empty);
        for(const item of items){const option=document.createElement('option');option.value=String(item.id);option.textContent=(item.name?.trim()?item.name+' (#'+item.id+')':'형상 #'+item.id)+' · '+item.type;select.append(option);}
      }
      select.value=valid&&current.selected_id!==null?String(current.selected_id):'';select.disabled=!ready||!items.length;select.dataset.geometryScope=scope;
    }
    this.geometrySignature=signature;
    for(const button of cameraElements('[data-scene-fit-selected]')){button.disabled=!ready||!selected?.visible||selected.type==='plane';button.dataset.geometryScope=scope;}
    for(const input of cameraElements('[data-scene-geometry-hidden]')){input.checked=selected?!selected.local_visible:false;input.disabled=!ready||!selected||review;input.dataset.geometryScope=scope;}
    for(const input of cameraElements('[data-scene-geometry-opacity]')){input.value=String(selected?.opacity_multiplier??1);input.disabled=!ready||!selected||review;input.dataset.geometryScope=scope;input.setAttribute('aria-valuetext',Math.round(Number(input.value)*100)+'%');}
    for(const output of cameraElements('[data-scene-geometry-opacity-value]'))output.textContent=Math.round((selected?.opacity_multiplier??1)*100)+'%';
    for(const button of cameraElements('[data-scene-geometry-reset]')){button.disabled=!ready||review;button.dataset.geometryScope=scope;}
    for(const help of cameraElements('[data-scene-geometry-help]'))help.textContent=review?'계획 검토 중에는 형상을 숨기거나 투명하게 바꿀 수 없습니다. 선택과 화면 맞춤은 사용할 수 있습니다.':selected&&!selected.visible?'선택 형상이 현재 보이지 않습니다. 숨김과 불투명도, 충돌 형상 표시를 확인하세요.':selected?.type==='plane'?'바닥 평면은 선택 맞춤에서 제외합니다.':'형상을 선택해 화면을 맞추거나 표시를 조절합니다. 불투명도는 원본 재질에 곱해 적용하며, 물리 장면에는 영향을 주지 않습니다.';
  }
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
    for(const button of cameraElements('[data-scene-fit-all]'))button.disabled=!this.cameraReady();
    for(const input of cameraElements('[data-scene-speed]')){const value=this.cameraSensitivity[input.dataset.sceneSpeed];
      input.disabled=!local;input.value=String(value);input.setAttribute('aria-valuetext',value.toFixed(2)+'배');}
    for(const output of cameraElements('[data-scene-speed-value]'))output.textContent=this.cameraSensitivity[output.dataset.sceneSpeedValue].toFixed(2)+'×';
    for(const help of cameraElements('[data-scene-camera-help]'))help.textContent=this.isImage()?
      '서버 이미지는 확대·축소와 감도 조절을 지원하지 않습니다.':'드래그 회전 · 휠/핀치 확대·축소 · 오른쪽 드래그 이동';
    this.updateGeometryControls();
  }
  resetPosePolling(){
    clearTimeout(this.poseTimer);this.poseTimer=null;this.poseTimerAt=null;this.poseEpoch++;this.poseNextAt=0;this.poseTarget=null;this.poseVisible=null;
  }
  sceneVisible(){return !document.hidden&&Boolean(this.viewer?.sized)&&(!$('operate')?.hidden||!$('demos')?.hidden);}
  canPollPose(){return Boolean(!this.destroyed&&!this.recovering&&this.viewer&&!this.viewer.disposed&&!this.viewer.sceneError&&!this.viewer.contextLost&&
    this.data?.enabled&&!this.isImage()&&!this.data.error&&this.data.status==='ready'&&this.expected&&this.data.binding_id===this.expected&&this.loadedBinding===this.expected&&!this.loading&&this.viewer.bindingID===this.expected);}
  previewPoseMatches(target=this.poseTarget){return Boolean(target&&this.viewer?.hasPose&&target.id===this.expected&&target.serial===this.seekSerial&&target.epoch===this.seekEpoch&&
    Number.isFinite(this.viewer.previewTimeSec)&&Math.abs(this.viewer.previewTimeSec-target.time)<1e-8);}
  sceneFrameSubmitted(frame,viewer){
    if(this.destroyed||viewer!==this.viewer||!frame.displaying_pose||frame.sequence!==viewer.sequence)return;
    if(this.submittedPoseViewer===viewer&&this.submittedPoseBinding===viewer.bindingID&&this.submittedPoseSequence===frame.sequence)return;
    this.submittedPoseViewer=viewer;this.submittedPoseBinding=viewer.bindingID;this.submittedPoseSequence=frame.sequence;
    this.renderPlanTime();this.updateCameraControls();this.onAvailabilityChange();
  }
  expirePoseTarget(target){
    if(target.exhausted)return;
    target.exhausted=true;this.viewer?.markStale('계획 자세 갱신 대기 · 시점을 다시 선택하세요.');this.notify('선택한 계획 자세를 확인하지 못했습니다. 시점을 다시 선택하세요.');
    this.renderPlanTime();this.updateCameraControls();this.onAvailabilityChange();
  }
  reconcilePreviewTime(data){
    const target=this.poseTarget,stamp=data?.preview_time_sec;
    if(data?.mode!=='plan_preview'||!target||target.id!==data.binding_id||!Number.isFinite(stamp)||stamp<0||this.ownsSeek(data.binding_id))return;
    const same=Math.abs(stamp-target.time)<1e-8;
    if(target.seek&&!target.stateConfirmed&&!target.exhausted){
      if(same)target.stateConfirmed=true;
      else if(performance.now()-target.started>=10000){this.expirePoseTarget(target);return;}
      else return; // A GET already in flight at our ACK may still carry its old time.
    }
    if(!same){this.poseTarget={id:data.binding_id,time:stamp,serial:this.seekSerial,epoch:this.seekEpoch,started:performance.now(),seek:false,external:true};this.poseNextAt=0;this.onAvailabilityChange();}
  }
  renderPlanTime(){
    if(this.data?.mode!=='plan_preview'||!Number.isFinite(this.data.duration_sec))return;
    const target=this.poseTarget,actual=this.viewer?.bindingID===this.data.binding_id&&this.viewer.hasPose?this.viewer.previewTimeSec:null;
    const shown=Number.isFinite(actual)?actual.toFixed(2)+' / '+this.data.duration_sec.toFixed(2)+' s':'자세 수신 대기';
    const waiting=target&&!this.previewPoseMatches(target);
    $('plan-time').textContent=shown+(target?.exhausted?' · 갱신 확인 중단':waiting?' · '+target.time.toFixed(2)+' s 갱신 대기':target&&!this.cameraReady()?' · 화면 표시 대기':'');
  }
  ownsSeek(id=this.data?.binding_id){return [this.activeSeek,this.pendingSeek].some(entry=>entry&&entry.expected===id&&entry.epoch===this.seekEpoch);}
  syncPosePolling(immediate=false){
    if(!this.canPollPose())return;
    const visible=this.sceneVisible(),becameVisible=visible&&this.poseVisible===false;this.poseVisible=visible;
    if(this.data.mode==='model_preview'&&this.viewer.hasPose)return;
    if(this.data.mode==='plan_preview'){
      if(!this.poseTarget){this.poseTarget={id:this.expected,time:this.data.preview_time_sec??0,serial:this.seekSerial,epoch:this.seekEpoch,started:performance.now(),seek:false};this.updateCameraControls();this.onAvailabilityChange();}
      const target=this.poseTarget;
      if(target.exhausted||target.serial!==this.seekSerial||target.epoch!==this.seekEpoch||this.previewPoseMatches(target))return;
      if((target.seek||target.external)&&performance.now()-target.started>=10000){this.expirePoseTarget(target);return;}
    }
    if(immediate||becameVisible)this.poseNextAt=0;
    if(this.poseRead)return;
    const delay=Math.max(0,this.poseNextAt-performance.now()),at=performance.now()+delay;
    if(this.poseTimer!==null&&this.poseTimerAt<=at)return;
    clearTimeout(this.poseTimer);this.poseTimerAt=at;
    this.poseTimer=setTimeout(()=>{this.poseTimer=null;this.poseTimerAt=null;this.pollScenePose();},delay);
  }
  pollScenePose(){
    if(this.poseRead||!this.canPollPose())return;
    if(this.data.mode==='model_preview'&&this.viewer.hasPose)return;
    const viewer=this.viewer,id=this.expected,epoch=this.poseEpoch,mode=this.data.mode,target=this.poseTarget;
    if(mode==='plan_preview'&&(!target||target.exhausted||target.serial!==this.seekSerial||target.epoch!==this.seekEpoch||this.previewPoseMatches(target)))return;
    this.poseNextAt=performance.now()+(this.sceneVisible()?250:1000);
    const current=()=>!this.destroyed&&viewer===this.viewer&&id===this.expected&&epoch===this.poseEpoch;
    const accept=envelope=>current()&&(mode!=='plan_preview'||(target===this.poseTarget&&target.serial===this.seekSerial&&target.epoch===this.seekEpoch&&Number.isFinite(envelope.preview_time_sec)&&Math.abs(envelope.preview_time_sec-target.time)<1e-8));
    const pending=viewer.pollPose('/api/view/pose?binding_id='+id,{accept})
      .then(applied=>{if(applied&&current()){this._restoreCamera(id);this.renderPlanTime();this.updateCameraControls();this.onAvailabilityChange();}})
      .catch(()=>{})
      .finally(()=>{if(this.poseRead===pending){this.poseRead=null;this.syncPosePolling();}});
    this.poseRead=pending;
  }
  mount(){
    const slot=$('demos').hidden?$('scene-slot'):$('demo-scene-slot');
    if(this.element.parentElement!==slot){slot.append(this.element);this.viewer?.resize();}
    this.syncPosePolling();
  }
  update(state){
    if(this.destroyed)return;
    const data=state.view;if(data?.binding&&this.data?.binding&&data.binding.operator_session_id===this.data.binding.operator_session_id&&data.binding.generation<this.data.binding.generation)return;
    if(this.recovering&&data?.binding_id!==this.expected){this.recoveryEpoch++;this.recovering=false;this.viewer?.clearBinding('표시 장면이 바뀌었습니다');this.pendingCamera=null;this.expected=this.loadedBinding=this.loading=null;}
    const previousError=Boolean(this.data?.status==='error'||this.data?.error),failed=Boolean(data?.status==='error'||data?.error);
    if(this.data?.binding_id!==data?.binding_id||failed||!data){this.pause();this._clearPendingSeek();this.resetPosePolling();}this.data=data;this.reconcilePreviewTime(data);this.viewer?.setReviewMode(data?.mode==='plan_preview');this.updateCameraControls();
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
    if(planReady){$('plan-seek').max=String(data.duration_sec);if(!this.playing&&!this.ownsSeek(data.binding_id)&&document.activeElement!==$('plan-seek'))$('plan-seek').value=String(this.poseTarget?.time??data.preview_time_sec??0);this.renderPlanTime();}
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
      .finally(()=>{if(expected===this.expected&&this.loading===pending){this.loading=null;this.syncPosePolling();}});
      this.loading=pending;
    }else if(this.loadedBinding===id&&!this.loading)this.syncPosePolling();
  }
  destroy(){if(this.destroyed)return;this.destroyed=true;this.recoveryEpoch++;this.recovering=false;this.pause();this._clearPendingSeek();this.resetPosePolling();for(const {element,event,callback} of this.cameraListeners)element.removeEventListener(event,callback);this.cameraListeners=[];
    for(const panel of cameraElements('[data-scene-camera-controls]'))panel.hidden=true;
    for(const input of cameraElements('[data-scene-zoom], [data-scene-reset], [data-scene-speed], [data-scene-fit-all], [data-scene-fit-selected], [data-scene-geometry], [data-scene-geometry-hidden], [data-scene-geometry-opacity], [data-scene-geometry-reset]'))input.disabled=true;
    this.viewer?.destroy();}
}
