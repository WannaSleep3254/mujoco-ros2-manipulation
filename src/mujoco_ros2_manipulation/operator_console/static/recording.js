import {LocalSceneView} from './viewer.js';
const $=id=>document.getElementById(id);
let token=null,binding=null,busy=false,loaded=null,stopped=false,revision=0;
let latestView=null,poseDirty=true,updating=false,updateTimer=null,pendingImmediate=false,stateRead=null;
const READ_TIMEOUT_MS=3000;
const viewer=new LocalSceneView($('recording-scene'),{onMetrics:value=>{if(!stopped)$('recording-metrics').textContent=JSON.stringify(value,null,2);}});
function cadence(){return !document.hidden&&latestView?.playing?50:1000;}
function scheduleUpdate(delay=cadence()){
  if(stopped)return;
  if(updateTimer!==null){clearTimeout(updateTimer);updateTimer=null;}
  if(updating||busy){if(delay===0)pendingImmediate=true;return;}
  updateTimer=setTimeout(()=>{updateTimer=null;update();},delay);
}
function controlsAvailable(){
  for(const id of ['recording-play','recording-pause','recording-restart','recording-seek'])$(id).disabled=stopped||busy||!token;
}
/** The one state GET deadline includes both headers and the complete JSON body. */
async function readState(){
  const controller=new AbortController();let rejectAbort;
  const aborted=new Promise((_,reject)=>{rejectAbort=reject;});
  const cancel=reason=>{if(!controller.signal.aborted){controller.abort(reason);rejectAbort(reason);}};
  const current={cancel};stateRead=current;
  const timer=setTimeout(()=>cancel(new DOMException('기록 상태 조회 시간이 초과되었습니다.','TimeoutError')),READ_TIMEOUT_MS);
  try{return await Promise.race([(async()=>{
    const response=await fetch('/api/recording',{cache:'no-store',signal:controller.signal});
    if(!response.ok)throw new Error('기록 연결을 확인할 수 없습니다.');
    return await response.json();
  })(),aborted]);}
  finally{clearTimeout(timer);if(stateRead===current)stateRead=null;}
}
function acceptView(view){
  latestView=view;
  if(view.binding_id!==binding){binding=view.binding_id;loaded=null;viewer.setExpectedBinding(binding,view.binding);}
  $('source-label').textContent=view.source.label+' · 읽기 전용';$('recording-seek').max=String(view.duration_sec);
  if(document.activeElement!==$('recording-seek'))$('recording-seek').value=String(view.position_sec);
  $('recording-time').textContent=view.position_sec.toFixed(2)+' / '+view.duration_sec.toFixed(2)+' s';
}
async function control(action,time_sec){
  if(stopped||busy||!token)return;busy=true;
  const epoch=++revision;poseDirty=true;
  if(updateTimer!==null){clearTimeout(updateTimer);updateTimer=null;}
  stateRead?.cancel(new DOMException('재생 조작으로 이전 조회를 종료했습니다.','AbortError'));
  viewer.cancelPoseRead?.();
  controlsAvailable();
  if(action==='seek'){binding=loaded=null;viewer.clearBinding('기록 시각 전환 중');}
  try{const response=await fetch('/api/recording/control',{method:'POST',headers:{'Content-Type':'application/json','X-Operator-Token':token},body:JSON.stringify(time_sec===undefined?{action}:{action,time_sec})});const result=await response.json();if(stopped||epoch!==revision)return;if(!response.ok)throw new Error(result.error);acceptView(result);}
  catch(error){if(!stopped&&epoch===revision)$('recording-error').textContent=error.message;}
  finally{busy=false;if(!stopped){controlsAvailable();scheduleUpdate(document.hidden?1000:0);}}
}
$('recording-play').addEventListener('click',()=>control('play'));
$('recording-pause').addEventListener('click',()=>control('pause'));
$('recording-restart').addEventListener('click',()=>control('seek',0));
$('recording-seek').addEventListener('change',()=>control('seek',Number($('recording-seek').value)));
async function update(){
  if(stopped)return;
  if(updateTimer!==null){clearTimeout(updateTimer);updateTimer=null;}
  if(updating||busy){pendingImmediate=true;return;}
  updating=true;pendingImmediate=false;
  const epoch=revision;
  try{
    const {view,token:next}=await readState();if(stopped||epoch!==revision||busy)return;token=next;acceptView(view);controlsAvailable();
    // Playback continues at the source while hidden; only inexpensive state
    // reads continue. Returning to the tab requests the latest pose immediately.
    if(document.hidden)return;
    if(loaded!==binding){const id=binding,d=view.manifest_descriptor;
      await viewer.loadManifest('/api/view/manifest?binding_id='+id,{binding_id:id,binding:view.binding,sha256:d.sha256,model_sha256:d.model_sha256,scene_sha256:d.scene_sha256});
      if(stopped||epoch!==revision||binding!==id)return;loaded=id;poseDirty=true;
    }
    if(stopped||epoch!==revision||busy||document.hidden)return;
    const newer=Number.isSafeInteger(view.sequence)&&view.sequence>viewer.sequence;
    if(poseDirty||view.playing||newer||!viewer.hasPose){
      const id=binding,applied=await viewer.pollPose('/api/view/pose?binding_id='+id);
      if(stopped||epoch!==revision||binding!==id)return;
      // A duplicate pose is a successful refresh; a busy/failed pose request
      // must not consume the pause/seek acknowledgement's refresh requirement.
      poseDirty=!(viewer.hasPose&&(Number.isSafeInteger(view.sequence)?viewer.sequence>=view.sequence:applied));
    }
    if(stopped||epoch!==revision)return;
    $('recording-error').textContent='';
  }catch(error){if(!stopped&&epoch===revision){poseDirty=true;viewer.markStale('기록 연결 중단 · 마지막 수신 자세');$('recording-error').textContent=error.message;}}
  finally{updating=false;if(!stopped&&!busy){const delay=pendingImmediate&&!document.hidden?0:cadence();pendingImmediate=false;scheduleUpdate(delay);}}
}
document.addEventListener('visibilitychange',()=>{if(stopped)return;if(document.hidden)pendingImmediate=false;else poseDirty=true;scheduleUpdate(document.hidden?1000:0);});
window.addEventListener('pagehide',()=>{stopped=true;revision++;pendingImmediate=false;if(updateTimer!==null){clearTimeout(updateTimer);updateTimer=null;}stateRead?.cancel(new DOMException('기록 화면을 종료했습니다.','AbortError'));viewer.cancelPoseRead?.();viewer.destroy();});
controlsAvailable();
update();
