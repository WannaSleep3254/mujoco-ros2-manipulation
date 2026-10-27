import {LocalSceneView} from './viewer.js';
const $=id=>document.getElementById(id);
let token=null,binding=null,busy=false,loaded=null,stopped=false,revision=0;
const viewer=new LocalSceneView($('recording-scene'),{onMetrics:value=>{$('recording-metrics').textContent=JSON.stringify(value,null,2);}});
function acceptView(view){
  if(view.binding_id!==binding){binding=view.binding_id;loaded=null;viewer.setExpectedBinding(binding,view.binding);}
  $('source-label').textContent=view.source.label+' · 읽기 전용';$('recording-seek').max=String(view.duration_sec);
  if(document.activeElement!==$('recording-seek'))$('recording-seek').value=String(view.position_sec);
  $('recording-time').textContent=view.position_sec.toFixed(2)+' / '+view.duration_sec.toFixed(2)+' s';
}
async function control(action,time_sec){
  if(busy||!token)return;busy=true;
  revision++;
  if(action==='seek'){binding=loaded=null;viewer.clearBinding('기록 시각 전환 중');}
  try{const response=await fetch('/api/recording/control',{method:'POST',headers:{'Content-Type':'application/json','X-Operator-Token':token},body:JSON.stringify(time_sec===undefined?{action}:{action,time_sec})});const result=await response.json();if(!response.ok)throw new Error(result.error);acceptView(result);}
  catch(error){$('recording-error').textContent=error.message;}finally{busy=false;}
}
$('recording-play').addEventListener('click',()=>control('play'));
$('recording-pause').addEventListener('click',()=>control('pause'));
$('recording-restart').addEventListener('click',()=>control('seek',0));
$('recording-seek').addEventListener('change',()=>control('seek',Number($('recording-seek').value)));
async function update(){
  const epoch=revision;
  try{
    if(busy)return;
    const response=await fetch('/api/recording',{cache:'no-store'});if(!response.ok)throw new Error('기록 연결을 확인할 수 없습니다.');
    const {view,token:next}=await response.json();if(epoch!==revision||busy)return;token=next;acceptView(view);
    if(loaded!==binding){const id=binding,d=view.manifest_descriptor;
      await viewer.loadManifest('/api/view/manifest?binding_id='+id,{binding_id:id,binding:view.binding,sha256:d.sha256,model_sha256:d.model_sha256,scene_sha256:d.scene_sha256});
      if(epoch!==revision||binding!==id)return;loaded=id;
    }
    if(epoch!==revision||busy)return;
    await viewer.pollPose('/api/view/pose?binding_id='+binding);
    if(epoch!==revision)return;
    $('recording-error').textContent='';
  }catch(error){if(epoch===revision){viewer.markStale('기록 연결 중단 · 마지막 수신 자세');$('recording-error').textContent=error.message;}}
  finally{if(!stopped)setTimeout(update,50);}
}
window.addEventListener('pagehide',()=>{stopped=true;viewer.destroy();});
update();
