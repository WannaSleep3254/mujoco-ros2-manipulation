import {LocalSceneView} from './viewer.js';
import {SynchronizedPlayer} from './synchronized.js';

const recordingID=value=>typeof value==='string'&&/^[0-9a-f]{32}$/.test(value);

/** One explicitly selected immutable record, separate from the live view. */
export class WorkspaceReplay {
  constructor({panel,doc=globalThis.document,
    makeViewer=(container,options)=>new LocalSceneView(container,options),
    makePlayer=options=>new SynchronizedPlayer(options)}){
    this.panel=panel;this.doc=doc;this.makeViewer=makeViewer;this.makePlayer=makePlayer;
    this.player=null;this.recording=null;this.disposed=false;this.returnFocus=null;
    this.nodes=new Map();this.root=this.build();panel.append(this.root);
  }
  node(id){return this.nodes.get(id);}
  build(){
    const create=(tag,text='',id=null,parent=null)=>{
      const node=this.doc.createElement(tag);node.textContent=text;
      if(id){node.id='workspace-'+id;this.nodes.set(id,node);}
      if(parent)parent.append(node);return node;
    };
    const root=create('article');root.className='workspace-replay inspection-card';root.hidden=true;root.tabIndex=-1;
    root.setAttribute('aria-labelledby','workspace-sync-title');
    const heading=create('div','',null,root);heading.className='workspace-replay-heading';
    create('h3','센서·3D 보존 기록 재생','sync-title',heading);
    const close=create('button','재생 닫기',null,heading);close.type='button';
    close.addEventListener('click',()=>this.close({restoreFocus:true}));
    create('p','기록을 선택하세요.','sync-source',root);
    create('p','현재 실시간 관측과 별개인 보존 기록입니다. 재생·탐색·시점 조작은 로봇 명령이나 취소를 보내지 않습니다.',null,root).className='help';
    const error=create('p','','sync-error',root);error.className='error-text';error.hidden=true;error.setAttribute('role','alert');
    create('p','연결 대기','sync-playback-state',root).className='help';
    const panels=create('div','',null,root);panels.className='workspace-replay-panels';
    for(const [kind,label] of [['pose','3D 자세'],['image','센서 영상']]){
      const section=create('section','',null,panels);
      const header=create('div','',null,section);header.className='workspace-replay-track-heading';
      const trackTitle=create('h4',label,'sync-'+kind+'-title',header);
      section.setAttribute('aria-labelledby',trackTitle.id);
      create('span','준비 대기','sync-'+kind+'-status',header);
      const media=create('div','',null,section);media.className='workspace-replay-media';
      create('div','',kind==='pose'?'sync-scene':'sync-image-frame',media).className=kind==='pose'?'local-scene':'';
      create('p','기록을 준비하고 있습니다.','sync-'+kind+'-empty',media).className='workspace-replay-empty';
      create('p','표시 시각 —','sync-'+kind+'-time',section).className='help';
    }
    const transport=create('div','',null,root);transport.className='workspace-replay-transport';transport.setAttribute('aria-label','보존 기록 공통 시간축');
    const label=create('label','재생 위치',null,transport);label.htmlFor='workspace-sync-seek';
    create('output','0.000 / 0.000 s','sync-time',transport);
    const seek=create('input','','sync-seek',transport);seek.type='range';seek.min='0';seek.max='0';seek.step='0.001';seek.value='0';seek.disabled=true;
    seek.addEventListener('input',()=>{if(this.player)this.player.scrubbing=true;});
    seek.addEventListener('change',()=>this.player?.control('seek',Number(seek.value)));
    seek.addEventListener('pointercancel',()=>{if(this.player){this.player.scrubbing=false;this.player.schedule(0);}});
    const buttons=create('div','',null,transport);buttons.className='workspace-replay-buttons';
    for(const [id,label,action] of [['sync-restart','처음으로','seek'],['sync-previous','이전 프레임','previous'],
      ['sync-play','재생','play'],['sync-pause','일시 정지','pause'],['sync-next','다음 프레임','next']]){
      const button=create('button',label,id,buttons);button.type='button';button.disabled=true;
      button.addEventListener('click',()=>this.player?.control(action,action==='seek'?0:undefined));
    }
    const speedLabel=create('label','속도 ',null,buttons);speedLabel.htmlFor='workspace-sync-speed';
    const speed=create('select','','sync-speed',speedLabel);speed.disabled=true;
    for(const value of [.25,.5,1,2]){const option=create('option',value+'×',null,speed);option.value=String(value);option.selected=value===1;}
    speed.value='1';speed.addEventListener('change',()=>this.player?.control('speed',Number(speed.value)));
    const status=create('p','기록을 준비하고 있습니다.','sync-status',root);status.setAttribute('role','status');status.dataset.state='waiting';
    create('p','표시된 선택 시각 —','sync-displayed-time',root).className='help';
    create('p','선택 시각보다 앞선 0.5초 이내 표본을 표시합니다. 빠진 자세·영상은 보간하지 않습니다. 3D 드래그 회전 · 휠 확대·축소 · 오른쪽 드래그 이동',null,root).className='help';
    const details=create('details','',null,root);create('summary','기록 출처와 표시 정보',null,details);
    create('p','','sync-identity',details).className='workspace-replay-identity';
    create('p','원래 수집 시각과 공급원 식별자를 유지합니다. 이 재생만으로 실제 카메라 보정이나 하드웨어 동기화를 확인하지 않습니다.',null,details).className='help';
    create('pre','표시 준비 대기','sync-metrics',details);
    return root;
  }
  open(id){
    if(this.disposed||!recordingID(id)||this.panel.hidden||this.doc.hidden)return false;
    if(this.recording===id&&this.player){this.root.focus?.();return true;}
    this.close();this.recording=id;this.returnFocus=this.doc.activeElement;this.root.hidden=false;
    this.node('sync-error').textContent='';this.node('sync-error').hidden=true;
    const prefix='/api/sync-recordings/'+id;
    let viewer=null;
    try{
      viewer=this.makeViewer(this.node('sync-scene'),{recordedPlayback:true,
        assetURL:hash=>prefix+'/assets/'+hash,
        onMetrics:value=>{this.node('sync-metrics').textContent=JSON.stringify(value,null,2);}});
      this.player=this.makePlayer({id,viewer,element:key=>this.node(key),
        hidden:()=>this.disposed||this.doc.hidden||this.panel.hidden||this.root.hidden});
      void this.player.update();this.root.focus?.();this.root.scrollIntoView?.({block:'nearest'});return true;
    }catch(error){
      if(this.player)this.player.destroy();else viewer?.destroy();this.player=null;
      this.node('sync-error').textContent='기록 표시를 준비하지 못했습니다: '+error.message;this.node('sync-error').hidden=false;return false;
    }
  }
  close({restoreFocus=false}={}){
    this.player?.destroy();this.player=null;this.recording=null;this.root.hidden=true;
    this.node('sync-image-frame').replaceChildren();this.node('sync-scene').replaceChildren();
    if(restoreFocus&&!this.panel.hidden&&this.returnFocus?.isConnected)this.returnFocus.focus?.();
    this.returnFocus=null;
  }
  destroy(){this.close();this.disposed=true;}
}

export function mountWorkspaceReplay({doc=globalThis.document,events=globalThis.window}={}){
  const panel=doc.getElementById('analysis');if(!panel)return null;
  const replay=new WorkspaceReplay({panel,doc});
  events.addEventListener('workspace-replay-open',event=>replay.open(event.detail?.id));
  events.addEventListener('console-navigation',event=>{if(event.detail?.view!=='analysis')replay.close();});
  doc.addEventListener('visibilitychange',()=>{if(doc.hidden)replay.close();});
  events.addEventListener('pagehide',()=>replay.destroy());
  return replay;
}
if(typeof document!=='undefined'&&typeof window!=='undefined')mountWorkspaceReplay();
