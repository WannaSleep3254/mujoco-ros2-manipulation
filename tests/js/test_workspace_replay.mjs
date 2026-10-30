/** Embedded replay owns one read-only player and disposes hidden/old views. */
import test from 'node:test';
import assert from 'node:assert/strict';
import {readFile} from 'node:fs/promises';

const directory=new URL('../../src/mujoco_ros2_manipulation/operator_console/static/',import.meta.url);
const source=(await readFile(new URL('workspace_replay.js',directory),'utf8')).replace(/^import .*?;\n/gm,'').replace(/export /g,'');
const api=new Function('LocalSceneView','SynchronizedPlayer',source+'\nreturn {WorkspaceReplay,mountWorkspaceReplay};')();
const ID='a'.repeat(32),OTHER='b'.repeat(32);
class Events {
  constructor(){this.events=new Map();}
  addEventListener(type,callback){if(!this.events.has(type))this.events.set(type,[]);this.events.get(type).push(callback);}
  emit(type,detail){for(const callback of this.events.get(type)||[])callback({type,detail});}
}
class Element extends Events {
  constructor(tag){super();this.tag=tag;this.children=[];this.hidden=false;this.dataset={};this.attributes={};this.value='';this.disabled=false;this.isConnected=true;}
  append(...children){this.children.push(...children);}
  replaceChildren(...children){this.children=children;}
  setAttribute(name,value){this.attributes[name]=String(value);}
  focus(){this.focused=true;}
  scrollIntoView(){}
}
function harness(){
  const panel=new Element('section'),events=new Events(),doc=new Events(),viewers=[],players=[];
  doc.hidden=false;doc.activeElement=new Element('button');doc.createElement=tag=>new Element(tag);doc.getElementById=id=>id==='analysis'?panel:null;
  const makeViewer=(container,options)=>{const viewer={container,options,destroyed:0,destroy(){this.destroyed++;}};viewers.push(viewer);return viewer;};
  const makePlayer=options=>{const player={options,destroyed:0,updates:0,controls:[],
    update(){this.updates++;},control(...action){this.controls.push(action);},schedule(){},
    destroy(){this.destroyed++;options.viewer.destroy();}};players.push(player);return player;};
  const replay=new api.WorkspaceReplay({panel,doc,makeViewer,makePlayer});
  return {replay,panel,doc,events,viewers,players};
}

test('explicit selection creates an independent recorded view with fixed asset scope',()=>{
  const h=harness();assert.equal(h.players.length,0);assert.equal(h.replay.root.hidden,true);
  assert.equal(h.replay.open('../other'),false);assert.equal(h.players.length,0);
  h.replay.open(ID);assert.equal(h.players.length,1);assert.equal(h.players[0].updates,1);
  assert.equal(h.viewers[0].options.recordedPlayback,true);
  assert.equal(h.viewers[0].options.assetURL('f'.repeat(64)),'/api/sync-recordings/'+ID+'/assets/'+'f'.repeat(64));
  assert.equal(h.players[0].options.id,ID);assert.equal(h.players[0].options.element('sync-seek').id,'workspace-sync-seek');
  assert.equal(h.players[0].options.element('sync-back'),undefined);
  h.replay.open(ID);assert.equal(h.players.length,1,'Selecting the same record does not start a second loop');
  h.replay.destroy();
});

test('changing the selected record destroys its old loop and GL context first',()=>{
  const h=harness();h.replay.open(ID);const old=h.players[0];h.replay.open(OTHER);
  assert.equal(old.destroyed,1);assert.equal(h.viewers[0].destroyed,1);assert.equal(h.replay.recording,OTHER);
  assert.equal(h.players.length,2);h.replay.close();
  assert.equal(h.players[1].destroyed,1);assert.equal(h.replay.root.hidden,true);
  assert.equal(h.replay.node('sync-image-frame').children.length,0);assert.equal(h.replay.node('sync-scene').children.length,0);
});

test('embedded controls call only the selected replay player, never robot commands',()=>{
  const h=harness();h.replay.open(ID);const player=h.players[0];
  for(const id of ['sync-play','sync-pause','sync-previous','sync-next','sync-restart'])h.replay.node(id).emit('click');
  h.replay.node('sync-seek').value='1.25';h.replay.node('sync-seek').emit('input');assert.equal(player.scrubbing,true);h.replay.node('sync-seek').emit('change');
  h.replay.node('sync-speed').value='.25';h.replay.node('sync-speed').emit('change');
  assert.deepEqual(player.controls,[['play',undefined],['pause',undefined],['previous',undefined],['next',undefined],['seek',0],['seek',1.25],['speed',.25]]);
  h.replay.close();h.replay.node('sync-play').emit('click');assert.equal(player.controls.length,7);
  assert.equal(/fetch\s*\(|\/api\/(command|cancel|grasp|tasks)/.test(source),false);
});

test('hidden and disposed panels cannot create a new replay or retain active loops',()=>{
  const h=harness();h.panel.hidden=true;assert.equal(h.replay.open(ID),false);h.panel.hidden=false;
  h.doc.hidden=true;assert.equal(h.replay.open(ID),false);h.doc.hidden=false;h.replay.open(ID);
  h.doc.hidden=true;assert.equal(h.players[0].options.hidden(),true);
  h.replay.destroy();h.doc.hidden=false;assert.equal(h.replay.open(OTHER),false);assert.equal(h.players[0].destroyed,1);
});

test('partial initialization failure releases the newly allocated renderer',()=>{
  const h=harness();h.replay.makePlayer=()=>{throw Error('player init failed');};assert.equal(h.replay.open(ID),false);
  assert.equal(h.viewers[0].destroyed,1);assert.equal(h.replay.node('sync-error').hidden,false);assert.match(h.replay.node('sync-error').textContent,/init failed/);
});

test('workspace navigation, hidden tabs and page exit dispose embedded playback',()=>{
  const h=harness(),players=[];
  class Viewer {destroy(){this.destroyed=true;}}
  class Player {
    constructor(options){this.options=options;this.destroyed=0;players.push(this);}
    update(){}
    destroy(){this.destroyed++;this.options.viewer.destroy();}
  }
  const mountedApi=new Function('LocalSceneView','SynchronizedPlayer',source+'\nreturn {mountWorkspaceReplay};')(Viewer,Player);
  const replay=mountedApi.mountWorkspaceReplay({doc:h.doc,events:h.events});
  h.events.emit('workspace-replay-open',{id:ID});assert.equal(players.length,1);
  h.events.emit('console-navigation',{view:'analysis'});assert.equal(players[0].destroyed,0);
  h.events.emit('console-navigation',{view:'operate'});assert.equal(players[0].destroyed,1);assert.equal(replay.root.hidden,true);
  h.events.emit('workspace-replay-open',{id:OTHER});assert.equal(players.length,2);
  h.doc.hidden=true;h.doc.emit('visibilitychange');assert.equal(players[1].destroyed,1);
  h.doc.hidden=false;h.events.emit('workspace-replay-open',{id:ID});assert.equal(players.length,3);
  h.events.emit('pagehide');assert.equal(players[2].destroyed,1);assert.equal(replay.disposed,true);
  h.events.emit('workspace-replay-open',{id:OTHER});assert.equal(players.length,3);
});

test('importing synchronized player into the workspace cannot bootstrap standalone UI',async()=>{
  const standalone=(await readFile(new URL('synchronized.js',directory),'utf8')).replace(/^import .*?;\n/gm,'').replace(/export /g,'');
  let touched=0;
  new Function('document','location','LocalSceneView',standalone)({getElementById(){touched++;throw Error('Unexpected standalone bootstrap');}}, {pathname:'/',search:''},()=>{touched++;});
  assert.equal(touched,0);
});
