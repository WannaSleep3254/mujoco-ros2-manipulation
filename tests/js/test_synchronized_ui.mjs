/** Selection identity, delayed image decode, gaps and playback controls without GL/ROS. */
import test from 'node:test';
import assert from 'node:assert/strict';
import {readFile} from 'node:fs/promises';
import {createHash,webcrypto} from 'node:crypto';

const source=(await readFile(new URL('../../src/mujoco_ros2_manipulation/operator_console/static/synchronized.js',import.meta.url),'utf8'))
 .replace(/^import[^\n]*\n/,'').replace(/export /g,'').replace(/\nbootstrap\(\);\s*$/,'');
const api=new Function(source+'\nreturn {SynchronizedPlayer,loadImage};')();
const deferred=()=>{let resolve,reject;const promise=new Promise((yes,no)=>{resolve=yes;reject=no;});return {promise,resolve,reject};};
const id='a'.repeat(32),digest='f'.repeat(64);
const imageDescriptor={stamp_ns:'1000000000',sequence:1,jpeg_sha256:digest,byte_length:128,width:4,height:3};
function view({binding='b',sequence=1,time=0,playing=false,pose=true,image=true}={}){
 return {binding_id:binding.repeat(64),binding:{generation:binding==='b'?1:2},source:{label:'검증 기록'},
  manifest_descriptor:{sha256:'1'.repeat(64),model_sha256:'2'.repeat(64),scene_sha256:'3'.repeat(64)},
  position_sec:time,duration_sec:4,playing,speed:1,recording_sha256:'4'.repeat(64),
  selection:{cursor_ns:String(1000000000+Math.round(time*1e9)),time_sec:time,
   pose_time_sec:time,image_time_sec:time,pose_delta_sec:0,image_delta_sec:0,
   pose_status:pose?'ready':'missing',image_status:image?'ready':'missing',
   pose:pose?{binding_id:binding.repeat(64),sequence}:null,image:image?{...imageDescriptor,sequence}:null}};
}
function response(value,ok=true){return {ok,status:ok?200:409,json:async()=>value};}
function decoded(name='image'){return {element:{name},destroyed:0,destroy(){this.destroyed++;}};}
async function drain(){for(let i=0;i<30;i++)await Promise.resolve();}
function harness(){
 const elements=new Map(),requests=[],images=[],timers=new Map();let timerID=0,hidden=false;
 const element=key=>{if(!elements.has(key))elements.set(key,{textContent:'',value:'',hidden:false,disabled:false,dataset:{},children:[],replaceChildren(...values){this.children=values;}});return elements.get(key);};
 const viewer={binding:null,hasPose:false,sequence:-1,applied:[],loads:[],destroyed:false,loadGate:null,
  setExpectedBinding(binding){this.binding=binding;this.hasPose=false;this.sequence=-1;},
  async loadManifest(url,options){this.loads.push({url,options});if(this.loadGate)await this.loadGate.promise;},
  updatePose(value){if(value.sequence<=this.sequence)return false;this.applied.push(value);this.sequence=value.sequence;this.hasPose=true;return true;},
  cancelPoseRead(){},destroy(){this.destroyed=true;}};
 const fetcher=(url,options={})=>{const pending=deferred();requests.push({url,options,...pending});return pending.promise;};
 const imageLoader=(url,descriptor,options)=>{const pending=deferred();images.push({url,descriptor,options,...pending});return pending.promise;};
 const player=new api.SynchronizedPlayer({id,viewer,element,fetcher,imageLoader,hidden:()=>hidden,
  timers:{set:(fn,delay)=>{const key=++timerID;timers.set(key,{fn,delay});return key;},clear:key=>timers.delete(key)}});
 return {player,viewer,element,requests,images,timers,setHidden:value=>hidden=value,
  get(url=undefined){return requests.filter(item=>!item.options.method&&(!url||item.url===url));},
  post(){return requests.filter(item=>item.options.method==='POST');},
  runTimer(delay){const entry=[...timers].find(([,item])=>item.delay===delay);assert.ok(entry,'Expected timer '+delay);timers.delete(entry[0]);entry[1].fn();}};
}
async function state(h,value=view(),image=decoded()){
 const update=h.player.update();h.get().at(-1).resolve(response({view:value,token:'fixture-token'}));await drain();
 if(value.selection.image_status==='ready'&&!h.player.hidden()){h.images.at(-1).resolve(image);}
 await update;return image;
}

test('one selected descriptor commits pose and decoded image together',async()=>{
 const h=harness(),update=h.player.update();h.get()[0].resolve(response({view:view(),token:'token'}));await drain();
 assert.equal(h.images.length,1);assert.equal(h.viewer.applied.length,0);
 assert.notEqual(h.element('sync-status').dataset.state,'ready');
 const image=decoded();h.images[0].resolve(image);await update;
 assert.equal(h.viewer.applied.length,1);assert.equal(h.element('sync-image-frame').children[0],image.element);
 assert.equal(h.element('sync-status').dataset.state,'ready');assert.match(h.element('sync-displayed-time').textContent,/0.000/);
 assert.equal(h.get().length,1,'Pose is from the same descriptor, not a later latest-pose GET');
});

test('seek clears displayed image and rejects a late old-binding decode',async()=>{
 const h=harness(),first=await state(h);
 const update=h.player.update();h.get().at(-1).resolve(response({view:view({time:1,sequence:2}),token:'token'}));await drain();
 const oldImage=decoded('late'),seek=h.player.control('seek',2);
 assert.equal(first.destroyed,1);assert.equal(h.element('sync-image-frame').children.length,0);
 h.post()[0].resolve(response(view({binding:'c',time:2,sequence:3})));await seek;
 h.images.at(-1).resolve(oldImage);await update;
 assert.equal(oldImage.destroyed,1);assert.equal(h.viewer.applied.length,1);
 assert.equal(h.player.binding,'c'.repeat(64));assert.notEqual(h.element('sync-status').dataset.state,'ready');
 const fresh=await state(h,view({binding:'c',time:2,sequence:3}));
 assert.equal(h.viewer.applied.at(-1).sequence,3);assert.equal(h.element('sync-image-frame').children[0],fresh.element);
});

test('gap selection removes prior image and never marks a partial pair synchronized',async()=>{
 const h=harness(),image=await state(h);
 await state(h,view({time:1,sequence:2,image:false}));
 assert.equal(image.destroyed,1);assert.equal(h.element('sync-image-frame').children.length,0);
 assert.equal(h.element('sync-image-empty').hidden,false);assert.equal(h.element('sync-pose-empty').hidden,true);
 assert.equal(h.element('sync-status').dataset.state,'partial');
 assert.match(h.element('sync-image-time').textContent,/^마지막 표본 시각 /);
 assert.match(h.element('sync-pose-time').textContent,/^표시 시각 /);
 const next=await state(h,view({time:2,sequence:3,pose:false}));
 assert.equal(h.element('sync-pose-empty').hidden,false);assert.equal(h.element('sync-image-frame').children[0],next.element);
 assert.equal(h.element('sync-status').dataset.state,'partial');
});

test('decode failure reveals neither the new pose nor a mismatched old image; recovery retries the pair',async()=>{
 const h=harness(),first=await state(h),update=h.player.update();
 h.get().at(-1).resolve(response({view:view({time:1,sequence:2}),token:'token'}));await drain();
 h.images.at(-1).reject(Error('decode failed'));await update;
 assert.equal(h.viewer.applied.length,1);assert.equal(first.destroyed,1);assert.equal(h.element('sync-image-frame').children.length,0);
 assert.equal(h.element('sync-status').dataset.state,'error');assert.match(h.element('sync-error').textContent,/decode failed/);
 await state(h,view({time:1,sequence:2}));assert.equal(h.viewer.applied.length,2);assert.equal(h.element('sync-status').dataset.state,'ready');
});

test('new image on a repeated pose remains valid without inventing a pose sequence',async()=>{
 const h=harness();await state(h);await state(h,view({time:.1,sequence:1}));
 assert.equal(h.viewer.applied.length,1);assert.equal(h.element('sync-status').dataset.state,'ready');
 assert.match(h.element('sync-displayed-time').textContent,/0.100/);
});

test('hidden tab reads playback state without image fetch then refreshes the current pair',async()=>{
 const h=harness();await state(h);h.setHidden(true);h.player.visibility();
 await state(h,view({time:1,sequence:2,playing:true}));assert.equal(h.images.length,1);
 assert.equal(h.viewer.applied.length,1);assert.equal([...h.timers.values()].at(-1).delay,1000);
 h.setHidden(false);h.player.visibility();await state(h,view({time:2,sequence:3,playing:true}));
 assert.equal(h.images.length,2);assert.equal(h.viewer.applied.at(-1).sequence,3);assert.equal([...h.timers.values()].at(-1).delay,50);
});

test('read updates are single flight and a control does not resend on failed POST',async()=>{
 const h=harness();await state(h);
 const first=h.player.update();await h.player.update();assert.equal(h.get().length,2);
 const pause=h.player.control('pause');h.post()[0].resolve(response({error:'pause refused'},false));await pause;
 h.get().at(-1).resolve(response({view:view({time:1,sequence:2}),token:'old'}));await first;
 assert.equal(h.post().length,1);assert.match(h.element('sync-error').textContent,/pause refused/);
 await state(h,view({time:2,sequence:3}));assert.equal(h.post().length,1);
 assert.match(h.element('sync-error').textContent,/pause refused/,'Automatic state refresh must not erase an unresolved control error');
});

test('playback operations send only narrow read-only playback payloads',async()=>{
 const h=harness();await state(h);
 for(const [action,value,expected] of [['speed',.5,{action:'speed',speed:.5}],['next',undefined,{action:'next'}],['previous',undefined,{action:'previous'}],['seek',0,{action:'seek',time_sec:0}]]){
  const job=h.player.control(action,value),request=h.post().at(-1);
  assert.equal(request.url,'/api/sync-recordings/'+id+'/control');assert.deepEqual(JSON.parse(request.options.body),expected);
  request.resolve(response(view({binding:'c'})));await job;
 }
 assert.equal(h.requests.some(item=>/command|approve|cancel/.test(item.url)),false);
});

test('late manifest completion after a seek cannot apply either stale track',async()=>{
 const h=harness();h.viewer.loadGate=deferred();const update=h.player.update();
 h.get()[0].resolve(response({view:view(),token:'token'}));await drain();
 const seek=h.player.control('seek',2);h.post()[0].resolve(response(view({binding:'c',time:2})));await seek;
 h.viewer.loadGate.resolve();await update;
 assert.equal(h.images.length,0);assert.equal(h.viewer.applied.length,0);assert.equal(h.player.loaded,null);
});

test('page exit releases displayed and late decoded media and destroys the renderer',async()=>{
 const h=harness(),first=await state(h),update=h.player.update();
 h.get().at(-1).resolve(response({view:view({time:1,sequence:2}),token:'token'}));await drain();h.player.destroy();
 const late=decoded();h.images.at(-1).resolve(late);await update;
 assert.equal(first.destroyed,1);assert.equal(late.destroyed,1);assert.equal(h.viewer.destroyed,true);
 assert.equal(h.timers.size,0);assert.equal(h.element('sync-play').disabled,true);
});

function mediaHarness({bytes=new Uint8Array([255,216,1,2,255,217]),decodeError=false,width=4,height=3}={}){
 const urls={created:0,revoked:[],createObjectURL(){return 'blob:'+ ++this.created;},revokeObjectURL(value){this.revoked.push(value);}};
 class Image {constructor(){this.naturalWidth=width;this.naturalHeight=height;}async decode(){if(decodeError)throw Error('decode invalid');}}
 const media=new Function('Image','URL','crypto','Blob',source+'\nreturn {loadImage};')(Image,urls,webcrypto,Blob);
 const descriptor={...imageDescriptor,byte_length:bytes.byteLength,jpeg_sha256:createHash('sha256').update(bytes).digest('hex')};
 const fetcher=async()=>({ok:true,headers:{get:name=>name==='Content-Type'?'image/jpeg':String(bytes.byteLength)},arrayBuffer:async()=>bytes.buffer});
 return {media,urls,descriptor,fetcher};
}

test('image loader verifies immutable bytes and releases its object URL',async()=>{
 const h=mediaHarness(),image=await h.media.loadImage('/frame.jpg',h.descriptor,{fetcher:h.fetcher});
 assert.equal(h.urls.created,1);assert.equal(h.urls.revoked.length,0);image.destroy();assert.deepEqual(h.urls.revoked,['blob:1']);
});

test('digest mismatch, decode failure and resolution mismatch cannot become a ready image',async()=>{
 const hash=mediaHarness();await assert.rejects(()=>hash.media.loadImage('/frame.jpg',{...hash.descriptor,jpeg_sha256:'0'.repeat(64)},{fetcher:hash.fetcher}),/해시/);assert.equal(hash.urls.created,0);
 for(const options of [{decodeError:true},{width:5}]){
  const h=mediaHarness(options);await assert.rejects(()=>h.media.loadImage('/frame.jpg',h.descriptor,{fetcher:h.fetcher}));assert.deepEqual(h.urls.revoked,['blob:1']);
 }
});

test('state deadline covers stalled headers or body and a late response cannot restore the old cursor',async()=>{
 for(const stallBody of [false,true]){
  const h=harness(),late=deferred(),update=h.player.update();
  if(stallBody){h.get()[0].resolve({ok:true,json:()=>late.promise});await drain();}
  h.runTimer(4000);await update;
  assert.equal(h.get()[0].options.signal.aborted,true);assert.equal(h.element('sync-status').dataset.state,'error');
  await state(h,view({binding:'c',time:2,sequence:3}));
  const old={view:view(),token:'late-token'};
  if(stallBody)late.resolve(old);else h.get()[0].resolve(response(old));await drain();
  assert.equal(h.player.binding,'c'.repeat(64));assert.equal(h.player.token,'fixture-token');assert.equal(h.viewer.applied.at(-1).sequence,3);
 }
});

const inspector=(await readFile(new URL('../../src/mujoco_ros2_manipulation/operator_console/static/inspector.js',import.meta.url),'utf8'));
const captureScript=inspector.slice(inspector.indexOf('// Capture controls preserve'));
function captureHarness(hidden=false){
 const nodes=new Map(),events={},documentEvents={},requests=[],timers=new Map();let timerID=0,now=10000;
 const node=()=>({textContent:'',disabled:true,hidden:false,children:[],events:{},className:'',
  addEventListener(name,callback){this.events[name]=callback;},replaceChildren(...items){this.children=items;},append(...items){this.children.push(...items);},get childElementCount(){return this.children.length;}});
 const get=key=>{if(!nodes.has(key))nodes.set(key,node());return nodes.get(key);};
 get('analysis').hidden=hidden;const analysisButton=node();
 const document={hidden:false,getElementById:get,createElement:node,querySelectorAll:()=>[analysisButton],
  addEventListener:(name,callback)=>{documentEvents[name]=callback;}};
 const window={addEventListener:(name,callback)=>{events[name]=callback;}};
 const fetcher=(url,options={})=>{const pending=deferred();requests.push({url,options,...pending});return pending.promise;};
 new Function('document','window','fetch','setTimeout','clearTimeout','Date',captureScript)(document,window,fetcher,
  (fn,delay)=>{const key=++timerID;timers.set(key,{fn,delay});return key;},key=>timers.delete(key),{now:()=>now});
 return {get,document,events,documentEvents,requests,timers,analysisButton,advance:()=>{now+=1100;},
  posts:()=>requests.filter(item=>item.options.method==='POST'),gets:()=>requests.filter(item=>!item.options.method),
  click:key=>get(key).events.click()};
}
const catalog=(capture={status:'idle',id:null,reason:null,pose_count:0,image_count:0},recordings=[])=>({capture,recordings,token:'capture-token'});

test('capture catalog exposes only explicit checked recording links and recording start/stop requests',async()=>{
 const h=captureHarness();h.gets()[0].resolve(response(catalog(undefined,[{id,label:'고정 입력',duration_sec:3.5,ended_reason:'user_stop'},{id:'../../other',label:'invalid'}])));await drain();
 assert.equal(h.get('sync-capture-start').disabled,false);assert.equal(h.get('sync-capture-stop').disabled,true);
 assert.equal(h.get('sync-capture-list').children.length,1);
 assert.equal(h.get('sync-capture-list').children[0].children[0].href,'/synchronized?id='+id);
 h.click('sync-capture-start');assert.equal(h.posts().length,1);assert.deepEqual(JSON.parse(h.posts()[0].options.body),{});
 h.posts()[0].resolve(response({status:'recording'}));await drain();
 h.gets().at(-1).resolve(response(catalog({status:'recording',id,pose_count:3,image_count:2})));await drain();
 assert.equal(h.get('sync-capture-start').disabled,true);assert.equal(h.get('sync-capture-stop').disabled,false);
 h.click('sync-capture-stop');assert.equal(h.posts().at(-1).url,'/api/sync-recordings/stop');assert.deepEqual(JSON.parse(h.posts().at(-1).options.body),{id});
 h.posts().at(-1).resolve(response({status:'saving'}));await drain();h.gets().at(-1).resolve(response(catalog({status:'saving',id})));await drain();
 assert.equal(h.get('sync-capture-start').disabled,true);assert.equal(h.get('sync-capture-stop').disabled,true);
 assert.equal(h.requests.some(item=>/command|approve|cancel/.test(item.url)),false);
});

test('capture failure remains visible through status refresh and never retries its POST',async()=>{
 const h=captureHarness();h.gets()[0].resolve(response(catalog()));await drain();h.click('sync-capture-start');
 h.posts()[0].resolve(response({error:'unknown acquisition clock'},false));await drain();
 h.gets().at(-1).resolve(response(catalog()));await drain();
 assert.equal(h.posts().length,1);assert.match(h.get('sync-capture-error').textContent,/unknown acquisition clock/);
 h.advance();h.events['console-observation']();h.gets().at(-1).resolve(response(catalog()));await drain();
 assert.equal(h.posts().length,1);assert.match(h.get('sync-capture-error').textContent,/unknown acquisition clock/);
});

test('hidden analysis does not poll the catalog and loss disables capture controls',async()=>{
 const h=captureHarness(true);assert.equal(h.requests.length,0);h.events['console-observation']();assert.equal(h.requests.length,0);
 h.get('analysis').hidden=false;h.events['console-observation']();h.gets()[0].resolve(response(catalog()));await drain();
 assert.equal(h.get('sync-capture-start').disabled,false);
 h.advance();h.events['console-observation']();h.gets().at(-1).resolve(response({},false));await drain();
 assert.equal(h.get('sync-capture-start').disabled,true);assert.equal(h.get('sync-capture-stop').disabled,true);
 h.events.pagehide();h.advance();h.events['console-observation']();assert.equal(h.requests.length,2);
});


test('default native fetch and timer functions keep their global receiver semantics',async()=>{
 const h=harness(),calls={fetch:0,set:0,clear:0};let timerID=0;
 function nativeFetch(url,options={}){assert.equal(this,undefined,'Native fetch must not receive the player as receiver');calls.fetch++;
  return Promise.resolve(response(options.method==='POST'?view({pose:false,image:false}):{view:view({pose:false,image:false}),token:'native-token'}));
 }
 function nativeSet(fn,delay){'use strict';assert.equal(this,undefined,'Native timer must not receive the timer options object');calls.set++;return ++timerID;}
 function nativeClear(timer){'use strict';assert.equal(this,undefined,'Native timer clear must not receive the timer options object');calls.clear++;}
 const nativeApi=new Function('fetch','setTimeout','clearTimeout',source+'\nreturn {SynchronizedPlayer};')(nativeFetch,nativeSet,nativeClear);
 const player=new nativeApi.SynchronizedPlayer({id,viewer:h.viewer,element:h.element,hidden:()=>false});
 await player.update();assert.equal(player.token,'native-token');assert.equal(h.element('sync-error').hidden,true);
 await player.control('pause');assert.equal(calls.fetch,2);assert.ok(calls.set>=4);assert.ok(calls.clear>=3);
 player.destroy();assert.equal(h.element('sync-play').disabled,true);
});

test('polling during timeline scrubbing preserves the chosen thumb until seek is submitted',async()=>{
 const h=harness();await state(h,view({time:1,sequence:1,playing:true}));
 h.player.scrubbing=true;h.element('sync-seek').value='3';
 h.player.acceptView(view({time:1.05,sequence:2,playing:true}));
 assert.equal(h.element('sync-seek').value,'3');
 const action=h.player.control('seek',Number(h.element('sync-seek').value));
 assert.equal(JSON.parse(h.post().at(-1).options.body).time_sec,3);
 h.post().at(-1).resolve(response(view({binding:'c',time:3,sequence:3})));await action;
 assert.equal(h.player.scrubbing,false);assert.equal(h.element('sync-seek').value,'3');
});
