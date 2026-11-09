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
 const element=key=>{if(!elements.has(key))elements.set(key,{textContent:'',value:'',hidden:false,disabled:false,dataset:{},children:[],append(...values){this.children.push(...values);},replaceChildren(...values){this.children=values;}});return elements.get(key);};
 const viewer={binding:null,hasPose:false,sequence:-1,applied:[],loads:[],destroyed:false,loadGate:null,
  setExpectedBinding(binding){this.binding=binding;this.hasPose=false;this.sequence=-1;},
  async loadManifest(url,options){this.loads.push({url,options});if(this.loadGate)await this.loadGate.promise;},
  updatePose(value){if(value.sequence<=this.sequence)return false;this.applied.push(value);this.sequence=value.sequence;this.hasPose=true;return true;},
  cancelPoseRead(){},destroy(){this.destroyed=true;}};
 const fetcher=(url,options={})=>{const pending=deferred();requests.push({url,options,...pending});return pending.promise;};
 const imageLoader=(url,descriptor,options)=>{const pending=deferred();images.push({url,descriptor,options,...pending});return pending.promise;};
 const player=new api.SynchronizedPlayer({id,viewer,element,fetcher,imageLoader,createElement:eventElement,hidden:()=>hidden,
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
 const nodes=new Map(),events={},documentEvents={},requests=[],timers=new Map(),dispatched=[];let timerID=0,now=10000;
 const node=()=>({textContent:'',disabled:true,hidden:false,children:[],events:{},className:'',
  addEventListener(name,callback){this.events[name]=callback;},replaceChildren(...items){this.children=items;},append(...items){this.children.push(...items);},get childElementCount(){return this.children.length;}});
 const get=key=>{if(!nodes.has(key))nodes.set(key,node());return nodes.get(key);};
 get('analysis').hidden=hidden;const analysisButton=node();
 const document={hidden:false,getElementById:get,createElement:node,querySelectorAll:()=>[analysisButton],
  addEventListener:(name,callback)=>{documentEvents[name]=callback;}};
 const window={addEventListener:(name,callback)=>{events[name]=callback;},dispatchEvent:event=>{dispatched.push(event);}};
 const fetcher=(url,options={})=>{const pending=deferred();requests.push({url,options,...pending});return pending.promise;};
 new Function('document','window','fetch','setTimeout','clearTimeout','Date','CustomEvent',captureScript)(document,window,fetcher,
  (fn,delay)=>{const key=++timerID;timers.set(key,{fn,delay});return key;},key=>timers.delete(key),{now:()=>now},function(type,options){this.type=type;this.detail=options.detail;});
 return {get,document,events,documentEvents,requests,timers,analysisButton,dispatched,advance:()=>{now+=1100;},
  posts:()=>requests.filter(item=>item.options.method==='POST'),gets:()=>requests.filter(item=>!item.options.method),
  click:key=>get(key).events.click()};
}
const catalog=(capture={status:'idle',id:null,reason:null,pose_count:0,image_count:0},recordings=[])=>({capture,recordings,token:'capture-token'});

test('capture catalog exposes only explicit checked recording links and recording start/stop requests',async()=>{
 const h=captureHarness();h.gets()[0].resolve(response(catalog(undefined,[{id,label:'고정 입력',duration_sec:3.5,ended_reason:'user_stop'},{id:'../../other',label:'invalid'}])));await drain();
 assert.equal(h.get('sync-capture-start').disabled,false);assert.equal(h.get('sync-capture-stop').disabled,true);
 assert.equal(h.get('sync-capture-list').children.length,1);
 assert.equal(h.get('sync-capture-list').children[0].children[1].href,'/synchronized?id='+id);
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

test('selecting a saved record requests same-workspace replay without any command POST',async()=>{
 const h=captureHarness();h.gets()[0].resolve(response(catalog(undefined,[{id,label:'보존 기록',duration_sec:3.5}])));await drain();
 const button=h.get('sync-capture-list').children[0].children[0];assert.equal(button.textContent,'보존 기록 · 작업실에서 재생');button.events.click();
 assert.equal(h.dispatched.length,1);assert.equal(h.dispatched[0].type,'workspace-replay-open');assert.deepEqual(h.dispatched[0].detail,{id});
 assert.equal(h.posts().length,0);assert.equal(h.requests.length,1);
});

test('capture mutations pin the observed workspace selection and cannot use a lost epoch',async()=>{
 const h=captureHarness();h.gets()[0].resolve(response(catalog()));await drain();
 h.events['console-observation']({detail:{task:{selection_id:id,grasp:{source:{id:'scene:grasp'}}}}});
 h.click('sync-capture-start');assert.equal(h.posts()[0].options.headers['X-Workspace-Selection'],id);
 h.events['console-observation']({detail:{task:{selection_id:'b'.repeat(32),grasp:{source:{id:'scene:grasp'}}}}});
 assert.equal(h.posts()[0].options.headers['X-Workspace-Selection'],id,'In-flight mutation cannot adopt a later selection');
 h.posts()[0].resolve(response({status:'recording'}));await drain();h.gets().at(-1).resolve(response(catalog({status:'recording',id})));await drain();
 h.events['console-observation-lost']();assert.equal(h.get('sync-capture-stop').disabled,true);const count=h.posts().length;
 h.click('sync-capture-stop');assert.equal(h.posts().length,count);
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


function eventElement(tag){
 return {tag,children:[],textContent:'',dataset:{},attributes:{},events:{},disabled:false,
  append(...items){this.children.push(...items);},setAttribute(name,value){this.attributes[name]=String(value);},
  addEventListener(name,handler){this.events[name]=handler;}};
}
const eventText=node=>[node.textContent,...(node.children||[]).map(eventText)].join(' ');
function taskEvents(){
 return [
  {event_id:'event-1',event_sequence:1,emitted_monotonic_ns:'9007199254740993123',parent_received_monotonic_ns:'9007199254740993223',
   stage:'approach',status:'running',details:{original:'first'},time_sec:0,media_seek_reason:null,
   native_association:{available:true,reason:'fresh_native',stamp_ns:'1000000000',sequence:1,capture_monotonic_ns:'9007199254740993023'}},
  {event_id:'event-2',event_sequence:2,emitted_monotonic_ns:'9007199254740993123',parent_received_monotonic_ns:'9007199254740993224',
   stage:'transport',status:'running',details:{},time_sec:1,media_seek_reason:null,
   native_association:{available:true,reason:'fresh_native',stamp_ns:'2000000000',sequence:2,capture_monotonic_ns:'9007199254740993123'}},
  {event_id:'event-3',event_sequence:3,emitted_monotonic_ns:'9007199254740993124',parent_received_monotonic_ns:'9007199254740993225',
   stage:'cleanup',status:'cancelled',details:{original:'held'},time_sec:null,media_seek_reason:'native_held',
   native_association:{available:false,reason:'native_held',stamp_ns:null,sequence:null,capture_monotonic_ns:null}}
 ];
}
function graspView({events=taskEvents(),selected=['event-1'],unavailable=false,...options}={}){
 const value={...view(options),recording_version:2,recording_kind:'synchronized-grasp-recording-v2',media_state:'ready',media_reason:null,
  events,task_status:'cancelled',acquisition_identity:{job_id:'c'.repeat(32),job_generation:1}};
 value.selection.event_ids=selected;
 if(unavailable){
  value.media_state='unavailable';value.media_reason='before_native';value.origin_ns=null;value.duration_sec=null;value.position_sec=null;value.playing=false;
  value.selection={cursor_ns:null,time_sec:null,pose_status:'missing',image_status:'missing',pose:null,image:null,event_ids:[],
   pose_time_sec:null,image_time_sec:null,pose_delta_sec:null,image_delta_sec:null};
  value.events=events.map(event=>({...event,time_sec:null,media_seek_reason:'before_native',
   native_association:{available:false,reason:'before_native',stamp_ns:null,sequence:null,capture_monotonic_ns:null}}));
 }
 return value;
}
function currentEvents(h){return [...h.player.eventRows].filter(([,entry])=>entry.row.dataset.current==='true').map(([id])=>id);}

test('v2 event highlights commit with decoded media, preserving equal timestamps and original ns',async()=>{
 const h=harness(),events=taskEvents();events[1].time_sec=0;events[1].native_association.stamp_ns='1000000000';
 const value=graspView({events,selected:['event-1','event-2']}),update=h.player.update();
 h.get()[0].resolve(response({view:value,token:'token'}));await drain();
 assert.equal(h.element('sync-events-panel').hidden,false);assert.equal(h.element('sync-events').children.length,2);
 assert.equal(h.element('sync-untimed-events').children.length,1);assert.deepEqual(currentEvents(h),[]);
 assert.match(eventText(h.element('sync-events').children[0]),/9007199254740993123 ns/);
 assert.equal(h.viewer.applied.length,0);
 h.images[0].resolve(decoded());await update;
 assert.deepEqual(currentEvents(h),['event-1','event-2']);assert.equal(h.viewer.applied.length,1);
 assert.equal(h.player.eventRows.get('event-3').button,null);assert.match(eventText(h.player.eventRows.get('event-3').row),/3D 시계 정지/);
});

test('v1 playback hides the event track and keeps its original two-track presentation',async()=>{
 const h=harness();await state(h);assert.equal(h.element('sync-events-panel').hidden,true);
 assert.equal(h.player.eventRows.size,0);assert.equal(h.element('sync-status').dataset.state,'ready');
});

test('event-only diagnostics never load a manifest, apply a pose, fetch media or enable playback',async()=>{
 const h=harness();await state(h,graspView({unavailable:true}));
 assert.equal(h.viewer.loads.length,0);assert.equal(h.viewer.applied.length,0);assert.equal(h.images.length,0);
 for(const key of ['sync-play','sync-pause','sync-restart','sync-previous','sync-next','sync-seek','sync-speed'])assert.equal(h.element(key).disabled,true,key);
 assert.equal(h.element('sync-seek').max,'0');assert.match(h.element('sync-time').textContent,/기록 없음/);
 assert.equal(h.element('sync-untimed-events').children.length,3);assert.equal(h.element('sync-untimed-panel').open,true);assert.deepEqual(currentEvents(h),[]);
 for(const action of ['play','pause','seek','speed','previous','next'])await h.player.control(action,0);
 assert.equal(h.post().length,0);assert.equal(h.player.view.selection.cursor_ns,null);
 assert.match(h.element('sync-identity').textContent,/원래 작업 상태 취소/);
});

test('seek and late old-binding media cannot restore an earlier event highlight',async()=>{
 const h=harness();await state(h,graspView());assert.deepEqual(currentEvents(h),['event-1']);
 const update=h.player.update();h.get().at(-1).resolve(response({view:graspView({time:1,sequence:2,selected:['event-2']}),token:'token'}));await drain();
 const old=decoded(),seek=h.player.control('seek',2);assert.deepEqual(currentEvents(h),[]);
 h.post()[0].resolve(response(graspView({binding:'c',time:2,sequence:3,selected:['event-2']})));await seek;
 h.images.at(-1).resolve(old);await update;assert.equal(old.destroyed,1);assert.deepEqual(currentEvents(h),[]);
 await state(h,graspView({binding:'c',time:2,sequence:3,selected:['event-2']}));assert.deepEqual(currentEvents(h),['event-2']);
});

test('hidden tabs and decode errors clear current events until a matching visible pair commits',async()=>{
 const h=harness();await state(h,graspView());h.setHidden(true);h.player.visibility();assert.deepEqual(currentEvents(h),[]);
 await state(h,graspView({time:1,sequence:2,selected:['event-2'],playing:true}));assert.deepEqual(currentEvents(h),[]);
 h.setHidden(false);h.player.visibility();const update=h.player.update();
 h.get().at(-1).resolve(response({view:graspView({time:1,sequence:2,selected:['event-2']}),token:'token'}));await drain();
 h.images.at(-1).reject(Error('v2 decode failure'));await update;assert.deepEqual(currentEvents(h),[]);
 assert.equal(h.element('sync-status').dataset.state,'error');
 await state(h,graspView({time:1,sequence:2,selected:['event-2']}));assert.deepEqual(currentEvents(h),['event-2']);
 h.player.destroy();assert.equal(h.player.eventRows.size,0);assert.equal(h.element('sync-events-panel').hidden,true);
});

test('event clicks seek only by server relative time and never convert source epochs or send robot commands',async()=>{
 const h=harness(),value=graspView();value.events[1].time_sec=1.234;value.events[1].native_association.stamp_ns='9007199254740993123';
 await state(h,value);const button=h.player.eventRows.get('event-2').button;button.events.click();
 assert.deepEqual(JSON.parse(h.post()[0].options.body),{action:'seek',time_sec:1.234});
 assert.equal(h.post()[0].url,'/api/sync-recordings/'+id+'/control');
 h.post()[0].resolve(response(graspView({events:value.events,binding:'c',time:1.234,selected:['event-2']})));await drain();
 assert.equal(h.requests.some(request=>/^\/api\/(?:command|task|grasp|cancel|approve)/.test(request.url)),false);
 h.player.destroy();const count=h.post().length;button.events.click();assert.equal(h.post().length,count);
});

test('mutated immutable event catalogs and missing selected event identities fail closed',async()=>{
 for(const mutate of [value=>{value.events[0].stage='changed';},value=>{value.selection.event_ids=['unknown'];}]){
  const h=harness();await state(h,graspView());const bad=graspView();mutate(bad);
  const update=h.player.update();h.get().at(-1).resolve(response({view:bad,token:'token'}));await update;
  assert.equal(h.element('sync-status').dataset.state,'error');assert.deepEqual(currentEvents(h),[]);assert.equal(h.post().length,0);
 }
});

const selection='c'.repeat(32),jobA='d'.repeat(32),jobB='e'.repeat(32),ownerA='a'.repeat(64),ownerB='b'.repeat(64);
const captureOwner=(job=jobA,generation=1)=>({selection_id:selection,job_id:job,job_generation:generation,binding_id:'f'.repeat(64),source_registration_epoch:'epoch'});
const captureTask=(job=jobA,generation=1)=>({detail:{task:{selected:'grasp',selection_id:selection,grasp:{job:job===null?null:{job_id:job,generation}}}}});
const graspCapture=(extra={})=>({status:'idle',id:null,pose_count:0,image_count:0,event_count:0,can_arm:true,can_stop:false,
 expected_owner_token:ownerA,expected_owner:captureOwner(),owner_token:null,owner:null,awaiting_sources:true,media_ready:false,...extra});
async function readyCapture(h,capture=graspCapture()){
 h.gets()[0].resolve(response(catalog()));await drain();h.events['console-observation'](captureTask());
 h.gets().at(-1).resolve(response(catalog(capture)));await drain();
}

test('v2 capture echoes the exact expected owner and stops an armed capture with its retained token',async()=>{
 const h=captureHarness();await readyCapture(h);assert.equal(h.get('sync-capture-start').disabled,false);
 h.click('sync-capture-start');assert.deepEqual(JSON.parse(h.posts()[0].options.body),{expected_owner_token:ownerA});
 assert.equal(h.posts()[0].options.headers['X-Workspace-Selection'],selection);
 h.posts()[0].resolve(response({status:'armed',id,owner_token:ownerA}));await drain();
 const armed=graspCapture({status:'armed',id,can_arm:false,can_stop:true,owner_token:ownerA,owner:captureOwner(),expected_owner_token:ownerB,event_count:3});
 h.gets().at(-1).resolve(response(catalog(armed)));await drain();
 assert.equal(h.get('sync-capture-stop').disabled,false);assert.match(h.get('sync-capture-state').textContent,/영상과 3D 관측을 기다리고/);
 h.events['console-observation'](captureTask(null));h.gets().at(-1).resolve(response(catalog(armed)));await drain();
 assert.equal(h.get('sync-capture-stop').disabled,false,'A disappeared source must not remove retained stop ownership');
 h.click('sync-capture-stop');assert.deepEqual(JSON.parse(h.posts().at(-1).options.body),{id,expected_owner_token:ownerA});
 h.posts().at(-1).resolve(response({status:'saving'}));await drain();assert.equal(h.posts().length,2);
});

test('v2 availability for a different current job cannot enable recording under an older observation',async()=>{
 const h=captureHarness();await readyCapture(h,graspCapture({expected_owner:captureOwner(jobB,2),expected_owner_token:ownerB}));
 assert.equal(h.get('sync-capture-start').disabled,true);h.click('sync-capture-start');assert.equal(h.posts().length,0);
 assert.match(h.get('sync-capture-state').textContent,/화면의 작업과 기록 공급원/);
 h.events['console-observation'](captureTask(jobB,2));h.gets().at(-1).resolve(response(catalog(graspCapture({expected_owner:captureOwner(jobB,2),expected_owner_token:ownerB}))));await drain();
 assert.equal(h.get('sync-capture-start').disabled,false);h.click('sync-capture-start');assert.deepEqual(JSON.parse(h.posts()[0].options.body),{expected_owner_token:ownerB});
});

test('a delayed old-job availability cannot restore an owner token after the observed job changes',async()=>{
 const h=captureHarness();await readyCapture(h);h.advance();h.events['console-observation'](captureTask());const old=h.gets().at(-1);
 h.events['console-observation'](captureTask(jobB,2));assert.equal(h.get('sync-capture-start').disabled,true);
 old.resolve(response(catalog(graspCapture())));await drain();assert.equal(h.get('sync-capture-start').disabled,true);
 const fresh=h.gets().at(-1);assert.notEqual(fresh,old);fresh.resolve(response(catalog(graspCapture({expected_owner:captureOwner(jobB,2),expected_owner_token:ownerB}))));await drain();
 assert.equal(h.get('sync-capture-start').disabled,false);h.click('sync-capture-start');assert.deepEqual(JSON.parse(h.posts()[0].options.body),{expected_owner_token:ownerB});
});

test('an ambiguous v2 start response is read back without automatically resending the request',async()=>{
 const h=captureHarness();await readyCapture(h);h.click('sync-capture-start');
 h.posts()[0].reject(Object.assign(Error('lost response'),{name:'AbortError'}));await drain();
 assert.equal(h.posts().length,1);assert.match(h.get('sync-capture-error').textContent,/응답이 지연/);
 h.gets().at(-1).resolve(response(catalog(graspCapture({status:'armed',id,can_arm:false,can_stop:true,owner_token:ownerA,owner:captureOwner()}))));await drain();
 assert.equal(h.posts().length,1);assert.equal(h.get('sync-capture-stop').disabled,false);
});


test('a catalog reply received after observation loss cannot restore v2 recording controls',async()=>{
 const h=captureHarness();await readyCapture(h);h.advance();h.events['console-observation'](captureTask());const pending=h.gets().at(-1);
 h.events['console-observation-lost']();pending.resolve(response(catalog(graspCapture())));await drain();
 assert.equal(h.get('sync-capture-start').disabled,true);assert.equal(h.get('sync-capture-stop').disabled,true);
 h.click('sync-capture-start');assert.equal(h.posts().length,0);
 h.events['console-observation'](captureTask());const fresh=h.gets().at(-1);assert.notEqual(fresh,pending);
 fresh.resolve(response(catalog(graspCapture())));await drain();assert.equal(h.get('sync-capture-start').disabled,false);
});


test('returning to motion uses legacy capture requests after a completed v2 owner is retained',async()=>{
 const h=captureHarness();await readyCapture(h);
 h.events['console-observation']({detail:{task:{selected:'motion',selection_id:'f'.repeat(32),grasp:{job:{job_id:jobA,generation:1}}}}});
 const completed=graspCapture({status:'completed',id,can_start:true,can_arm:false,expected_owner_token:null,expected_owner:null,
  owner_token:ownerA,owner:captureOwner(),can_stop:false});
 h.gets().at(-1).resolve(response(catalog(completed)));await drain();assert.equal(h.get('sync-capture-start').disabled,false);
 h.click('sync-capture-start');assert.deepEqual(JSON.parse(h.posts()[0].options.body),{});
 assert.equal(h.posts()[0].options.headers['X-Workspace-Selection'],'f'.repeat(32));
});


test('native-associated events outside the retained media remain visible without a seek action',async()=>{
 const h=harness(),events=taskEvents();events[1].time_sec=null;events[1].media_seek_reason='outside_media';
 events[1].native_association.stamp_ns='9007199254740993123';
 await state(h,graspView({events}));const row=h.player.eventRows.get('event-2');
 assert.equal(row.button,null);assert.match(eventText(row.row),/9007199254740993123 ns/);
 assert.match(eventText(row.row),/보존한 재생 구간 밖/);assert.equal(h.element('sync-untimed-events').children.length,2);
 assert.equal(h.post().length,0);
});

test('an armed grasp without its retained stop token never falls back to a legacy stop request',async()=>{
 const h=captureHarness();await readyCapture(h,graspCapture({status:'armed',id,can_stop:true,owner_token:null}));
 assert.equal(h.get('sync-capture-stop').disabled,true);h.click('sync-capture-stop');assert.equal(h.posts().length,0);
});


test('a late capture mutation error cannot replace feedback for another observed job',async()=>{
 const h=captureHarness();await readyCapture(h);h.click('sync-capture-start');
 h.events['console-observation'](captureTask(jobB,2));
 h.posts()[0].resolve(response({error:'old owner rejected'},false));await drain();
 assert.equal(h.get('sync-capture-error').textContent,'');assert.equal(h.posts().length,1);
 h.gets().at(-1).resolve(response(catalog(graspCapture({expected_owner_token:ownerB,expected_owner:captureOwner(jobB,2)}))));await drain();
 assert.equal(h.get('sync-capture-start').disabled,false);assert.equal(h.posts().length,1);
});


test('known v2 completion and failure reasons use clear Korean recording feedback',async()=>{
 for(const [reason,label] of [['task_finished','작업 종료에 따라 기록 종료'],['recording_source_fault','기록 공급원 오류'],
  ['source_journal_limit','작업 이벤트·자세 보존 상한 도달'],['insufficient_paired_native_and_image_observations','영상과 3D 표본을 함께 확보하지 못했습니다']]){
  const h=captureHarness();h.gets()[0].resolve(response(catalog({status:'completed',id,reason,task_status:'cancelled'})));await drain();
  assert.ok(h.get('sync-capture-state').textContent.includes(label));assert.match(h.get('sync-capture-state').textContent,/원래 작업 취소/);
 }
});


test('advanced v2 provenance distinguishes retained queue exclusions and source sequence gaps',async()=>{
 const h=harness(),value=graspView();value.pose_sample_semantics='parent_accepted_latest_ipc';
 value.observation_counters={poses_dropped:2,events_dropped:3,poses_source_gaps:4,image_transport_gaps:5,image_counters:{received:20,encoded:10}};
 await state(h,value);const info=h.element('sync-identity').textContent;
 assert.match(info,/부모가 수용한 최신 IPC 관측/);assert.match(info,/부모 자세 큐 제외 2개/);
 assert.match(info,/원천 자세 순번 건너뜀 4개/);assert.match(info,/이벤트 큐 제외 3개/);assert.match(info,/영상 전송 누락 5개/);
 assert.ok(info.includes('"received":20'));assert.equal(h.post().length,0);
});
