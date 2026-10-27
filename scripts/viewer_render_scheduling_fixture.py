#!/usr/bin/env python3
"""Isolated read-only LocalSceneView scheduling observation; never connects to ROS.

The selected static source files are frozen in memory at server startup so a
later checkout edit cannot silently change the code behind an open baseline tab.
Restart with --root pointing at another checkout for an after-change comparison.
"""

from argparse import ArgumentParser
from hashlib import sha256
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import json
from pathlib import Path
import subprocess
from urllib.parse import urlsplit


DEFAULT_ROOT = Path(__file__).resolve().parents[1] / 'src/mujoco_ros2_manipulation/operator_console/static'
STATIC_FILES = ('viewer.js', 'viewer_contract.mjs', 'vendor/OrbitControls.js',
                'vendor/three.module.js', 'vendor/three.core.js')


def encode(value):
    return json.dumps(value, sort_keys=True, separators=(',', ':'), ensure_ascii=False).encode('utf-8')


MANIFEST = {
    'schema_version': 1, 'export_version': 'compiled-scene-v1',
    'model_sha256': '1' * 64, 'scene_sha256': '2' * 64,
    'compiled_model_sha256': '3' * 64,
    'units': {'length': 'm', 'angle': 'rad', 'up_axis': 'Z', 'rotation': 'row-major'},
    'joint_names': [], 'home_rad': {}, 'default_visible_groups': [0, 1, 2],
    'geoms': [
        {'id': 0, 'name': 'fixture_cube', 'type': 'box', 'group': 0,
         'default_visible': True, 'size': [.18, .18, .18],
         'rgba': [.2, .65, .85, 1], 'asset_sha256': None},
        {'id': 1, 'name': 'fixture_base', 'type': 'box', 'group': 0,
         'default_visible': True, 'size': [.8, .8, .04],
         'rgba': [.35, .38, .4, 1], 'asset_sha256': None},
    ], 'assets': [],
}
MANIFEST_BYTES = encode(MANIFEST)

HTML = '''<!doctype html>
<html lang="ko"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>격리 뷰어 제출 횟수 관찰</title>
<script type="importmap">{"imports":{"three":"/vendor/three.module.js"}}</script>
<style>
*{box-sizing:border-box}body{margin:20px;background:#edf2f4;color:#142d38;font:15px/1.5 system-ui,sans-serif}
main{max-width:1050px;margin:auto}h1{font-size:25px}button{padding:10px 15px;margin:4px 8px 8px 0;font:inherit;cursor:pointer}
.note{padding:12px;border-left:4px solid #008090;background:white}.code{font:12px/1.5 monospace;overflow-wrap:anywhere}
#scene{width:640px;height:480px;position:relative;background:#17242a;border:1px solid #54717b;max-width:100%}
.local-scene-status{position:absolute;left:8px;bottom:8px;color:white;background:#122831d9;padding:5px 8px;font-size:12px}
pre{padding:12px;background:#fff;white-space:pre-wrap;overflow-wrap:anywhere;font-size:12px}#status{font-weight:600}
td,th{padding:7px 10px;border:1px solid #c9d6dc;text-align:left}table{border-collapse:collapse;background:#fff;margin-top:12px}
</style></head><body><main>
<h1>격리 읽기 전용 뷰어 · 3D 렌더 제출 횟수 관찰</h1>
<p class="note">단순 큐브와 받침만 사용합니다. 실제 로봇 성능평가가 아닙니다. ROS 연결 없음 · 로봇 명령 <strong>0</strong>.<br>
측정값은 실제 LocalSceneView의 CPU 렌더 제출입니다. GPU 완료·실제 화면 paint·VRAM·CPU 사용률을 측정하지 않습니다.</p>
<p id="source" class="code">코드 출처 확인 중</p>
<details open><summary>실제 WebGL context 정보 · 시작 시 한 번 조회</summary><pre id="gl-info">확인 중</pre></details>
<div><button id="static" disabled>정지 장면 10초 측정</button><button id="moving" disabled>4 Hz 자세 입력 10초 측정</button><button id="download" disabled>결과 JSON 저장</button></div>
<p id="status" role="status">장면 준비 중</p>
<p>각 측정은 동일한 카메라로 재설정하고 입력을 잠근 뒤 1초간 damping을 정리합니다. 측정 중 이 탭을 유지하세요.</p>
<div id="scene" aria-label="단순 도형 측정 장면"></div>
<table><thead><tr><th>조건</th><th>제출 횟수 / 초</th><th>제출된 고유 자세</th><th>입력된 자세</th><th>CPU 제출 p95</th><th>유효성</th></tr></thead><tbody id="rows"></tbody></table>
<h2>최신 뷰어 메트릭</h2><pre id="metrics">아직 없음</pre>
<h2>측정 결과 JSON</h2><pre id="results">준비 중</pre>
<script type="module" src="/fixture.js"></script></main></body></html>'''.encode('utf-8')

SCRIPT = r'''
import {LocalSceneView} from '/viewer.js';
const $=id=>document.getElementById(id);
const metadata=await (await fetch('/fixture/metadata',{cache:'no-store'})).json();
const bindingID='b'.repeat(64);
const binding={operator_session_id:'a'.repeat(32),generation:1,mode:'model_preview',source_id:'manual',job_id:null,run_id:null,motion_session_id:null,request_id:null};
let sequence=0,latestMetrics=null,run=null,preparing=false,sourceTimer=null,results=[];
const limitations=['Two analytic boxes, not a robot or sensor dataset.',
 'Pose delivery is in-memory schema-1 updatePose at nominal 4 Hz; no network, ROS or control pipeline is timed.',
 'onRenderFrame observes CPU render submission, not GPU completion or physical display paint.',
 'No CPU profile, VRAM measurement, or fresh-browser repetition is performed.',
 'Page visibility changes, resize, or context loss invalidate an active observation.',
 'The existing onRenderFrame hook is enabled in both comparison versions.'];
function percentile95(values){if(!values.length)return null;const ordered=[...values].sort((a,b)=>a-b);return ordered[Math.ceil(ordered.length*.95)-1];}
function bundle(){return {fixture_version:1,robot_commands:0,metadata,graphics_info:graphicsInfo,limitations,results};}
function showResults(){$('results').textContent=JSON.stringify(bundle(),null,2);$('download').disabled=results.length===0;}
function poseAt(motionTime=0){
 const angle=motionTime*.7,c=Math.cos(angle),s=Math.sin(angle);
 return {schema_version:1,binding_id:bindingID,binding,sequence:++sequence,
  source_state_age_sec:null,simulation_sec:null,
  pose:{schema_version:1,model_sha256:'1'.repeat(64),scene_sha256:'2'.repeat(64),geoms:[
   {id:0,position:[.35*Math.sin(motionTime),0,.25],rotation:[c,-s,0,s,c,0,0,0,1]},
   {id:1,position:[0,0,0],rotation:[1,0,0,0,1,0,0,0,1]}]}};
}
const viewer=new LocalSceneView($('scene'),{
 onRenderFrame:sample=>{const active=run;if(!active||performance.now()>=active.end||!sample.displaying_pose)return;
  active.submissions++;active.uniqueSequences.add(sample.sequence);
  if(Number.isFinite(sample.render_submit_ms)&&active.cpuSubmitMs.length<2000)active.cpuSubmitMs.push(sample.render_submit_ms);},
 onMetrics:metrics=>{latestMetrics=metrics;$('metrics').textContent=JSON.stringify(metrics,null,2);},
 onContextLost:()=>{if(run)run.invalid.add('graphics context lost');$('status').textContent='그래픽 context 손실: 페이지를 다시 여세요.';}
});
const graphicsInfo=(()=>{
 try{
  const gl=viewer.renderer.getContext(),debug=gl.getExtension('WEBGL_debug_renderer_info');
  return {available:true,renderer:gl.getParameter(gl.RENDERER),vendor:gl.getParameter(gl.VENDOR),
   version:gl.getParameter(gl.VERSION),shading_language_version:gl.getParameter(gl.SHADING_LANGUAGE_VERSION),
   unmasked_renderer:debug?gl.getParameter(debug.UNMASKED_RENDERER_WEBGL):null,
   unmasked_vendor:debug?gl.getParameter(debug.UNMASKED_VENDOR_WEBGL):null,
   debug_renderer_info_available:Boolean(debug),context_attributes:gl.getContextAttributes(),
   note:'One-time context identity from this renderer, not GPU timing, hardware utilization or VRAM usage.'};
 }catch(error){return {available:false,error:String(error.message||error)};}
})();
$('gl-info').textContent=JSON.stringify(graphicsInfo,null,2);
const codeHash=metadata.source_sha256['viewer.js'];
$('source').textContent=`${metadata.label} | git ${metadata.git_head||'unavailable'} | viewer.js SHA256 ${codeHash} | 서버 시작 시 코드 스냅샷`;
function invalidate(reason){if(run)run.invalid.add(reason);}
document.addEventListener('visibilitychange',()=>invalidate('document visibility changed'));
globalThis.addEventListener('resize',()=>invalidate('browser viewport resized'));
const fixedRect=$('scene').getBoundingClientRect();
let observedWidth=fixedRect.width,observedHeight=fixedRect.height;
const sizeObserver=new ResizeObserver(entries=>{for(const entry of entries){const {width,height}=entry.contentRect;if(Math.abs(width-observedWidth)>1||Math.abs(height-observedHeight)>1)invalidate('scene CSS dimensions changed');observedWidth=width;observedHeight=height;}});
sizeObserver.observe($('scene'));
function addRow(result){const row=document.createElement('tr');for(const text of [result.mode,
 `${result.render_submissions} / ${result.render_submissions_per_sec.toFixed(3)}`,
 String(result.unique_submitted_sequences),String(result.applied_pose_count),
 result.cpu_render_submit_ms_p95===null?'표본 없음':result.cpu_render_submit_ms_p95.toFixed(3)+' ms',
 result.valid?'유효':result.invalid_reasons.join(', ')]){const cell=document.createElement('td');cell.textContent=text;row.append(cell);}$('rows').append(row);}
function finish(){
 const active=run;if(!active)return;clearInterval(sourceTimer);sourceTimer=null;clearTimeout(active.finishTimer);run=null;
 viewer.controls.enabled=true;
 const measured=Math.max(1,Math.min(performance.now(),active.end)-active.started);
 const result={mode:active.mode,requested_duration_ms:10000,observed_duration_ms:measured,
  completion_timer_delay_ms:Math.max(0,performance.now()-active.end),render_submissions:active.submissions,
  render_submissions_per_sec:active.submissions*1000/measured,unique_submitted_sequences:active.uniqueSequences.size,
  applied_pose_count:active.appliedPoses,applied_poses_per_sec:active.appliedPoses*1000/measured,
  cpu_render_submit_ms_p95:percentile95(active.cpuSubmitMs),cpu_render_submit_sample_count:active.cpuSubmitMs.length,
  valid:active.invalid.size===0,invalid_reasons:[...active.invalid],scene_css_pixels:active.scenePixels,
  canvas_pixels:active.canvasPixels,device_pixel_ratio:devicePixelRatio,camera:active.camera,
  latest_viewer_metrics:latestMetrics,measurement_endpoint:'actual LocalSceneView onRenderFrame CPU submission'};
 results.push(result);if(results.length>12)results.shift();addRow(result);showResults();
 $('status').textContent=`${active.mode} 완료: ${result.render_submissions}회 제출, CPU p95 ${result.cpu_render_submit_ms_p95===null?'표본 없음':result.cpu_render_submit_ms_p95.toFixed(3)+' ms'}${result.valid?'':' · 무효 조건 확인'}`;
 $('static').disabled=$('moving').disabled=false;
}
async function start(mode){
 if(run||preparing)return;preparing=true;$('static').disabled=$('moving').disabled=true;
 clearInterval(sourceTimer);sourceTimer=null;viewer.controls.enabled=false;
 viewer.updatePose(poseAt(0));viewer.resetCamera();
 $('status').textContent='입력을 잠그고 damping 정리 중 (1초)…';
 await new Promise(resolve=>setTimeout(resolve,1000));
 if(document.hidden||!viewer.sized||viewer.contextLost){preparing=false;viewer.controls.enabled=true;$('status').textContent='보이는 장면에서 다시 측정하세요.';$('static').disabled=$('moving').disabled=false;return;}
 const now=performance.now(),rect=$('scene').getBoundingClientRect();
 run={mode,started:now,end:now+10000,submissions:0,uniqueSequences:new Set(),cpuSubmitMs:[],appliedPoses:0,invalid:new Set(),
  camera:viewer.cameraSnapshot(),scenePixels:[rect.width,rect.height],canvasPixels:[viewer.canvas.width,viewer.canvas.height]};
 preparing=false;$('status').textContent=`${mode} 측정 중 (10초) · 탭과 창 크기를 유지하세요.`;
 if(mode==='moving-4Hz'){
  const emit=()=>{const active=run;if(!active||performance.now()>=active.end)return;const elapsed=(performance.now()-active.started)/1000;
   if(viewer.updatePose(poseAt(elapsed)))active.appliedPoses++;};
  emit();sourceTimer=setInterval(emit,250);
 }
 run.finishTimer=setTimeout(finish,10000);
}
$('static').addEventListener('click',()=>start('static'));
$('moving').addEventListener('click',()=>start('moving-4Hz'));
$('download').addEventListener('click',()=>{const link=document.createElement('a'),url=URL.createObjectURL(new Blob([JSON.stringify(bundle(),null,2)+'\n'],{type:'application/json'}));link.href=url;link.download='viewer-render-scheduling-'+metadata.label+'.json';link.click();setTimeout(()=>URL.revokeObjectURL(url),1000);});
await viewer.loadManifest('/fixture/manifest',{binding_id:bindingID,binding,
 sha256:metadata.manifest_sha256,model_sha256:'1'.repeat(64),scene_sha256:'2'.repeat(64)});
viewer.updatePose(poseAt(0));
$('status').textContent='준비 완료 · 조건 버튼을 누르면 1초 정리 후 10초간 측정합니다.';
$('static').disabled=$('moving').disabled=false;showResults();
globalThis.addEventListener('pagehide',()=>{clearInterval(sourceTimer);if(run)clearTimeout(run.finishTimer);sizeObserver.disconnect();viewer.destroy();},{once:true});
'''


def main():
    parser = ArgumentParser(description=__doc__)
    parser.add_argument('--root', type=Path, default=DEFAULT_ROOT)
    parser.add_argument('--port', type=int, default=0)
    parser.add_argument('--label', default='manual-observation')
    args = parser.parse_args()
    root = args.root.resolve(strict=True)
    files = {'/' + name: (root / name).read_bytes() for name in STATIC_FILES}
    git = subprocess.run(['git', '-C', str(root), 'rev-parse', 'HEAD'],
                         capture_output=True, text=True, check=False)
    metadata = {
        'label': args.label, 'static_root': str(root),
        'git_head': git.stdout.strip() if git.returncode == 0 else None,
        'source_sha256': {name.lstrip('/'): sha256(content).hexdigest()
                          for name, content in files.items()},
        'manifest_sha256': sha256(MANIFEST_BYTES).hexdigest(),
        'fixture_python_sha256': sha256(Path(__file__).read_bytes()).hexdigest(),
        'fixture_javascript_sha256': sha256(SCRIPT.encode('utf-8')).hexdigest(),
        'fixture_html_sha256': sha256(HTML).hexdigest(),
        'source_snapshot_policy': 'Static files frozen in server memory at startup.',
        'nominal_pose_frequency_hz': 4, 'warmup_ms': 1000, 'measurement_ms': 10000,
    }

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_args):
            pass

        def send(self, content, media_type='application/json', status=200):
            self.send_response(status)
            self.send_header('Content-Type', media_type + '; charset=utf-8')
            self.send_header('Content-Length', str(len(content)))
            self.send_header('Cache-Control', 'no-store')
            self.send_header('X-Content-Type-Options', 'nosniff')
            self.end_headers()
            try:
                self.wfile.write(content)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def do_GET(self):
            route = urlsplit(self.path).path
            if route == '/':
                return self.send(HTML, 'text/html')
            if route == '/fixture.js':
                return self.send(SCRIPT.encode('utf-8'), 'text/javascript')
            if route == '/fixture/metadata':
                return self.send(encode(metadata))
            if route == '/fixture/manifest':
                return self.send(MANIFEST_BYTES)
            if route in files:
                return self.send(files[route], 'text/javascript')
            return self.send(encode({'error': 'Fixed read-only fixture routes only.'}), status=404)

        def do_POST(self):
            return self.send(encode({'error': 'No mutations or robot API are implemented.'}), status=405)

    server = ThreadingHTTPServer(('127.0.0.1', args.port), Handler)
    print(json.dumps({'url': f'http://127.0.0.1:{server.server_port}/', **metadata}, ensure_ascii=False), flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()


if __name__ == '__main__':
    main()
