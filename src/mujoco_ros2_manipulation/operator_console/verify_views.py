"""Actual loopback CPU-scene/ROS acceptance, without a browser or GL renderer.

Starts one fresh owned console; reads existing generated models only. Runtime
records stay in VerificationReport's new ignored directory. No raw publication,
model rebuild, arbitrary attach, EGL, JPEG or native GUI is requested here.
"""
import argparse
from copy import deepcopy
import hashlib
import json
import math
import os
from pathlib import Path
import re
import signal
import socket
import subprocess
import time
from urllib.error import HTTPError, URLError
from urllib.parse import urlencode
from urllib.request import Request, urlopen

from ..model_build.integrity import file_sha256
from ..paths import ROOT
from ..profiles import check_generated_model, load_profile
from ..verification.diagnostics import check_physics
from ..verification.reports import VerificationReport, atomic_json, utc_now
from ..verification.runner import group_gone, stop_process
from ..workcells.config import load_workcell
from ..workcells.contracts import describe_workcell
from ..workcells.integrity import check_workcell
from ..workcells.motion_request import command
from .demo_jobs import matching_process, process_identity
from .scene_host import SceneSource, _manifest
from .view_binding import binding_id, envelope
from .view_plan import SavedPlan


MAX_JSON = 8*1024*1024
MAX_TRACE = 32*1024*1024
MAX_ASSET = 48*1024*1024
MAX_TOTAL_ASSETS = 128*1024*1024
WORKER_MODULE = 'mujoco_ros2_manipulation.operator_console.scene_worker'


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def digest(raw):
    return hashlib.sha256(raw).hexdigest()


def bounded_file(path, maximum):
    require(path.is_file() and not path.is_symlink() and path.stat().st_size <= maximum,
            'Owned verification source is missing, linked or too large')
    raw=path.read_bytes()
    require(len(raw)<=maximum,'Owned verification source grew beyond its limit')
    return raw


def checked_pose(value, manifest, binding, *, playback=None, live=False, allow_stale=False):
    require(type(value) is dict and value.get('binding_id')==binding_id(binding), 'CPU pose belongs to another binding')
    checked=envelope(binding,value['pose'],value['sequence'],
        source_state_age_sec=value.get('source_state_age_sec'),simulation_sec=value.get('simulation_sec'),
        preview_time_sec=value.get('preview_time_sec'))
    require(value==checked,'CPU pose contains a different or incomplete identity envelope')
    pose=value['pose']
    require(pose['model_sha256']==manifest['model_sha256'] and pose['scene_sha256']==manifest['scene_sha256']
            and len(pose['geoms'])==len(manifest['geoms']),'CPU pose compiled model/catalog changed')
    if live:
        require(type(value['source_state_age_sec']) in (int,float) and 0<=value['source_state_age_sec'] and (allow_stale or value['source_state_age_sec']<.5)
                and value['simulation_sec'] is not None,'CPU live pose is stale or missing its simulation stamp')
    else:
        require(value['source_state_age_sec'] is None,'Preview invented a live source age')
    if playback is not None:
        require(abs(value['preview_time_sec']-playback)<=1e-8,'CPU preview uses another playback time')
    return value


def scene_workers(console_pid, session, *, proc=Path('/proc')):
    """Identify only direct children running our exact worker in our new folder."""
    try:
        threads=list((proc/str(console_pid)/'task').iterdir())
    except FileNotFoundError:
        return []
    children=set()
    for thread in threads:
        try:
            children.update((thread/'children').read_text().split())
        except FileNotFoundError:
            continue
    result=[]
    root=(session/'scene-view').resolve()
    for value in sorted(children,key=int):
        pid=int(value)
        try:
            raw=bounded_file(proc/str(pid)/'cmdline',65536)
            argv=[item.decode() for item in raw.split(b'\0') if item]
            if WORKER_MODULE not in argv:
                continue
            directory=Path(argv[argv.index('--directory')+1]).resolve()
            identity=process_identity(pid)
            require(directory.is_relative_to(root) and directory.is_dir() and not directory.is_symlink()
                    and identity is not None and identity['ppid']==console_pid
                    and identity['pgid']==pid and identity['sid']==pid,'Scene worker ownership proof failed')
            result.append(dict(identity,directory=str(directory)))
        except (FileNotFoundError,ProcessLookupError):
            continue  # A replaced owned child may disappear while being inspected.
    return result


def checked_demo_launch(launch, variant, *, root, base_model):
    """Validate the exact already-owned launch file, never discover another model."""
    require(type(launch) is dict and variant in ('contact','control','prepare','baseline','timing'),
            'Unknown demonstration launch/variant')
    run=launch.get('run_id')
    require(isinstance(run,str) and re.fullmatch(r'[0-9]{8}T[0-9]{6}\.[0-9]{6}Z-[0-9a-f]{8}',run),
            'Malformed fresh demonstration run identity')
    directory=Path(root)/'runtime/validation/runs'/run
    require(Path(launch['report_path']).resolve()==(directory/'result.json').resolve(),
            'Demonstration report escaped its exact run directory')
    expected=directory/'contact_test.xml' if variant in ('contact','control') else Path(base_model)
    model=Path(launch['model_path'])
    require(model.is_absolute() and not model.is_symlink() and model.resolve()==expected.resolve(),
            'Demonstration model is not the exact allowed run/model file')
    require(type(launch.get('pid')) is int and launch['pid']>1
            and type(launch.get('start_ticks')) is int and launch['start_ticks']>0,
            'Demonstration launch has no stable owned process identity')
    sha=launch.get('model_sha256')
    require(isinstance(sha,str) and re.fullmatch('[0-9a-f]{64}',sha)
            and digest(bounded_file(model,16*1024*1024))==sha,
            'Demonstration model bytes changed or differ from the owned launch')
    return model.resolve()


def checked_demo_binding(view, *, operator_session_id, source_id, job_id, run_id, model_sha256):
    """A transition may be blank; it may never show the previous job/run pose."""
    require(view.get('display_mode')=='web3d' and not view.get('error') and view.get('status')!='error',
            'Demonstration CPU view failed or selected a renderer')
    binding=view.get('binding');identifier=view.get('binding_id')
    if binding is None or identifier is None:
        require(binding is None and identifier is None and not view.get('pose_ready')
                and not view.get('manifest_descriptor'),'Blank demonstration view retained old compiled data')
        return None
    require(binding_id(binding)==identifier and binding['operator_session_id']==operator_session_id
            and binding['mode']=='live' and binding['source_id']==source_id,
            'Demonstration view belongs to another console/source/mode')
    if binding['job_id'] is None:
        require(binding['run_id'] is None and not view.get('pose_ready') and not view.get('manifest_descriptor'),
                'Waiting demonstration binding retained a previous model or pose')
        return None
    require(run_id is not None and binding['job_id']==job_id and binding['run_id']==run_id,
            'Demonstration view mixed a previous job/run into the current transition')
    require(model_sha256 is not None,'Demonstration view appeared before its owned model was verified')
    descriptor=view.get('manifest_descriptor')
    if descriptor:
        require(descriptor.get('model_sha256')==model_sha256,'Demonstration manifest uses another run model')
    return binding


def checked_pose_progress(previous, current):
    if previous is not None:
        require(current['binding_id']==previous['binding_id'] and current['sequence']>=previous['sequence']
                and current['simulation_sec']>=previous['simulation_sec'],'CPU run pose identity/sequence/time went backwards')
        if current['sequence']==previous['sequence']:
            require(current['pose']==previous['pose'] and current['simulation_sec']==previous['simulation_sec']
                    and current['source_state_age_sec']>=previous['source_state_age_sec'],
                    'CPU pose bytes changed or source age reset without a new source sequence')
    return current


def checked_final_read(view, pose, expected_binding_id):
    require(view.get('status')=='ready' and view.get('pose_ready') is True
            and view.get('binding_id')==expected_binding_id and pose is not None
            and pose.get('binding_id')==expected_binding_id,
            'Completed comparison cleared its last actual CPU view or returned no current pose')
    return pose


def checked_retained_pose(first, current, *, minimum_growth=.5):
    require(current['binding_id']==first['binding_id'] and current['binding']==first['binding']
            and current['sequence']==first['sequence'] and current['pose']==first['pose']
            and current['simulation_sec']==first['simulation_sec'],
            'Completed demonstration cleared or replaced its last actual pose')
    require(current['source_state_age_sec']>.5
            and current['source_state_age_sec']-first['source_state_age_sec']>=minimum_growth,
            'Completed demonstration reset/froze source age instead of retaining its original timestamp')
    return current


def ready_view_state(state, source_id, mode, *, live=False):
    """A ready command server may precede the scene monitor's new run epoch."""
    view = state['view']
    require(view.get('status') != 'error', 'CPU scene host reported an error')
    binding = view.get('binding') or {}
    ready = (view.get('status') == 'ready' and view.get('binding_id')
        and binding.get('source_id') == source_id and view.get('mode') == mode
        and view.get('manifest_descriptor'))
    if live:
        motion = (state.get('server') or {}).get('session_id')
        ready = ready and motion is not None and binding.get('motion_session_id') == motion and view.get('pose_ready') is True
    return state if ready else None


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--domain',type=int,default=168)
    parser.add_argument('--port',type=int,default=8884)
    parser.add_argument('--with-demos',action='store_true',help='Also verify fresh contact/timing CPU views and retained final poses')
    args=parser.parse_args()
    if not 0<=args.domain<=232 or not 1024<=args.port<=65535:
        parser.error('Invalid isolated domain or loopback port')
    report=VerificationReport(ROOT,'operator_views','cpu',args.domain)
    process=session=None
    token=None  # Never printed or recorded in verification evidence.
    failed=False;cleanup=[];workers={};checks=[];models={};initial=None;verified_views={};demo_jobs=[];all_demo_runs=set()
    url=f'http://127.0.0.1:{args.port}'

    def http(route,body=None,maximum=MAX_JSON):
        request=Request(url+route) if body is None else Request(url+route,
            data=json.dumps(body,allow_nan=False).encode(),headers={'Content-Type':'application/json',
                'Origin':url,'X-Operator-Token':token})
        try:
            with urlopen(request,timeout=5) as response:
                raw=response.read(maximum+1)
                require(len(raw)<=maximum,'Loopback acceptance response exceeded its bound')
                return response.status,response.headers.get_content_type(),raw
        except HTTPError as error:
            error.verification_route=route
            raise

    def api(route,body=None):
        status,mime,raw=http(route,body)
        require(status in (200,202) and mime=='application/json','Unexpected console JSON response')
        return json.loads(raw)

    def until(predicate,timeout,label):
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            require(process is not None and process.poll() is None,'Owned console exited before '+label)
            try:
                value=predicate()
                if value:return value
            except HTTPError:
                raise
            except (URLError,TimeoutError):
                pass
            time.sleep(.05)
        raise RuntimeError('Timed out: '+label)

    def reject(route,body,code,*,preview=False):
        try:
            api(route,body)
        except HTTPError as error:
            require(error.code==code,'Wrong HTTP rejection for '+route)
            if preview:
                payload=json.loads(error.read(65537))
                require('미리보기' in payload.get('error',''),'Request was rejected for another reason, not preview admission')
        else:
            raise RuntimeError('Unexpectedly accepted '+route)

    def observe_workers():
        current=scene_workers(process.pid,session)
        for value in current:workers[(value['pid'],value['start_ticks'])]=value
        return current

    def no_image_worker(state):
        require(state['view']['display_mode']=='web3d' and not state['lifecycle']['owned'].get('renderer'),
                'CPU-only acceptance unexpectedly selected an image worker')
        require(not (session/'frame.jpg').exists(),'CPU-only acceptance generated a JPEG')
        events=[json.loads(line) for line in bounded_file(session/'events.jsonl',16*1024*1024).splitlines()]
        require(not any(row.get('action')=='process_started' and row.get('kind')=='renderer' for row in events),
                'An EGL/JPEG renderer was spawned during CPU-only acceptance')

    def ready_view(source_id,mode,timeout=35,*,live=False):
        def ready():
            state=api('/api/state');no_image_worker(state)
            return ready_view_state(state,source_id,mode,live=live)
        state=until(ready,timeout,'ready CPU scene '+source_id)
        return check_view(state,source_id,mode)

    def check_view(state,source_id,mode,*,source=None):
        view=state['view'];binding=view['binding'];identifier=view['binding_id'];descriptor=view['manifest_descriptor']
        require(binding_id(binding)==identifier and binding['operator_session_id']==state['lifecycle']['session_id']
                and binding['mode']==mode,
                'View belongs to another console or has a malformed binding')
        source=source or models[source_id]
        if identifier in verified_views:return verified_views[identifier]
        status,mime,raw=http('/api/view/manifest?'+urlencode({'binding_id':identifier}))
        require(status==200 and mime=='application/json' and digest(raw)==descriptor['sha256']
                and len(raw)==descriptor['byte_length'],'Manifest bytes differ from the current descriptor')
        manifest,catalog=_manifest(raw,source,file_sha256(source.xml_path))
        require(descriptor['model_sha256']==manifest['model_sha256'] and descriptor['scene_sha256']==manifest['scene_sha256']
                and descriptor['compiled_model_sha256']==(manifest.get('compiled_model_sha256') or manifest['provenance']['engine']['compiled_sha256']),'Manifest compiled identity mismatch')
        assets=[];total=0
        for sha,entry in catalog.items():
            status,mime,blob=http('/api/view/assets/'+sha,maximum=MAX_ASSET)
            require(status==200 and mime=='application/octet-stream' and len(blob)==entry['byte_length']
                    and digest(blob)==sha,'Compiled asset bytes differ from the manifest')
            total+=len(blob);require(total<=MAX_TOTAL_ASSETS,'Compiled assets exceed acceptance byte budget')
            assets.append(dict(sha256=sha,bytes=len(blob)))
        owned=observe_workers();require(len(owned)==1,'Expected exactly one current owned CPU scene worker')
        folder=report.directory/'view-evidence'/f'{len(checks):02d}'
        folder.mkdir(parents=True)
        (folder/'manifest.json').write_bytes(raw)
        result=dict(source_id=source_id,mode=mode,binding_id=identifier,binding=deepcopy(binding),
            manifest_sha256=digest(raw),model_sha256=manifest['model_sha256'],scene_sha256=manifest['scene_sha256'],
            compiled_model_sha256=(manifest.get('compiled_model_sha256') or manifest['provenance']['engine']['compiled_sha256']),geoms=len(manifest['geoms']),assets=assets,
            asset_bytes=total,worker_identity={k:v for k,v in owned[0].items() if k!='directory'})
        checks.append(result);report.update(view_checks=checks)
        verified_views[identifier]=(state,manifest,folder)
        return state,manifest,folder

    def pose_for(state,manifest,*,playback=None,live=False,after=0):
        identifier=state['view']['binding_id'];binding=state['view']['binding']
        def current():
            status,_,raw=http('/api/view/pose?'+urlencode({'binding_id':identifier}))
            if status==204:
                require(not raw,'204 CPU pose carried a body');return None
            require(status==200,'Unexpected CPU pose HTTP status')
            value=json.loads(raw)
            if value.get('sequence',0)<=after:return None
            if playback is not None and value.get('preview_time_sec')!=playback:return None
            return checked_pose(value,manifest,binding,playback=playback,live=live)
        return until(current,15,'matching CPU pose')

    def input_matches(pose,expected):
        owned=observe_workers();require(len(owned)==1,'Lost the current owned CPU worker')
        value=json.loads(bounded_file(Path(owned[0]['directory'])/'input.json',256*1024))
        require(value['sequence']==pose['sequence'] and value['positions']==expected,
                'Worker pose was not sourced from the expected complete named state')

    def history_ids(state):return [row['request_id'] for row in state['history']]

    def owned_launch(state):
        folder=Path(state['lifecycle']['last_launch_directory']).resolve()
        require(folder.parent==session and re.fullmatch(r'launch-[0-9]{3}',folder.name),
                'Simulation launch belongs to another console session')
        return folder

    def quiet_state():
        previous=None;stable_at=None
        def quiet():
            nonlocal previous,stable_at
            state=api('/api/state');positions=state['positions_rad']
            require(state['active_request_id'] is None and positions is not None,'Robot is still active before preview')
            if previous is None or max(abs(positions[n]-previous[n]) for n in positions)>.0005:
                previous=positions;stable_at=time.monotonic();return None
            return state if time.monotonic()-stable_at>=.5 else None
        return until(quiet,10,'stationary observed state before preview')

    def run_cpu_demo(scenario,variants,timeout):
        from .verify_demos import completed_job,checked_reports
        until(lambda:api('/api/state')['demo']['can_start'],20,'empty domain before '+scenario)
        previous=api('/api/state')['view']['binding_id']
        started=utc_now()
        response=api('/api/demos/start',dict(scenario=scenario,native_gui=False));job_id=response['job_id']
        require(re.fullmatch('[0-9a-f]{32}',job_id) and job_id not in {row['job_id'] for row in demo_jobs},
                'Comparison did not create a fresh job')
        runs={};proofs={};retired=set();last_binding=previous;rejected_old=set();transitions=[];latest_pose_response=None
        # POST changes ownership synchronously; the monitor publishes its blank
        # scene epoch on its next 100ms tick. Do not consume the preceding view.
        until(lambda:(state if (state:=api('/api/state'))['view']['binding_id']!=previous else None),
              3,'new CPU view epoch for '+scenario)

        def old_binding_rejected(identifier):
            if identifier is None or identifier in rejected_old:return
            for route in ('/api/view/manifest?','/api/view/pose?'):
                try:http(route+urlencode({'binding_id':identifier}))
                except HTTPError as error:
                    require(error.code in (400,409),'Old well-formed CPU binding was rejected for an unrelated HTTP reason')
                else:raise RuntimeError('A previous CPU binding remained readable after a scene transition')
            rejected_old.add(identifier)

        def flush():
            for run_id,proof in proofs.items():
                if proof.get('last_pose') is not None:atomic_json(proof['folder']/'last-observed-pose.json',proof['last_pose'])
            report.update(demo_cpu_runs=[deepcopy(row) for row in runs.values()],demo_cpu_transitions=transitions)

        def observe():
            nonlocal last_binding,latest_pose_response
            latest_pose_response=None
            state=api('/api/state');no_image_worker(state)
            job=(state.get('demo') or {}).get('current') or {}
            require(job.get('job_id')==job_id and job.get('scenario')==scenario,'CPU view lost its active comparison owner')
            view=state['view'];identifier=view.get('binding_id')
            if identifier!=last_binding:
                if last_binding is not None:retired.add(last_binding);old_binding_rejected(last_binding)
                require(identifier not in retired,'Comparison restored a retired CPU binding')
                transitions.append(dict(job_id=job_id,variant=job.get('variant'),run_id=job.get('run_id'),
                    previous_binding_id=last_binding,binding_id=identifier,status=view.get('status')))
                last_binding=identifier;flush()
            launch=job.get('active_launch');variant=job.get('variant');run_id=job.get('run_id')
            if launch and launch.get('model_sha256'):
                require(launch['run_id']==run_id and variant in variants,'Current comparison launch/variant changed identity')
                if run_id not in runs:
                    require(run_id not in all_demo_runs and variant not in {row['variant'] for row in runs.values()},
                            'Comparison reused a run or launched a variant twice')
                    model=checked_demo_launch(launch,variant,root=ROOT,base_model=cell.output/'cell.xml')
                    require(matching_process(launch),'New comparison launch is not a live owned process group')
                    all_demo_runs.add(run_id)
                    source=SceneSource(base,cell.label,model,tuple(cell.joints),dict(zip(cell.joints,cell.home)),
                        lambda model=model,sha=launch['model_sha256']:require(file_sha256(model)==sha,'Owned comparison model changed'))
                    runs[run_id]=dict(job_id=job_id,run_id=run_id,variant=variant,model_path=str(model),
                        model_sha256=launch['model_sha256'],launch_identity={key:launch[key] for key in ('pid','start_ticks')},
                        binding_id=None,manifest_sha256=None,fresh_sequences=0,first_sequence=None,last_sequence=None)
                    proofs[run_id]=dict(source=source,last_pose=None,fresh=set(),folder=None,manifest=None)
                    flush()
                record=runs[run_id]
                require(record['variant']==variant and record['model_path']==str(Path(launch['model_path']).resolve())
                        and record['model_sha256']==launch['model_sha256']
                        and record['launch_identity']=={key:launch[key] for key in ('pid','start_ticks')},
                        'One comparison run changed its owned model/process identity')
            model_sha=runs[run_id]['model_sha256'] if run_id in runs else None
            binding=checked_demo_binding(view,operator_session_id=state['lifecycle']['session_id'],source_id=base,
                job_id=job_id,run_id=run_id,model_sha256=model_sha)
            if binding is None:return state
            require(run_id in proofs,'CPU view has no previously observed owned launch')
            proof=proofs[run_id];record=runs[run_id]
            if view.get('status')!='ready' or not view.get('manifest_descriptor'):return state
            try:
                if proof['manifest'] is None:
                    _,manifest,folder=check_view(state,base,'live',source=proof['source'])
                    require(manifest['model_sha256']==record['model_sha256'],'CPU scene compiled another owned demo model')
                    proof.update(manifest=manifest,folder=folder)
                    record.update(binding_id=identifier,manifest_sha256=view['manifest_descriptor']['sha256'],
                        scene_sha256=manifest['scene_sha256'],compiled_model_sha256=(manifest.get('compiled_model_sha256') or manifest['provenance']['engine']['compiled_sha256']))
                    flush()
                require(view['manifest_descriptor']['sha256']==record['manifest_sha256']
                        and view['manifest_descriptor']['scene_sha256']==record['scene_sha256']
                        and view['manifest_descriptor']['compiled_model_sha256']==record['compiled_model_sha256'],
                        'A comparison binding changed its verified manifest/compiled scene identity')
                require(record['binding_id']==identifier,'A run unexpectedly replaced its already verified CPU binding')
                status,mime,raw=http('/api/view/pose?'+urlencode({'binding_id':identifier}))
            except HTTPError as error:
                # A new monitor tick may retire this exact snapshot during its
                # bounded asset/pose GETs. It cannot turn a current-scene error
                # into a successful observation or satisfy this run's coverage.
                if error.code in (400,404,409) and api('/api/state')['view']['binding_id']!=identifier:return state
                raise
            if status==204:
                require(not raw,'204 demo CPU pose carried a body');return state
            require(status==200 and mime=='application/json','Unexpected comparison CPU pose response')
            pose=checked_pose(json.loads(raw),proof['manifest'],binding,live=True,allow_stale=True)
            checked_pose_progress(proof['last_pose'],pose);proof['last_pose']=pose;latest_pose_response=pose
            record['last_sequence']=pose['sequence']
            if pose['source_state_age_sec']<.5:
                if not proof['fresh']:atomic_json(proof['folder']/'first-fresh-pose.json',pose)
                proof['fresh'].add(pose['sequence']);record['fresh_sequences']=len(proof['fresh'])
                if record['first_sequence'] is None:record['first_sequence']=pose['sequence']
            return state

        def finished():
            state=observe();job=state['demo']['current']
            if job.get('status')=='failed' and job.get('cleanup_clean') is False:
                raise RuntimeError('CPU comparison could not clean its owned launches')
            return completed_job(state,job_id)

        job=until(finished,timeout,'CPU comparison and owned cleanup '+scenario)
        require(job['status']=='succeeded' and job.get('comparison') is not None,
                'CPU comparison did not succeed')
        originals=checked_reports(job,variants,ROOT,args.domain,started)
        require(list(row['variant'] for row in runs.values())==list(variants)
                and set(runs)=={summary['run_id'] for summary in job['results']},
                'A comparison variant had no independently observed fresh CPU run')
        require(all(row['fresh_sequences']>=2 and row['last_sequence']>row['first_sequence']
                    and row['manifest_sha256'] for row in runs.values()),
                'A comparison variant had no advancing fresh CPU poses and verified scene assets')
        last_run=job['results'][-1]['run_id']
        require(job['run_id']==last_run and last_run==next(reversed(runs)),'Completed view is not the last actual comparison run')
        expected=runs[last_run];proof=proofs[last_run];stable=None;stable_at=None
        def stable_terminal():
            nonlocal stable,stable_at
            state=observe();current=completed_job(state,job_id)
            require(current is not None and current['status']=='succeeded','Completed comparison lost clean terminal ownership')
            value=checked_final_read(state['view'],latest_pose_response,expected['binding_id'])
            if stable is None or value['sequence']!=stable['sequence']:
                stable=deepcopy(value);stable_at=time.monotonic();return None
            checked_pose_progress(stable,value)
            return value if time.monotonic()-stable_at>=.2 else None
        first=deepcopy(until(stable_terminal,5,'stable final CPU sequence '+scenario));held_at=time.monotonic()
        def retained():
            state=observe()
            require(completed_job(state,job_id) is not None,'Completed comparison lost clean terminal ownership')
            value=checked_final_read(state['view'],latest_pose_response,expected['binding_id'])
            require(value['sequence']==first['sequence'] and value['pose']==first['pose'],
                    'Completed comparison resumed/replaced a settled CPU sequence')
            if time.monotonic()-held_at<.75:return None
            return checked_retained_pose(first,value)
        final=until(retained,4,'retained final pose and increasing stale age '+scenario)
        atomic_json(proof['folder']/'retained-final-pose.json',final)
        flush()
        summary=dict(job_id=job_id,scenario=scenario,status=job['status'],cleanup_clean=job['cleanup_clean'],
            results=[dict(variant=variant,run_id=value['run_id'],report_sha256=file_sha256(path))
                for variant,(path,value) in originals.items()],last_run_id=last_run,last_binding_id=expected['binding_id'],
            retained_sequence=final['sequence'],initial_final_age_sec=first['source_state_age_sec'],
            retained_final_age_sec=final['source_state_age_sec'],old_binding_queries_rejected=len(rejected_old),
            cpu_pose_run_ids=list(runs))
        demo_jobs.append(summary);report.update(checked_cpu_demo_jobs=demo_jobs,demo_cpu_views_checked=True)


    try:
        report.update(code_commit=subprocess.check_output(['git','-C',str(ROOT),'rev-parse','HEAD'],text=True).strip(),
            cpu_views_requested=True,egl_requested=False,jpeg_requested=False,browser_requested=False,
            demo_cpu_views_requested=args.with_demos)
        report.stage('fresh_domain_and_existing_model_preflight')
        from ..workcells.verify_motion_reproduction import require_unused_domain
        require_unused_domain(args.domain)
        with socket.socket() as probe:probe.bind(('127.0.0.1',args.port))
        cell=load_workcell();metadata=check_workcell(cell);contract=describe_workcell(cell,metadata)
        base='workcell:'+cell.id
        models[base]=SceneSource(base,cell.label,cell.output/'cell.xml',tuple(cell.joints),dict(zip(cell.joints,cell.home)),lambda:check_workcell(cell))
        for name in ('fr5','fr10','ur5e'):
            profile=load_profile(name);check_generated_model(profile)
            models['profile:'+name]=SceneSource('profile:'+name,profile.label,profile.scene_path,tuple(profile.joints),
                dict(zip(profile.joints,profile.home)),lambda profile=profile:check_generated_model(profile))
        report.update(model_manifest_sha256=file_sha256(cell.output/'source.json'),workcell_contract=contract)
        report.stage('owned_cpu_console_start')
        with report.log_path.open('w') as log:
            process=subprocess.Popen(['/usr/bin/python3',str(ROOT/'operator_console.py'),'--domain',str(args.domain),
                '--port',str(args.port)],cwd=ROOT,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        def owned_session():
            text=bounded_file(report.log_path,16*1024*1024).decode(errors='replace')[-65536:]
            match=re.search(r'OPERATOR_READY http://127\.0\.0\.1:'+str(args.port)+r'/ domain='+str(args.domain)+r' session=([^\n]+)',text)
            if match:
                path=Path(match.group(1)).resolve()
                require(path.is_relative_to((ROOT/'runtime/operator').resolve()),'Console advertised a foreign directory')
                return path
        session=until(owned_session,20,'owned console port reservation')
        bootstrap=api('/api/bootstrap');token=bootstrap['token']
        require(bootstrap['config']['domain']==args.domain and bootstrap['config']['workcell_contract']==contract,
                'Bootstrap differs from the requested isolated workcell')
        initial,manifest,_=ready_view(base,'live')
        require(not initial['can_submit'] and not initial['history'] and not any(initial['graph'].values()),
                'Initial console reused a ROS scene or history')
        status,_,raw=http('/api/view/pose?'+urlencode({'binding_id':initial['view']['binding_id']}))
        require(status==204 and not raw,'Live view invented a pose before ROS observation')
        report.update(initial_live_without_ros_checked=True)

        report.stage('single_robot_model_previews')
        for name in ('fr5','fr10','ur5e'):
            identifier='profile:'+name;api('/api/view/select',dict(mode='model_preview',source_id=identifier))
            selected,manifest,folder=ready_view(identifier,'model_preview')
            pose=pose_for(selected,manifest);input_matches(pose,models[identifier].home)
            atomic_json(folder/'home-pose.json',pose)
            before=api('/api/state');same=before['view']['binding_id'];history=history_ids(before)
            reject('/api/view/select',dict(mode='model_preview',source_id='profile:unknown'),400)
            require(api('/api/state')['view']['binding_id']==same,'Invalid source selection changed the binding')
            for route,body in (('/api/command',command(bootstrap['config']['home_rad'])),
                ('/api/start',dict(native_gui=False,planning_case='dual_arm_crossing')),('/api/attach',{}),
                ('/api/demos/start',dict(scenario='contact_comparison',native_gui=False))):
                reject(route,body,409,preview=True)
            after=api('/api/state');no_image_worker(after)
            require(history_ids(after)==history and not after['lifecycle']['owned'] and not any(after['graph'].values())
                    and after['demo']['current'] is None,'Preview rejection published or started a process')
        report.update(model_previews_checked=True,preview_admission_checked=True)

        report.stage('owned_live_simulation_and_cpu_observations')
        api('/api/view/select',dict(mode='live',source_id=base))
        until(lambda:api('/api/state')['demo']['can_start'],10,'empty isolated ROS graph')
        api('/api/start',dict(native_gui=False,planning_case='dual_arm_crossing'))
        until(lambda:api('/api/state')['can_submit'],70,'normal command readiness')
        current,manifest,folder=ready_view(base,'live',live=True)
        launch=owned_launch(current)
        first_pose=pose_for(current,manifest,live=True)
        next_pose=pose_for(current,manifest,live=True,after=first_pose['sequence'])
        require(next_pose['simulation_sec']>=first_pose['simulation_sec'],'CPU live pose source simulation time went backwards')
        atomic_json(folder/'live-pose.json',next_pose)
        require(set(current['lifecycle']['owned'])=={'simulation','commands'},'Unexpected owned child or renderer')
        before=deepcopy(current['positions_rad']);target=dict(bootstrap['config']['home_rad'])
        for arm in cell.instances:target[arm.joints[0]]+=.12
        payload=command(target);api('/api/command',payload)
        require(api('/api/state')['active_request_id']==payload['request_id'],'Normal HTTP command was not admitted')
        reject('/api/view/select',dict(mode='model_preview',source_id='profile:fr5'),409)
        reject('/api/view/display',dict(mode='image'),409)
        def terminal():
            state=api('/api/state');no_image_worker(state)
            return next((row for row in state['history'] if row['request_id']==payload['request_id']
                and row['status'] in ('succeeded','failed','cancelled','rejected')),None)
        record=until(terminal,80,'successful ordinary robot motion')
        details=api('/api/requests/'+payload['request_id'])
        require(record['status']=='succeeded' and details['action_results']==[dict(status=4,error_code=0),dict(status=4,error_code=0)]
                and details['metrics']['received_to_first_motion_wall_sec'] is not None,'Normal motion did not complete on both arms')
        settled=quiet_state()
        require(all(abs(settled['positions_rad'][arm.joints[0]]-before[arm.joints[0]])>.05 for arm in cell.instances),
                'No physical motion was observed in both arms')
        report.update(checked_requests=[details],motion_checks_passed=True,active_view_changes_rejected=True)

        report.stage('saved_plan_replay_without_publication_or_physical_motion')
        stationary=deepcopy(settled['positions_rad']);history=history_ids(settled);old_binding=settled['view']['binding_id']
        trace_path=Path(record['trace_path']).resolve()
        require(owned_launch(settled)==launch and trace_path.parent==launch/'commands'
                and trace_path.name==payload['request_id']+'.json','Observed trace belongs to another launch')
        trace_raw=bounded_file(trace_path,MAX_TRACE);session_raw=bounded_file(trace_path.parent/'session.json',4*1024*1024)
        plan=SavedPlan(trace_raw,session_raw,request_id=payload['request_id'],motion_session_id=record['session_id'],
            model_manifest=metadata,joint_names=cell.joints,joint_limits=metadata['joint_limits'])
        api('/api/view/plan',dict(request_id=payload['request_id']))
        preview,manifest,folder=ready_view(base,'plan_preview')
        require(preview['view']['binding']['motion_session_id']==record['session_id']
                and preview['view']['binding']['request_id']==payload['request_id'],'Saved plan binding lost its observed request/session')
        identifier=preview['view']['binding_id'];require(identifier!=old_binding,'Saved plan reused the live binding')
        for index,stamp in enumerate((0.,plan.duration/2,plan.duration)):
            response=api('/api/view/seek',dict(time_sec=stamp,binding_id=identifier))
            require(response==dict(preview_time_sec=stamp,duration_sec=plan.duration),'Seek acknowledgement changed time/duration')
            pose=pose_for(preview,manifest,playback=stamp);input_matches(pose,plan.positions(stamp))
            atomic_json(folder/f'seek-{index}.json',pose)
        reject('/api/view/seek',dict(time_sec=0.,binding_id=old_binding),400)
        require(api('/api/state')['view']['preview_time_sec']==plan.duration,'Rejected old binding seek changed the current preview')
        time.sleep(.5)
        after=api('/api/state');no_image_worker(after)
        drift=max(abs(after['positions_rad'][name]-stationary[name]) for name in cell.joints)
        require(history_ids(after)==history and after['active_request_id'] is None and drift<.001
                and not after['can_submit'],'Saved preview published another command or moved the actual robot')
        report.update(saved_plan_read_only_checked=True,preview_joint_drift_rad=drift,old_binding_seek_rejected=True,
            selected_trace_sha256=digest(trace_raw),selected_session_sha256=digest(session_raw))

        report.stage('restore_live_readiness_and_owned_shutdown')
        api('/api/view/select',dict(mode='live',source_id=base))
        until(lambda:api('/api/state')['can_submit'],15,'restored ordinary live readiness')
        restored,manifest,_=ready_view(base,'live',live=True);pose_for(restored,manifest,live=True)
        require(history_ids(restored)==history and restored['view']['binding_id']!=identifier,
                'Returning live replayed a command or reused a preview identity')
        api('/api/stop',{})
        stopped=until(lambda:(state if (state:=api('/api/state'))['lifecycle']['phase'] in ('idle','failed') else None),40,'owned simulation shutdown')
        require(stopped['lifecycle']['phase']=='idle' and not stopped['lifecycle']['owned'] and not stopped['can_submit'],
                'Owned shutdown did not disable normal admission')
        require(owned_launch(stopped)==launch,'Shutdown selected another launch history')
        physics_path=launch/'physics.json'
        physics=json.loads(bounded_file(physics_path,16*1024*1024))
        expected_run_id=stopped['lifecycle']['session_id']+'-'+str(int(launch.name.removeprefix('launch-')))
        check_physics(physics,run_id=expected_run_id,source_sha256=file_sha256(ROOT/'native/step_monitor.hpp'),
            timestep=cell.data['physics']['timestep'],require_interlock=True)
        require(physics['contact_steps']==0 and physics['invalid_values']==0,'Ordinary view acceptance had a physical fault')
        no_image_worker(stopped)
        report.update(physics=physics,physics_checks_passed=True,cpu_views_checked=True,no_jpeg_renderer_checked=True,
            live_restored_checked=True,view_checks=checks)
        if args.with_demos:
            report.stage('contact_comparison_cpu_views_and_retained_final_pose')
            run_cpu_demo('contact_comparison',('contact','control'),300)
            report.stage('timing_comparison_cpu_views_and_retained_final_pose')
            run_cpu_demo('timing_comparison',('prepare','baseline','timing'),900)
            report.update(view_checks=checks,no_jpeg_renderer_checked=True)
    except (Exception,KeyboardInterrupt) as error:
        failed=True;details=dict(type=type(error).__name__,message=str(error))
        if isinstance(error,HTTPError):
            details.update(http_route=getattr(error,'verification_route',None),http_status=error.code,
                http_payload=error.read(65537).decode('utf-8',errors='replace')[:65536])
        report.update(error=details,failure_stage=report.data['stage'])
    finally:
        token=None
        if process is not None:
            if session is not None:
                try:observe_workers()
                except Exception as error:cleanup.append('Worker observation: '+str(error))
            try:
                if process.poll() is None:process.send_signal(signal.SIGINT)
                deadline=time.monotonic()+45
                while process.poll() is None and time.monotonic()<deadline:
                    if session is not None:observe_workers()
                    time.sleep(.05)
                process.wait(timeout=max(.1,deadline-time.monotonic()))
                require(process.returncode==0 and group_gone(process.pid),'Owned console shutdown was not clean')
            except Exception as error:
                cleanup.append(str(error))
                try:stop_process(process)
                except Exception as fallback:cleanup.append('Owned console fallback: '+str(fallback))
        if session is not None:
            try:
                deadline=time.monotonic()+5
                while any(matching_process(row) for row in workers.values()) and time.monotonic()<deadline:time.sleep(.05)
                require(all(not matching_process(row) and group_gone(row['pid']) for row in workers.values()),
                        'A proven owned CPU scene worker remains alive')
                events=[json.loads(line) for line in bounded_file(session/'events.jsonl',16*1024*1024).splitlines()]
                require(all(group_gone(row['pid']) for row in events if row['action']=='process_started'),
                        'An owned console child process group remains')
                require(not any(row.get('cleanup_errors') for row in events),'Console recorded forced/incomplete cleanup')
                (report.directory/'console-events.jsonl').write_bytes((session/'events.jsonl').read_bytes())
            except Exception as error:cleanup.append(str(error))
        report.update(status='failed' if failed or cleanup else 'passed',stage='complete',completed_utc=utc_now(),
            cleanup_errors=cleanup,shutdown_clean=not cleanup,process_group_gone=process is not None and group_gone(process.pid),
            launch_return_code=None if process is None else process.returncode,owned_scene_workers=list(workers.values()),
            console_session=None if session is None else str(session))
    print(report.data['status'].upper()+': '+str(report.path),flush=True)
    if report.data['error']:print(report.data['error'],flush=True)
    if cleanup:print(cleanup,flush=True)
    return int(failed or bool(cleanup))


if __name__=='__main__':raise SystemExit(main())
