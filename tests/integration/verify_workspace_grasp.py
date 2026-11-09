"""Actual same-origin workspace task ownership, native grasp and shared 3D.

No browser renderer is claimed here. The public HTTP routes used by the UI drive
real native ROS tasks. Every run retains its first result; no retry or simulated
joint/object pose is introduced by this verifier.
"""
import argparse
import hashlib
import json
import os
import re
import statistics
from pathlib import Path
import signal
import subprocess
import time
from urllib.error import HTTPError, URLError
from urllib.request import Request, urlopen
import uuid

from mujoco_ros2_manipulation.paths import ROOT
from mujoco_ros2_manipulation.verification.reports import atomic_json, utc_now
from mujoco_ros2_manipulation.workcells.motion_request import command as motion_command
from mujoco_ros2_manipulation.verification.runner import group_gone
from mujoco_ros2_manipulation.operator_console.synchronized_recording import (
    SyncRecording, _directory, _read, _publish)
from mujoco_ros2_manipulation.operator_console.verify_image_transport import (
    owned_console_session, preserve_synchronized_originals)


def require(ok, message):
    if not ok:
        raise AssertionError(message)



def preserve_grasp_originals(output, receipt, capture_id, *, port, domain):
    """Keep bounded native/event source originals after this console has closed."""
    evidence={'status':'not_started','files':[],'errors':[]}
    receipt['grasp_source_originals']=evidence
    checks=receipt['checks']
    job_ids={item['job_id'] for item in checks
             if item['name']=='active_owner_blocks_switch_and_rejects_wrong_cancel'}
    if not job_ids:
        return
    try:
        require(len(job_ids)==1,'More than one native source owner was recorded')
        jid=next(iter(job_ids))
        require(type(jid) is str and re.fullmatch(r'[0-9a-f]{32}',jid),
                'Native evidence owner is not an exact job ID')
        session=owned_console_session(output/'console.log',ROOT,port,domain)
        source=session.with_name(session.name+'-grasp')/jid
        destination=output/'grasp-originals'/jid
        members={'capture-registration.json':8192,'owner-receipt.json':1024*1024,
                 'source-pin.json':1024*1024,'run/result.json':8*1024*1024,
                 'run/latest.json':512*1024,'run/capture/latest.json':1024*1024,
                 'run/native-observations.jsonl':64*1024*1024,
                 'run/native-receipts.jsonl':2*1024*1024,
                 'run/launch.log':4*1024*1024,'run/physics.json':8*1024*1024}
        try:
            transport=_directory(source/'run/capture')
        except FileNotFoundError:
            transport=None
        if transport is not None:
            try:
                names=[]
                with os.scandir(transport) as entries:
                    for entry in entries:
                        require(len(names)<10,'Recording transport has too many retained members')
                        names.append(entry.name)
                jpegs=[name for name in names if re.fullmatch(r'[0-9a-f]{64}\.jpg',name)]
                require(len(jpegs)<=4,'Recording transport exceeds its bounded JPEG evidence set')
                for name in names:
                    if name=='latest.json':continue
                    if name in jpegs or name=='image-writing':
                        members['run/capture/'+name]=512*1024
                    elif re.fullmatch(r'tmp[a-zA-Z0-9_-]{8}',name):
                        members['run/capture/'+name]=1024*1024
                    else:
                        raise ValueError('Unexpected recording transport evidence member')
            finally:os.close(transport)
        total=0
        for relative,maximum in members.items():
            name=Path(relative)
            try:
                folder=_directory(source/name.parent)
            except FileNotFoundError:
                continue
            try:
                try:raw=_read(folder,name.name,maximum)
                except FileNotFoundError:continue
                total+=len(raw)
                require(total<=96*1024*1024,'Native source evidence exceeds its job budget')
                target=_directory(destination/name.parent,create=True)
                try:
                    _publish(target,name.name,raw)
                    copied=_read(target,name.name,maximum)
                finally:os.close(target)
                require(raw==copied and raw==_read(folder,name.name,maximum),
                        'Native source bytes changed during preservation')
                evidence['files'].append({'path':relative,'bytes':len(raw),
                    'sha256':hashlib.sha256(raw).hexdigest()})
            finally:os.close(folder)
        evidence.update(status='preserved',source_job_id=jid,byte_length=total)
        final_result=destination/'run/result.json'
        if final_result.exists():
            native_result=json.loads(final_result.read_bytes())
            retained={row['path']:row['sha256'] for row in evidence['files']}
            confirmed=[item['job']['worker_result_sha256'] for item in checks
                       if item['name']=='native_result_and_owned_cleanup']
            require(len(confirmed)<=1,'Native HTTP result identity is ambiguous')
            if confirmed:
                require(confirmed[0]==retained.get('run/result.json'),
                        'Retained native result differs from its confirmed HTTP identity')
            for field,relative in (('launch_log_sha256','run/launch.log'),
                                  ('native_observations_sha256','run/native-observations.jsonl'),
                                  ('native_receipts_sha256','run/native-receipts.jsonl'),
                                  ('physics_sha256','run/physics.json')):
                if field in native_result:
                    require(native_result[field]==retained.get(relative),
                            'Native result hash differs from retained original '+relative)
        record_path=session/'synchronized'/capture_id/'recording.json' if capture_id else None
        if record_path is not None and record_path.exists():
            recording=SyncRecording.load(record_path)
            journal=json.loads((destination/'run/capture/latest.json').read_bytes())
            source_events={event['event_id']:event for event in journal['events']}
            for event in recording.events:
                emitted={key:value for key,value in event.items()
                         if key!='parent_received_monotonic_ns'}
                require(emitted==source_events.get(event['event_id']),
                        'Recorded task event differs from the original worker journal')
            required={pose['sequence']:pose for pose in recording.poses}
            if required:
                with (destination/'run/native-observations.jsonl').open('rb') as stream:
                    for line in stream:
                        native=json.loads(line)
                        sample=required.pop(native['sequence'],None)
                        if sample is None:continue
                        require(sample['stamp_ns']==native['stamp_ns'] and
                                sample['native']['time_sec']==native['time_sec'] and
                                sample['native']['capture_monotonic_ns']==native['capture_monotonic_ns'] and
                                sample['positions_rad']==native['arm_positions_rad'],
                                'Recorded native pose differs from its original source sample')
            require(not required,'Recorded pose sequence is absent from native source originals')
            evidence.update(original_event_rows_matched=len(recording.events),
                            original_native_pose_rows_matched=len(recording.poses))
    except Exception as error:
        evidence['status']='failed'
        evidence['errors'].append({'type':type(error).__name__,'message':str(error)})
        receipt['status']='failed'


def verify(args):
    from mujoco_ros2_manipulation.workcells.verify_motion_reproduction import require_unused_domain
    output=args.output.resolve()
    require(output.is_relative_to(ROOT/'runtime') and not output.exists(), 'Select a new retained runtime output')
    require(args.domain != args.grasp_domain, 'Use distinct unused domains')
    require_unused_domain(args.domain); require_unused_domain(args.grasp_domain)
    output.mkdir(parents=True)
    receipt={'schema_version':1,'kind':'workspace-grasp-verification-v1','case':args.case,
             'status':'running','started_utc':utc_now(),'automatic_retry':False,
             'domain':args.domain,'grasp_domain':args.grasp_domain,'checks':[],
             'source_commit':subprocess.check_output(['git','rev-parse','HEAD'],text=True).strip()}
    source_files=['operator_console/__main__.py','operator_console/workspace_tasks.py',
                  'operator_console/application.py','operator_console/http_server.py','grasping/ros_demo_server.py']
    receipt['working_tree_diff_sha256']=hashlib.sha256(subprocess.check_output(['git','diff','HEAD','--','src'])).hexdigest()
    receipt['source_sha256']={name:hashlib.sha256((ROOT/'src/mujoco_ros2_manipulation'/name).read_bytes()).hexdigest() for name in source_files}
    result=output/'result.json';process=None;token=None;selection_header=None;latencies=[];root_url=f'http://127.0.0.1:{args.port}'
    capture_id=None
    producer=None
    producer_stream=None
    def save(): atomic_json(result,receipt)
    def api(route, body=None, *, expect=None, selection_override=None, raw_response=False):
        before=time.perf_counter()
        request=Request(root_url+route) if body is None else Request(root_url+route,
            data=json.dumps(body,allow_nan=False).encode(),headers={'Content-Type':'application/json','Origin':root_url,'X-Operator-Token':token,
                     'X-Workspace-Selection':selection_override or selection_header or ''})
        try:
            with urlopen(request,timeout=15) as reply:
                raw=reply.read()
                if raw_response:
                    value=raw
                else:
                    value=json.loads(raw) if raw else None
                status=reply.status
        except HTTPError as error:
            status=error.code;value=json.loads(error.read())
        latencies.append({'route':route.split('?')[0],'method':'GET' if body is None else 'POST','status':status,'wall_ms':(time.perf_counter()-before)*1000})
        if expect is None:
            require(200<=status<300,f'{route} unexpected {status}: {value}')
        else:
            require(status in expect,f'{route} expected {expect}, got {status}: {value}')
        return value
    def until(check, seconds, label):
        deadline=time.monotonic()+seconds
        while time.monotonic()<deadline:
            require(process.poll() is None,'Workspace exited while waiting for '+label)
            try:
                value=check()
                if value:return value
            except (URLError,TimeoutError):pass
            time.sleep(.1)
        raise AssertionError('Timeout: '+label)
    def note(name, **values):
        receipt['checks'].append(dict(name=name,**values));save()
    def state():
        nonlocal selection_header
        value=api('/api/state');selection_header=value['task']['selection_id'];return value
    def job():return state()['task']['grasp']['job']
    started=time.monotonic();save()
    stream=(output/'console.log').open('w')
    try:
        command=['/usr/bin/python3',str(ROOT/'operator_console.py'),'--domain',str(args.domain),'--port',str(args.port),
                 '--ros-grasp-domain',str(args.grasp_domain),'--ros-grasp-environment',str(args.target.absolute()),
                 '--ros-grasp-manifest-sha256',args.sha]
        if args.seed:command+=['--saved-recording',str(args.seed.absolute())]
        if args.synchronized:
            command+=['--ros-grasp-image-topic','/diagnostic/image',
                      '--ros-grasp-image-clock-domain','ros_simulation']
        process=subprocess.Popen(command,cwd=ROOT,stdout=stream,stderr=subprocess.STDOUT,start_new_session=True)
        receipt['owned_console_pid']=process.pid;save()
        bootstrap=until(lambda:api('/api/bootstrap'),40,'workspace readiness');token=bootstrap['token']
        require(bootstrap['task']['selected']=='motion','Default task changed')
        initial=until(lambda:(s if (s:=state())['task']['can_select'] else None),10,'initial task selection')
        old_selection=initial['task']['selection_id']
        api('/api/task/select',{'task':'grasp','selection_id':old_selection})
        selected=state();selection=selected['task']['selection_id']
        require(selection!=old_selection and selected['task']['selected']=='grasp','Selection epoch did not change')
        api('/api/task/select',{'task':'motion','selection_id':old_selection},expect=(400,409))
        api('/api/start',{'native_gui':False,'planning_case':'none'},expect=(409,))
        binding=selected['view']['binding_id']
        manifest=api('/api/view/manifest?binding_id='+binding)
        require(manifest['provenance']['engine']['version']=='3.12.0','Shared view did not select native geometry')
        require(any(g['name']=='fr5_block_geom' for g in manifest['geoms']),'Free block missing from shared scene')
        api('/api/view/pose?binding_id='+binding)
        note('same_shared_native_scene_and_scoped_task_selection',binding_id=binding,geom_count=len(manifest['geoms']))
        if args.seed:
            library=api('/api/sync-recordings');require(len(library['recordings'])==1,'Explicit saved recording missing')
            rid=library['recordings'][0]['id'];saved=api('/api/sync-recordings/'+rid)
            replay=api('/api/sync-recordings/'+rid+'/control',{'action':'next'})
            require(replay['source']['kind']=='synchronized-recording' and replay['source']['readonly'] is True
                    and replay['binding']['source_id']=='scene:synchronized-recording','Replay source is not the scoped read-only record')
            require(job() is None,'Reading a saved record started a robot task')
            note('saved_sensor_and_pose_replay_remains_readonly',recording_id=rid,recording_sha256=saved['view']['recording_sha256'])
        model=selected['task']['grasp']['source']['model_sha256'];request_id=uuid.uuid4().hex
        api('/api/task/grasp/start',{'selection_id':selection,'request_id':uuid.uuid4().hex,'model_sha256':'0'*64},expect=(400,))
        payload={'selection_id':selection,'request_id':request_id,'model_sha256':model}
        api('/api/task/grasp/start',payload)
        current=until(job,10,'owned job');jid=current['job_id'];generation=current['generation']
        api('/api/task/grasp/start',payload)
        require(job()['job_id']==jid,'Duplicate request launched a second task')
        api('/api/task/select',{'task':'motion','selection_id':selection},expect=(409,))
        api('/api/task/grasp/cancel',{'selection_id':selection,'job_id':jid,'generation':generation+1,'model_sha256':model},expect=(400,))
        note('active_owner_blocks_switch_and_rejects_wrong_cancel',job_id=jid,generation=generation)
        if args.synchronized:
            available=api('/api/sync-recordings')['capture']
            require(available['can_arm'] and available['expected_owner_token'],
                    'The exact grasp owner cannot arm before the first source sample')
            api('/api/sync-recordings/start',{'expected_owner_token':'0'*64},expect=(400,409))
            require(job()['job_id']==jid and not job()['cancel_requested'],
                    'Invalid recording ownership affected the native task')
            receipt['synchronized_capture_readiness']={'start_requests':1,
                'invalid_owner_start_requests':1,
                'expected_owner_token':available['expected_owner_token'],
                'scope':'Exact reserved grasp owner, before native/image readiness'}
            save()
            capture=api('/api/sync-recordings/start',
                        {'expected_owner_token':available['expected_owner_token']})
            capture_id=capture['id']
            api('/api/sync-recordings/stop',{'id':capture_id,
                'expected_owner_token':'0'*64},expect=(400,409))
            require(api('/api/sync-recordings')['capture']['can_stop'],
                    'Wrong recording owner stopped the active acquisition')
            note('grasp_synchronized_capture_armed',id=capture_id,
                 owner_token=capture['owner_token'],awaiting_sources=capture['awaiting_sources'])
        cancelled=False;live_binding=None;first_block=None;moving=False;last=None
        deadline=time.monotonic()+320
        while time.monotonic()<deadline:
            value=state();last=value['task']['grasp']['job']
            require(last['job_id']==jid and last['generation']==generation,'Job identity changed')
            view=value['view']
            if view['mode']=='live' and live_binding is None:
                live_binding=view['binding_id'];pose=api('/api/view/pose?binding_id='+live_binding)
                require(pose['binding_id']==live_binding and pose.get('source',{}).get('clock_domain')=='mujoco-native:'+jid,'Pose belongs to another source')
            if args.synchronized and args.case!='cancel_startup' and live_binding and producer is None:
                # The production worker has already passed unused-domain admission.
                # This source publishes only diagnostic RGB8 with observed /clock stamps.
                producer_stream=(output/'diagnostic-rgb8.log').open('wb')
                producer_command=['/usr/bin/python3','-m',
                    'mujoco_ros2_manipulation.operator_console.diagnostic_image',
                    '--domain',str(args.grasp_domain),'--topic','/diagnostic/image',
                    '--duration','120','--rate','10','--gap-at','2','--gap-duration','1.2']
                producer=subprocess.Popen(producer_command,cwd=ROOT,stdout=producer_stream,
                    stderr=subprocess.STDOUT,start_new_session=True)
                note('late_readonly_diagnostic_rgb8_source_started',pid=producer.pid,
                     command=producer_command,control_commands=0)
            if producer is not None:
                require(producer.poll() is None,'Diagnostic RGB8 producer exited before its owned stop')
            if args.case=='cancel_startup' and not cancelled and last['process_running']:
                api('/api/task/grasp/cancel',{'selection_id':selection,'job_id':jid,'generation':generation,'model_sha256':model});cancelled=True
                note('startup_cancel_requested',job_before=last)
            elif args.case=='cancel_transport' and not cancelled and last['phase']=='transport':
                block=last['evidence'].get('block_position_m')
                if block and first_block is None:first_block=block
                if block and first_block and sum((a-b)**2 for a,b in zip(block,first_block))**.5>=.005:
                    require(last['task']['physical_grasp']=='confirmed' and last['task']['planning_attachment']=='attached','Moving cancel lacks grasp/attachment evidence')
                    api('/api/task/grasp/cancel',{'selection_id':selection,'job_id':jid,'generation':generation,'model_sha256':model});cancelled=True;moving=True
                    note('moving_transport_cancel_requested',job_before=last)
            if last['finished']:break
            time.sleep(.15)
        require(last and last['finished'],'Job did not finish')
        match=re.search(r'OPERATOR_READY .* session=(.+)',(output/'console.log').read_text())
        require(match is not None,'Owned session receipt missing')
        session=Path(match.group(1)).absolute()
        require(session.is_relative_to(ROOT/'runtime/operator'),'Unexpected owned session root')
        raw_result=session.with_name(session.name+'-grasp')/jid/'run/result.json'
        raw=raw_result.read_bytes();proof=json.loads(raw)
        require(proof.get('kind')=='ros-grasp-task-v1' and proof.get('run_id')==jid
                and proof.get('domain')==args.grasp_domain and proof.get('model_sha256')==model
                and proof.get('execution_manifest_sha256')==args.sha
                and hashlib.sha256(raw).hexdigest()==last.get('worker_result_sha256'),
                'Final HTTP result does not match the exact retained native worker result')
        guard=last.get('native_guard') or {}
        stopped=(guard.get('latched') is True and guard.get('phase')=='stopped'
                 and all(guard.get('controllers',{}).get(name)=='inactive'
                         for name in ('left_controller','right_controller','gripper_controller')))
        expected='succeeded' if args.case=='normal' else 'cancelled'
        require(last['status']==expected,f'Unexpected task result: {last}')
        require(last['owner_cleanup_clean'] is True and last['process_group_gone'] is True and not last['process_running'],'Owned process cleanup incomplete')
        require(last['cleanup'].get('clean') is True and last['cleanup'].get('process_group_gone') is True
                and last['cleanup'].get('native_process_clean') is True
                and last.get('validation_process_group_gone') is True,'Native or validation-worker cleanup incomplete')
        require(not last['automatic_release'] and not last['automatic_detach'] and not last['automatic_retry'],'Cancellation changed object or restarted task')
        if args.case=='normal':
            require(last['task']['physical_grasp']=='released' and last['task']['planning_attachment']=='world','Physical release/world-object state missing')
            require(live_binding is not None,'No actual native live scene was shown')
        else:
            require(cancelled,'Cancellation was never issued')
            if args.case=='cancel_transport':require(moving,'Cancel occurred before observed transport motion')
            if args.case=='cancel_startup':
                require(proof.get('actions')==[],'Startup cancellation dispatched a robot Action')
            if not stopped:
                require(args.case=='cancel_startup','Three inactive controllers with latched stop were not observed')
                require(proof.get('owned_process_group') is None and proof.get('actions')==[]
                        and proof.get('cleanup',{}).get('launch_started') is False
                        and proof.get('cleanup',{}).get('native_process_clean') is True
                        and last.get('validation_process_group_gone') is True,
                        'No proof that cancellation preceded every native launch/action')
                note('startup_cancel_preceded_native_launch_not_physical_stop',
                     worker_result_sha256=hashlib.sha256(raw).hexdigest())
        note('native_result_and_owned_cleanup',job=last)
        if args.synchronized:
            finished_capture=until(lambda:(c if (c:=api('/api/sync-recordings')['capture'])['status']
                in ('completed','failed') else None),20,'automatic grasp recording completion')
            require(finished_capture['status']=='completed',
                    'Grasp recording did not preserve a checked record: '+str(finished_capture))
            require(finished_capture['reason']=='task_finished',
                    'Recording ended before the exact task terminal source journal')
            prefix='/api/sync-recordings/'+capture_id
            captured=SyncRecording.load(session/'synchronized'/capture_id/'recording.json')
            descriptor=api(prefix)['view']
            require(captured.document['schema_version']==2 and descriptor['recording_version']==2,
                    'Grasp acquisition was labelled as an old normal-motion recording')
            require(captured.sha256==descriptor['recording_sha256'],'Saved grasp recording identity differs')
            require(descriptor['events'],'No original task event was preserved')
            event_sequences=[event['event_sequence'] for event in descriptor['events']]
            require(event_sequences==sorted(set(event_sequences)),
                    'Original event order or identity was lost')
            require(all(event['time_sec'] is None for event in descriptor['events']
                        if not event['native_association']['available']),
                    'An unassociated event was assigned a native cursor time')
            require(captured.document['task_status']==expected,
                    'Recording completion hid the original task terminal status')
            if args.case=='cancel_startup':
                require(descriptor['media_state']=='unavailable' and captured.origin_ns is None,
                        'Pre-native cancellation manufactured a media clock origin')
                require(not captured.poses and not captured.image_frames,
                        'Pre-native cancellation manufactured a pose or image')
                require(all(event['time_sec'] is None for event in descriptor['events']),
                        'Unassociated startup events acquired a native timestamp')
            else:
                require(len(captured.poses)>=2 and len(captured.image_frames)>=3,
                        'Native grasp and the declared RGB source were not both recorded')
                frame=captured.image_frames[-1]
                chosen=api(prefix+'/control',{'action':'seek',
                    'time_sec':(int(frame['stamp_ns'])-captured.origin_ns)/1e9})
                require(chosen['selection']['image_status']=='ready' and
                        chosen['selection']['pose_status']=='ready','Grasp image/native pose selection missing')
                digest=chosen['selection']['image']['jpeg_sha256']
                require(hashlib.sha256(api(prefix+'/images/'+digest+'.jpg',raw_response=True)).hexdigest()==digest,
                        'Grasp replay JPEG differs from its original digest')
                gaps=[(a,b) for a,b in zip(captured.image_frames,captured.image_frames[1:])
                      if int(b['stamp_ns'])-int(a['stamp_ns'])>700_000_000]
                if gaps:
                    gap_time=(int(gaps[0][0]['stamp_ns'])+600_000_000-captured.origin_ns)/1e9
                    gap=api(prefix+'/control',{'action':'seek','time_sec':gap_time})['selection']
                    require(gap['image_status']=='missing' and gap['image'] is None,
                            'A missing RGB interval was filled with an old frame')
                associated=next((event for event in descriptor['events']
                                 if event['time_sec'] is not None),None)
                require(associated is not None,'No task event retained a native association')
                event_selection=api(prefix+'/control',{'action':'seek',
                    'time_sec':associated['time_sec']})['selection']
                require(associated['event_id'] in event_selection['event_ids'],
                        'The common cursor lost the selected native-associated task event')
                note('native_event_replay_and_rgb_gap_observation',id=capture_id,
                     gap_count=len(gaps),gap_verification='passed' if gaps else 'not_exercised',
                     event_id=associated['event_id'],
                     source_stamp_ns=associated['native_association']['stamp_ns'],
                     note='Diagnostic input gap is scheduled in wall time. Detected gaps retain actual source stamps.')
            require(job()['job_id']==jid and job()['finished'],'Read-only replay started a new grasp job')
            note('grasp_synchronized_capture_and_readonly_replay',id=capture_id,
                 recording_sha256=captured.sha256,pose_count=len(captured.poses),
                 image_count=len(captured.image_frames),events=descriptor['events'],
                 media_state=descriptor['media_state'],control_commands=0,
                 note='64x48 diagnostic RGB8, same-run association only; no camera calibration or exact native-step claim')
        returned=until(lambda:(s if (s:=state())['task']['can_select'] else None),12,'post-job task switch')
        api('/api/task/select',{'task':'motion','selection_id':returned['task']['selection_id']})
        require(state()['task']['selected']=='motion','Could not return to ordinary task')
        api('/api/start',{'native_gui':False,'planning_case':'none'},selection_override=old_selection,expect=(400,409))
        api('/api/task/grasp/start',payload,expect=(400,409))
        if live_binding:api('/api/view/pose?binding_id='+live_binding,expect=(400,))
        note('completed_owner_returns_to_motion_and_old_bindings_rejected')
        if args.case=='normal':
            api('/api/start',{'native_gui':False,'planning_case':'none'})
            ready=until(lambda:(s if (s:=state())['can_submit'] else None),70,'ordinary motion after grasp')
            api('/api/task/select',{'task':'grasp','selection_id':ready['task']['selection_id']},expect=(409,))
            request=motion_command(bootstrap['config']['targets']['sample'])
            api('/api/plan',request)
            review=until(lambda:(s['pending_plan'] if ((s:=state()).get('pending_plan') or {}).get('state')=='awaiting_approval' and not s.get('active_request_id') else None),60,'ordinary plan review after grasp')
            require(review['plan_request_id']==request['request_id'],'Review belongs to another request')
            api('/api/plan/discard',{'request_id':request['request_id']})
            api('/api/stop',{})
            ordinary=until(lambda:(s if (s:=state())['lifecycle']['phase']=='idle' and s['task']['can_select'] else None),40,'ordinary process shutdown')
            require(not ordinary['lifecycle']['owned'],'Ordinary owned processes remain')
            note('ordinary_motion_start_plan_review_discard_stop_after_grasp',request_id=request['request_id'])
        receipt.update(status='passed',finished_utc=utc_now(),wall_sec=time.monotonic()-started)
    except Exception as error:
        receipt.update(status='failed',error=repr(error),finished_utc=utc_now());raise
    finally:
        if producer is not None:
            if producer.poll() is None:
                producer.send_signal(signal.SIGINT)
                try:producer.wait(timeout=12)
                except subprocess.TimeoutExpired:
                    receipt['diagnostic_source_cleanup_error']='Owned diagnostic producer exceeded shutdown bound'
                    receipt['diagnostic_source_cleanup_escalation']=[]
                    for stop_signal in (signal.SIGTERM,signal.SIGKILL):
                        if producer.poll() is not None:break
                        receipt['diagnostic_source_cleanup_escalation'].append(stop_signal.name)
                        producer.send_signal(stop_signal)
                        try:producer.wait(timeout=5)
                        except subprocess.TimeoutExpired:continue
                    receipt['status']='failed'
            receipt['diagnostic_source_exit_code']=producer.poll()
            receipt['diagnostic_source_group_gone']=group_gone(producer.pid)
            if producer.poll()!=0 or not receipt['diagnostic_source_group_gone']:
                receipt['status']='failed'
                receipt.setdefault('cleanup_error','Read-only diagnostic producer clean exit not confirmed')
        if producer_stream is not None:producer_stream.close()
        if process is not None and process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:process.wait(timeout=80)
            except subprocess.TimeoutExpired:
                receipt['cleanup_error']='Owned workspace did not close within 80 seconds'
        if process is not None:
            receipt['console_exit_code']=process.poll()
            if process.poll()!=0:
                receipt['status']='failed';receipt.setdefault('cleanup_error','Workspace clean exit not confirmed')
        stream.close()
        if args.synchronized:
            preserve_synchronized_originals(output,receipt,capture_id,root=ROOT,
                port=args.port,domain=args.domain)
            preserve_grasp_originals(output,receipt,capture_id,port=args.port,domain=args.domain)
        receipt['http_timings']={'scope':'same-host HTTP round trip including JSON decode; not browser/paint/control latency',
            'samples':latencies,'state_get_median_ms':statistics.median([r['wall_ms'] for r in latencies if r['route']=='/api/state']) if any(r['route']=='/api/state' for r in latencies) else None}
        save()
    require(receipt['status']=='passed','Workspace verification failed')
    print(json.dumps({'status':receipt['status'],'case':args.case,'output':str(result),'wall_sec':receipt['wall_sec']}))


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--target',type=Path,required=True);parser.add_argument('--sha',required=True)
    parser.add_argument('--output',type=Path,required=True);parser.add_argument('--port',type=int,default=8934)
    parser.add_argument('--domain',type=int,default=216);parser.add_argument('--grasp-domain',type=int,default=217)
    parser.add_argument('--synchronized',action='store_true',help='Arm grasp RGB/native/event acquisition and retain every original bundle')
    parser.add_argument('--seed',type=Path);parser.add_argument('--case',choices=('normal','cancel_transport','cancel_startup'),default='normal')
    verify(parser.parse_args())
if __name__=='__main__':main()
