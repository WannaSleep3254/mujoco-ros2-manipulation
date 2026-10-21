"""Actual loopback HTTP -> ROS commands -> observed two-arm motion regression."""
import argparse
import json
from pathlib import Path
import shutil
import signal
import subprocess
import time
from urllib.error import URLError, HTTPError
from urllib.request import Request, urlopen

from mujoco_ros2_manipulation.paths import ROOT
from mujoco_ros2_manipulation.model_build.integrity import file_sha256
from mujoco_ros2_manipulation.verification.reports import VerificationReport, utc_now
from mujoco_ros2_manipulation.verification.runner import group_gone, stop_process
from mujoco_ros2_manipulation.verification.diagnostics import check_physics
from mujoco_ros2_manipulation.workcells.config import load_workcell
from mujoco_ros2_manipulation.workcells.integrity import check_workcell
from mujoco_ros2_manipulation.workcells.motion_request import command


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--domain',type=int,default=143)
    parser.add_argument('--port',type=int,default=8878)
    parser.add_argument('--no-render',action='store_true')
    args=parser.parse_args()
    if not 0<=args.domain<=232 or not 1024<=args.port<=65535:parser.error('Invalid domain or port')
    cell=load_workcell();check_workcell(cell)
    report=VerificationReport(ROOT,'operator_console','headless' if args.no_render else 'render',args.domain)
    process=None;session=None;failed=False;cleanup=[];token=None
    url=f'http://127.0.0.1:{args.port}'
    def api(route,body=None):
        request=Request(url+route) if body is None else Request(url+route,data=json.dumps(body,allow_nan=False).encode(),
            headers={'Content-Type':'application/json','Origin':url,'X-Operator-Token':token})
        with urlopen(request,timeout=5) as response:return json.load(response)
    def until(predicate,timeout,label):
        deadline=time.monotonic()+timeout
        while time.monotonic()<deadline:
            if process.poll() is not None:raise RuntimeError('Console exited before '+label)
            try:
                result=predicate()
                if result:return result
            except (URLError,TimeoutError):pass
            time.sleep(.05)
        raise RuntimeError('Timed out: '+label)
    def ready():return api('/api/state')['can_submit']
    def terminal(identifier):
        state=api('/api/state');record=next((r for r in state['history'] if r['request_id']==identifier),None)
        return record if record and record['status'] in ('succeeded','failed','cancelled','rejected') else None
    checked=[]
    def completed(payload,expected):
        until(lambda:ready(),10,'command readiness')
        api('/api/command',payload)
        reply=until(lambda:terminal(payload['request_id']),70,'command terminal status')
        details=api('/api/requests/'+payload['request_id'])
        require(reply['status']==expected,f'Expected {expected}: {reply}')
        checked.append(details);report.update(checked_requests=checked)
        return details
    try:
        report.update(code_commit=subprocess.check_output(['git','-C',str(ROOT),'rev-parse','HEAD'],text=True).strip(),
            model_manifest_sha256=file_sha256(cell.output/'source.json'))
        with report.log_path.open('w') as log:
            process=subprocess.Popen(['/usr/bin/python3',str(ROOT/'operator_console.py'),'--domain',str(args.domain),'--port',str(args.port)]+(['--no-render'] if args.no_render else []),
                cwd=ROOT,stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
        bootstrap=until(lambda:api('/api/bootstrap'),15,'console HTTP readiness');token=bootstrap['token']
        until(lambda:api('/api/state')['reason']!='ROS 연결을 확인하고 있습니다.',5,'ROS graph discovery')
        report.stage('start_owned_simulation')
        api('/api/start',dict(native_gui=False,planning_case='dual_arm_crossing'))
        until(ready,65,'MuJoCo identity / shared-state / command readiness')
        initial=api('/api/state');session=Path(initial['lifecycle']['last_launch_directory']).parent
        require(session.resolve().is_relative_to((ROOT/'runtime/operator').resolve()),'Unexpected console session path')
        if not args.no_render:
            until(lambda:api('/api/state')['frame'].get('sequence',0)>2,25,'live joint-state renderer')
        report.stage('strict_target_rejection')
        invalid=dict(bootstrap['config']['home_rad']);name=cell.joints[0];invalid[name]=bootstrap['config']['joint_limits'][name]['upper']+1
        try:api('/api/command',command(invalid))
        except HTTPError as error:require(error.code==400,'Wrong invalid-target rejection')
        else:raise RuntimeError('Invalid target was accepted')
        require(not api('/api/state')['history'],'Invalid target entered dispatch history')
        target=dict(bootstrap['config']['home_rad'])
        for arm in cell.instances:target[arm.joints[0]]+=.3
        payload=command(target)
        report.stage('coordinated_http_command')
        first=completed(payload,'succeeded')
        require([r['status'] for r in first['action_results']]==[4,4] and first['metrics']['received_to_first_motion_wall_sec'] is not None,'No verified two-arm physical completion')
        api('/api/command',payload)
        require(len(api('/api/state')['history'])==1,'Duplicate HTTP request added another motion')
        report.stage('moving_http_cancel_and_busy_rejection')
        until(ready,10,'cancel test readiness');before=api('/api/state')['positions_rad']
        target=dict(bootstrap['config']['home_rad'])
        for arm in cell.instances:target[arm.joints[0]]-=.6
        payload=command(target);api('/api/command',payload)
        def moving():
            current=api('/api/state')
            return current['active_request_id']==payload['request_id'] and max(abs(current['positions_rad'][n]-before[n]) for n in cell.joints)>.02
        until(moving,45,'observed physical motion before cancellation')
        try:api('/api/command',command(bootstrap['config']['home_rad']))
        except HTTPError as error:require(error.code==409,'Wrong busy rejection')
        else:raise RuntimeError('Busy request was accepted')
        api('/api/cancel',dict(request_id=payload['request_id']))
        result=until(lambda:terminal(payload['request_id']),30,'both-arm cancellation')
        cancelled=api('/api/requests/'+payload['request_id']);checked.append(cancelled);report.update(checked_requests=checked)
        require(result['status']=='cancelled' and [r['status'] for r in cancelled['action_results']]==[5,5],'Both active action goals were not cancelled')
        require(cancelled['metrics']['received_to_first_motion_wall_sec'] is not None,'Cancellation occurred before motion')
        report.stage('departure_timing_recovery')
        recovery=completed(command(bootstrap['config']['home_rad'],'departure_timing'),'succeeded')
        require(recovery['departure_search']['native_distance_calls']>0,'Departure search was not exercised')
        report.update(motion_checks_passed=True,http_target_rejection=True,http_busy_rejection=True,http_duplicate_did_not_redispatch=True)
        report.stage('owned_process_shutdown')
        api('/api/stop',{})
        stopped=until(lambda:(s if (s:=api('/api/state'))['lifecycle']['phase'] in ('idle','failed') else None),35,'owned process shutdown')
        require(stopped['lifecycle']['phase']=='idle' and not stopped['lifecycle']['owned'] and not stopped['can_submit'],'Owned shutdown did not disable dispatch cleanly')
        folder=Path(stopped['lifecycle']['last_launch_directory']);physics=json.loads((folder/'physics.json').read_text())
        check_physics(physics,run_id=physics['run_id'],source_sha256=file_sha256(ROOT/'native/step_monitor.hpp'),timestep=cell.data['physics']['timestep'],require_interlock=True)
        require(physics['run_id']==initial['lifecycle']['session_id']+'-1','Native physics belongs to another console launch')
        report.update(physics=physics,physics_checks_passed=True,console_session=str(session),renderer_checked=not args.no_render)
    except (Exception,KeyboardInterrupt) as error:
        failed=True;report.update(error=dict(type=type(error).__name__,message=str(error)),failure_stage=report.data['stage'])
    finally:
        if process:
            try:
                process.send_signal(signal.SIGINT) if process.poll() is None else None
                process.wait(timeout=35)
                require(process.returncode==0 and group_gone(process.pid),'Console process shutdown was not clean')
            except Exception as error:
                cleanup.append(str(error));stop_process(process)
        if session and session.is_dir():
            try:
                events=[json.loads(line) for line in (session/'events.jsonl').read_text().splitlines()]
                require(all(group_gone(e['pid']) for e in events if e['action']=='process_started'),'Owned child process group remains')
                require(not any(e.get('cleanup_errors') for e in events),'Console needed forced shutdown')
                shutil.copytree(session,report.directory/'console-session')
            except Exception as error:cleanup.append(str(error))
        report.update(status='failed' if failed or cleanup else 'passed',stage='complete',completed_utc=utc_now(),cleanup_errors=cleanup,
            shutdown_clean=not cleanup,launch_return_code=None if process is None else process.returncode)
    print(report.data['status'].upper()+': '+str(report.path),flush=True)
    if report.data['error']:print(report.data['error'],flush=True)
    if cleanup:print(cleanup,flush=True)
    return 1 if failed or cleanup else 0


if __name__=='__main__':raise SystemExit(main())
