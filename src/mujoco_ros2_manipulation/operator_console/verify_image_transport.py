"""Actual ROS RGB8 -> console worker -> read-only HTTP functional verification."""
import argparse
from concurrent.futures import ThreadPoolExecutor
import hashlib
import io
import json
import os
from pathlib import Path
import re
import signal
import subprocess
import time
from urllib.error import HTTPError
from urllib.request import Request, urlopen
import uuid

from ..paths import ROOT
from ..verification.reports import atomic_json
from ..workcells.motion_request import command
from .diagnostic_image import pattern
from .synchronized_capture import MAX_CAPTURE_BYTES, MAX_IMAGES, MAX_RECORDINGS
from .synchronized_recording import SyncRecording, _directory, _read, _publish, MAX_RECORDING_BYTES, MAX_JPEG_BYTES
from .scene_host import MAX_ASSET_BYTES


def http_request(base, path, payload=None, *, token=None, raw=False):
    headers={} if payload is None else {"Content-Type":"application/json","Origin":base,"X-Operator-Token":token}
    request=Request(base+path,data=None if payload is None else json.dumps(payload).encode(),headers=headers)
    try:
        with urlopen(request,timeout=3) as reply:
            data=reply.read()
            return data if raw else json.loads(data)
    except HTTPError as error:
        # A status alone cannot distinguish a correct freshness refusal from a
        # transport or implementation failure. Keep the bounded server reason.
        body=error.read(8193)
        error.verification_detail=dict(path=path,status=error.code,
            response=body[:8192].decode("utf-8",errors="replace"),response_truncated=len(body)>8192)
        raise


def request_while_pumping(operation, wait, label, seconds=15):
    """Send once while the main thread keeps the owned ROS input advancing.

    The operation retains its HTTP transport timeout. A wait timeout drains
    that one request before the caller's console shutdown handles any late
    capture; it must not retry a start whose outcome may be unknown.
    """
    with ThreadPoolExecutor(max_workers=1,thread_name_prefix="image-verification-http") as pool:
        future=pool.submit(operation)
        # Return the completed Future, rather than raising inside wait(), which
        # deliberately tolerates transient OSError during read-only discovery.
        wait(lambda:future if future.done() else None,label,seconds,frames=True)
        return future.result()


def capture_readiness(state, catalog):
    """Observe one current motion/view handshake and the capture owner's gate.

    A ready pose can precede the motion server's first status. It is not enough
    until its binding names that same server. The public capture availability
    remains authoritative for source freshness and all other admission checks.
    """
    def mapping(value):
        return value if type(value) is dict else {}
    def bounded(value):
        return value[:256] if type(value) is str else None
    state,catalog=mapping(state),mapping(catalog)
    lifecycle,server=mapping(state.get("lifecycle")),mapping(state.get("server"))
    view=mapping(state.get("view"));binding=mapping(view.get("binding"))
    capture=mapping(catalog.get("capture"))
    motion=server.get("session_id")
    owner_ready=(lifecycle.get("phase")=="running" and type(motion) is str and bool(motion)
        and view.get("status")=="ready" and view.get("pose_ready") is True
        and view.get("mode")=="live" and binding.get("mode")=="live"
        and binding.get("motion_session_id")==motion and binding.get("job_id") is None)
    can_start=capture.get("can_start") is True
    return dict(ready=bool(owner_ready and can_start),owner_ready=bool(owner_ready),can_start=can_start,
        lifecycle_phase=bounded(lifecycle.get("phase")),motion_session_id=bounded(motion),
        view_status=bounded(view.get("status")),pose_ready=view.get("pose_ready") is True,
        view_mode=bounded(view.get("mode")),binding_mode=bounded(binding.get("mode")),
        binding_id=bounded(view.get("binding_id")),binding_motion_session_id=bounded(binding.get("motion_session_id")),
        binding_job_id=bounded(binding.get("job_id")),availability_reason=bounded(capture.get("availability_reason")))


def start_synchronized_capture(http, wait, result):
    """Read-only readiness polling, followed by exactly one owned start POST."""
    evidence=dict(poll_count=0,start_requests=0,last_observed=None)
    result["synchronized_capture_readiness"]=evidence
    def probe():
        state=http("/api/state")
        catalog=http("/api/sync-recordings")
        evidence["poll_count"]+=1
        evidence["last_observed"]=capture_readiness(state,catalog)
        return evidence["last_observed"]["ready"]
    wait(probe,"same motion live 3D and synchronized capture readiness",30,frames=True)
    # Asset preparation can outlast image freshness. Continue publishing during
    # this one request; a later refusal remains evidence and is never retried.
    evidence["start_requests"]=1
    return request_while_pumping(lambda:http("/api/sync-recordings/start",{}),
        wait,"synchronized capture start")["id"]


def owned_console_session(log_path, root, port, domain):
    """Accept only this verifier's single current console readiness marker."""
    descriptor = _directory(Path(log_path).parent)
    try:
        text = _read(descriptor, Path(log_path).name, 4 * 1024 * 1024).decode('utf-8', errors='replace')
    finally:
        os.close(descriptor)
    lines = [line for line in text.splitlines() if line.startswith('OPERATOR_READY ')]
    prefix = f'OPERATOR_READY http://127.0.0.1:{port}/ domain={domain} session='
    if len(lines) != 1 or not lines[0].startswith(prefix):
        raise ValueError('Missing or ambiguous current console session marker')
    session = Path(lines[0][len(prefix):])
    operator = Path(root).resolve() / 'runtime/operator'
    if (not session.is_absolute() or '..' in session.parts or session.parent != operator
            or not re.fullmatch(r'[0-9]{8}T[0-9]{6}Z-[0-9a-f]{8}', session.name)):
        raise ValueError('Console capture session escaped the current owned runtime')
    descriptor = _directory(session)
    os.close(descriptor)
    return session


def _capture_names(directory, maximum):
    names = []
    with os.scandir(directory) as entries:
        for entry in entries:
            if len(names) >= maximum:
                raise ValueError('Capture evidence member count exceeds its bound')
            names.append(entry.name)
    return sorted(names)


def _capture_inventory(directory):
    """Inventory only bounded capture JSON, digest assets and JPEG originals."""
    names = _capture_names(directory, 5)
    if set(names) - {'recording.json', 'capture.json', 'partial.json', 'assets', 'images'}:
        raise ValueError('Unexpected member in owned synchronized capture')
    inventory = dict(directories=[], files=[], byte_length=0)
    for folder, maximum_count in (('', 3), ('assets', 1024), ('images', MAX_IMAGES)):
        if folder and folder not in names:
            continue
        child = os.open(folder, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW, dir_fd=directory) if folder else directory
        try:
            members = _capture_names(child, maximum_count) if folder else [name for name in names if name not in ('assets', 'images')]
            if folder:
                inventory['directories'].append(folder)
            for name in members:
                if folder and not re.fullmatch(r'[0-9a-f]{64}' + (r'\.jpg' if folder == 'images' else r'\.bin'), name):
                    raise ValueError('Capture asset or image name is not a digest')
                maximum = (MAX_JPEG_BYTES if folder == 'images' else MAX_ASSET_BYTES if folder else
                           64 * 1024 if name == 'capture.json' else MAX_RECORDING_BYTES)
                raw = _read(child, name, maximum)
                inventory['byte_length'] += len(raw)
                if inventory['byte_length'] > MAX_CAPTURE_BYTES + 128 * 1024:
                    raise ValueError('Capture evidence byte budget exceeded')
                inventory['files'].append(dict(relative_path=(folder+'/' if folder else '')+name,
                    file_kind='regular', byte_length=len(raw), sha256=hashlib.sha256(raw).hexdigest()))
        finally:
            if folder:
                os.close(child)
    return inventory


def preserve_capture_bundle(session, capture_id, output, *, root=ROOT, expected_recording_sha256=None):
    """Keep exact original bytes and any copied prefix, including failed capture data."""
    root, session, output = Path(root).resolve(), Path(session).absolute(), Path(output).absolute()
    evidence = dict(capture_id=capture_id, status='failed', copied=False, byte_agreement=False,
                    recording_valid=False, expected_recording_sha256=expected_recording_sha256, errors=[])
    source = destination = archive = None
    before = after = retained = None
    try:
        if (not re.fullmatch(r'[0-9a-f]{32}', capture_id) or session.parent != root/'runtime/operator'
                or '..' in session.parts or '..' in output.parts
                or not output.is_relative_to(root/'runtime') or output.is_relative_to(session)):
            raise ValueError('Capture preservation requires exact owned runtime paths and capture ID')
        source_path = session/'synchronized'/capture_id
        archive_path = output/'capture-originals'
        evidence.update(source=str(source_path.relative_to(root)), destination=str((archive_path/capture_id).relative_to(root)))
        source = _directory(source_path)
        archive = _directory(archive_path, create=True)
        os.mkdir(capture_id, 0o700, dir_fd=archive)
        destination = os.open(capture_id, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW, dir_fd=archive)
        before = _capture_inventory(source)
        if not before['files']:
            raise ValueError('Owned synchronized capture has no retained original members')
        for folder in before['directories']:
            os.mkdir(folder, 0o700, dir_fd=destination)
        for item in before['files']:
            parts = item['relative_path'].split('/')
            source_child, destination_child = source, destination
            try:
                if len(parts) == 2:
                    source_child = os.open(parts[0], os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW, dir_fd=source)
                    destination_child = os.open(parts[0], os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW, dir_fd=destination)
                raw = _read(source_child, parts[-1], item['byte_length'])
                if len(raw) != item['byte_length'] or hashlib.sha256(raw).hexdigest() != item['sha256']:
                    raise ValueError('Capture source changed after its original inventory')
                _publish(destination_child, parts[-1], raw)
            finally:
                if source_child != source:
                    os.close(source_child)
                if destination_child != destination:
                    os.close(destination_child)
        evidence['copied'] = True
        after, retained = _capture_inventory(source), _capture_inventory(destination)
        if before != after or before != retained:
            raise ValueError('Capture source/destination file sets, sizes or SHA256 digests differ')
        evidence['byte_agreement'] = True
        if any(item['relative_path'] == 'recording.json' for item in before['files']):
            original = SyncRecording.load(source_path/'recording.json')
            copied = SyncRecording.load(archive_path/capture_id/'recording.json')
            if original.sha256 != copied.sha256 or (expected_recording_sha256 is not None and original.sha256 != expected_recording_sha256):
                raise ValueError('Preserved recording differs from the verified recording identity')
            evidence.update(recording_valid=True, recording_sha256=original.sha256)
        elif expected_recording_sha256 is not None:
            raise ValueError('Previously verified synchronized recording is missing')
        evidence['status'] = 'preserved'
    except Exception as error:
        evidence['errors'].append(dict(type=type(error).__name__, message=str(error)[:512]))
    finally:
        if destination is not None:
            if retained is None:
                try:
                    retained = _capture_inventory(destination)
                except Exception as error:
                    evidence['errors'].append(dict(type=type(error).__name__, message=str(error)[:512]))
            source_inventory = dict(before=before, after=after)
            for name, value in (('source-inventory', source_inventory), ('destination-inventory', retained), ('preservation', evidence)):
                filename = capture_id+'-'+name+'.json'
                try:
                    _publish(archive, filename, (json.dumps(value, indent=2)+'\n').encode())
                    evidence[name.replace('-', '_')] = str((output/'capture-originals'/filename).relative_to(root))
                except Exception as error:
                    evidence['status'] = 'failed'
                    evidence['errors'].append(dict(type=type(error).__name__, message=str(error)[:512]))
        for descriptor in (destination, archive, source):
            if descriptor is not None:
                os.close(descriptor)
    return evidence


def preserve_synchronized_originals(output, result, capture_id, *, root=ROOT, port, domain):
    """Run after console exit, retaining partial or unknown-start outcomes too."""
    evidence = dict(status='not_started', bundles=[], errors=[])
    result['synchronized_originals'] = evidence
    try:
        requested = result.get('synchronized_capture_readiness', {}).get('start_requests') == 1
        if capture_id is None and not requested:
            return evidence
        session = owned_console_session(Path(output)/'console.log', root, port, domain)
        identifiers = [capture_id] if capture_id is not None else []
        if capture_id is None:
            descriptor = _directory(session/'synchronized')
            try:
                identifiers = _capture_names(descriptor, MAX_RECORDINGS)
            finally:
                os.close(descriptor)
        if not identifiers:
            raise ValueError('Requested synchronized capture has no preserved output')
        for identifier in identifiers:
            verified = [item['recording_sha256'] for item in result['checks']
                        if item.get('id') == identifier and item.get('recording_sha256') is not None]
            if (any(type(digest) is not str or not re.fullmatch(r'[0-9a-f]{64}', digest) for digest in verified)
                    or len(set(verified)) > 1):
                raise ValueError('Conflicting or invalid verified recording identities')
            expected = verified[0] if verified else None
            bundle = preserve_capture_bundle(session, identifier, output, root=root, expected_recording_sha256=expected)
            evidence['bundles'].append(bundle)
            if bundle['status'] != 'preserved':
                evidence['errors'].extend(bundle['errors'])
        evidence['status'] = 'preserved' if not evidence['errors'] else 'failed'
    except Exception as error:
        evidence['status'] = 'failed'
        evidence['errors'].append(dict(type=type(error).__name__, message=str(error)[:512]))
    if evidence['status'] == 'failed':
        result['status'] = 'failed'
    return evidence


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--domain",type=int,required=True)
    parser.add_argument("--port",type=int,required=True)
    parser.add_argument("--output",type=Path,required=True)
    parser.add_argument("--synchronized",action="store_true",help="Also capture actual published 3D poses and replay the immutable image/pose bundle")
    args=parser.parse_args()
    if not 0<=args.domain<=232 or not 1024<=args.port<=65535:
        parser.error("Invalid isolated domain or port")
    output=args.output.absolute()
    if output.exists() or not output.is_relative_to((ROOT/"runtime").resolve()):
        parser.error("Output must be a new owned runtime directory")
    from ..workcells.verify_motion_reproduction import require_unused_domain
    require_unused_domain(args.domain)
    output.mkdir(parents=True)
    os.environ["ROS_DOMAIN_ID"]=str(args.domain)
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile,ReliabilityPolicy
    from sensor_msgs.msg import Image
    from rosgraph_msgs.msg import Clock
    from PIL import Image as Pixels
    rclpy.init()
    node=Node("image_transport_verification")
    qos=QoSProfile(depth=1,reliability=ReliabilityPolicy.BEST_EFFORT)
    publisher=node.create_publisher(Image,"/diagnostic/image",qos)
    latest_clock=[None]
    node.create_subscription(Clock,"/clock",lambda message:latest_clock.__setitem__(0,message.clock),qos)
    base=f"http://127.0.0.1:{args.port}"
    token=None
    capture_id=None
    records=[]
    result=dict(schema_version=1,kind="diagnostic-rgb8-transport-verification",status="running",
        domain=args.domain,checks=records,note="Synthetic image transport only; no hardware camera, calibration, native-step alignment or latency claim")
    log=(output/"console.log").open("wb")
    process=subprocess.Popen(["/usr/bin/python3","-m","mujoco_ros2_manipulation.operator_console",
        "--domain",str(args.domain),"--port",str(args.port)]+([] if args.synchronized else ["--no-render"])+[
        "--image-topic","/diagnostic/image","--image-clock-domain","ros_simulation"],
        cwd=ROOT,env=os.environ.copy(),stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    def http(path,payload=None,raw=False):
        return http_request(base,path,payload,token=token,raw=raw)
    sequence=0
    def publish(encoding="rgb8"):
        nonlocal sequence
        if latest_clock[0] is None:return
        image=Image();image.header.stamp=latest_clock[0];image.header.frame_id="diagnostic_optical"
        image.width,image.height,image.step,image.encoding,image.is_bigendian=64,48,192,encoding,0
        image.data=pattern(sequence);sequence+=1;publisher.publish(image)
    def wait(predicate,label,seconds=15,*,frames=False):
        deadline=time.monotonic()+seconds;next_frame=0
        while time.monotonic()<deadline:
            if process.poll() is not None:raise RuntimeError("Console exited: "+label)
            rclpy.spin_once(node,timeout_sec=.01)
            if frames and time.monotonic()>=next_frame:
                publish();next_frame=time.monotonic()+.06
            try:
                value=predicate()
                if value:return value
            except (ConnectionError,OSError):pass
            time.sleep(.025)
        raise TimeoutError(label)
    try:
        boot=wait(lambda:http("/api/bootstrap"),"console bootstrap")
        token=boot["token"]
        wait(lambda:http("/api/state")["lifecycle"]["phase"]=="idle","idle")
        time.sleep(2.1)
        http("/api/start",dict(native_gui=False,planning_case="none"))
        ready=wait(lambda:(lambda d:d if d["can_submit"] else None)(http("/api/state")),"simulation ready",70,frames=True)
        source=wait(lambda:(lambda d:d if d.get("status")=="ready" and d.get("latest") else None)(http("/api/image-source")),"encoded image",frames=True)
        old_generation,old_sequence=source["generation"],source["latest"]["sequence"]
        pixels=http(f"/api/image-source/frame?generation={old_generation}&sequence={old_sequence}",raw=True)
        with Pixels.open(io.BytesIO(pixels)) as decoded:
            assert decoded.size==(64,48) and decoded.format=="JPEG"
        records.append(dict(name="actual_ros_rgb8_to_http_jpeg",generation=old_generation,sequence=old_sequence,
            jpeg_sha256=hashlib.sha256(pixels).hexdigest(),status=source["status"],source_clock_domain=source["clock_domain"]))
        if args.synchronized:
            capture_id=start_synchronized_capture(http,wait,result)
            wait(lambda:http("/api/sync-recordings")["capture"]["image_count"]>=3,"initial synchronized image samples",frames=True)
            records.append(dict(name="synchronized_capture_started",id=capture_id))
        before=source["counters"]["rejected"]
        for _ in range(8):
            rclpy.spin_once(node,timeout_sec=.025);publish("bgr8")
        invalid=wait(lambda:(lambda d:d if d["counters"]["rejected"]>before else None)(http("/api/image-source")),"invalid encoding rejected")
        assert invalid["invalid_reasons"].get("invalid_encoding",0)>0
        records.append(dict(name="invalid_encoding_rejected",rejected=invalid["counters"]["rejected"]))
        stale=wait(lambda:(lambda d:d if d["status"]=="stale" and (not args.synchronized or d["latest"]["receive_age_sec"]>1.) else None)(http("/api/image-source")),"publisher present but frames stale")
        records.append(dict(name="stale_without_publisher_loss",status=stale["status"],publisher_count=stale["publisher_count"]))
        wait(lambda:http("/api/image-source")["status"]=="ready","recovered frame",frames=True)
        target=dict(boot["config"]["home_rad"])
        for instance in boot["config"]["instances"]:target[instance["joints"][0]]+=.03
        payload=command(target);http("/api/command",payload)
        terminal=wait(lambda:(lambda d:next((r for r in d["history"] if r["request_id"]==payload["request_id"] and r["status"] in ("succeeded","failed","cancelled","rejected")),None))(http("/api/state")),"actual motion terminal",50,frames=True)
        assert terminal["status"]=="succeeded",terminal
        analysis=http("/api/analysis/"+payload["request_id"])
        assert analysis["read_only"] and analysis["request_id"]==payload["request_id"]
        assert any(item["name"]=="goal_result" for item in analysis["timeline"]["items"])
        records.append(dict(name="request_image_joint_clock_observations",request_id=payload["request_id"],
            result=terminal["status"],analysis=analysis))
        if args.synchronized:
            wait(lambda:http("/api/sync-recordings")["capture"]["image_count"]>=3,"captured image samples",frames=True)
            http("/api/sync-recordings/stop",dict(id=capture_id))
            catalog=wait(lambda:(lambda d:d if d["capture"]["status"] in ("completed","failed") else None)(http("/api/sync-recordings")),"saved synchronized capture")
            assert catalog["capture"]["status"]=="completed",catalog
            prefix="/api/sync-recordings/"+capture_id
            view=http(prefix)["view"]
            count_before=len(http("/api/state")["history"])
            # Locate only this owned console's output named in its own log.
            folder=owned_console_session(output/"console.log",ROOT,args.port,args.domain)
            recording=SyncRecording.load(folder/"synchronized"/capture_id/"recording.json")
            assert recording.sha256==view["recording_sha256"]
            image_sample=recording.image_frames[1]
            at=(int(image_sample["stamp_ns"])-recording.origin_ns)/1e9
            chosen=http(prefix+"/control",dict(action="seek",time_sec=at))
            assert chosen["selection"]["image"]["stamp_ns"]==image_sample["stamp_ns"]
            assert chosen["selection"]["image_status"]=="ready"
            assert chosen["selection"]["pose_status"]=="ready",chosen["selection"]
            digest=chosen["selection"]["image"]["jpeg_sha256"]
            assert hashlib.sha256(http(prefix+"/images/"+digest+".jpg",raw=True)).hexdigest()==digest
            pose_stamp=int(chosen["selection"]["pose"]["source"]["stamp_ns"])
            assert pose_stamp<=int(chosen["selection"]["cursor_ns"])
            old_binding=chosen["binding_id"]
            advanced=http(prefix+"/control",dict(action="next"))
            assert advanced["binding_id"]!=old_binding
            # The deliberate input pause above must remain a missing image gap.
            gaps=[(a,b) for a,b in zip(recording.image_frames,recording.image_frames[1:])
                if int(b["stamp_ns"])-int(a["stamp_ns"])>700_000_000]
            assert gaps,"No preserved synthetic image gap"
            gap_at=(int(gaps[0][0]["stamp_ns"])+600_000_000-recording.origin_ns)/1e9
            gap=http(prefix+"/control",dict(action="seek",time_sec=gap_at))["selection"]
            assert gap["image_status"]=="missing" and gap["image"] is None
            assert len(http("/api/state")["history"])==count_before
            records.append(dict(name="synchronized_capture_and_readonly_replay",id=capture_id,
                recording_path=str(folder/"synchronized"/capture_id/"recording.json"),
                recording_sha256=recording.sha256,pose_count=len(recording.poses),image_count=len(recording.image_frames),
                gap_count=len(gaps),chosen_cursor_ns=chosen["selection"]["cursor_ns"],pose_stamp_ns=str(pose_stamp),
                image_stamp_ns=image_sample["stamp_ns"],no_command_replay=True))
        history_count=len(http("/api/state")["history"])
        node.destroy_publisher(publisher)
        disconnected=wait(lambda:(lambda d:d if d["publisher_count"]==0 else None)(http("/api/image-source")),"publisher disconnected")
        try:http(f"/api/image-source/frame?generation={old_generation}&sequence={old_sequence}",raw=True)
        except HTTPError as error:assert error.code==410
        else:raise AssertionError("Old generation frame was accepted")
        publisher=node.create_publisher(Image,"/diagnostic/image",qos)
        new=wait(lambda:(lambda d:d if d["status"]=="ready" and d["generation"]!=old_generation else None)(http("/api/image-source")),"new publisher generation",frames=True)
        assert len(http("/api/state")["history"])==history_count
        records.append(dict(name="publisher_reconnected_without_command_replay",old_generation=old_generation,
            disconnected_generation=disconnected["generation"],new_generation=new["generation"],history_count=history_count))
        assert new["memory"]["encoded_bytes"]<=8*1024*1024 and new["memory"]["encoded_frames"]<=64
        result.update(status="passed",image_snapshot=new)
    except BaseException as error:
        result.update(status="failed",error=dict(type=type(error).__name__,message=str(error)))
        if getattr(error,"verification_detail",None) is not None:
            result["http_failure"]=error.verification_detail
    finally:
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:process.wait(timeout=30)
            except subprocess.TimeoutExpired:
                result["cleanup_forced"]=True;os.killpg(process.pid,signal.SIGKILL);process.wait(timeout=5)
        result["console_exit_code"]=process.returncode
        result["cleanup_clean"]=process.returncode==0 and not result.get("cleanup_forced",False)
        log.close()
        if args.synchronized:
            preserve_synchronized_originals(output,result,capture_id,root=ROOT,port=args.port,domain=args.domain)
        node.destroy_node();rclpy.shutdown()
        if not result["cleanup_clean"]:result["status"]="failed"
        atomic_json(output/"result.json",result)
    print(result["status"].upper()+": "+str(output/"result.json"),flush=True)
    return 0 if result["status"]=="passed" else 1


if __name__=="__main__":raise SystemExit(main())
