"""Actual ROS RGB8 -> console worker -> read-only HTTP functional verification."""
import argparse
import hashlib
import io
import json
import os
from pathlib import Path
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
    records=[]
    result=dict(schema_version=1,kind="diagnostic-rgb8-transport-verification",status="running",
        domain=args.domain,checks=records,note="Synthetic image transport only; no hardware camera, calibration, native-step alignment or latency claim")
    log=(output/"console.log").open("wb")
    process=subprocess.Popen(["/usr/bin/python3","-m","mujoco_ros2_manipulation.operator_console",
        "--domain",str(args.domain),"--port",str(args.port)]+([] if args.synchronized else ["--no-render"])+[
        "--image-topic","/diagnostic/image","--image-clock-domain","ros_simulation"],
        cwd=ROOT,env=os.environ.copy(),stdout=log,stderr=subprocess.STDOUT,start_new_session=True)
    def http(path,payload=None,raw=False):
        headers={} if payload is None else {"Content-Type":"application/json","Origin":base,"X-Operator-Token":token}
        request=Request(base+path,data=None if payload is None else json.dumps(payload).encode(),headers=headers)
        with urlopen(request,timeout=3) as reply:
            data=reply.read()
            return data if raw else json.loads(data)
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
        capture_id=None
        if args.synchronized:
            wait(lambda:http("/api/state")["view"].get("pose_ready"),"published 3D pose",30,frames=True)
            capture_id=http("/api/sync-recordings/start",{})["id"]
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
            text=(output/"console.log").read_text()
            folder=Path(next(line.split(" session=",1)[1] for line in text.splitlines() if line.startswith("OPERATOR_READY ")))
            from .synchronized_recording import SyncRecording
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
    finally:
        if process.poll() is None:
            process.send_signal(signal.SIGINT)
            try:process.wait(timeout=30)
            except subprocess.TimeoutExpired:
                result["cleanup_forced"]=True;os.killpg(process.pid,signal.SIGKILL);process.wait(timeout=5)
        result["console_exit_code"]=process.returncode
        result["cleanup_clean"]=process.returncode==0 and not result.get("cleanup_forced",False)
        log.close();node.destroy_node();rclpy.shutdown()
        if not result["cleanup_clean"]:result["status"]="failed"
        atomic_json(output/"result.json",result)
    print(result["status"].upper()+": "+str(output/"result.json"),flush=True)
    return 0 if result["status"]=="passed" else 1


if __name__=="__main__":raise SystemExit(main())
