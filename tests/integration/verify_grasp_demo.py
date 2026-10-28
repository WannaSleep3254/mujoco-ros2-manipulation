"""Verify the owned loopback FR5 demo's normal run and mid-carry cancellation.

This uses Python MuJoCo 3.14.0, a synthetic tool/block and local DLS. It does
not start ROS, native controllers or any existing viewer server. All outcomes,
including failures and partial jobs, remain in a new explicitly named folder.
"""
import argparse
import hashlib
import http.client
import json
from pathlib import Path
import threading
import time
import uuid

from mujoco_ros2_manipulation.grasping.demo_server import DemoApplication, serve
from mujoco_ros2_manipulation.grasping.live_worker import ENGINE, source_digests, source_documents
from mujoco_ros2_manipulation.paths import ROOT
from mujoco_ros2_manipulation.verification.reports import atomic_json


def require(condition, message):
    if not condition:
        raise AssertionError(message)


def verify(source, output):
    output = Path(output).resolve()
    output.mkdir(parents=True, exist_ok=False)
    raw_source = Path(__file__).read_bytes()
    (output/"verify_grasp_demo.py").write_bytes(raw_source)
    receipt = dict(schema_version=1, passed=False, engine=ENGINE,
        verifier_sha256=hashlib.sha256(raw_source).hexdigest(), checks=[], jobs=[],
        scope="Independent fixed FR5/synthetic-gripper/block local DLS demo; not ROS/native/MoveIt/physical robot validation",
        physical_stop_inferred_from_process_exit=False, automatic_retry=False, w07_complete=False,
        earlier_prototypes_note="Earlier a/b/c retain uncommitted implementation hashes only; current sources do not replace those originals")
    app = server = thread = None
    try:
        app = DemoApplication(source, output/"jobs")
        server = serve(app, 0)
        port = server.server_address[1]
        origin = "http://127.0.0.1:"+str(port)
        thread = threading.Thread(target=server.serve_forever, daemon=True)
        thread.start()
        receipt["ephemeral_port"] = port
        receipt["source"] = app.source
        receipt["implementation_sha256"] = source_digests(source_documents())
        token = server.token

        def request(method, path, value=None, *, headers=None, expected=200, binary=False):
            extra = {} if headers is None else dict(headers)
            if method == "POST":
                extra = dict({"Origin": origin, "X-Operator-Token": token,
                              "Content-Type": "application/json"}, **extra)
            client = http.client.HTTPConnection("127.0.0.1", port, timeout=3.)
            try:
                client.request(method, path, None if value is None else json.dumps(value).encode(), extra)
                response = client.getresponse()
                raw = response.read(4*1024*1024+1)
                require(len(raw) <= 4*1024*1024, "Unbounded HTTP verification response")
                require(response.status == expected,
                        f"Unexpected HTTP {response.status} for {method} {path}: "+raw[:400].decode(errors="replace"))
                return raw if binary else json.loads(raw)
            finally:
                client.close()

        initial = request("GET", "/api/demo")
        require(initial["source"]["engine"] == ENGINE and initial["job"] is None and initial["view"] is None,
                "Before start the fixed source must have no fabricated live pose")
        require(initial["source"]["model_sha256"] == app.physical_manifest["mjcf_sha256"], "Initial model SHA differs")
        receipt["checks"].append("Engine/model SHA checked; no pre-start live pose")
        model_sha = initial["source"]["model_sha256"]
        first_body = dict(request_id=uuid.uuid4().hex, model_sha256=model_sha)
        for headers in ({"Origin": "https://foreign.invalid"}, {"X-Operator-Token": "invalid"}):
            request("POST", "/api/demo/start", first_body, headers=headers, expected=403)
        request("POST", "/api/demo/start", dict(first_body, model_sha256="0"*64), expected=400)
        require(not app.jobs, "Rejected starts created a child process")
        receipt["checks"].append("Wrong origin/token/model refuse start without spawning")

        first = request("POST", "/api/demo/start", first_body)
        first_id, first_binding = first["job"]["job_id"], first["view"]["binding_id"]
        duplicate = request("POST", "/api/demo/start", first_body)
        require(duplicate["job"]["job_id"] == first_id and len(app.jobs) == 1
                and duplicate["request_replay"]["new_job_started"] is False, "Duplicate start spawned another worker")
        request("POST", "/api/demo/start", dict(first_body, request_id=uuid.uuid4().hex), expected=409)
        manifest_raw = request("GET", "/api/view/manifest?binding_id="+first_binding, binary=True)
        require(hashlib.sha256(manifest_raw).hexdigest() == first["view"]["manifest_descriptor"]["sha256"],
                "HTTP manifest is not bound to the declared digest")
        manifest = json.loads(manifest_raw)
        total_assets = 0
        for asset in manifest["assets"]:
            digest = asset["sha256"]
            raw = request("GET", "/api/view/assets/"+digest, binary=True)
            require(hashlib.sha256(raw).hexdigest() == digest, "Mesh response SHA differs")
            total_assets += len(raw)
        require(total_assets <= 2*1024*1024, "Unexpected demo geometry byte growth")
        request("GET", "/api/view/assets/../../README.md", expected=409)
        receipt["display_assets"] = dict(count=len(manifest["assets"]), byte_length=total_assets,
            manifest_sha256=hashlib.sha256(manifest_raw).hexdigest())
        receipt["checks"].append("Idempotent start, one owned child, bound manifest and content-addressed meshes")

        def wait_for(outcome=None, phase=None, timeout=27.):
            deadline, last_sequence = time.monotonic()+timeout, -1
            pose_count = 0
            while time.monotonic() < deadline:
                state = request("GET", "/api/demo")
                job, view = state["job"], state["view"]
                require(job["model_sha256"] == model_sha and not job["timed_out"], "Worker model changed or watchdog expired")
                require(not job["error"] or job["status"] == "cancelled", "Worker/display failure: "+str(job["error"]))
                if view["pose_ready"]:
                    pose = request("GET", "/api/view/pose?binding_id="+view["binding_id"])
                    require(pose["binding_id"] == view["binding_id"] and pose["binding"] == view["binding"],
                            "Pose binding does not match the active job")
                    require(pose["sequence"] >= last_sequence, "HTTP source sequence regressed")
                    last_sequence = pose["sequence"]
                    pose_count += 1
                if phase is not None and job["phase"] == phase and job["process_running"]:
                    return state, pose_count
                if not job["process_running"] and job["process_group_gone"] is True:
                    require(outcome is not None and job["status"] == outcome,
                            "Unexpected terminal worker result: "+str(job))
                    return state, pose_count
                time.sleep(.12)
            raise TimeoutError("Bounded wait exceeded for fixed demo "+str((outcome, phase)))

        normal, normal_poses = wait_for(outcome="succeeded")
        require(normal_poses > 10 and normal["job"]["worker_result_observed"], "Normal run lacks actual HTTP poses/result")
        normal_result = app.jobs[first_id].worker_result
        require(normal_result["physics"]["integrated_steps"] == 6300 and normal_result["task"]["physical_grasp"] == "released"
                and normal_result["task"]["planning_attachment"] == "world", "Normal grasp/release evidence differs")
        receipt["jobs"].append(dict(kind="normal", job=normal["job"], http_pose_samples=normal_poses))
        receipt["checks"].append("Real FR5 six-axis 1x simulation: grasp, lift, carry, release, settle; 6300 actual steps")

        second_body = dict(request_id=uuid.uuid4().hex, model_sha256=model_sha)
        second = request("POST", "/api/demo/start", second_body)
        require(second["job"]["generation"] == normal["job"]["generation"]+1
                and second["view"]["binding_id"] != first_binding, "A new run reused an old generation/binding")
        request("GET", "/api/view/pose?binding_id="+first_binding, expected=409)
        cancel_body = dict(job_id=second["job"]["job_id"], generation=second["job"]["generation"], model_sha256=model_sha)
        request("POST", "/api/demo/cancel", dict(cancel_body, job_id=first_id), expected=400)
        request("POST", "/api/demo/cancel", dict(cancel_body, model_sha256="0"*64), expected=400)
        require(not app.current.cancel_requested, "Invalid cancellation affected the current job")
        carrying, carrying_poses = wait_for(phase="carry", timeout=17.)
        request("POST", "/api/demo/cancel", cancel_body)
        request("POST", "/api/demo/cancel", cancel_body)
        stopped, cancel_poses = wait_for(outcome="cancelled", timeout=7.)
        result = app.current.worker_result
        require(result["cancellation"]["request_observed"] is True
                and result["cancellation"]["phase"] == "carry"
                and result["cancellation"]["physical_stop_confirmed"] is False
                and result["cancellation"]["action_state"] == "not_applicable", "Cancellation/physical-stop scope differs")
        require(not any(row["phase"] in {"lower", "open", "retract"} for row in result["segments"]),
                "A subsequent ordinary task stage ran after cancellation")
        require(result["task"]["planning_attachment"] == "attached" and result["task"]["physical_grasp"] != "released",
                "Cancellation automatically released or erased attachment evidence")
        require(stopped["job"]["cancel_delivery"] == "worker_observed" and not stopped["job"]["timed_out"],
                "Owned worker did not distinguish user cancellation")
        receipt["jobs"].append(dict(kind="mid-carry-cancel", job=stopped["job"],
            http_pose_samples=carrying_poses+cancel_poses, cancellation=result["cancellation"]))
        receipt["checks"].append("Mid-carry cancellation acknowledged; no lower/open/retract, no Action, physical stop unconfirmed")

        for job in app.jobs.values():
            require(job.finished and job.process_group_gone and job.process.poll() is not None, "Owned process group remains")
            original = json.loads((job.directory/"source-pin.json").read_text())
            snapshot = job.directory/"implementation_sources"
            for name, digest in original.items():
                require(hashlib.sha256((snapshot/name).read_bytes()).hexdigest() == digest, "Source snapshot digest differs")
            require((job.directory/"run/fr5_grasp.xml").is_file() and (job.directory/"run/manifest.json").is_file()
                    and (job.directory/"latest.json").is_file() and (job.directory/"stdout.log").is_file(),
                    "Original job artifact is missing")
        require(source_digests(source_documents()) == receipt["implementation_sha256"], "Sources changed during actual HTTP verification")
        receipt["checks"].append("Both owned process groups gone; exact implementation snapshots, MJCF, manifest, qpos and raw results retained")
        receipt["passed"] = True
    except Exception as error:
        receipt["error"] = dict(type=type(error).__name__, message=str(error)[:4096])
    finally:
        if server is not None:
            server.shutdown()
            server.server_close()
        if thread is not None:
            thread.join(timeout=2.)
        if app is not None:
            app.close()
            receipt["cleanup"] = [dict(job_id=job.identifier, process_exited=job.process.poll() is not None,
                process_group_gone=job.process_group_gone) for job in app.jobs.values()]
        atomic_json(output/"verification.json", receipt)
    return receipt


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", default=str(ROOT/"runtime/generated/workcells/fr5_ur5e"))
    parser.add_argument("--output", required=True, help="A new directory; all failures and partial jobs are retained")
    args = parser.parse_args(argv)
    result = verify(args.source, args.output)
    print(json.dumps(dict(passed=result["passed"], output=str(Path(args.output).resolve()),
                         error=result.get("error"), w07_complete=False), indent=2))
    return 0 if result["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
