"""Loopback-only fixed FR5 grasp simulation, separate from ROS manipulation.

The browser can start this declared scenario or request cancellation of its
current owned job. It cannot choose a path, model, goal, executable or ROS API.
"""
import argparse
from copy import deepcopy
from dataclasses import dataclass
import hashlib
import hmac
from http.server import ThreadingHTTPServer
from importlib.resources import files
import json
import math
import os
from pathlib import Path
import re
import secrets
import signal
import subprocess
import sys
import threading
import time
from urllib.parse import parse_qs, urlsplit
import uuid

from ..operator_console.recorded_server import RecordingHandler
from ..operator_console.scene_host import _json, _regular, _require
from ..operator_console.view_binding import binding_id, envelope, make_binding
from ..verification.reports import atomic_json
from ..workcells.motion_request import request_id
from .display_export import LiveGraspDisplay
from .fr5_model import ARM_JOINTS, build_model
from .live_worker import ENGINE, MAX_LATEST_BYTES, MAX_RESULT_BYTES, source_digests, source_documents


ACTIVE = frozenset(("starting", "running", "cancelling"))
MAX_JOBS = 32
FRAME_FIELDS = frozenset(("schema_version", "job_id", "generation", "binding_id", "model_sha256",
    "manifest_sha256", "engine", "sequence", "observed_monotonic_ns", "frame", "task",
    "integrated_steps", "terminal_outcome"))
STAGES = frozenset(("approaching", "closing", "extracting", "holding", "carrying", "releasing",
                    "settling", "succeeded", "failed", "cancelled"))


def _sha(raw):
    return hashlib.sha256(raw).hexdigest()


def _process_group_gone(pid):
    try:
        os.killpg(pid, 0)
    except ProcessLookupError:
        return True
    except PermissionError:
        return False
    return False


@dataclass
class OwnedJob:
    identifier: str
    request_identifier: str
    generation: int
    binding: dict
    binding_identifier: str
    directory: Path
    process: object
    log: object
    started: float
    status: str = "starting"
    cancel_requested: bool = False
    cancel_at: object = None
    signal_stage: int = 0
    cancel_delivery: str = "not_requested"
    timed_out: bool = False
    frame: object = None
    frame_digest: object = None
    pose: object = None
    display_error: object = None
    worker_result: object = None
    result_sha256: object = None
    error: object = None
    finished: bool = False
    process_group_gone: bool = False


class DemoApplication:
    """Own one fixed subprocess at a time and preserve every job directory."""
    def __init__(self, source_directory, output_root, *, clock=time.monotonic,
                 _process_factory=subprocess.Popen, _timeout_sec=60.):
        if type(_timeout_sec) not in (int, float) or not 1 <= _timeout_sec <= 60:
            raise ValueError("Owned demo timeout must be within 1..60 seconds")
        import mujoco
        if "MuJoCo "+mujoco.__version__ != ENGINE or mujoco.mj_versionString() != "3.14.0":
            raise ValueError("This independent demo requires the tested MuJoCo 3.14.0 engine")
        self.source_directory = Path(source_directory).resolve()
        self.workspace = self.source_directory.parents[3]
        self.model_raw, self.physical_manifest = build_model(self.source_directory)
        self.display = LiveGraspDisplay(self.model_raw, self.physical_manifest)
        self.source_pin = source_digests(source_documents())
        self.output_root = Path(output_root).resolve()
        self.output_root.mkdir(parents=True, exist_ok=False)
        atomic_json(self.output_root/"source-pin.json", self.source_pin)
        self.clock, self.process_factory, self.timeout_sec = clock, _process_factory, float(_timeout_sec)
        self.session_id = uuid.uuid4().hex
        self.jobs, self.requests = {}, {}
        self.current = None
        self.generation = 0
        self.lock = threading.RLock()
        self.closing = threading.Event()
        self.source = dict(id="scene:fr5_grasp_live", label="FR5 · 검증용 평행 그리퍼 · 블록",
            model_sha256=self.physical_manifest["mjcf_sha256"],
            manifest_sha256=self.physical_manifest["manifest_sha256"], engine=ENGINE,
            method="local site-Jacobian DLS", realtime_factor=1,
            scope="독립 MuJoCo 집기 시연 · ROS·MoveIt·실물 로봇과 연결되지 않음")
        self.monitor = threading.Thread(target=self._watch, daemon=True)
        self.monitor.start()

    def _identity(self, value):
        if type(value) is not str or value != self.source["model_sha256"]:
            raise ValueError("고정된 시연 모델 SHA가 다릅니다. 이 화면을 다시 확인하세요.")

    def start(self, value):
        if type(value) is not dict or set(value) != {"request_id", "model_sha256"}:
            raise ValueError("고정 시연의 새 요청 ID와 모델 SHA만 받습니다.")
        identifier = request_id(value["request_id"])
        self._identity(value["model_sha256"])
        with self.lock:
            if self.closing.is_set():
                raise RuntimeError("시연 서버를 종료하고 있습니다.")
            if identifier in self.requests:
                if self.requests[identifier] not in self.jobs:
                    raise RuntimeError("이 요청은 프로세스 시작에 실패했습니다. 기록을 확인하고 새 요청 ID를 사용하세요.")
                result = self.snapshot()
                result["request_replay"] = dict(request_id=identifier, job_id=self.requests[identifier], new_job_started=False)
                return result
            if self.current is not None and not self.current.finished:
                raise RuntimeError("이 서버가 시작한 시연이 아직 실행·정리 중입니다.")
            if len(self.requests) >= MAX_JOBS:
                raise RuntimeError("이 서버의 시연 보존 상한에 도달했습니다. 새 서버 작업 공간이 필요합니다.")
            if source_digests(source_documents()) != self.source_pin:
                raise RuntimeError("시연 코드가 바뀌었습니다. 이 전용 서버를 다시 시작하세요.")
            # Recheck manufacturer/model identity before spawning a process.
            model, manifest = build_model(self.source_directory)
            if model != self.model_raw or manifest != self.physical_manifest:
                raise RuntimeError("시연 원본 모델이 바뀌었습니다. 새 서버에서 다시 확인하세요.")
            self.generation += 1
            job_id = uuid.uuid4().hex
            binding = make_binding(self.session_id, self.generation, self.source["id"], "live",
                                   job_id=job_id, run_id=job_id)
            bound = binding_id(binding)
            directory = self.output_root/job_id
            directory.mkdir(exist_ok=False)
            self.requests[identifier] = job_id
            atomic_json(directory/"source-pin.json", self.source_pin)
            log = (directory/"stdout.log").open("w")
            command = [sys.executable, "-m", "mujoco_ros2_manipulation.grasping.live_worker",
                "--source", str(self.source_directory), "--job-directory", str(directory),
                "--job-id", job_id, "--generation", str(self.generation), "--binding-id", bound,
                "--model-sha256", self.source["model_sha256"],
                "--manifest-sha256", self.source["manifest_sha256"]]
            environment = dict(os.environ)
            for name in ("AMENT_PREFIX_PATH", "COLCON_PREFIX_PATH", "CMAKE_PREFIX_PATH", "LD_LIBRARY_PATH",
                         "LD_PRELOAD", "ROS_DOMAIN_ID", "ROS_LOG_DIR", "RMW_IMPLEMENTATION",
                         "CYCLONEDDS_URI", "FASTRTPS_DEFAULT_PROFILES_FILE"):
                environment.pop(name, None)
            environment.update(PYTHONPATH=str(Path(__file__).resolve().parents[2]),
                MUJOCO_ROS2_WORKSPACE=str(self.workspace), PYTHONDONTWRITEBYTECODE="1", PYTHONUNBUFFERED="1",
                OPENBLAS_NUM_THREADS="1", OMP_NUM_THREADS="1")
            try:
                process = self.process_factory(command, cwd=self.workspace, env=environment,
                    stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            except Exception as error:
                log.close()
                atomic_json(directory/"owner-receipt.json", dict(schema_version=1, job_id=job_id,
                    status="failed", error=dict(type=type(error).__name__, message=str(error)[:1024]),
                    process_started=False, physical_stop_confirmed=False))
                raise RuntimeError("소유 시연 프로세스를 시작하지 못했습니다.") from error
            job = OwnedJob(job_id, identifier, self.generation, binding, bound,
                           directory, process, log, self.clock())
            self.jobs[job_id] = self.current = job
            self._save_owner(job)
            return self.snapshot()

    def _send_owned(self, job, sig):
        if job.process.poll() is not None:
            return
        try:
            os.killpg(job.process.pid, sig)
        except ProcessLookupError:
            pass

    def _begin_cancel(self, job, *, timeout=False):
        if not job.cancel_requested:
            job.cancel_requested, job.cancel_at = True, self.clock()
            job.status, job.cancel_delivery = "cancelling", "signal_sent"
            job.timed_out = timeout
            self._send_owned(job, signal.SIGINT)
            job.signal_stage = 1
            self._save_owner(job)

    def cancel(self, value):
        if type(value) is not dict or set(value) != {"job_id", "generation", "model_sha256"}:
            raise ValueError("현재 시연 작업·세대·모델 SHA만 받습니다.")
        self._identity(value["model_sha256"])
        with self.lock:
            job = self.current
            if (job is None or value["job_id"] != job.identifier or type(value["generation"]) is not int
                    or value["generation"] != job.generation):
                raise ValueError("취소할 현재 시연의 식별자가 다릅니다.")
            if not job.finished:
                self._begin_cancel(job)
            return self.snapshot()

    def _read_latest(self, job):
        path = job.directory/"latest.json"
        if not path.exists() or job.display_error is not None:
            return
        try:
            raw = _regular(path, MAX_LATEST_BYTES)
            digest = _sha(raw)
            if digest == job.frame_digest:
                return
            value = _json(raw)
            _require(type(value) is dict and set(value) == FRAME_FIELDS, "잘못된 시연 관측 형식입니다.")
            _require(type(value["schema_version"]) is int and value["schema_version"] == 1
                and value["job_id"] == job.identifier and type(value["generation"]) is int
                and value["generation"] == job.generation and value["binding_id"] == job.binding_identifier
                and value["model_sha256"] == self.source["model_sha256"]
                and value["manifest_sha256"] == self.source["manifest_sha256"] and value["engine"] == ENGINE,
                "시연 관측의 작업·모델·엔진 식별자가 다릅니다.")
            sequence, observed = value["sequence"], value["observed_monotonic_ns"]
            _require(type(sequence) is int and 1 <= sequence <= 5000
                and type(observed) is int and 0 < observed <= time.monotonic_ns()+100_000_000,
                "시연 관측의 순번·시각이 잘못됐습니다.")
            frame, task = value["frame"], value["task"]
            _require(type(frame) is dict and frame.get("observed_monotonic_ns") == observed
                and type(frame.get("simulation_sec")) in (int, float)
                and math.isfinite(frame["simulation_sec"]) and 0 <= frame["simulation_sec"] <= 60
                and type(task) is dict and task.get("stage") in STAGES
                and type(task.get("events")) is list and len(task["events"]) <= 32
                and type(value["integrated_steps"]) is int and 0 <= value["integrated_steps"] <= 30000
                and value["terminal_outcome"] in (None, "succeeded", "failed", "cancelled"),
                "시연 관측의 단계·시계·물리 상태가 잘못됐습니다.")
            if job.frame is not None:
                _require(sequence > job.frame["sequence"] and observed >= job.frame["observed_monotonic_ns"]
                    and frame["simulation_sec"] >= job.frame["frame"]["simulation_sec"],
                    "시연 순번·시계가 재전송되거나 역행했습니다.")
            pose = self.display.pose(frame)
            job.frame, job.frame_digest, job.pose = value, digest, pose
            if not job.cancel_requested:
                job.status = "running"
        except (ValueError, KeyError, OSError, TypeError) as error:
            job.display_error = str(error)[:1024]
            self._save_owner(job)

    def _finish(self, job):
        if job.finished:
            return
        job.process.wait(timeout=0)
        result_path = job.directory/"run/result.json"
        if result_path.exists():
            try:
                raw = _regular(result_path, MAX_RESULT_BYTES)
                result = _json(raw)
                identity = result.get("live_job") if type(result) is dict else None
                _require(type(identity) is dict and identity.get("job_id") == job.identifier
                    and type(identity.get("generation")) is int and identity.get("generation") == job.generation
                    and identity.get("binding_id") == job.binding_identifier and identity.get("engine") == ENGINE
                    and identity.get("model_sha256") == self.source["model_sha256"]
                    and identity.get("manifest_sha256") == self.source["manifest_sha256"]
                    and type(result.get("schema_version")) is int and result.get("schema_version") == 1
                    and result.get("prototype") == "fr5_parallel_gripper_v1"
                    and result.get("manifest_sha256") == self.source["manifest_sha256"]
                    and result.get("engine") == ENGINE and result.get("outcome") in ("succeeded", "failed", "cancelled"),
                    "최종 시연 결과의 식별자가 다릅니다.")
                if result["outcome"] == "cancelled":
                    cancellation = result.get("cancellation") or {}
                    _require(cancellation.get("request_observed") is True
                        and cancellation.get("physical_stop_confirmed") is False
                        and cancellation.get("action_state") == "not_applicable",
                        "취소 관측과 물리 정지 미확인 상태를 구분할 수 없습니다.")
                job.worker_result = result
                job.result_sha256 = _sha(raw)
                job.status = result["outcome"]
                job.error = deepcopy(result.get("error"))
                if (result.get("cancellation") or {}).get("request_observed") is True:
                    job.cancel_delivery = "worker_observed"
                if job.process.returncode not in (0, 130) and job.status == "succeeded":
                    job.status = "failed"
                    job.error = dict(type="WorkerExit", message="성공 결과 후 프로세스 종료가 정상적이지 않습니다.")
            except (ValueError, TypeError, KeyError, OSError) as error:
                job.status, job.error = "failed", dict(type=type(error).__name__, message=str(error)[:1024])
        else:
            job.status = "interrupted" if job.cancel_requested else "failed"
            job.error = dict(type="MissingWorkerResult", message="프로세스는 종료됐지만 최종 물리 결과는 확인하지 못했습니다.")
        if job.timed_out:
            job.status = "failed"
            job.error = dict(type="DemoTimeout", message="시연 제한 시간을 초과해 소유 프로세스를 정리했습니다.",
                             worker_outcome=None if job.worker_result is None else job.worker_result["outcome"])
        job.finished = True
        job.process_group_gone = _process_group_gone(job.process.pid)
        if not job.log.closed:
            job.log.close()
        self._save_owner(job)

    def _update(self, job):
        self._read_latest(job)
        if job.process.poll() is not None:
            self._finish(job)
            return
        now = self.clock()
        if now-job.started > self.timeout_sec:
            self._begin_cancel(job, timeout=True)
        if job.cancel_at is not None:
            elapsed = now-job.cancel_at
            if elapsed >= 4 and job.signal_stage < 3:
                self._send_owned(job, signal.SIGKILL)
                job.signal_stage = 3
                self._save_owner(job)
            elif elapsed >= 2 and job.signal_stage < 2:
                self._send_owned(job, signal.SIGTERM)
                job.signal_stage = 2
                self._save_owner(job)

    def _watch(self):
        while not self.closing.wait(.05):
            with self.lock:
                if self.current is not None and not self.current.finished:
                    try:
                        self._update(self.current)
                    except Exception as error:
                        self.current.error = dict(type=type(error).__name__, message=str(error)[:1024])
                        self._begin_cancel(self.current)

    def _job_snapshot(self, job):
        observed = job.frame
        frame = observed["frame"] if observed else {}
        result = job.worker_result
        task = (result or {}).get("task") or (observed or {}).get("task")
        cancellation = (result or {}).get("cancellation") or {}
        age = None if observed is None else max(0., (time.monotonic_ns()-observed["observed_monotonic_ns"])/1e9)
        evidence = {name: deepcopy(frame.get(name)) for name in ("finger_force_n", "relative_position_error_m",
            "relative_angle_error_rad", "tcp_position_m", "block_position_m")}
        return dict(job_id=job.identifier, request_id=job.request_identifier, generation=job.generation,
            model_sha256=self.source["model_sha256"], status=job.status, process_running=job.process.poll() is None,
            cancel_requested=job.cancel_requested, cancel_delivery=job.cancel_delivery, timed_out=job.timed_out,
            phase=frame.get("phase", "initializing"), sequence=0 if observed is None else observed["sequence"],
            source_age_sec=age, freshness="unknown" if age is None else "fresh" if age <= .5 else "stale",
            simulation_sec=frame.get("simulation_sec"), steps=(observed or {}).get("integrated_steps", 0),
            task=deepcopy(task), evidence=evidence, physical_stop_confirmed=cancellation.get("physical_stop_confirmed"),
            action_state="not_applicable", error=job.display_error or deepcopy(job.error),
            worker_result_observed=result is not None, process_exit_code=job.process.poll(),
            worker_result_sha256=job.result_sha256,
            process_group_gone=job.process_group_gone if job.finished else None)

    def _save_owner(self, job):
        atomic_json(job.directory/"owner-receipt.json", dict(schema_version=1,
            source=deepcopy(self.source), job=self._job_snapshot(job),
            output_note="latest.json, stdout.log, run/ and implementation_sources/ are retained without overwrite",
            physical_stop_inferred_from_process_exit=False, automatic_retry=False))

    def snapshot(self):
        with self.lock:
            job = self.current
            if job is not None and not job.finished:
                self._update(job)
            view = None
            if job is not None:
                manifest = self.display.manifest
                view = dict(binding_id=job.binding_identifier, binding=deepcopy(job.binding),
                    manifest_descriptor=dict(sha256=_sha(self.display.manifest_raw),
                        byte_length=len(self.display.manifest_raw), model_sha256=manifest["model_sha256"],
                        scene_sha256=manifest["scene_sha256"]),
                    sequence=0 if job.frame is None else job.frame["sequence"],
                    pose_ready=job.pose is not None and job.display_error is None)
            return dict(schema_version=1, source=deepcopy(self.source),
                job=None if job is None else self._job_snapshot(job), view=view)

    def _bound_job(self, identifier):
        job = self.current
        if job is None or identifier != job.binding_identifier:
            raise ValueError("현재 시연의 표시 식별자가 다릅니다.")
        return job

    def manifest_bytes(self, identifier):
        with self.lock:
            self._bound_job(identifier)
            return self.display.manifest_raw

    def asset_bytes(self, digest):
        if digest not in self.display.assets:
            raise ValueError("등록하지 않은 시연 자산입니다.")
        return self.display.assets[digest]

    def pose_envelope(self, identifier):
        with self.lock:
            job = self._bound_job(identifier)
            if not job.finished:
                self._update(job)
            if job.frame is None or job.pose is None or job.display_error is not None:
                raise ValueError(job.display_error or "실제 시연 자세를 기다리고 있습니다.")
            observed = job.frame
            frame = observed["frame"]
            age = max(0., (time.monotonic_ns()-observed["observed_monotonic_ns"])/1e9)
            value = envelope(job.binding, job.pose, observed["sequence"],
                source_state_age_sec=age, simulation_sec=frame["simulation_sec"])
            value.update(schema_version=2, source=dict(session_id=job.identifier,
                clock_domain="mujoco-local:"+job.identifier, stamp_sec=frame["simulation_sec"],
                sequence=observed["sequence"],
                joint_positions_rad={name: frame["scalar_joint_positions"][name] for name in ARM_JOINTS}))
            return value

    def close(self):
        self.closing.set()
        with self.lock:
            job = self.current
            if job is not None and job.process.poll() is None:
                self._begin_cancel(job)
        if job is not None and job.process.poll() is None:
            for sig, wait in ((None, 2.), (signal.SIGTERM, 1.), (signal.SIGKILL, 1.)):
                if sig is not None:
                    self._send_owned(job, sig)
                try:
                    job.process.wait(timeout=wait)
                    break
                except subprocess.TimeoutExpired:
                    continue
        self.monitor.join(timeout=1.)
        with self.lock:
            if job is not None and job.process.poll() is not None:
                self._read_latest(job)
                self._finish(job)


class DemoHandler(RecordingHandler):
    """Reuse the recording handler's loopback Host check and response/CSP policy."""
    def _origin(self, required=False):
        port = self.server.server_address[1]
        values = self.headers.get_all("Origin", [])
        return ((not required and not values) or (len(values) == 1
            and values[0] in (f"http://127.0.0.1:{port}", f"http://localhost:{port}")))

    def _host(self):
        return len(self.headers.get_all("Host", [])) == 1 and self.valid_host()

    def do_GET(self):
        if not self._host() or not self._origin():
            return self.respond(dict(error="허용되지 않은 시연 주소입니다."), 403)
        parsed = urlsplit(self.path)
        app = self.server.app
        try:
            if parsed.path == "/api/demo" and not parsed.query:
                return self.respond(dict(app.snapshot(), token=self.server.token))
            if parsed.path in ("/api/view/manifest", "/api/view/pose"):
                query = parse_qs(parsed.query, keep_blank_values=True)
                if set(query) != {"binding_id"} or len(query["binding_id"]) != 1:
                    raise ValueError("현재 시연의 표시 식별자가 필요합니다.")
                method = app.manifest_bytes if parsed.path.endswith("manifest") else app.pose_envelope
                return self.respond(method(query["binding_id"][0]))
            if parsed.path.startswith("/api/view/assets/") and not parsed.query:
                digest = parsed.path.removeprefix("/api/view/assets/")
                if not re.fullmatch(r"[0-9a-f]{64}", digest):
                    raise ValueError("잘못된 시연 자산 식별자입니다.")
                return self.respond(app.asset_bytes(digest), mime="application/octet-stream")
            local = {"/": "demo.html", "/demo.html": "demo.html", "/demo.js": "demo.js", "/demo.css": "demo.css"}
            shared = {"/viewer.js": "viewer.js", "/viewer_contract.mjs": "viewer_contract.mjs",
                "/vendor/three.module.js": "vendor/three.module.js", "/vendor/three.core.js": "vendor/three.core.js",
                "/vendor/OrbitControls.js": "vendor/OrbitControls.js"}
            if not parsed.query and (parsed.path in local or parsed.path in shared):
                if parsed.path in local:
                    name = local[parsed.path]
                    raw = files(__package__).joinpath("static", name).read_bytes()
                else:
                    name = shared[parsed.path]
                    raw = files("mujoco_ros2_manipulation.operator_console").joinpath("static", name).read_bytes()
                    if name == "viewer.js":
                        raw = raw.replace(b"from 'three'", b"from './vendor/three.module.js'")
                    elif name == "vendor/OrbitControls.js":
                        raw = raw.replace(b"from 'three'", b"from './three.module.js'")
                mime = "text/javascript" if name.endswith((".js", ".mjs")) else "text/css" if name.endswith(".css") else "text/html"
                return self.respond(raw, mime=mime+"; charset=utf-8")
            return self.respond(dict(error="시연 자료를 찾을 수 없습니다."), 404)
        except (ValueError, KeyError, OSError, TypeError) as error:
            return self.respond(dict(error=str(error)), 409)

    def do_POST(self):
        tokens = self.headers.get_all("X-Operator-Token", [])
        if (not self._host() or not self._origin(required=True) or len(tokens) != 1
                or not tokens[0].isascii() or not hmac.compare_digest(tokens[0], self.server.token)):
            return self.respond(dict(error="이 전용 화면의 시연 요청만 허용합니다."), 403)
        parsed = urlsplit(self.path)
        if parsed.query or parsed.path not in ("/api/demo/start", "/api/demo/cancel"):
            return self.respond(dict(error="고정된 시연 시작과 취소만 지원합니다."), 404)
        try:
            lengths = self.headers.get_all("Content-Length", [])
            if len(lengths) != 1 or not lengths[0].isascii() or not lengths[0].isdigit():
                raise ValueError("요청 길이를 확인할 수 없습니다.")
            length = int(lengths[0])
            if (not 0 < length <= 1024 or self.headers.get_content_type() != "application/json"
                    or len(self.headers.get_all("Content-Type", [])) != 1
                    or self.headers.get("Transfer-Encoding") is not None):
                raise ValueError("작은 JSON 시연 요청이 필요합니다.")
            self.connection.settimeout(3.)
            value = _json(self.rfile.read(length))
            method = self.server.app.start if parsed.path.endswith("start") else self.server.app.cancel
            return self.respond(dict(method(value), token=self.server.token))
        except (ValueError, TypeError, KeyError, UnicodeError, OSError) as error:
            return self.respond(dict(error=str(error)), 400)
        except RuntimeError as error:
            return self.respond(dict(error=str(error)), 409)


def serve(app, port=8918):
    server = ThreadingHTTPServer(("127.0.0.1", port), DemoHandler)
    server.daemon_threads = True
    server.app, server.token = app, secrets.token_urlsafe(32)
    return server


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", required=True, help="Explicit checked FR5 workcell directory; never accepted by HTTP")
    parser.add_argument("--output", required=True, help="New owned root for retained simulation jobs")
    parser.add_argument("--port", type=int, default=8918)
    args = parser.parse_args(argv)
    if not 1024 <= args.port <= 65535:
        parser.error("Port must be 1024..65535")
    app = DemoApplication(args.source, args.output)
    server = None
    previous_term = signal.signal(signal.SIGTERM, lambda *_: (_ for _ in ()).throw(KeyboardInterrupt()))
    try:
        server = serve(app, args.port)
        print(f"INDEPENDENT_FR5_GRASP_READY http://127.0.0.1:{args.port}/", flush=True)
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        signal.signal(signal.SIGTERM, previous_term)
        if server is not None:
            server.server_close()
        app.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
