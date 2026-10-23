"""Own only processes started by this console; reuse the existing command protocol."""
from copy import deepcopy
from datetime import datetime, timezone
import json
import os
from pathlib import Path
import queue
import signal
import subprocess
import threading
import time
import uuid

from ..verification.reports import atomic_json
from ..verification.runner import stop_process
from ..workcells.motion_request import request_id
from .demo_jobs import DemoJobs
from .console_views import ConsoleViews


class ConsoleApplication:
    def __init__(self, cell, state, bridge, directory, *, render=True):
        self.cell, self.state, self.bridge = cell, state, bridge
        self.directory = Path(directory)
        self.directory.mkdir(parents=True, exist_ok=False)
        self.session_id = uuid.uuid4().hex
        self.render_enabled = render
        self.lock = threading.RLock()
        self.cleanup_lock = threading.Lock()
        self.children = {}
        self.files = []
        self.phase = "idle"
        self.error = None
        self.generation = 0
        self.last_launch = None
        self.closing = threading.Event()
        self.worker = None
        self.render_attempted = False
        self.render_suspended = False
        self.demos = DemoJobs(self)
        self.display_mode = "web3d"
        self.image_render_enabled = False
        self.views = ConsoleViews(self, enabled=render)
        self.audit("console_started")
        self.monitor = threading.Thread(target=self.watch, daemon=True)
        self.monitor.start()

    def audit(self, action, **details):
        with self.lock:
            with (self.directory / "events.jsonl").open("a") as stream:
                stream.write(json.dumps(dict(action=action, utc=datetime.now(timezone.utc).isoformat(), **details), allow_nan=False)+"\n")

    def spawn(self, kind, args, folder, **environment):
        log = (folder / (kind+".log")).open("w")
        self.files.append(log)
        process = subprocess.Popen(args, cwd=self.cell.root, stdout=log, stderr=subprocess.STDOUT,
            env=dict(os.environ, **environment), start_new_session=True)
        self.children[kind] = process
        self.audit("process_started", kind=kind, pid=process.pid, log=str(log.name))
        return process

    def start(self, options, *, attach=False):
        if set(options) != {"native_gui", "planning_case"} or type(options["native_gui"]) is not bool or options["planning_case"] not in ("none", "dual_arm_crossing"):
            raise ValueError("실행 옵션이 올바르지 않습니다.")
        with self.lock, self.state.lock:
            if self.views.preview:
                raise RuntimeError("미리보기를 종료하고 현재 작업장면의 실시간 보기로 돌아오세요.")
            if self.demos.busy or self.phase in ("starting", "running", "stopping") or any(p.poll() is None for k,p in self.children.items() if k != "renderer"):
                raise RuntimeError("이 화면에서 시작한 실행이 이미 있습니다.")
            if self.state.clock()-self.state.started < 2:
                raise RuntimeError("ROS 연결 확인이 끝난 뒤 실행하세요.")
            if attach:
                if not self.state.identity or not self.state.positions or self.state.graph["status"] != 0:
                    raise RuntimeError("검증된 MuJoCo 장면에 명령 노드가 없을 때만 연결할 수 있습니다.")
            elif self.state.graph["clock"] or self.state.graph["joints"] or self.state.graph["status"]:
                raise RuntimeError("이 ROS 도메인에 기존 실행이 있습니다. 새 실행 대신 기존 상태를 확인하세요.")
            else:
                self.reset_observation()
                self.clear_frames()
            self.phase, self.error = "starting", None
            self.state.admission_suspended = False
            self.render_suspended = False
            self.generation += 1
            folder = self.directory / f"launch-{self.generation:03d}"
            folder.mkdir()
            self.last_launch = folder
            self.worker = threading.Thread(target=self.start_worker, args=(options, folder, attach, self.generation), daemon=True)
            self.worker.start()
            return dict(phase=self.phase)

    def process_failure(self, *, preparing=False):
        exited = {kind: process.poll() for kind, process in self.children.items()
                  if kind != "renderer" and process.poll() is not None}
        if not exited:
            return None
        phase = "준비 중" if preparing else "실행 중"
        detail = ", ".join(f"{kind} (exit={code})" for kind, code in exited.items())
        cause = "핵심 프로세스가 종료됐습니다."
        if self.last_launch:
            # Read a bounded tail from our own logs; the command traceback often
            # contains only a secondary timeout after a controller spawner fails.
            for kind in ("simulation", "commands"):
                path = self.last_launch / (kind+".log")
                try:
                    with path.open("rb") as stream:
                        stream.seek(max(0, path.stat().st_size-65536))
                        tail = stream.read(65536).decode(errors="replace")
                except OSError:
                    continue
                if "Failed to find a free participant index" in tail:
                    cause = "ROS DDS 참가자 슬롯 부족으로 제어기 초기화가 실패했습니다."
                    break
            detail += f". 로그: {self.last_launch}"
        return f"{phase} {cause} {detail}"

    def start_worker(self, options, folder, attach, generation):
        try:
            with self.lock:
                if self.phase != "starting" or self.generation != generation or self.closing.is_set():
                    return
                if not attach:
                    native = str(options["native_gui"]).lower()
                    self.spawn("simulation", ["bash", str(self.cell.root / "run_scene.sh"), f"workcell:={self.cell.id}",
                        f"gui:={native}", f"rviz:={native}", "planning_case:="+options["planning_case"]], folder,
                        MANIPULATION_DIAGNOSTICS_PATH=str(folder/"physics.json"),
                        MANIPULATION_DIAGNOSTICS_RUN_ID=self.session_id+f"-{generation}")
                self.spawn("commands", ["/usr/bin/python3", "-m", "mujoco_ros2_manipulation.workcells.motion_server",
                    "--workcell", self.cell.id, "--output", str(folder/"commands")], folder)
            deadline = time.monotonic()+55
            while time.monotonic() < deadline and not self.closing.is_set():
                with self.lock:
                    if self.phase != "starting" or self.generation != generation:
                        return
                    failure = self.process_failure(preparing=True)
                    if failure:
                        raise RuntimeError(failure)
                    if self.state.snapshot()["can_submit"]:
                        self.phase = "running"
                        self.audit("simulation_ready", attached=attach)
                        return
                self.closing.wait(0.1)
            if not self.closing.is_set():
                raise RuntimeError("시뮬레이터·명령 노드 준비 시간이 초과됐습니다.")
        except Exception as error:
            with self.lock:
                if self.generation != generation:
                    return
                self.phase = "stopping"
            self.stop(error=str(error))

    def submit(self, payload):
        with self.lock, self.state.lock:
            if self.views.preview:
                raise RuntimeError("미리보기를 종료하고 현재 작업장면의 실시간 보기로 돌아오세요.")
            if self.demos.busy or self.phase in ("starting", "stopping") or self.closing.is_set():
                raise RuntimeError("실행 준비 또는 종료 중입니다.")
            if self.state.reserve(payload):
                try:
                    self.bridge.outbox.put_nowait(("command", payload))
                except queue.Full:
                    with self.state.lock:
                        self.state.history[payload["request_id"]].update(status="rejected", stage="local_queue_full")
                        self.state.active = None
                    raise RuntimeError("명령 전송 대기 공간이 가득 찼습니다.")
                self.audit("submitted", request_id=payload["request_id"], strategy=payload["strategy"])
            return dict(request_id=payload["request_id"])

    def cancel(self, identifier):
        if self.demos.busy:
            raise RuntimeError("시연 종료 버튼으로 현재 시연을 종료하세요.")
        request_id(identifier)
        with self.state.lock:
            if identifier != self.state.active:
                raise RuntimeError("현재 진행 중인 요청만 취소할 수 있습니다.")
        try:
            self.bridge.outbox.put_nowait(("cancel", dict(request_id=identifier)))
        except queue.Full:
            raise RuntimeError("취소 요청이 이미 대기 중입니다. 완료 상태를 확인하세요.") from None
        self.audit("cancel_requested", request_id=identifier)
        return dict(request_id=identifier, status="cancel_requested")

    def request_details(self, identifier):
        request_id(identifier)
        with self.state.lock:
            record = deepcopy(self.state.history.get(identifier))
        if record is None:
            raise ValueError("이 화면에서 관측하지 않은 요청입니다.")
        if record.get("trace_path"):
            path = Path(record["trace_path"]).resolve()
            if path.is_relative_to((self.cell.root/"runtime").resolve()) and path.name == identifier+".json" and path.is_file() and path.stat().st_size <= 32*1024*1024:
                data = json.loads(path.read_text())
                if data["request_id"] == identifier:
                    record.update({k:data.get(k) for k in ("events", "spans", "plans", "planned_clearance", "observed_clearance", "compute_budget", "action_results", "plan_selection", "failure_kind", "clearance_diagnostic", "endpoint_moveit_validity")})
                    record["departure_search"] = (data.get("departure_search") or {}).get("profiling")
        return record

    def camera(self, view):
        if view not in ("overview", "front", "top"):
            raise ValueError("지원하지 않는 카메라입니다.")
        atomic_json(self.directory/"camera.json", dict(view=view))
        return dict(view=view)

    def set_display(self, mode):
        if mode not in ("web3d", "image") or not isinstance(mode, str):
            raise ValueError("지원하는 장면 표시 방식을 선택하세요.")
        with self.lock, self.state.lock:
            if not self.render_enabled:
                raise RuntimeError("이 실행은 장면 표시가 비활성화되어 있습니다.")
            if self.demos.busy or self.state.active or self.phase in ("starting", "stopping", "demo") or self.closing.is_set():
                raise RuntimeError("진행 중인 요청이나 시연이 끝난 뒤 표시 방식을 바꾸세요.")
            if mode == "image" and self.views.preview:
                raise RuntimeError("모델·계획 미리보기는 웹 3D에서 표시합니다.")
            if mode != self.display_mode:
                renderer = self.children.get("renderer")
                try:
                    self.demos.stop_renderer(clear=True)  # Only our renderer, never simulation/commands.
                except Exception:
                    if renderer is not None:
                        self.children["renderer"] = renderer  # Retain ownership if cleanup failed.
                    raise
                # A JPEG worker starts its sequence at one. Use a new generation
                # so a late frame from an earlier worker cannot look current.
                self.generation += 1
                self.display_mode = mode
                self.image_render_enabled = mode == "image"
                self.audit("display_selected", mode=mode)
            return dict(display_mode=self.display_mode)

    def update_views(self):
        """A display failure must not kill supervision or stop robot processes."""
        try:
            self.views.tick()
        except Exception as error:
            failure = dict(code="view_update_failed", message="장면 상태를 갱신하지 못했습니다. 실시간 보기를 다시 선택하세요.")
            changed = self.views.error != failure
            self.views.error = failure
            try:
                if self.views.host is not None:
                    self.views.host.invalidate()
            except Exception:
                pass
            if changed:
                try:
                    self.audit("view_update_failed", error_type=type(error).__name__)
                except Exception:
                    pass  # Even a display-log failure must not kill supervision.

    def watch(self):
        next_supervision = 0.
        while not self.closing.wait(0.1):
            failure = None
            with self.lock:
                now = time.monotonic()
                supervise = now >= next_supervision
                if supervise:
                    next_supervision = now + 0.5
                    if self.phase == "running":
                        failure = self.process_failure()
                    self.demos.tick()
                # Feed current observed states more often than the 500ms stale
                # limit. Child/source validation remains in SceneHost's thread.
                self.update_views()
                live = self.state.snapshot() if supervise else None
                if supervise and not self.render_suspended and self.phase != "stopping" and self.image_render_enabled and not self.render_attempted and live["simulation_verified"] and live["state_age_sec"] is not None and live["state_age_sec"] < 0.5:
                    self.render_attempted = True
                    folder = self.directory/"frames"
                    folder.mkdir(exist_ok=True)
                    try:
                        self.spawn("renderer", [str(self.cell.root/".venv/bin/python"), "-m", "mujoco_ros2_manipulation.operator_console.render_worker",
                            "--workcell", self.cell.id, "--directory", str(self.directory),
                            "--generation", str(self.generation), "--operator-session-id", self.session_id], folder, MUJOCO_GL="egl")
                    except Exception as error:
                        atomic_json(self.directory/"frame.json", dict(error=str(error)))
            if failure:
                with self.lock:
                    failed = self.phase == "running"
                    if failed:
                        self.phase = "stopping"
                if failed:
                    self.stop(error=failure)

    def reset_observation(self):
        with self.state.lock:
            self.state.clock_backwards = False
            self.state.simulation_sec = self.state.clock_at = self.state.clock_advanced_at = None
            self.state.joint_state_stamp_simulation_sec = None
            self.state.positions = self.state.joint_at = self.state.guard = self.state.server = None
            self.state.identity = False
            self.state.observed_model_path = None
            self.state.identity_requested_at = self.state.identity_checked_at = None

    def clear_frames(self):
        for name in ("frame.json", "frame.jpg"):
            (self.directory/name).unlink(missing_ok=True)

    def suspend_dispatch(self):
        with self.state.lock:
            self.state.admission_suspended = True
            while True:
                try:
                    kind, payload = self.bridge.outbox.get_nowait()
                except queue.Empty:
                    break
                if kind == "command":
                    identifier = payload["request_id"]
                    if identifier in self.state.history:
                        self.state.history[identifier].update(status="cancelled", stage="console_stopped_before_publish")
                    if self.state.active == identifier:
                        self.state.active = None

    def stop(self, *, error=None):
        with self.cleanup_lock:
            return self._stop(error=error)

    def _stop(self, *, error=None):
        self.demos._stop_sync()
        with self.lock:
            self.phase = "stopping"
            self.generation += 1
            self.render_suspended = True
            owned_commands = "commands" in self.children
            self.suspend_dispatch()
        failures = []
        # Stop the command owner first so it can cancel active actions with ROS alive.
        for kind in ("commands", "simulation", "renderer"):
            with self.lock:
                process = self.children.get(kind)
            if process:
                try:
                    forced = stop_process(process)
                    if forced or process.returncode not in (0, -signal.SIGINT):
                        failures.append(kind+": forced/abnormal shutdown")
                except Exception as problem:
                    failures.append(kind+": "+str(problem))
        with self.lock:
            with self.state.lock:
                if owned_commands and self.state.active:
                    self.state.history[self.state.active].update(status="unknown", stage="owner_stopped", error=dict(message="명령 노드가 종료되어 마지막 요청의 종료 결과를 수신하지 못했습니다."))
                    self.state.active = None
            self.children.clear()
            self.render_attempted = False
            self.phase = "failed" if error or failures else "idle"
            self.error = error or ("; ".join(failures) if failures else None)
            self.audit("processes_stopped", error=self.error, cleanup_errors=failures)
        return dict(phase=self.phase, cleanup_errors=failures)

    def stop_async(self):
        with self.lock:
            if self.phase == "stopping":
                return dict(phase="stopping")
            if not any(k in self.children for k in ("simulation", "commands")):
                raise RuntimeError("이 화면에서 시작한 시뮬레이터·명령 노드가 없습니다.")
            self.phase = "stopping"
            self.suspend_dispatch()
            self.worker = threading.Thread(target=self.stop, daemon=True)
            self.worker.start()
            return dict(phase="stopping")

    def snapshot(self):
        with self.lock:
            data = self.state.snapshot()
            frame = {}
            if (self.directory/"frame.json").is_file():
                frame = json.loads((self.directory/"frame.json").read_text())
                if frame.get("rendered_monotonic") is not None:
                    frame["age_sec"] = time.monotonic()-frame["rendered_monotonic"]
            renderer = self.children.get("renderer")
            if renderer and renderer.poll() is not None and renderer.returncode != 0:
                frame["error"] = frame.get("error", "렌더러가 종료됐습니다. 실행 로그를 확인하세요.")
            data.update(lifecycle=dict(phase=self.phase, error=self.error,
                owned={k:p.poll() is None for k,p in self.children.items()}, session_id=self.session_id,
                generation=self.generation,
                last_launch_directory=None if self.last_launch is None else str(self.last_launch)), frame=frame)
            data["demo"] = self.demos.snapshot()
            data["view"] = self.views.snapshot()
            base = self.views.base
            data["view"].update(display_mode=self.display_mode,
                config_source_id=base.id if base is not None else self.state.config.get("workcell"))
            if self.demos.busy or self.phase in ("starting", "stopping"):
                data["can_submit"] = False
                if self.demos.busy:
                    data["reason"] = "시연 진행 중입니다. 종료 후 일반 관절 조작을 시작하세요."
            if self.views.preview:
                data["can_submit"] = False
                data["reason"] = "미리보기를 종료하고 현재 작업장면의 실시간 보기로 돌아오세요."
            return data

    def close(self):
        self.closing.set()
        self.suspend_dispatch()
        if self.worker and self.worker is not threading.current_thread():
            self.worker.join(timeout=1)
        self.demos.stop_sync()
        with self.state.lock:
            active = self.state.active if self.state.active in self.state.ui_ids else None
        if active and "commands" not in self.children:
            self.cancel(active)
            deadline = time.monotonic()+8
            while self.state.active == active and time.monotonic() < deadline:
                time.sleep(0.05)
        try:
            self.views.close()
        finally:
            self.stop()
            self.monitor.join(timeout=2)
            for stream in self.files:
                stream.close()
