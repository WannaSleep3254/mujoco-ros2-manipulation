"""Exclusive, owned live demonstration jobs for the loopback console."""
from copy import deepcopy
import json
import os
from pathlib import Path
import signal
import subprocess
import threading
import time
import uuid

from ..model_build.integrity import file_sha256
from ..verification.reports import atomic_json
from ..verification.runner import group_gone, stop_process
from .catalog import describe_scenarios

ACTIVE = {"starting", "running", "cancel_requested"}


def process_identity(pid):
    """Linux process identity survives PID reuse checks; no command line is read."""
    try:
        parts = Path(f"/proc/{int(pid)}/stat").read_text().rpartition(") ")[2].split()
        return dict(pid=int(pid), start_ticks=int(parts[19]), pgid=int(parts[2]), sid=int(parts[3]), ppid=int(parts[1]))
    except (OSError, ValueError, IndexError):
        return None


def matching_process(record):
    current = process_identity(record["pid"])
    return current is not None and current["start_ticks"] == record["start_ticks"] and current["pgid"] == record["pid"] and current["sid"] == record["pid"]


class DemoJobs:
    def __init__(self, app):
        self.app = app
        self.current = None
        self.history = []
        self.process = None
        self.folder = None
        self.launches = {}
        self.members = {}
        self.render_key = None
        self.cancel_requested = False
        self.stopping = False
        self.cleanup_blocked = False
        self.monitor_error = None

    @property
    def busy(self):
        return self.process is not None or self.stopping

    def reason(self):
        with self.app.lock, self.app.state.lock:
            return self._reason()

    def _reason(self):
        app = self.app
        if getattr(app, "execution_target", None) is not None:
            return "선택한 설치 환경에서는 기존 고정 환경 시연을 실행할 수 없습니다."
        if app.views.preview:
            return "미리보기를 종료하고 현재 작업 환경의 실시간 보기로 돌아오세요."
        if self.busy or app.phase in ("starting", "running", "stopping"):
            return "진행 중인 실행을 종료한 뒤 시연을 시작하세요."
        if app.closing.is_set():
            return "화면을 종료하고 있습니다."
        if app.state.clock()-app.state.started < 2:
            return "ROS 연결을 확인하고 있습니다."
        if any(p.poll() is None for k,p in app.children.items() if k != "renderer"):
            return "진행 중인 실행을 종료한 뒤 시연을 시작하세요."
        if any(app.state.graph[k] for k in ("clock", "joints", "status", "guards")):
            return "이 ROS 도메인의 기존 실행을 종료한 뒤 시연을 시작하세요."
        return None

    def start(self, options):
        app = self.app
        if not isinstance(options, dict) or set(options) != {"scenario", "native_gui"} or type(options["native_gui"]) is not bool or options["scenario"] not in {s["id"] for s in describe_scenarios()}:
            raise ValueError("지원하는 시연과 실행 옵션을 선택하세요.")
        if app.cell.id != "fr5_ur5e":
            raise ValueError("이 시연은 FR5 + UR5e 작업 환경에서 지원합니다.")
        with app.lock, app.state.lock:
            reason = self.reason()
            if reason:
                raise RuntimeError(reason)
            identifier = uuid.uuid4().hex
            folder = app.directory/"demos"/identifier
            folder.mkdir(parents=True)
            self.current = dict(schema_version=1, job_id=identifier, scenario=options["scenario"], status="starting", stage="starting", variant=None, run_id=None, active_launch=None, results=[], comparison=None, error=None)
            atomic_json(folder/"status.json", self.current)
            app.suspend_dispatch()
            app.reset_observation()
            self.stop_renderer(clear=True)
            self.folder, self.launches, self.render_key = folder, {}, None
            self.members = {}
            self.cancel_requested = False
            self.cleanup_blocked = False
            self.monitor_error = None
            app.phase, app.error = "demo", None
            app.render_suspended = True
            args = ["/usr/bin/python3", "-m", "mujoco_ros2_manipulation.operator_console.demo_worker", "--directory", str(folder), "--scenario", options["scenario"], "--domain", str(app.state.config["domain"]), "--console-directory", str(app.directory)]
            if options["native_gui"]:
                args.append("--native-gui")
            if not app.image_render_enabled:
                args.append("--no-render")
            try:
                self.process = app.spawn("demo", args, folder)
            except Exception as error:
                self.current.update(status="failed", stage="launch_failed", error=str(error))
                app.phase, app.error = "failed", str(error)
                atomic_json(folder/"status.json", self.current)
                raise
            app.audit("demo_started", job_id=identifier, scenario=options["scenario"])
            return dict(job_id=identifier, status="starting")

    def read_status(self):
        if not self.folder:
            return
        path = self.folder/"status.json"
        if path.stat().st_size > 4*1024*1024:
            raise ValueError("시연 상태 기록의 크기가 올바르지 않습니다.")
        data = json.loads(path.read_text())
        if data.get("job_id") != self.current["job_id"] or data.get("scenario") != self.current["scenario"]:
            raise ValueError("시연 상태 기록이 현재 작업과 다릅니다.")
        launch = data.get("active_launch")
        if launch:
            self.validate_launch(launch)
            key = (launch["pid"], launch["start_ticks"])
            if key not in self.launches:
                identity = process_identity(launch["pid"])
                if identity is None and group_gone(launch["pid"]):
                    data["active_launch"] = None
                elif identity is None or identity["ppid"] != self.process.pid or not matching_process(launch):
                    raise ValueError("시연 worker의 직접 자식이 아닌 실행은 소유할 수 없습니다.")
            if data["active_launch"] is not None:
                self.launches[key] = dict(launch)
                if matching_process(launch):
                    self.members[key] = self.group_members(launch)
        self.current = data
        # Keep the worker's outcome for finish(); snapshot() overlays the
        # pending cancellation until owned cleanup has completed.

    def refresh_for_cleanup(self):
        try:
            self.read_status()
        except Exception as error:
            # A corrupt sidecar must not prevent cleanup of already proven children.
            self.monitor_error = self.monitor_error or str(error)

    def validate_launch(self, launch):
        app = self.app
        run = app.cell.root/"runtime/validation/runs"/launch["run_id"]
        if run.parent != app.cell.root/"runtime/validation/runs" or Path(launch["report_path"]).resolve() != (run/"result.json").resolve():
            raise ValueError("시연 실행 기록 경로가 올바르지 않습니다.")
        model = Path(launch["model_path"]).resolve()
        if model not in {(app.cell.output/"cell.xml").resolve(), (run/"contact_test.xml").resolve()}:
            raise ValueError("시연 모델 경로가 올바르지 않습니다.")
        if type(launch["pid"]) is not int or type(launch["start_ticks"]) is not int or launch["pid"] <= 1 or launch["start_ticks"] <= 0:
            raise ValueError("시연 실행 소유권이 올바르지 않습니다.")
        sha = launch.get("model_sha256")
        if sha is not None and (not isinstance(sha,str) or len(sha) != 64 or any(c not in "0123456789abcdef" for c in sha)):
            raise ValueError("시연 모델 해시가 올바르지 않습니다.")

    def stop_renderer(self, *, clear=False):
        app = self.app
        process = app.children.pop("renderer", None)
        if process:
            stop_process(process)
        app.render_attempted = False
        self.render_key = None
        if clear:
            app.clear_frames()

    def tick(self):
        if self.process is None or self.stopping or self.cleanup_blocked:
            return
        app = self.app
        try:
            self.read_status()
            launch = self.current.get("active_launch")
            if self.process.poll() is not None:
                self.stopping = True
                threading.Thread(target=self.finish_exited, daemon=True).start()
                return
            if self.cancel_requested:
                return
            key = None if not launch or not launch.get("model_sha256") else (self.current["job_id"], launch["run_id"], launch["model_sha256"])
            if key != self.render_key:
                self.stop_renderer(clear=key is not None)
                self.render_key = key
            if not key or not app.image_render_enabled or app.render_attempted:
                return
            live = app.state.snapshot()
            if (not matching_process(launch) or live["graph"]["clock"] != 1 or live["graph"]["joints"] != 1 or live["state_age_sec"] is None or live["state_age_sec"] > .5 or app.state.observed_model_path != launch["model_path"]):
                return
            if file_sha256(launch["model_path"]) != launch["model_sha256"]:
                raise ValueError("시연 모델이 준비 후 변경됐습니다.")
            app.render_attempted = True
            folder = app.directory/"frames"
            folder.mkdir(exist_ok=True)
            app.spawn("renderer", [str(app.cell.root/".venv/bin/python"), "-m", "mujoco_ros2_manipulation.operator_console.render_worker", "--workcell", app.cell.id, "--directory", str(app.directory), "--model", launch["model_path"], "--model-sha256", key[2], "--job-id", key[0], "--run-id", key[1], "--generation", str(app.generation), "--operator-session-id", app.session_id], folder, MUJOCO_GL="egl")
        except Exception as error:
            self.monitor_error = str(error)
            self.current.update(error=str(error))
            app.audit("demo_monitor_failed", job_id=self.current["job_id"], error=str(error))
            self.request_stop(self.current["job_id"])

    @staticmethod
    def group_members(record):
        result = []
        for path in Path("/proc").iterdir():
            if path.name.isdecimal():
                member = process_identity(int(path.name))
                if member and member["pgid"] == record["pid"] and member["sid"] == record["pid"]:
                    result.append(member)
        return result

    def group_owned(self, record):
        if matching_process(record):
            return True
        for previous in self.members.get((record["pid"], record["start_ticks"]), []):
            current = process_identity(previous["pid"])
            if current and all(current[k] == previous[k] for k in ("pid", "start_ticks", "pgid", "sid")):
                return True
        return False

    def cleanup_launches(self):
        forced = False
        for record in self.launches.values():
            if group_gone(record["pid"]):
                continue
            forced = True
            for sig in (signal.SIGTERM, signal.SIGKILL):
                if not self.group_owned(record):
                    break
                try:
                    os.killpg(record["pid"], sig)
                except ProcessLookupError:
                    break
                deadline = time.monotonic()+5
                while not group_gone(record["pid"]) and time.monotonic()<deadline:
                    time.sleep(.05)
                if group_gone(record["pid"]):
                    break
        return forced

    def finish_exited(self):
        app = self.app
        with app.cleanup_lock:
            with app.lock:
                self.refresh_for_cleanup()
            forced = self.cleanup_launches()
            with app.lock:
                self.finish(forced=forced)
                self.stopping = False

    def finish(self, *, forced=False):
        app = self.app
        code = self.process.poll()
        unclean = forced or not group_gone(self.process.pid) or any(not group_gone(r["pid"]) for r in self.launches.values())
        self.stop_renderer()
        data = self.current
        if unclean:
            data.update(status="failed", error="시연 프로세스의 종료를 확인하지 못했습니다.")
        elif self.monitor_error:
            data.update(status="failed", error=self.monitor_error, comparison=None)
        elif self.cancel_requested and (
                (code == 130 and data.get("status") == "cancelled"
                 and isinstance(data.get("error"), dict)
                 and data["error"].get("type") == "Cancelled")
                or (code == 0 and data.get("status") == "succeeded")):
            data.update(status="cancelled", comparison=None, error=None)
        elif code != 0 or data.get("status") != "succeeded":
            data.update(status="failed", comparison=None,
                        error=data.get("error") or f"시연 실행이 종료됐습니다 (exit={code}).")
        data.update(active_launch=None, cleanup_clean=not unclean)
        atomic_json(self.folder/"status.json", data)
        self.history.insert(0, deepcopy(data))
        del self.history[10:]
        app.audit("demo_finished", job_id=data["job_id"], status=data["status"], cleanup_clean=not unclean)
        # Retain ownership and block new jobs while any launch remains alive.
        if any(not group_gone(r["pid"]) for r in self.launches.values()):
            self.cleanup_blocked = True
            app.phase, app.error = "failed", data["error"]
            return
        self.process = None
        app.children.pop("demo", None)
        app.phase = "failed" if unclean else "idle"
        app.error = data.get("error") if unclean else None

    def request_stop(self, identifier):
        app = self.app
        with app.lock:
            if not self.current or identifier != self.current["job_id"] or self.process is None:
                raise RuntimeError("현재 진행 중인 시연만 종료할 수 있습니다.")
            if self.cancel_requested:
                return dict(job_id=identifier, status="cancel_requested")
            self.cancel_requested = True
            self.process.send_signal(signal.SIGINT)
            app.audit("demo_cancel_requested", job_id=identifier)
            threading.Thread(target=self.stop_sync, daemon=True).start()
            return dict(job_id=identifier, status="cancel_requested")

    def stop_sync(self):
        app = self.app
        with app.cleanup_lock:
            self._stop_sync()

    def _audit_worker_wait(self, action, process, *, elapsed_sec, return_code=None, timed_out=False):
        try:
            self.app.audit(action, job_id=self.current["job_id"], worker_pid=process.pid,
                           timeout_sec=45, elapsed_sec=elapsed_sec,
                           return_code=return_code, timed_out=timed_out)
        except Exception:
            # Diagnostics must not interrupt cleanup or alter its clean verdict.
            pass

    def _stop_sync(self):
        app = self.app
        with app.lock:
            if self.process is None:
                return
            self.stopping = True
            already_requested = self.cancel_requested
            self.cancel_requested = True
            process = self.process
            if not already_requested and process.poll() is None:
                process.send_signal(signal.SIGINT)
        forced = False
        wait_started = time.monotonic()
        self._audit_worker_wait("demo_worker_wait_started", process, elapsed_sec=0.0)
        try:
            return_code = process.wait(timeout=45)
        except subprocess.TimeoutExpired:
            self._audit_worker_wait("demo_worker_wait_finished", process,
                                    elapsed_sec=time.monotonic()-wait_started,
                                    return_code=process.returncode, timed_out=True)
            forced = True
            # Only a still-identical launch leader authorizes signaling its group.
            with app.lock:
                self.refresh_for_cleanup()
            self.cleanup_launches()
            stop_process(process)
        else:
            self._audit_worker_wait("demo_worker_wait_finished", process,
                                    elapsed_sec=time.monotonic()-wait_started,
                                    return_code=return_code)
        with app.lock:
            self.refresh_for_cleanup()
        forced = self.cleanup_launches() or forced
        with app.lock:
            self.finish(forced=forced)
            self.stopping = False

    def snapshot(self):
        reason = self.reason()
        data = deepcopy(dict(current=self.current, history=self.history, can_start=reason is None, reason=reason))
        if self.busy and data["current"]:
            data["current"]["status"] = "cancel_requested" if self.cancel_requested else "running"
            if self.current.get("status") in ("succeeded", "failed", "cancelled"):
                data["current"]["stage"] = "cleanup"
        return data
