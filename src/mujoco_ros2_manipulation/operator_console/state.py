"""Thread-safe console state and admission rules, independent of ROS/HTTP."""
from collections import OrderedDict
from copy import deepcopy
import math
import threading
import time

from ..workcells.motion_request import validate_command

TERMINAL = {"succeeded", "failed", "cancelled", "rejected"}


class ConsoleState:
    def __init__(self, config, *, clock=time.monotonic):
        self.config = config
        self.clock = clock
        self.lock = threading.RLock()
        self.started = clock()
        self.positions = self.velocities = None
        self.joint_at = self.clock_at = self.clock_advanced_at = None
        self.simulation_sec = None
        self.joint_state_stamp_simulation_sec = None
        self.clock_backwards = False
        self.graph = dict(clock=0, joints=0, status=0, commands=0, guards=0)
        self.identity = False
        self.observed_model_path = None
        self.guard = None
        self.server = None
        self.server_at = None
        self.history = OrderedDict()
        self.active = None
        self.ui_ids = set()
        self.admission_suspended = False

    def joints(self, names, positions, velocities, *, source_stamp=None):
        with self.lock:
            if len(names) != len(set(names)) or set(names) != set(self.config["home_rad"]) or len(positions) != len(names):
                return
            if not all(math.isfinite(v) for v in positions):
                return
            self.positions = dict(zip(names, positions))
            self.velocities = dict(zip(names, velocities)) if len(velocities) == len(names) and all(math.isfinite(v) for v in velocities) else None
            self.joint_at = self.clock()
            self.joint_state_stamp_simulation_sec = (source_stamp
                if type(source_stamp) in (int, float) and math.isfinite(source_stamp) and source_stamp >= 0
                else None)

    def tick(self, stamp):
        if not math.isfinite(stamp):
            return
        with self.lock:
            if self.simulation_sec is not None and stamp < self.simulation_sec:
                self.clock_backwards = True
            if self.simulation_sec is None or stamp > self.simulation_sec:
                self.clock_advanced_at = self.clock()
            self.simulation_sec, self.clock_at = stamp, self.clock()

    def status(self, record):
        if not isinstance(record, dict) or not isinstance(record.get("session_id"), str) or record.get("status") not in TERMINAL | {"ready", "blocked", "queued", "running"}:
            return
        identifier = record.get("request_id")
        if identifier:
            from ..workcells.motion_request import request_id
            try:
                request_id(identifier)
            except ValueError:
                return
        with self.lock:
            if self.server and self.server["session_id"] != record["session_id"] and self.active:
                self.history[self.active].update(status="unknown", stage="server_changed", error=dict(message="명령 노드가 바뀌어 이전 요청의 종료 결과를 확인할 수 없습니다."))
                self.active = None
            self.server, self.server_at = deepcopy(record), self.clock()
            identifier = record.get("request_id")
            if identifier:
                old = self.history.get(identifier, {})
                self.history[identifier] = dict(old, **deepcopy(record))
                self.history.move_to_end(identifier)
                if record["status"] in ("queued", "running"):
                    self.active = identifier
                elif record["status"] in TERMINAL and self.active == identifier:
                    self.active = None
                while len(self.history) > 40:
                    key = next(k for k in self.history if k != self.active)
                    self.history.pop(key)
                    self.ui_ids.discard(key)

    def reason(self, *, ignore_request=None):
        now = self.clock()
        if self.admission_suspended:
            return "실행을 종료했습니다. 시뮬레이션을 다시 시작하거나 연결하세요."
        if now - self.started < 2:
            return "ROS 연결을 확인하고 있습니다."
        if any(self.graph[k] != 1 for k in ("clock", "joints", "status", "guards")) or self.graph["commands"] < 1:
            return "공유 시계·관절 상태·명령 노드가 각각 하나씩 연결되어야 합니다."
        if not self.identity:
            return "현재 작업장면의 MuJoCo 시뮬레이션 연결을 확인하지 못했습니다."
        if self.positions is None or self.joint_at is None or now - self.joint_at > 0.5:
            return "관절 상태 수신이 멈췄습니다."
        if self.clock_at is None or now - self.clock_at > 0.5 or self.clock_advanced_at is None or now - self.clock_advanced_at > 0.5 or self.clock_backwards:
            return "시뮬레이션 시계가 멈췄거나 역행했습니다."
        if not self.guard or self.guard.get("latched") or self.guard.get("phase") != "ready":
            return "접촉·시계 감시 상태를 확인하세요. 정지 잠금은 재시작이 필요합니다."
        if self.active and self.active != ignore_request:
            return "양팔이 이전 요청을 처리하고 있습니다."
        if not self.server or self.server.get("status") != "ready" or now - self.server_at > 3:
            return "명령 노드가 준비되지 않았습니다."
        return None

    def reserve(self, payload):
        validate_command(payload, self.config["joint_limits"])
        with self.lock:
            identifier = payload["request_id"]
            if identifier in self.history:
                previous = self.history[identifier]
                if previous.get("command") != payload:
                    raise ValueError("이미 사용한 요청 ID입니다. 새 목표에는 새 ID를 사용하세요.")
                return False
            reason = self.reason()
            if reason:
                raise RuntimeError(reason)
            self.active = identifier
            self.ui_ids.add(identifier)
            self.history[identifier] = dict(request_id=identifier, status="submitting", stage="submitted", command=deepcopy(payload), metrics={}, error=None)
            return True

    def snapshot(self):
        with self.lock:
            now = self.clock()
            reason = self.reason()
            return deepcopy(dict(positions_rad=self.positions, velocities_rad_sec=self.velocities,
                simulation_sec=self.simulation_sec, joint_state_stamp_simulation_sec=self.joint_state_stamp_simulation_sec,
                state_age_sec=None if self.joint_at is None else now-self.joint_at,
                clock_age_sec=None if self.clock_advanced_at is None else now-self.clock_advanced_at,
                graph=self.graph, simulation_verified=self.identity, guard=self.guard,
                server=self.server, active_request_id=self.active, can_submit=reason is None,
                reason=reason, history=list(reversed(list(self.history.values())))))
