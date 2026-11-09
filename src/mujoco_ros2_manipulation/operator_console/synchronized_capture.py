"""Console-owned, bounded acquisition records; no robot commands or ROS imports.

The recorder copies published poses with their *original* joint acquisition
stamp, and encoded images from the explicitly declared ROS simulation clock.
Discovery association is recorded; it is not camera calibration or hardware
synchronization proof. Saving and sampling run independently of supervision.
"""
from copy import deepcopy
import hashlib
import re
import threading
import time
import uuid
import os

from .scene_host import _json, _canonical
from .synchronized_recording import SyncRecording, SynchronizedSceneHost, _directory, _publish

MAX_RECORDINGS = 8
MAX_CAPTURE_SEC = 120
MAX_WALL_SEC = 180
MAX_CAPTURE_BYTES = 64 * 1024 * 1024
MAX_LIBRARY_BYTES = 512 * 1024 * 1024
MAX_POSES, MAX_IMAGES = 2400, 600


def _saved_size(recording):
    if type(recording) is not SyncRecording:
        raise ValueError("시작 설정에서 검증한 동기 기록만 등록하세요.")
    return (len(_canonical(recording.document)) + sum(map(len, recording.assets.values()))
            + sum(map(len, recording.images.values())))


def load_saved_recordings(paths):
    """Load a bounded startup registry in sequence before creating ROS entities.

    Stop at the first invalid or over-budget bundle. Never eagerly retain eight
    maximum-size loader results before checking the tighter console budgets.
    The paths are explicit trusted CLI inputs, not HTTP request fields.
    """
    if type(paths) not in (list, tuple) or len(paths) > MAX_RECORDINGS:
        raise ValueError("시작 설정의 동기 기록은 최대 8개까지 지정하세요.")
    loaded, total = [], 0
    for path in paths:
        value = SyncRecording.load(path)
        size = _saved_size(value)
        if size > MAX_CAPTURE_BYTES or total + size > MAX_LIBRARY_BYTES:
            raise ValueError("시작 설정 동기 기록의 보존 용량 상한을 초과했습니다.")
        loaded.append(value)
        total += size
    return tuple(loaded)


class CaptureLibrary:
    def __init__(self, app, *, clock=time.monotonic):
        self.app, self.clock = app, clock
        self.lock = threading.RLock()
        self.records, self.hosts = {}, {}
        self.current = dict(status="idle", id=None, reason=None, pose_count=0,
                            image_count=0, duration_sec=0.)
        self.stop_event = threading.Event()
        self.thread = None
        self.closed = False
        self.bytes = 0
        self.retained_count = 0

    def add_saved(self, recording):
        """Register one explicitly loaded startup record, without a file API.

        A separate immutable copy keeps caller-owned maps from changing a
        registered source. Saved acquisition identities and times are retained;
        they never become the current sensor or command execution session.
        """
        if _saved_size(recording) > MAX_CAPTURE_BYTES:
            raise ValueError("등록할 기록의 보존 용량 상한을 초과했습니다.")
        value = SyncRecording(recording.document, dict(recording.assets), dict(recording.images))
        size = _saved_size(value)
        if size > MAX_CAPTURE_BYTES:
            raise ValueError("등록할 기록의 보존 용량 상한을 초과했습니다.")
        identifier = value.sha256[:32]
        with self.lock:
            if self.closed or self.thread is not None or self.current["status"] != "idle":
                raise ValueError("동기 기록은 수집을 시작하기 전 시작 설정에서만 등록하세요.")
            if identifier in self.records:
                if self.records[identifier].sha256 != value.sha256:
                    raise ValueError("동기 기록 식별자가 기존 기록과 다릅니다.")
                return identifier
            if self.retained_count >= MAX_RECORDINGS or self.bytes + size > MAX_LIBRARY_BYTES:
                raise ValueError("이 콘솔의 기록 보존 상한에 도달했습니다.")
            self.records[identifier] = value
            self.hosts[identifier] = SynchronizedSceneHost(value)
            self.bytes += size
            self.retained_count += 1
            return identifier

    def _context(self, *, collecting=False):
        app = self.app
        with app.lock, app.state.lock:
            guard = getattr(app, "require_motion_task", None)
            if callable(guard):
                try:
                    guard()
                except RuntimeError as error:
                    raise ValueError(str(error)) from error
            if (app.closing.is_set() or app.phase != "running" or app.demos.busy
                    or app.display_mode != "web3d" or app.views.mode != "live"
                    or not app.state.identity or app.state.clock_backwards
                    or app.state.graph.get("clock") != 1 or app.state.graph.get("joints") != 1):
                raise ValueError("실행 중인 검증된 작업 환경의 실시간 3D 보기에서 기록하세요.")
            host = app.views.host
            view = host.snapshot() if host is not None else {}
            if view.get("status") != "ready" or not view.get("pose_ready"):
                raise ValueError("3D 관절 자세를 받은 뒤 기록하세요.")
            binding = view["binding"]
            motion = (app.state.server or {}).get("session_id")
            if not motion or binding.get("motion_session_id") != motion or binding.get("job_id"):
                raise ValueError("같은 명령 실행의 실시간 3D 자세가 필요합니다.")
            image = getattr(app.bridge, "image_source", None)
            info = None if image is None else image.snapshot()
            if (not info or info.get("clock_domain") != "ros_simulation"
                    or info.get("publisher_count") != 1 or info.get("clock_invalid")
                    or info.get("status") not in (("ready", "encoding", "stale") if collecting else ("ready", "encoding"))
                    or not info.get("latest")):
                raise ValueError("ROS 시뮬레이션 시각을 선언한 RGB8 센서 입력이 필요합니다.")
            sample = host.capture_sample(view["binding_id"])
            if not sample or sample.get("source_stamp_ns") is None:
                raise ValueError("원본 관절 시각이 있는 3D 자세를 기다리세요.")
            if not collecting:
                now = self.clock()
                received = sample.get("source_received_monotonic")
                advanced = getattr(app.state,"clock_advanced_at",None)
                if (received is None or advanced is None or not 0 <= now-received <= .5
                        or not 0 <= now-advanced <= .5):
                    raise ValueError("관절 자세와 시뮬레이션 시계가 갱신된 뒤 기록하세요.")
            identity = dict(binding=deepcopy(binding), operator_session_id=app.session_id,
                motion_session_id=motion, app_generation=app.generation,
                image_generation=info["generation"], image_source_id=info["source_id"],
                topic=info["topic"], publisher_endpoint_sha256=info["publisher_endpoint_sha256"],
                clock_domain="ros_simulation")
            return host, image, view, info, sample, identity

    def _grasp_selected(self):
        return getattr(getattr(self.app, "tasks", None), "selected", None) == "grasp"

    def _grasp_context(self):
        source = self.app.tasks.capture_context()
        info = source["image"].snapshot()
        if (source["finished"] or source.get("fault") or not info.get("configured")
                or info.get("clock_domain") != "ros_simulation" or info.get("clock_invalid")
                or info.get("publisher_count", 0) > 1):
            raise ValueError("실행 중인 집기에 명시적으로 등록한 단일 ROS 시뮬레이션 RGB8 입력이 필요합니다.")
        native = source["owner"]
        owner = dict(operator_session_id=self.app.session_id, selection_id=source["selection_id"],
            job_id=native["job_id"], job_generation=native["generation"], binding_id=native["binding_id"],
            model_sha256=native["model_sha256"], execution_manifest_sha256=native["execution_manifest_sha256"],
            domain=native["domain"], source_registration_epoch=native["source_registration_epoch"],
            native_owner=deepcopy(native))
        pair = self._image_epoch(info)
        token = hashlib.sha256(_canonical(dict(owner=owner, image_epoch=pair))).hexdigest()
        return source, info, owner, token

    @staticmethod
    def _image_epoch(info):
        publisher = info.get("publisher_endpoint_sha256")
        generation = info.get("generation")
        if (info.get("publisher_count") == 1 and type(publisher) is str
                and re.fullmatch("[0-9a-f]{64}", publisher) and type(generation) is str
                and re.fullmatch("[0-9a-f]{32}", generation)):
            return generation, publisher
        return None

    def image_source(self, expected_owner_token=None):
        """Resolve a fixed selected provider, never a browser-provided ROS topic."""
        if not self._grasp_selected():
            if expected_owner_token is not None:
                raise ValueError("센서 이미지의 집기 실행 소유자가 바뀌었습니다.")
            image = getattr(self.app.bridge, "image_source", None)
            return image, None if image is None else image.snapshot()
        source = self.app.tasks.capture_context()
        info = source["image"].snapshot()
        native = source["owner"]
        owner = dict(operator_session_id=self.app.session_id, selection_id=source["selection_id"],
            job_id=native["job_id"], job_generation=native["generation"], binding_id=native["binding_id"],
            model_sha256=native["model_sha256"], execution_manifest_sha256=native["execution_manifest_sha256"],
            domain=native["domain"], source_registration_epoch=native["source_registration_epoch"],
            native_owner=deepcopy(native))
        token = hashlib.sha256(_canonical(dict(owner=owner, image_epoch=self._image_epoch(info)))).hexdigest()
        if expected_owner_token is not None and expected_owner_token != token:
            raise ValueError("센서 이미지의 집기 실행 소유자가 바뀌었습니다.")
        info.update(owner_token=token, owner=owner)
        return source["image"], info

    def start(self, expected_owner_token=None):
        if self._grasp_selected():
            return self._start_grasp(expected_owner_token)
        if expected_owner_token is not None:
            raise ValueError("현재 일반 조작의 기록 요청에 집기 소유자를 사용할 수 없습니다.")
        with self.lock:
            if self.closed or self.current["status"] in ("preparing", "armed", "recording", "saving"):
                raise RuntimeError("현재 기록이 끝난 뒤 새 기록을 시작하세요.")
            if self.retained_count >= MAX_RECORDINGS or self.bytes + MAX_CAPTURE_BYTES > MAX_LIBRARY_BYTES:
                raise RuntimeError("이 콘솔의 기록 보존 상한에 도달했습니다.")
            identifier = uuid.uuid4().hex
            self.current = dict(status="preparing", id=identifier, reason=None,
                                pose_count=0, image_count=0, duration_sec=0.)
        try:
            host, image, view, info, sample, identity = self._context()
            manifest = _json(host.manifest_bytes(view["binding_id"]))
            assets = {item["sha256"]: host.asset_bytes(item["sha256"]) for item in manifest["assets"]}
            if sum(map(len, assets.values()))+len(_canonical(manifest))+len(_canonical(identity))+1024 > MAX_CAPTURE_BYTES:
                raise ValueError("표시 모델의 기록 자산 상한을 초과했습니다.")
            # Recheck lifecycle after potentially slow asset reads. Do not take
            # the application lock during hashing, file I/O or JPEG conversion.
            if self._context()[-1] != identity:
                raise ValueError("기록 준비 중 실행이나 센서 공급원이 바뀌었습니다.")
            with self.lock:
                if self.closed:
                    raise ValueError("콘솔이 종료 중입니다.")
                self.stop_event = threading.Event()
                self.current["status"] = "recording"
                # Failed acquisitions retain original bytes too. Reserve their
                # complete budget before collecting; failures keep the reserved
                # upper bound instead of allowing unlimited partial bundles.
                self.retained_count += 1
                self.bytes += MAX_CAPTURE_BYTES
                self.thread = threading.Thread(target=self._run,
                    args=(identifier, host, image, identity, manifest, assets),
                    name="synchronized-capture", daemon=True)
                self.thread.start()
                return deepcopy(self.current)
        except Exception:
            with self.lock:
                self.current.update(status="failed", reason="capture_not_started")
            raise

    def stop(self, identifier, expected_owner_token=None):
        with self.lock:
            tasks = getattr(self.app, "tasks", None)
            if tasks is not None:
                tasks.require_capture_selection()
            if expected_owner_token != self.current.get("owner_token"):
                raise ValueError("현재 기록의 소유자 식별자가 다릅니다.")
            if (self.current.get("owner") and tasks is not None
                    and tasks.selection_id != self.current["owner"]["selection_id"]):
                raise ValueError("기록 요청의 작업 선택이 바뀌었습니다.")
            if identifier != self.current["id"] or self.current["status"] not in ("armed", "recording"):
                raise ValueError("현재 수집 중인 기록을 선택하세요.")
            self.stop_event.set()
            return deepcopy(self.current)

    def _start_grasp(self, expected_owner_token):
        source, info, owner, token = self._grasp_context()
        if type(expected_owner_token) is not str or expected_owner_token != token:
            raise ValueError("현재 집기 기록의 소유자 식별자를 다시 확인하세요.")
        with self.lock:
            if self.closed or self.current["status"] in ("preparing", "armed", "recording", "saving"):
                raise RuntimeError("현재 기록이 끝난 뒤 새 기록을 시작하세요.")
            if self.retained_count >= MAX_RECORDINGS or self.bytes + MAX_CAPTURE_BYTES > MAX_LIBRARY_BYTES:
                raise RuntimeError("이 콘솔의 기록 보존 상한에 도달했습니다.")
            identifier = uuid.uuid4().hex
            self.current = dict(status="preparing", id=identifier, reason=None, pose_count=0, image_count=0,
                event_count=0, duration_sec=None, owner_token=token, owner=deepcopy(owner), awaiting_sources=True,
                media_ready=False, task_status=source["job_status"])
        try:
            manifest = deepcopy(source["manifest"])
            assets = {item["sha256"]: source["asset_bytes"](item["sha256"]) for item in manifest["assets"]}
            if sum(map(len, assets.values())) + len(_canonical(manifest)) + len(_canonical(owner)) + 8192 > MAX_CAPTURE_BYTES:
                raise ValueError("표시 모델의 기록 자산 상한을 초과했습니다.")
            current, current_info, current_owner, _ = self._grasp_context()
            if current_owner != owner or current["image"] is not source["image"]:
                raise ValueError("기록 준비 중 집기 실행 소유자가 바뀌었습니다.")
            epoch = self._image_epoch(info)
            if epoch is not None and self._image_epoch(current_info) != epoch:
                raise ValueError("기록 준비 중 센서 공급원이 바뀌었습니다.")
            with self.app.tasks.capture_admission(owner), self.lock:
                if self.closed or self.current.get("id") != identifier or self.current["status"] != "preparing":
                    raise ValueError("기록 준비가 종료됐습니다.")
                # The provider snapshot is memory-only. Pin the epoch again at
                # admission after asset I/O, without refreshing its transport.
                admitted_info = source["image"].snapshot()
                if (not admitted_info.get("configured") or admitted_info.get("clock_domain") != "ros_simulation"
                        or admitted_info.get("clock_invalid") or admitted_info.get("publisher_count", 0) > 1
                        or admitted_info.get("status") == "error"):
                    raise ValueError("기록 준비 중 센서 공급원의 시각이나 상태가 바뀌었습니다.")
                admitted_epoch = self._image_epoch(admitted_info)
                if epoch is not None and admitted_epoch != epoch:
                    raise ValueError("기록 준비 중 센서 공급원이 바뀌었습니다.")
                self.stop_event = threading.Event()
                self.current.update(status="armed", awaiting_sources=True)
                self.retained_count += 1
                self.bytes += MAX_CAPTURE_BYTES
                self.thread = threading.Thread(target=self._run_grasp,
                    args=(identifier, current, owner, manifest, assets, admitted_epoch),
                    name="synchronized-grasp-capture", daemon=True)
                self.thread.start()
                return deepcopy(self.current)
        except Exception:
            with self.lock:
                self.current.update(status="failed", reason="capture_not_started")
            raise

    def _run_grasp(self, identifier, source, owner, manifest, assets, epoch):
        poses, frames, events, pixels = [], [], [], {}
        last_pose = last_image = last_event = 0
        # The producer's eviction count plus deque length is an accepted-row
        # index, independent of the original native physics sequence. Existing
        # pre-arm rows are observed history, not newly captured samples.
        last_pose_index = source.get("poses_dropped", 0) + len(source["poses"])
        started = self.clock()
        armed_ns = round(started * 1_000_000_000)
        reason, task_status = "user_stop", source["job_status"]
        first_stamp = last_stamp = None
        image = source["image"]
        native = owner["native_owner"]
        identity = dict(operator_session_id=self.app.session_id, selection_id=owner["selection_id"],
            job_id=native["job_id"], job_generation=native["generation"], binding=deepcopy(native["binding"]),
            binding_id=native["binding_id"], model_sha256=native["model_sha256"],
            execution_manifest_sha256=native["execution_manifest_sha256"], domain=native["domain"],
            source_registration_epoch=native["source_registration_epoch"], native_clock_domain="mujoco-native:"+native["job_id"],
            native_stamp_semantics="rounded_native_mjData_time", image_clock_domain="ros_simulation",
            clock_association="declared_same_run", image_generation=None if epoch is None else epoch[0],
            publisher_endpoint_sha256=None if epoch is None else epoch[1],
            image_source_id=image.snapshot()["source_id"], topic=image.snapshot()["topic"])
        byte_count = sum(map(len, assets.values())) + len(_canonical(manifest)) + len(_canonical(identity)) + 8192
        counters = dict(poses_dropped=source.get("poses_dropped", 0), events_dropped=source.get("events_dropped", 0),
            poses_source_gaps=source.get("poses_source_gaps", 0),
            image_transport_gaps=image.snapshot().get("transport_gaps", 0),
            image_counters=deepcopy(image.snapshot().get("counters", {})))
        try:
            while True:
                # The final source journal is drained even after user_stop or
                # terminal cancellation. No control command is called here.
                current = self.app.tasks.capture_context()
                if current["owner"] != native or current["selection_id"] != owner["selection_id"] or current["image"] is not image:
                    reason = "source_changed"
                    break
                task_status = current["job_status"]
                info = image.snapshot()
                counters.update(poses_dropped=current.get("poses_dropped", 0), events_dropped=current.get("events_dropped", 0),
                    poses_source_gaps=current.get("poses_source_gaps", 0), image_transport_gaps=info.get("transport_gaps", 0),
                    image_counters=deepcopy(info.get("counters", {})))
                previous_event = last_event
                for item in current["events"]:
                    if item["event_sequence"] <= last_event:
                        continue
                    size = len(_canonical(item))
                    if byte_count + size > MAX_CAPTURE_BYTES or len(events) >= 128:
                        reason = "byte_or_sample_limit"
                        break
                    events.append(deepcopy(item))
                    byte_count += size
                    last_event = item["event_sequence"]
                if reason != "user_stop":
                    break
                info = image.snapshot()
                observed_epoch = self._image_epoch(info)
                if epoch is None and observed_epoch is not None:
                    epoch = observed_epoch
                    identity.update(image_generation=epoch[0], publisher_endpoint_sha256=epoch[1])
                if info.get("clock_invalid"):
                    reason = "clock_reversed"
                    break
                if epoch is not None and observed_epoch != epoch:
                    reason = "source_changed"
                    break
                if current.get("fault") or info.get("status") == "error":
                    reason = "recording_source_fault"
                    break
                pending_events = current["events"]
                lost_poses = current.get("poses_dropped", 0) > last_pose_index
                lost_events = (pending_events and pending_events[0]["event_sequence"] > previous_event + 1)
                if lost_poses or lost_events:
                    reason = "source_journal_limit"
                    break
                for index, item in enumerate(current["poses"], current.get("poses_dropped", 0) + 1):
                    if index <= last_pose_index:
                        continue
                    last_pose_index = index
                    if item["sequence"] <= last_pose or int(item["native"]["observed_monotonic_ns"]) < armed_ns:
                        continue
                    stamp = int(item["stamp_ns"])
                    if poses and stamp <= int(poses[-1]["stamp_ns"]):
                        reason = "clock_reversed"
                        break
                    if first_stamp is not None and max(last_stamp, stamp)-min(first_stamp, stamp) > MAX_CAPTURE_SEC*1_000_000_000:
                        reason = "duration_limit"
                        break
                    size = len(_canonical(item))
                    if byte_count + size > MAX_CAPTURE_BYTES or len(poses) >= MAX_POSES:
                        reason = "byte_or_sample_limit"
                        break
                    poses.append(deepcopy(item))
                    byte_count += size
                    first_stamp = stamp if first_stamp is None else min(first_stamp, stamp)
                    last_stamp = stamp if last_stamp is None else max(last_stamp, stamp)
                    last_pose = item["sequence"]
                if reason != "user_stop":
                    break
                if epoch is not None:
                    for metadata in info.get("history", []):
                        sequence = metadata["sequence"]
                        if (metadata["generation"] != epoch[0] or sequence <= last_image
                                or not metadata.get("image_available") or metadata["received_monotonic_sec"] < started):
                            continue
                        value = image.frame(epoch[0], sequence)
                        if value is None:
                            continue
                        descriptor, raw = value
                        stamp = descriptor["source_stamp_ns"]
                        number = int(stamp)
                        if frames and number <= int(frames[-1]["stamp_ns"]):
                            reason = "clock_reversed"
                            break
                        if first_stamp is not None and max(last_stamp, number)-min(first_stamp, number) > MAX_CAPTURE_SEC*1_000_000_000:
                            reason = "duration_limit"
                            break
                        digest = descriptor["jpeg_sha256"]
                        item = dict(stamp_ns=stamp, sequence=sequence, jpeg_sha256=digest, byte_length=len(raw),
                            width=descriptor["width"], height=descriptor["height"], source=dict(generation=epoch[0],
                                source_id=info["source_id"], topic=info["topic"], frame_id=descriptor["frame_id"],
                                publisher_endpoint_sha256=epoch[1], clock_domain=info["clock_domain"],
                                received_monotonic_sec=descriptor["received_monotonic_sec"],
                                encoded_monotonic_sec=descriptor["encoded_monotonic_sec"]))
                        size = len(_canonical(item)) + (0 if digest in pixels else len(raw))
                        if byte_count + size > MAX_CAPTURE_BYTES or len(frames) >= MAX_IMAGES:
                            reason = "byte_or_sample_limit"
                            break
                        frames.append(item)
                        pixels[digest] = raw
                        byte_count += size
                        first_stamp = number if first_stamp is None else min(first_stamp, number)
                        last_stamp = number if last_stamp is None else max(last_stamp, number)
                        last_image = sequence
                stamps = [int(item["stamp_ns"]) for item in poses + frames]
                duration = 0. if not stamps else (max(stamps)-min(stamps))/1e9
                media_ready = len(poses) >= 2 and bool(frames) and duration > 0
                with self.lock:
                    self.current.update(status="recording" if media_ready else "armed", awaiting_sources=not media_ready,
                        media_ready=media_ready, pose_count=len(poses), image_count=len(frames), event_count=len(events),
                        duration_sec=duration if media_ready else None, task_status=task_status)
                if reason != "user_stop":
                    break
                if current["finished"]:
                    reason = "task_finished"
                    break
                if duration >= MAX_CAPTURE_SEC or self.clock() - started >= MAX_WALL_SEC:
                    reason = "duration_limit"
                    break
                if self.stop_event.is_set():
                    break
                self.stop_event.wait(.05)
            with self.lock:
                self.current.update(status="saving", reason=reason, pose_count=len(poses),
                                    image_count=len(frames), event_count=len(events), task_status=task_status)
            stamps = [int(item["stamp_ns"]) for item in poses + frames]
            media_ready = len(poses) >= 2 and bool(frames) and bool(stamps) and 0 < max(stamps)-min(stamps) <= MAX_CAPTURE_SEC*1_000_000_000
            document = dict(schema_version=2, kind="synchronized-grasp-recording-v2",
                label="집기 센서·3D·이벤트 기록 · "+identifier[:8], manifest=manifest, identity=identity,
                origin_ns=str(min(stamps)) if media_ready else None, poses=poses, images=frames, events=events,
                ended_reason=reason, task_status=task_status, pose_sample_semantics="parent_accepted_latest_ipc",
                observation_counters=counters, media_state="ready" if media_ready else "unavailable",
                media_reason=None if media_ready else "insufficient_paired_native_and_image_observations")
            folder = self.app.directory / "synchronized" / identifier
            folder.mkdir(parents=True, exist_ok=False)
            (folder / "capture.json").write_bytes(_canonical(dict(id=identifier, reason=reason,
                pose_count=len(poses), image_count=len(frames), event_count=len(events), owner_token=self.current["owner_token"])))
            recording = SyncRecording(document, assets, pixels)
            actual_bytes = _saved_size(recording)
            if actual_bytes > MAX_CAPTURE_BYTES:
                raise ValueError("기록 보존 용량 상한을 초과했습니다.")
            recording.save(folder / "recording.json")
            with self.lock:
                self.records[identifier] = recording
                self.hosts[identifier] = SynchronizedSceneHost(recording)
                self.bytes += actual_bytes - MAX_CAPTURE_BYTES
                self.current.update(status="completed", reason=reason, awaiting_sources=False,
                                    media_ready=media_ready, task_status=task_status)
        except Exception as error:
            self._preserve_grasp_partial(identifier, identity, manifest, assets, pixels, poses, frames, events,
                                         reason, task_status, counters, error)
            with self.lock:
                self.current.update(status="failed", reason="insufficient_or_invalid_capture", awaiting_sources=False,
                                    pose_count=len(poses), image_count=len(frames), event_count=len(events), task_status=task_status)
            self.app.audit("synchronized_capture_failed", recording_id=identifier, error_type=type(error).__name__)

    def _preserve_grasp_partial(self, identifier, identity, manifest, assets, pixels, poses, frames, events,
                                reason, task_status, counters, error):
        folder = self.app.directory / "synchronized" / identifier
        try:
            directory = _directory(folder, create=True)
            try:
                _publish(directory, "partial.json", _canonical(dict(schema_version=2, identity=identity,
                    manifest=manifest, poses=poses, images=frames, events=events, reason=reason,
                    task_status=task_status, pose_sample_semantics="parent_accepted_latest_ipc",
                    observation_counters=counters, error_type=type(error).__name__)))
                for name, values, suffix in (("assets", assets, ".bin"), ("images", pixels, ".jpg")):
                    if not values:
                        continue
                    try:
                        os.mkdir(name, 0o700, dir_fd=directory)
                    except FileExistsError:
                        pass
                    child = os.open(name, os.O_RDONLY | os.O_DIRECTORY | os.O_NOFOLLOW, dir_fd=directory)
                    try:
                        for digest, raw in values.items():
                            _publish(child, digest + suffix, raw, reuse=True)
                    finally:
                        os.close(child)
            finally:
                os.close(directory)
        except OSError:
            pass

    def _run(self, identifier, host, image, identity, manifest, assets):
        poses, image_frames, pixels = [], [], {}
        started = self.clock()
        last_pose_seq = last_image_seq = 0
        byte_count = sum(map(len, assets.values()))+len(_canonical(manifest))+len(_canonical(identity))+1024
        first_stamp = last_pose_stamp = last_image_stamp = None
        reason = "user_stop"
        try:
            while not self.stop_event.is_set():
                if self.clock()-started >= MAX_WALL_SEC:
                    reason = "duration_limit"
                    break
                try:
                    current_host, current_image, view, info, sample, observed = self._context(collecting=True)
                except ValueError:
                    info = image.snapshot()
                    reason = ("clock_reversed" if info.get("clock_invalid") or self.app.state.clock_backwards
                              else "source_changed" if info.get("generation") != identity["image_generation"]
                              or self.app.generation != identity["app_generation"] else "source_unavailable")
                    break
                if current_host is not host or current_image is not image or observed != identity:
                    reason = "source_changed"
                    break
                stamp = int(sample["source_stamp_ns"])
                if first_stamp is not None and stamp-first_stamp > MAX_CAPTURE_SEC*1_000_000_000:
                    reason = "duration_limit"
                    break
                if last_pose_stamp is not None and stamp < last_pose_stamp:
                    reason = "clock_reversed"
                    break
                if sample["sequence"] > last_pose_seq and (last_pose_stamp is None or stamp > last_pose_stamp):
                    item = dict(stamp_ns=str(stamp), sequence=sample["sequence"],
                                positions_rad=sample["positions_rad"], pose=sample["pose"])
                    size = len(_canonical(item))
                    if byte_count + size > MAX_CAPTURE_BYTES or len(poses) >= MAX_POSES:
                        reason = "byte_or_sample_limit"
                        break
                    poses.append(item)
                    byte_count += size
                    first_stamp = stamp if first_stamp is None else min(first_stamp, stamp)
                    last_pose_seq, last_pose_stamp = sample["sequence"], stamp
                # Retain only images received during this capture; old live
                # cache frames cannot be relabelled as a new recorded run.
                for metadata in info["history"]:
                    sequence = metadata["sequence"]
                    if (metadata["generation"] != identity["image_generation"]
                            or sequence <= last_image_seq or not metadata["image_available"]
                            or metadata["received_monotonic_sec"] < started):
                        continue
                    value = image.frame(identity["image_generation"], sequence)
                    if value is None:
                        continue
                    descriptor, raw = value
                    stamp = int(descriptor["source_stamp_ns"])
                    if first_stamp is not None and stamp < first_stamp:
                        # Delayed acquisitions before the first captured pose
                        # cannot move this capture's origin backwards.
                        continue
                    if first_stamp is not None and stamp-first_stamp > MAX_CAPTURE_SEC*1_000_000_000:
                        reason = "duration_limit"
                        break
                    if last_image_stamp is not None and stamp <= last_image_stamp:
                        reason = "clock_reversed"
                        break
                    digest = descriptor["jpeg_sha256"]
                    item = dict(stamp_ns=str(stamp), sequence=sequence,
                        jpeg_sha256=digest, byte_length=len(raw), width=descriptor["width"], height=descriptor["height"])
                    size = (0 if digest in pixels else len(raw))+len(_canonical(item))+1
                    if byte_count + size > MAX_CAPTURE_BYTES or len(image_frames) >= MAX_IMAGES:
                        reason = "byte_or_sample_limit"
                        break
                    image_frames.append(item)
                    pixels[digest] = raw
                    byte_count += size
                    first_stamp = stamp if first_stamp is None else first_stamp
                    last_image_seq, last_image_stamp = sequence, stamp
                if reason != "user_stop":
                    break
                end = max(last_pose_stamp or 0, last_image_stamp or 0)
                duration = 0. if first_stamp is None else (end-first_stamp)/1e9
                with self.lock:
                    self.current.update(pose_count=len(poses), image_count=len(image_frames), duration_sec=duration)
                if duration >= MAX_CAPTURE_SEC or self.clock()-started >= MAX_WALL_SEC:
                    reason = "duration_limit"
                    break
                self.stop_event.wait(.1)
            with self.lock:
                self.current.update(status="saving", reason=reason, pose_count=len(poses), image_count=len(image_frames))
            document = dict(schema_version=1, kind="synchronized-recording-v1",
                label="센서·3D 관측 기록 · "+identifier[:8], manifest=manifest,
                identity=identity, origin_ns=str(first_stamp or 0), poses=poses,
                images=image_frames, ended_reason=reason)
            # Preserve even insufficient/failed acquisition evidence locally.
            folder = self.app.directory/"synchronized"/identifier
            folder.mkdir(parents=True, exist_ok=False)
            (folder/"capture.json").write_bytes(_canonical(dict(id=identifier,
                reason=reason, pose_count=len(poses), image_count=len(image_frames))))
            recording = SyncRecording(document, assets, pixels)
            actual_bytes = len(_canonical(recording.document))+sum(map(len,assets.values()))+sum(map(len,pixels.values()))
            if actual_bytes > MAX_CAPTURE_BYTES or self.bytes-MAX_CAPTURE_BYTES+actual_bytes > MAX_LIBRARY_BYTES:
                raise ValueError("기록 보존 용량 상한을 초과했습니다.")
            recording.save(folder/"recording.json")
            with self.lock:
                self.records[identifier] = recording
                self.hosts[identifier] = SynchronizedSceneHost(recording)
                self.bytes += actual_bytes-MAX_CAPTURE_BYTES
                self.current.update(status="completed", reason=reason)
        except Exception as error:
            # An incomplete capture is diagnostic data, never a playable record.
            try:
                folder = self.app.directory/"synchronized"/identifier
                folder.mkdir(parents=True, exist_ok=True)
                with (folder/"partial.json").open("xb") as stream:
                    stream.write(_canonical(dict(identity=identity, reason=reason,
                        poses=poses, images=image_frames, error_type=type(error).__name__)))
                directory = _directory(folder)
                try:
                    for name, values, suffix in (("assets",assets,".bin"),("images",pixels,".jpg")):
                        try:os.mkdir(name,0o700,dir_fd=directory)
                        except FileExistsError:pass
                        child=os.open(name,os.O_RDONLY|os.O_DIRECTORY|os.O_NOFOLLOW,dir_fd=directory)
                        try:
                            for digest,raw in values.items():_publish(child,digest+suffix,raw,reuse=True)
                        finally:os.close(child)
                finally:os.close(directory)
            except OSError:
                pass
            with self.lock:
                self.current.update(status="failed", reason="insufficient_or_invalid_capture",
                    pose_count=len(poses), image_count=len(image_frames))
            self.app.audit("synchronized_capture_failed", recording_id=identifier, error_type=type(error).__name__)

    def snapshot(self):
        with self.lock:
            capture = deepcopy(self.current)
            can_start = (not self.closed and capture["status"] not in ("preparing", "armed", "recording", "saving")
                and self.retained_count < MAX_RECORDINGS and self.bytes+MAX_CAPTURE_BYTES <= MAX_LIBRARY_BYTES)
            result = dict(capture=capture, recordings=[dict(id=identifier,
                label=value.label, duration_sec=value.duration, ended_reason=value.ended_reason,
                recording_version=value.version, recording_kind=value.kind,
                media_state=value.media_state, media_reason=value.media_reason,
                task_status=value.task_status)
                for identifier, value in self.records.items()])
            if not can_start and (self.retained_count >= MAX_RECORDINGS or self.bytes+MAX_CAPTURE_BYTES > MAX_LIBRARY_BYTES):
                capture["availability_reason"] = "이 콘솔의 완료·실패 기록 보존 상한에 도달했습니다."
        capture.update(can_arm=False, expected_owner_token=None, expected_owner=None,
                       can_stop=capture["status"] in ("armed", "recording"))
        if self._grasp_selected():
            try:
                source, info, owner, token = self._grasp_context()
                capture.update(expected_owner_token=token, expected_owner=owner, can_arm=can_start)
            except ValueError as error:
                can_start = False
                capture["availability_reason"] = str(error)
        elif can_start:
            try:
                self._context()
            except ValueError as error:
                can_start = False
                capture["availability_reason"] = str(error)
        capture["can_start"] = can_start
        capture.setdefault("awaiting_sources", False)
        capture.setdefault("media_ready", False)
        return result

    def get(self, identifier):
        with self.lock:
            if type(identifier) is not str or identifier not in self.hosts:
                raise ValueError("이 콘솔에서 보존한 기록을 선택하세요.")
            return self.hosts[identifier]

    def request_close(self):
        with self.lock:
            self.closed = True
            self.stop_event.set()
    def close(self):
        self.request_close()
        with self.lock:
            worker = self.thread
        if worker and worker is not threading.current_thread():
            worker.join(timeout=5)
        with self.lock:
            for host in self.hosts.values():
                host.close()
        return worker is None or not worker.is_alive()
