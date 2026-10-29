"""Console-owned, bounded acquisition records; no robot commands or ROS imports.

The recorder copies published poses with their *original* joint acquisition
stamp, and encoded images from the explicitly declared ROS simulation clock.
Discovery association is recorded; it is not camera calibration or hardware
synchronization proof. Saving and sampling run independently of supervision.
"""
from copy import deepcopy
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

    def _context(self, *, collecting=False):
        app = self.app
        with app.lock, app.state.lock:
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

    def start(self):
        with self.lock:
            if self.closed or self.current["status"] in ("preparing", "recording", "saving"):
                raise RuntimeError("현재 기록이 끝난 뒤 새 기록을 시작하세요.")
            if len(self.records) >= MAX_RECORDINGS or self.bytes + MAX_CAPTURE_BYTES > MAX_LIBRARY_BYTES:
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
                self.thread = threading.Thread(target=self._run,
                    args=(identifier, host, image, identity, manifest, assets),
                    name="synchronized-capture", daemon=True)
                self.thread.start()
                return deepcopy(self.current)
        except Exception:
            with self.lock:
                self.current.update(status="failed", reason="capture_not_started")
            raise

    def stop(self, identifier):
        with self.lock:
            if identifier != self.current["id"] or self.current["status"] != "recording":
                raise ValueError("현재 수집 중인 기록을 선택하세요.")
            self.stop_event.set()
            return deepcopy(self.current)

    def _run(self, identifier, host, image, identity, manifest, assets):
        poses, image_frames, pixels = [], [], {}
        started = self.clock()
        last_pose_seq = last_image_seq = 0
        byte_count = sum(map(len, assets.values()))+len(_canonical(manifest))+len(_canonical(identity))+1024
        first_stamp = last_pose_stamp = last_image_stamp = None
        reason = "user_stop"
        try:
            while not self.stop_event.is_set():
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
                    first_stamp = stamp if first_stamp is None else min(first_stamp, stamp)
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
            if actual_bytes > MAX_CAPTURE_BYTES or self.bytes+actual_bytes > MAX_LIBRARY_BYTES:
                raise ValueError("기록 보존 용량 상한을 초과했습니다.")
            recording.save(folder/"recording.json")
            with self.lock:
                self.records[identifier] = recording
                self.hosts[identifier] = SynchronizedSceneHost(recording)
                self.bytes += actual_bytes
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
            return dict(capture=deepcopy(self.current), recordings=[dict(id=identifier,
                label=value.label, duration_sec=value.duration, ended_reason=value.ended_reason)
                for identifier, value in self.records.items()])

    def get(self, identifier):
        with self.lock:
            if type(identifier) is not str or identifier not in self.hosts:
                raise ValueError("이 콘솔에서 보존한 기록을 선택하세요.")
            return self.hosts[identifier]

    def close(self):
        with self.lock:
            self.closed = True
            self.stop_event.set()
            worker = self.thread
        if worker and worker is not threading.current_thread():
            worker.join(timeout=5)
        with self.lock:
            for host in self.hosts.values():
                host.close()
        return worker is None or not worker.is_alive()
