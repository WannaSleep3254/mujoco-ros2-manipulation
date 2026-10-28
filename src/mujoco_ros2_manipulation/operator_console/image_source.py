"""Bounded RGB8 observations and a separate JPEG worker; no ROS/control imports."""
from collections import Counter, OrderedDict
from array import array
from copy import deepcopy
import hashlib
import io
import math
import re
import threading
import time
import uuid


MAX_WIDTH, MAX_HEIGHT, MAX_STEP = 640, 480, 1920
MAX_ROS_SECONDS = 2**31 - 1
MAX_RAW_BYTES = 921600
MAX_JPEG_BYTES = 512 * 1024
MAX_FRAMES, MAX_CACHE_BYTES, MAX_HISTORY = 64, 8 * 1024 * 1024, 256


def _require(condition, reason):
    if not condition:
        raise ValueError(reason)


def _finite(value, lower, upper):
    try:
        return type(value) in (int, float) and math.isfinite(value) and lower <= value <= upper
    except OverflowError:
        return False


def _name(value, maximum=128):
    return type(value) is str and 0 < len(value) <= maximum and re.fullmatch(r"[A-Za-z_][A-Za-z0-9_/]*", value) is not None


def _jpeg(width, height, step, data):
    # Input is already bounded raw RGB, never an arbitrary compressed file.
    from PIL import Image
    image = Image.frombytes("RGB", (width, height), data, "raw", "RGB", step, 1)
    try:
        output = io.BytesIO()
        image.save(output, format="JPEG", quality=85)
        return output.getvalue()
    finally:
        image.close()


class ImageSource:
    """One opt-in image stream with latest-only input and bounded encoded history.

    Publisher identity is discovery-associated, not proved for each ROS message.
    ``generation`` changes invalidate in-flight callbacks and HTTP frame lookups.
    Source stamps and monotonic receipt/encoding ages are never subtracted.
    """
    def __init__(self, source_id="camera0", topic="/camera/image", *, frame_id=None,
                 clock=time.monotonic, encoder=None, stale_after_sec=.5, frame_stale_after_sec=1.0,
                 clock_domain="unknown"):
        _require(type(source_id) is str and re.fullmatch(r"[a-z][a-z0-9_]{0,63}", source_id), "invalid_source_id")
        _require(type(topic) is str and len(topic) <= 256
                 and re.fullmatch(r"(?:/[A-Za-z_][A-Za-z0-9_]*)+", topic) is not None, "invalid_topic")
        _require(frame_id is None or _name(frame_id), "invalid_frame_id")
        _require(_finite(stale_after_sec, .05, 60) and _finite(frame_stale_after_sec, .05, 60), "invalid_stale_limit")
        _require(encoder is None or callable(encoder), "invalid_encoder")
        _require(callable(clock), "invalid_receipt_clock")
        _require(clock_domain in ("unknown", "ros_simulation"), "invalid_clock_domain")
        self.source_id, self.topic, self.expected_frame_id = source_id, topic, frame_id
        self.clock_domain = clock_domain
        self.clock, self.encoder = clock, encoder or _jpeg
        self.stale_after_sec, self.frame_stale_after_sec = float(stale_after_sec), float(frame_stale_after_sec)
        self.lock = threading.RLock()
        self._generation, self._sequence = uuid.uuid4().hex, 0
        self._publishers, self._publisher_count = (), 0
        self._ever_had_publisher = False
        self._closed, self._clock_invalid = False, False
        self._clock_ns = self._last_stamp_ns = self._last_receipt = None
        self._frame_id = frame_id
        self._pending, self._latest_received, self._latest_encoded = None, None, None
        self._frames, self._history = OrderedDict(), OrderedDict()
        self._cache_bytes, self._inflight_bytes = 0, 0
        self._encoding, self._next_encode_at = False, 0.
        self._last_error = None
        self._counts, self._invalid_reasons = Counter(), Counter()
        self._wake, self._stop = threading.Event(), threading.Event()
        self._worker = None

    @property
    def generation(self):
        with self.lock:
            return self._generation

    def _reset(self, reason, *, clear_clock):
        for metadata in self._history.values():
            if metadata["processing"] in ("pending", "encoding"):
                metadata["processing"] = "generation_invalidated"
        self._generation = uuid.uuid4().hex
        self._pending = self._latest_received = self._latest_encoded = None
        self._last_stamp_ns = self._last_receipt = None
        self._frame_id = self.expected_frame_id
        self._frames.clear()
        self._cache_bytes = 0
        self._last_error = None
        self._counts["generation_changes"] += 1
        if clear_clock:
            self._clock_invalid, self._clock_ns = False, None
        self._wake.set()

    def new_epoch(self):
        """Explicit new acquisition/execution epoch; never called by a frame stamp."""
        with self.lock:
            _require(not self._closed, "source_closed")
            self._reset("explicit_epoch", clear_clock=True)
            return self._generation

    def set_publishers(self, endpoint_ids):
        """Apply bounded discovery hashes. Only a unique endpoint can feed frames."""
        _require(type(endpoint_ids) in (list, tuple) and len(endpoint_ids) <= 8, "invalid_publisher_catalog")
        _require(all(type(value) is str and re.fullmatch("[0-9a-f]{64}", value) for value in endpoint_ids)
                 and len(set(endpoint_ids)) == len(endpoint_ids), "invalid_publisher_catalog")
        signature = tuple(sorted(endpoint_ids))
        with self.lock:
            if self._closed:
                return self._generation
            if signature != self._publishers or len(signature) != self._publisher_count:
                self._reset("publisher_change", clear_clock=True)
                self._counts["publisher_changes"] += 1
            self._publishers, self._publisher_count = signature, len(signature)
            self._ever_had_publisher |= bool(signature)
            return self._generation

    def discovery_failed(self):
        with self.lock:
            self._counts["discovery_errors"] += 1
        self.set_publishers(())

    def discovery_ambiguous(self):
        """A discovery catalog beyond the cap has at least nine publishers.

        Count nine is a lower bound, not an invented endpoint identity or exact
        graph size. Repeated over-cap discovery does not create new epochs.
        """
        with self.lock:
            if self._closed:
                return self._generation
            if self._publisher_count != 9:
                self._reset("publisher_change", clear_clock=True)
                self._counts["publisher_changes"] += 1
            self._publishers, self._publisher_count = (), 9
            self._ever_had_publisher = True
            return self._generation

    def _invalidate_clock(self, reason):
        if not self._clock_invalid:
            self._reset(reason, clear_clock=False)
            self._clock_invalid = True
            self._last_error = reason
            self._counts["clock_reversals"] += 1

    def observe_clock(self, seconds):
        """Compatibility input in approximate seconds; ROS uses integer parts."""
        if not _finite(seconds, 0, MAX_ROS_SECONDS + 1):
            return self._reject("invalid_clock")
        stamp = int(round(seconds * 1e9))
        if stamp > MAX_ROS_SECONDS * 1_000_000_000 + 999_999_999:
            return self._reject("invalid_clock")
        return self._observe_clock_ns(stamp)

    def observe_clock_stamp(self, seconds, nanoseconds):
        """Preserve the ROS integer stamp and detect even a 1 ns reversal."""
        if (type(seconds) is not int or not 0 <= seconds <= MAX_ROS_SECONDS
                or type(nanoseconds) is not int or not 0 <= nanoseconds < 1_000_000_000):
            return self._reject("invalid_clock")
        return self._observe_clock_ns(seconds * 1_000_000_000 + nanoseconds)

    def _observe_clock_ns(self, stamp):
        with self.lock:
            if self._closed:
                return False
            if self._clock_ns is not None and stamp < self._clock_ns:
                self._invalidate_clock("ros_clock_backwards")
            self._clock_ns = stamp
            return not self._clock_invalid

    def _reject(self, reason):
        with self.lock:
            self._counts["rejected"] += 1
            self._invalid_reasons[reason] += 1
        return False

    def receive_message(self, message, *, generation):
        """Bound ROS-like attributes before copying the image body."""
        try:
            return self.receive(width=message.width, height=message.height, encoding=message.encoding,
                step=message.step, data=message.data, stamp_sec=message.header.stamp.sec,
                stamp_nanosec=message.header.stamp.nanosec, frame_id=message.header.frame_id,
                is_bigendian=message.is_bigendian, generation=generation)
        except (AttributeError, TypeError, OverflowError):
            return self._reject("invalid_message")

    def receive(self, *, width, height, encoding, step, data, stamp_sec, stamp_nanosec,
                frame_id, generation, is_bigendian=0):
        """Validate one raw sample. Duplicate stamps never refresh receipt age."""
        with self.lock:
            try:
                _require(not self._closed, "source_closed")
                _require(type(generation) is str and generation == self._generation, "old_generation")
                _require(self._publisher_count == 1, "publisher_not_unique")
                _require(not self._clock_invalid, "clock_invalid")
                _require(encoding == "rgb8" and type(encoding) is str, "invalid_encoding")
                _require(type(width) is int and 1 <= width <= MAX_WIDTH
                         and type(height) is int and 1 <= height <= MAX_HEIGHT, "invalid_dimensions")
                _require(type(step) is int and width * 3 <= step <= MAX_STEP, "invalid_stride")
                _require(type(is_bigendian) is int and is_bigendian in (0, 1), "invalid_endian")
                _require(type(stamp_sec) is int and 0 <= stamp_sec <= MAX_ROS_SECONDS
                         and type(stamp_nanosec) is int and 0 <= stamp_nanosec < 1_000_000_000, "invalid_stamp")
                _require(_name(frame_id) and (self._frame_id is None or self._frame_id == frame_id), "frame_changed")
                _require(hasattr(data, "__len__") and len(data) == step * height
                         and len(data) <= MAX_RAW_BYTES, "invalid_data_length")
                stamp = stamp_sec * 1_000_000_000 + stamp_nanosec
                if self._last_stamp_ns is not None and stamp < self._last_stamp_ns:
                    self._invalidate_clock("image_stamp_backwards")
                    raise ValueError("clock_invalid")
                _require(stamp != self._last_stamp_ns, "duplicate_stamp")
                now = self.clock()
                _require(_finite(now, 0, 1e12), "invalid_receipt_clock")
                if self._last_receipt is not None and now < self._last_receipt:
                    self._invalidate_clock("receipt_clock_backwards")
                    raise ValueError("clock_invalid")
                # ROS uint8[] is array('B'). bytes()/bytearray/memoryview are
                # also bounded here; arbitrary iterables cannot request a copy.
                _require(type(data) in (bytes, bytearray, memoryview)
                         or type(data) is array and data.typecode == "B", "invalid_data_type")
                _require(not isinstance(data, memoryview) or data.itemsize == 1
                         and data.nbytes == step * height and data.contiguous, "invalid_data_type")
                raw = bytes(data)
                _require(len(raw) == step * height, "invalid_data_length")
            except (ValueError, TypeError, OverflowError) as error:
                reason = str(error) if isinstance(error, ValueError) else "invalid_message"
                return self._reject(reason if reason in _REASONS else "invalid_message")
            if self._pending is not None:
                self._counts["raw_overwrites"] += 1
                old = self._history.get(self._pending["sequence"])
                if old is not None:
                    old["processing"] = "overwritten_before_encoding"
            self._sequence += 1
            sequence = self._sequence
            metadata = dict(source_id=self.source_id, generation=self._generation, sequence=sequence,
                sequence_semantics="receiver_accepted_sample", source_stamp_ns=str(stamp),
                source_stamp_sec=stamp / 1e9, source_clock_domain=self.clock_domain,
                stamp_sec=stamp / 1e9, clock_domain=self.clock_domain,
                received_monotonic_sec=now, frame_id=frame_id, width=width, height=height,
                encoding="rgb8", step=step, is_bigendian=is_bigendian, raw_bytes=len(raw),
                processing="pending", calibration=None, transform_to_world=None)
            self._history[sequence] = metadata
            while len(self._history) > MAX_HISTORY:
                self._history.popitem(last=False)
                self._counts["metadata_evictions"] += 1
            self._pending = dict(sequence=sequence, generation=self._generation, width=width,
                                 height=height, step=step, raw=raw, metadata=deepcopy(metadata))
            self._latest_received = metadata
            self._last_stamp_ns, self._last_receipt, self._frame_id = stamp, now, frame_id
            self._counts["received"] += 1
            self._wake.set()
            return True

    def process_once(self):
        """Encode at most once per 0.2s. The worker owns this outside ROS locks."""
        with self.lock:
            now = self.clock()
            if (self._closed or self._encoding or self._pending is None or self._clock_invalid
                    or self._publisher_count != 1 or now < self._next_encode_at):
                return False
            sample, self._pending = self._pending, None
            if sample["sequence"] in self._history:
                self._history[sample["sequence"]]["processing"] = "encoding"
            self._encoding, self._inflight_bytes = True, len(sample["raw"])
            self._next_encode_at = now + .2
        jpeg, error = None, None
        try:
            jpeg = self.encoder(sample["width"], sample["height"], sample["step"], sample["raw"])
            _require(type(jpeg) is bytes and 4 <= len(jpeg) <= MAX_JPEG_BYTES
                     and jpeg.startswith(b"\xff\xd8") and jpeg.endswith(b"\xff\xd9"), "invalid_jpeg_output")
        except Exception:
            error = "jpeg_encode_failed"
        with self.lock:
            self._encoding, self._inflight_bytes = False, 0
            if self._closed or self._clock_invalid or sample["generation"] != self._generation:
                self._counts["obsolete_encodes"] += 1
                return True
            metadata = self._history.get(sample["sequence"])
            if error:
                self._counts["codec_errors"] += 1
                self._last_error = error
                if metadata is not None:
                    metadata["processing"] = "codec_failed"
                return True
            encoded_at = self.clock()
            if not _finite(encoded_at, 0, 1e12) or encoded_at < sample["metadata"]["received_monotonic_sec"]:
                self._invalidate_clock("receipt_clock_backwards")
                self._counts["obsolete_encodes"] += 1
                return True
            descriptor = dict(sample["metadata"], processing="encoded", encoded_monotonic_sec=encoded_at,
                              jpeg_sha256=hashlib.sha256(jpeg).hexdigest(), jpeg_bytes=len(jpeg))
            while self._frames and (len(self._frames) >= MAX_FRAMES or self._cache_bytes + len(jpeg) > MAX_CACHE_BYTES):
                _, (_, previous) = self._frames.popitem(last=False)
                self._cache_bytes -= len(previous)
                self._counts["frame_evictions"] += 1
            self._frames[sample["sequence"]] = (descriptor, jpeg)
            self._cache_bytes += len(jpeg)
            self._latest_encoded = descriptor
            if metadata is not None:
                metadata.update(descriptor)
            self._last_error = None
            self._counts["encoded"] += 1
            return True

    def start(self):
        with self.lock:
            _require(not self._closed, "source_closed")
            if self._worker is None:
                self._worker = threading.Thread(target=self._run, name="image-jpeg-"+self.source_id, daemon=True)
                self._worker.start()

    def _run(self):
        while not self._stop.is_set():
            self._wake.clear()
            if self.process_once():
                continue
            with self.lock:
                delay = min(.2, max(.01, self._next_encode_at - self.clock())) if self._pending is not None else .2
            self._wake.wait(delay)

    def _metadata(self, metadata, now):
        if metadata is None:
            return None
        result = deepcopy(metadata)
        result["receive_age_sec"] = max(0., now - metadata["received_monotonic_sec"])
        result["age_sec"] = result["receive_age_sec"]
        result["stale_after_sec"] = self.stale_after_sec
        result["encoded_age_sec"] = (None if "encoded_monotonic_sec" not in metadata
                                      else max(0., now - metadata["encoded_monotonic_sec"]))
        result["image_available"] = metadata["generation"] == self._generation and metadata["sequence"] in self._frames
        return result

    def frame(self, generation, sequence):
        with self.lock:
            if (type(generation) is not str or type(sequence) is not int or generation != self._generation
                    or self._closed or self._clock_invalid or self._publisher_count != 1):
                return None
            value = self._frames.get(sequence)
            return None if value is None else (self._metadata(value[0], self.clock()), value[1])

    def snapshot(self):
        with self.lock:
            now = self.clock()
            latest = self._metadata(self._latest_encoded, now)
            received = self._metadata(self._latest_received, now)
            if self._closed: status = "closed"
            elif self._clock_invalid: status = "invalid"
            elif self._publisher_count > 1: status = "ambiguous"
            elif not self._publisher_count: status = "disconnected" if self._ever_had_publisher else "waiting_source"
            elif received is None: status = "waiting_frame"
            elif received["receive_age_sec"] > self.stale_after_sec: status = "stale"
            elif self._last_error: status = "error"
            elif latest is None: status = "encoding"
            elif latest["receive_age_sec"] > self.frame_stale_after_sec: status = "stale"
            else: status = "ready"
            return dict(schema_version=1, kind="rgb8-image-source-v1", source_id=self.source_id, topic=self.topic,
                generation=self._generation, status=status, latest=latest, latest_received=received,
                stamp_sec=None if received is None else received["stamp_sec"],
                age_sec=None if received is None else received["age_sec"],
                sequence=None if received is None else received["sequence"],
                age_clock_domain="console_receiver_elapsed_monotonic",
                history=[self._metadata(metadata, now) for metadata in self._history.values()],
                publisher_count=self._publisher_count, publisher_endpoint_sha256=self._publishers[0] if self._publisher_count == 1 else None,
                publisher_count_is_lower_bound=self._publisher_count == 9,
                error=self._last_error, clock_invalid=self._clock_invalid, observed_clock_ns=None if self._clock_ns is None else str(self._clock_ns),
                stale_after_sec=self.stale_after_sec, frame_stale_after_sec=self.frame_stale_after_sec,
                clock_domain=self.clock_domain,
                counters={name:self._counts[name] for name in _COUNTERS}, invalid_reasons=dict(self._invalid_reasons),
                bounds=dict(max_width=MAX_WIDTH,max_height=MAX_HEIGHT,max_step=MAX_STEP,max_raw_bytes=MAX_RAW_BYTES,
                    max_jpeg_bytes=MAX_JPEG_BYTES,max_encode_hz=5,max_frames=MAX_FRAMES,max_cache_bytes=MAX_CACHE_BYTES,
                    max_metadata=MAX_HISTORY),
                memory=dict(pending_raw_bytes=0 if self._pending is None else len(self._pending["raw"]),
                    inflight_raw_bytes=self._inflight_bytes,encoded_bytes=self._cache_bytes,encoded_frames=len(self._frames),metadata=len(self._history)),
                worker_alive=bool(self._worker and self._worker.is_alive()), read_only=True,
                provenance="Publisher discovery association, not per-message sender identity proof",
                timing="Source acquisition stamp and receiver monotonic age are separate; latency and native step alignment are not measured",
                loss_semantics="Overwrite/eviction/rejection counts are local; DDS delivery loss is not measured")

    def close(self):
        with self.lock:
            if not self._closed:
                self._closed = True
                self._reset("closed", clear_clock=False)
            self._stop.set()
            self._wake.set()
            worker = self._worker
        if worker is not None and worker is not threading.current_thread():
            worker.join(timeout=2)
        return worker is None or not worker.is_alive()


_REASONS = {"source_closed", "old_generation", "publisher_not_unique", "clock_invalid", "invalid_encoding",
    "invalid_dimensions", "invalid_stride", "invalid_endian", "invalid_stamp", "frame_changed", "invalid_data_length",
    "duplicate_stamp", "invalid_receipt_clock", "invalid_data_type", "invalid_message", "invalid_clock"}
_COUNTERS = ("received","rejected","encoded","raw_overwrites","metadata_evictions","frame_evictions",
    "codec_errors","obsolete_encodes","generation_changes","publisher_changes","discovery_errors","clock_reversals")
