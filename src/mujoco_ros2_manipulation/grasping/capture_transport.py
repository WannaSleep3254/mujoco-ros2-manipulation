"""Bounded, read-only recording observations from one owned grasp worker.

JPEG encoding and sidecar writes run outside the ROS executor. These files are
recording inputs only. A failure here cannot authorize or cancel robot actions.
"""
from collections import OrderedDict, deque
from copy import deepcopy
import hashlib
import json
import math
import os
from pathlib import Path
import re
import threading
import time

from ..operator_console.synchronized_recording import _directory, _read
from ..operator_console.image_source import ImageSource, MAX_CACHE_BYTES, MAX_FRAMES, MAX_JPEG_BYTES
from ..verification.reports import atomic_json

MAX_JOURNAL = 128
MAX_SIDECAR_BYTES = 1024 * 1024
MAX_EVENT_DETAILS_BYTES = 4096
_SHA = re.compile(r'[0-9a-f]{64}')
_ID = re.compile(r'[0-9a-f]{32}')
OWNER_FIELDS = {'job_id', 'generation', 'binding_id', 'binding', 'model_sha256',
                'execution_manifest_sha256', 'domain', 'source_registration_epoch'}


def _read_confined(path, maximum):
    directory = _directory(Path(path).parent)
    try:
        return _read(directory, Path(path).name, maximum)
    finally:
        os.close(directory)


def checked_registration(path, output, run_id, domain, manifest_sha256):
    """Read only the fixed registration beside this task's output directory."""
    if Path(path) != Path(output).parent / 'capture-registration.json':
        raise ValueError('Recording registration is outside its owned job')
    owner = json.loads(_read_confined(path, 8192))
    if (type(owner) is not dict or set(owner) != OWNER_FIELDS
            or owner['job_id'] != run_id or owner['domain'] != domain
            or owner['execution_manifest_sha256'] != manifest_sha256
            or type(owner['generation']) is not int or owner['generation'] < 1
            or type(owner['binding']) is not dict
            or any(type(owner[name]) is not str or not _SHA.fullmatch(owner[name])
                   for name in ('binding_id', 'model_sha256', 'execution_manifest_sha256'))
            or type(owner['source_registration_epoch']) is not str
            or not _ID.fullmatch(owner['source_registration_epoch'])):
        raise ValueError('Recording registration does not match this task')
    return owner


def native_association(value, emitted_ns, *, native_alive):
    """Associate only a progressing, fresh original native observation."""
    result = dict(available=False, reason='pre_native', stamp_ns=None, sequence=None,
                  capture_monotonic_ns=None)
    native = value.get('native_observation')
    if not native:
        return result
    result['reason'] = 'native_not_live'
    if not native_alive:
        return result
    result['reason'] = 'native_cleanup_or_hold'
    guard = value.get('native_guard') or {}
    task = value.get('task') or {}
    if (native.get('phase') == 8 or guard.get('latched') is not False
            or guard.get('phase') != 'ready' or task.get('cleanup', {}).get('status') != 'not_started'):
        return result
    result['reason'] = 'native_not_fresh_and_progressing'
    observed = value.get('observed_monotonic_ns')
    progress = value.get('clock_progress_observed_monotonic_ns')
    capture = native.get('capture_monotonic_ns')
    if (not value.get('native_clock_has_advanced') or type(capture) is not str
            or not re.fullmatch(r'[1-9][0-9]{0,18}', capture)
            or type(observed) is not int or type(progress) is not int
            or any(not 0 <= emitted_ns - stamp <= 500_000_000
                   for stamp in (int(capture), observed, progress))):
        return result
    return dict(available=True, reason='fresh_progressing_native_observation',
                stamp_ns=native['stamp_ns'], sequence=native['sequence'],
                capture_monotonic_ns=capture)


class CapturePublisher:
    """One bounded source-event journal and latest-only JPEG transport."""
    def __init__(self, directory, owner, *, image_topic=None, image_clock_domain='unknown'):
        self.directory = Path(directory) / 'capture'
        self.directory.mkdir(mode=0o700)
        self.owner = deepcopy(owner)
        self.source = ImageSource(source_id='grasp_rgb', topic=image_topic or '/camera/image',
                                  clock_domain=image_clock_domain)
        self.image_enabled = image_topic is not None
        self.sensor = self.timer = self.node = None
        self.events = deque(maxlen=MAX_JOURNAL)
        self.events_dropped = self.event_sequence = 0
        self.fault = None
        self.lock = threading.RLock()
        self.wake, self.stop = threading.Event(), threading.Event()
        self.final_image = None
        self.final_frame = None
        self.blobs = deque(maxlen=2)
        self.closed = False
        self.thread = threading.Thread(target=self._run, name='grasp-recording-export', daemon=True)
        self.thread.start()

    def fail(self, error):
        with self.lock:
            if self.fault is None:
                self.fault = (type(error).__name__ + ': ' + str(error))[:512]

    def attach(self, node):
        """Use the task's existing context, only after its unused-domain gate."""
        if not self.image_enabled:
            return
        try:
            from ..operator_console.sensor_ros import attach
            self.node = node
            self.sensor = attach(node, self.source)
            def discover():
                try:
                    self.sensor.discover()
                except Exception as error:
                    self.fail(error)
            self.timer = node.create_timer(.25, discover)
            discover()
        except Exception as error:
            self.fail(error)

    def event(self, value, *, emitted_ns, native_alive):
        if value.get('kind') != 'task':
            return
        try:
            model = value.get('model_sha256')
            if model is not None and model != self.owner['model_sha256']:
                raise ValueError('Recording model differs from native model')
            task = value['task']
            details = {name: deepcopy(task[name]) for name in ('error', 'failure_stage', 'physical_grasp',
                'planning_attachment', 'action_terminal', 'current_hold_verification') if name in task}
            if len(json.dumps(details, allow_nan=False).encode()) > MAX_EVENT_DETAILS_BYTES:
                raise ValueError('Recording event details exceed their bound')
            with self.lock:
                self.event_sequence += 1
                if len(self.events) == MAX_JOURNAL:
                    self.events_dropped += 1
                self.events.append(dict(event_id=self.owner['job_id'] + ':' + str(self.event_sequence),
                    event_sequence=self.event_sequence, emitted_monotonic_ns=str(emitted_ns),
                    stage=task['stage'], status=task['status'], details=details,
                    native_association=native_association(value, emitted_ns, native_alive=native_alive)))
            self.wake.set()
        except Exception as error:
            self.fail(error)

    def stop_sensor(self):
        """Freeze the last image metadata before source close invalidates caches."""
        try:
            snapshot = self.source.snapshot()
            latest = snapshot.get('latest')
            frame = None if latest is None else self.source.frame(latest['generation'], latest['sequence'])
            with self.lock:
                self.final_image, self.final_frame = snapshot, frame
            if self.timer is not None:
                self.node.destroy_timer(self.timer)
                self.timer = None
            if self.sensor is not None:
                if not self.sensor.close():
                    raise RuntimeError('Recording image subscriptions did not close cleanly')
                self.sensor = None
            else:
                self.source.close()
        except Exception as error:
            self.fail(error)

    def publish_once(self):
        with self.lock:
            snapshot = deepcopy(self.final_image)
            frame = self.final_frame
            events, dropped, fault = list(self.events), self.events_dropped, self.fault
        if snapshot is None:
            snapshot = self.source.snapshot()
            latest = snapshot.get('latest')
            frame = None if latest is None else self.source.frame(latest['generation'], latest['sequence'])
        if frame is None:
            snapshot['latest'] = None
        snapshot.update(age_clock_domain='grasp_worker_receiver_elapsed_monotonic',
                        configured=self.image_enabled, source_registration_epoch=self.owner['source_registration_epoch'])
        if not self.image_enabled:
            snapshot['status'] = 'disabled'
        blob_name = None
        if frame is not None:
            metadata, jpeg = frame
            metadata = dict(metadata, topic=snapshot['topic'],
                            publisher_endpoint_sha256=snapshot['publisher_endpoint_sha256'])
            snapshot['latest'] = metadata
            digest = hashlib.sha256(jpeg).hexdigest()
            if digest != metadata['jpeg_sha256'] or len(jpeg) != metadata['jpeg_bytes']:
                raise ValueError('Encoded JPEG identity changed before export')
            blob_name = digest + '.jpg'
            path = self.directory / blob_name
            if blob_name not in self.blobs:
                # Two retained blobs plus one in publication bound the total
                # to three. An overwritten descriptor remains an explicit gap.
                temporary = self.directory / 'image-writing'
                with temporary.open('xb') as stream:
                    stream.write(jpeg)
                temporary.replace(path)
                old = self.blobs[0] if len(self.blobs) == self.blobs.maxlen else None
                self.blobs.append(blob_name)
                if old is not None and old != blob_name:
                    (self.directory / old).unlink(missing_ok=True)
        value = dict(schema_version=1, kind='grasp-recording-observations-v1', owner=self.owner,
            image=snapshot, image_blob=blob_name, events=events, events_dropped=dropped,
            fault=fault, closed=self.closed, published_monotonic_ns=str(time.monotonic_ns()))
        if len(json.dumps(value, indent=2, allow_nan=False).encode()) + 1 > MAX_SIDECAR_BYTES:
            raise ValueError('Recording observation sidecar exceeds its bound')
        atomic_json(self.directory / 'latest.json', value)

    def _run(self):
        while not self.stop.is_set():
            self.wake.clear()
            try:
                self.publish_once()
            except Exception as error:
                self.fail(error)
            self.wake.wait(.1)
        try:
            self.publish_once()
        except Exception as error:
            self.fail(error)

    def close(self):
        self.closed = True
        self.stop.set()
        self.wake.set()
        self.thread.join(timeout=2.)
        if self.thread.is_alive():
            self.fail(RuntimeError('Recording export did not stop within its bound'))
        return dict(clean=not self.thread.is_alive() and self.fault is None, fault=self.fault,
                    events=self.event_sequence, events_dropped=self.events_dropped)


class CaptureReceiver:
    """Read only the exact worker registration; retain bounded encoded frames."""
    def __init__(self, directory, owner, *, image_topic=None, image_clock_domain='unknown'):
        self.directory = Path(directory) / 'run/capture'
        self.owner = deepcopy(owner)
        self.topic, self.clock_domain = image_topic or '/camera/image', image_clock_domain
        self.enabled = image_topic is not None
        self.lock = threading.RLock()
        self.frames = OrderedDict()
        self.cache_bytes = 0
        self.events = deque(maxlen=MAX_JOURNAL)
        self.events_dropped = self.last_event = self.transport_gaps = 0
        self.fault = None
        self.finished = False
        self.state = ImageSource(source_id='grasp_rgb', topic=image_topic or '/camera/image',
                                 clock_domain=image_clock_domain).snapshot()
        self.state.update(configured=image_topic is not None,
            source_registration_epoch=owner['source_registration_epoch'],
            age_clock_domain='grasp_worker_receiver_elapsed_monotonic')
        if image_topic is None:
            self.state['status'] = 'disabled'
        self.digest = None
        self.closed = False

    def refresh(self, *, finished=False):
        """Contain all recording failures; the caller's native gates are separate."""
        with self.lock:
            self.finished = finished
            try:
                path = self.directory / 'latest.json'
                try:
                    raw = _read_confined(path, MAX_SIDECAR_BYTES)
                except FileNotFoundError:
                    if finished and self.enabled:
                        raise ValueError('Finished worker has no final recording observation journal')
                    return
                digest = hashlib.sha256(raw).hexdigest()
                if digest == self.digest:
                    if finished and self.enabled and not self.closed:
                        raise ValueError('Finished worker did not close its recording observation journal')
                    return
                value = json.loads(raw)
                if (type(value) is not dict or value.get('schema_version') != 1
                        or value.get('kind') != 'grasp-recording-observations-v1'
                        or value.get('owner') != self.owner):
                    raise ValueError('Recording transport owner differs from this exact job')
                state, events = value.get('image'), value.get('events')
                if (type(events) is not list or len(events) > MAX_JOURNAL
                        or type(value.get('events_dropped')) is not int or value['events_dropped'] < 0):
                    raise ValueError('Invalid recording source event journal')
                # Task diagnostics are independently validated. A corrupt JPEG
                # must not discard a same-owner terminal event from this batch.
                self._accept_events(events, value['events_dropped'])
                if (type(state) is not dict or state.get('kind') != 'rgb8-image-source-v1'
                        or state.get('source_registration_epoch') != self.owner['source_registration_epoch']
                        or state.get('source_id') != 'grasp_rgb' or state.get('topic') != self.topic
                        or state.get('clock_domain') != self.clock_domain
                        or type(state.get('history')) is not list or len(state['history']) > 256
                        or type(state.get('publisher_count')) is not int or not 0 <= state['publisher_count'] <= 9
                        or type(state.get('clock_invalid')) is not bool):
                    raise ValueError('Invalid recording image or event journal')
                for metadata in [state.get('latest'), state.get('latest_received'), *state['history']]:
                    if metadata is not None:
                        self._check_metadata(metadata)
                self._accept_image(state, value.get('image_blob'))
                self.state, self.digest, self.closed = deepcopy(state), digest, value.get('closed') is True
                if finished and self.enabled and not self.closed:
                    raise ValueError('Finished worker did not close its recording observation journal')
                if value.get('fault'):
                    raise ValueError(str(value['fault'])[:512])
            except Exception as error:
                self.fault = self.fault or (type(error).__name__ + ': ' + str(error))[:512]

    @staticmethod
    def _check_metadata(metadata):
        now = time.monotonic()
        if (type(metadata) is not dict or metadata.get('source_id') != 'grasp_rgb'
                or type(metadata.get('generation')) is not str or not _ID.fullmatch(metadata['generation'])
                or type(metadata.get('sequence')) is not int or metadata['sequence'] < 1
                or type(metadata.get('source_stamp_ns')) is not str
                or not re.fullmatch(r'0|[1-9][0-9]{0,18}', metadata['source_stamp_ns'])
                or type(metadata.get('received_monotonic_sec')) not in (int, float)
                or not math.isfinite(metadata['received_monotonic_sec'])
                or not 0 <= metadata['received_monotonic_sec'] <= now + .1):
            raise ValueError('Invalid original image metadata or receipt clock')
        encoded = metadata.get('encoded_monotonic_sec')
        if encoded is not None and (type(encoded) not in (int, float) or not math.isfinite(encoded)
                or not metadata['received_monotonic_sec'] <= encoded <= now + .1):
            raise ValueError('Invalid original image encoding clock')

    def _accept_image(self, state, blob):
        generation = state.get('generation')
        if type(generation) is not str or not _ID.fullmatch(generation):
            raise ValueError('Invalid image source generation')
        if generation != self.state.get('generation'):
            self.frames.clear()
            self.cache_bytes = 0
        latest = state.get('latest')
        if latest is None:
            if blob is not None:
                raise ValueError('Image blob has no encoded descriptor')
            return
        if (type(latest) is not dict or latest.get('generation') != generation
                or type(latest.get('sequence')) is not int or latest['sequence'] < 1
                or type(latest.get('jpeg_sha256')) is not str or not _SHA.fullmatch(latest['jpeg_sha256'])
                or blob != latest['jpeg_sha256'] + '.jpg'):
            raise ValueError('Invalid encoded image descriptor')
        key = (generation, latest['sequence'])
        if key in self.frames:
            if self.frames[key][0].get('jpeg_sha256') != latest['jpeg_sha256']:
                raise ValueError('Same image sequence changed its JPEG identity')
            return
        try:
            jpeg = _read_confined(self.directory / blob, MAX_JPEG_BYTES)
        except FileNotFoundError:
            # A descriptor can race the latest-only writer's three-blob bound.
            # Preserve a visible gap without reading an arbitrary fallback file.
            self.transport_gaps += 1
            return
        if (hashlib.sha256(jpeg).hexdigest() != latest['jpeg_sha256'] or len(jpeg) != latest.get('jpeg_bytes')
                or not jpeg.startswith(b'\xff\xd8') or not jpeg.endswith(b'\xff\xd9')):
            raise ValueError('Recording JPEG bytes differ from their descriptor')
        while self.frames and (len(self.frames) >= MAX_FRAMES or self.cache_bytes + len(jpeg) > MAX_CACHE_BYTES):
            _, (_, old) = self.frames.popitem(last=False)
            self.cache_bytes -= len(old)
        self.frames[key] = (deepcopy(latest), jpeg)
        self.cache_bytes += len(jpeg)

    def _accept_events(self, events, dropped):
        now = str(time.monotonic_ns())
        retained = {item['event_sequence']: item for item in self.events}
        previous = 0
        for event in events:
            if (type(event) is not dict or type(event.get('event_sequence')) is not int
                    or event['event_sequence'] < 1
                    or event.get('event_id') != self.owner['job_id'] + ':' + str(event['event_sequence'])
                    or type(event.get('emitted_monotonic_ns')) is not str
                    or not re.fullmatch(r'[1-9][0-9]{0,18}', event['emitted_monotonic_ns'])
                    or int(event['emitted_monotonic_ns']) > int(now)
                    or type(event.get('native_association')) is not dict
                    or event['event_sequence'] <= previous):
                raise ValueError('Invalid source-emitted recording event')
            association = event['native_association']
            if (type(event.get('stage')) is not str or not 0 < len(event['stage']) <= 128
                    or event.get('status') not in ('running', 'succeeded', 'failed', 'cancelled')
                    or type(event.get('details')) is not dict
                    or len(json.dumps(event['details'], allow_nan=False).encode()) > MAX_EVENT_DETAILS_BYTES
                    or type(association.get('available')) is not bool
                    or type(association.get('reason')) is not str or len(association['reason']) > 128):
                raise ValueError('Invalid source event details or native association')
            stamps = ('stamp_ns', 'capture_monotonic_ns')
            if association['available']:
                if (any(type(association.get(name)) is not str
                        or not re.fullmatch(r'0|[1-9][0-9]{0,18}', association[name]) for name in stamps)
                        or type(association.get('sequence')) is not int or association['sequence'] < 1
                        or int(association['capture_monotonic_ns']) > int(event['emitted_monotonic_ns'])):
                    raise ValueError('Invalid associated native source time')
            elif any(association.get(name) is not None for name in (*stamps, 'sequence')):
                raise ValueError('Unassociated event must retain null native timing')
            previous = event['event_sequence']
            if event['event_sequence'] <= self.last_event:
                old = deepcopy(retained.get(event['event_sequence']))
                if old is not None:
                    old.pop('parent_received_monotonic_ns', None)
                    if old != event:
                        raise ValueError('Source event changed within one event sequence')
                continue
            if self.last_event and event['event_sequence'] != self.last_event + 1:
                self.events_dropped += event['event_sequence'] - self.last_event - 1
            if len(self.events) == MAX_JOURNAL:
                self.events_dropped += 1
            self.events.append(dict(deepcopy(event), parent_received_monotonic_ns=now))
            self.last_event = event['event_sequence']
        self.events_dropped = max(self.events_dropped, dropped)

    def _metadata(self, metadata, now):
        result = deepcopy(metadata)
        receipt = metadata.get('received_monotonic_sec')
        if type(receipt) in (int, float):
            result['receive_age_sec'] = result['age_sec'] = max(0., now - receipt)
        encoded = metadata.get('encoded_monotonic_sec')
        result['encoded_age_sec'] = None if encoded is None else max(0., now - encoded)
        result['image_available'] = (metadata.get('generation'), metadata.get('sequence')) in self.frames
        return result

    def snapshot(self):
        with self.lock:
            now = time.monotonic()
            state = deepcopy(self.state)
            state['latest'] = None if state.get('latest') is None else self._metadata(state['latest'], now)
            state['latest_received'] = (None if state.get('latest_received') is None
                                        else self._metadata(state['latest_received'], now))
            state['history'] = [self._metadata(value, now) for value in state.get('history', [])]
            received = state['latest_received']
            state['age_sec'] = None if received is None else received['age_sec']
            if self.fault:
                state['status'], state['error'] = 'error', self.fault
            elif state.get('status') == 'ready' and received and received['age_sec'] > state['stale_after_sec']:
                state['status'] = 'stale'
            state.update(read_only=True, worker_finished=self.finished, transport_closed=self.closed,
                         transport_gaps=self.transport_gaps)
            return state

    def frame(self, generation, sequence):
        with self.lock:
            if (generation != self.state.get('generation') or self.fault or self.state.get('clock_invalid')
                    or self.state.get('publisher_count') != 1):
                return None
            value = self.frames.get((generation, sequence))
            return None if value is None else (self._metadata(value[0], time.monotonic()), value[1])
