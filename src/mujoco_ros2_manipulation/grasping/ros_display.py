"""Native 3.12 geometry and observed full-geometry poses for the ROS grasp view.

Geometry is compiled once by the isolated native exporter. Dynamic poses copy
complete transforms from checked native observations, including both arms,
finger slides and the free block. No Python MuJoCo import, FK, physics step,
command, attachment, or source-freshness decision belongs to this adapter.
"""
from array import array
from copy import deepcopy
import hashlib
import math
from pathlib import Path
import re
import subprocess
import sys
import tempfile
import threading

from ..model_build.integrity import file_sha256
from ..operator_console.display_contract import WORLD, VERSION, scene_digest, validate_manifest, validate_pose
from ..operator_console.scene_host import _canonical, _json, MAX_MANIFEST_BYTES
from ..operator_console.scene_view import SceneLimits, DEFAULT_VISIBLE_GROUPS
from .ros_model import ARTIFACT
from .ros_observation import GraspObservation, GraspObservationStore


MAX_EXPORT_JSON_BYTES = 128 * 1024 * 1024
_NATIVE = '3.12.0'
_SHA = re.compile(r'[0-9a-f]{64}')
_RUN = re.compile(r'[0-9a-f]{32}')


def _require(value, message):
    if not value:
        raise ValueError(message)


def _sha(raw):
    return hashlib.sha256(raw).hexdigest()


def _bounded_vector(value, count, label, *, maximum=1e9):
    _require(type(value) is list and len(value) == count
             and all(type(item) in (int, float) and math.isfinite(item) and abs(item) <= maximum for item in value),
             'Invalid native display ' + label)
    return list(value)


def _packed(values, kind):
    output = array(kind, values)
    _require(output.itemsize == 4, 'Expected four-byte display buffer components')
    if sys.byteorder != 'little':
        output.byteswap()
    return output.tobytes()


def _compiled_sha(path, expected_size, maximum):
    _require(type(expected_size) is int and path.is_file() and not path.is_symlink()
             and 0 < path.stat().st_size == expected_size <= maximum,
             'Native compiled model snapshot differs from its declared size')
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        while block := stream.read(1024 * 1024):
            digest.update(block)
    return digest.hexdigest()


def _native_export(target, limits):
    binary = target.root / 'runtime/native/grasp_scene_export'
    source = target.root / 'native/grasp_scene_export.cpp'
    _require(binary.is_file(), 'Build the isolated native ROS grasp scene exporter before opening this view')
    source_sha = file_sha256(source)
    binary_sha = file_sha256(binary)
    identity = subprocess.run([str(binary), '--identity'], check=True, capture_output=True, timeout=5.)
    _require(len(identity.stdout) <= 4096, 'Native scene exporter identity exceeds its byte limit')
    identity = _json(identity.stdout)
    _require(identity == dict(schema_version=1, native_version=_NATIVE, source_sha256=source_sha),
             'Native scene exporter is stale or uses another engine')
    model_sha = file_sha256(target.model_path)
    with tempfile.TemporaryDirectory(prefix='ros-grasp-static-scene-') as temporary:
        output = subprocess.run([str(binary), str(target.model_path), model_sha, temporary],
                                check=True, capture_output=True, timeout=30.)
        _require(0 < len(output.stdout) <= MAX_EXPORT_JSON_BYTES, 'Native scene export exceeds its byte limit')
        value = _json(output.stdout)
        _require(type(value) is dict and value.get('schema_version') == 1
                 and value.get('kind') == 'native-ros-grasp-static-scene-v1'
                 and value.get('native_version') == _NATIVE and value.get('source_sha256') == source_sha
                 and value.get('model_sha256') == model_sha
                 and type(value.get('physics_steps')) is int and value['physics_steps'] == 0,
                 'Native scene export identity or static-only scope differs')
        expected = target.manifest['qpos_state_contract']
        _require(value.get('dimensions') == {key: expected[key] for key in ('nq', 'nv', 'nu', 'njnt')}
                 and value.get('qpos_layout') == expected['layout'],
                 'Native scene export has another full-state layout')
        compiled_sha = _compiled_sha(Path(temporary) / 'compiled.mjb', value.get('compiled_bytes'), limits.max_compiled_bytes)
    _require(file_sha256(binary) == binary_sha and file_sha256(source) == source_sha
             and file_sha256(target.model_path) == model_sha, 'Native display input changed during compilation')
    target.validate()
    return value, dict(model_sha256=model_sha, compiled_sha256=compiled_sha,
                       exporter_source_sha256=source_sha, exporter_binary_sha256=binary_sha)


def _mesh_assets(value, limits):
    meshes = value.get('meshes')
    _require(type(meshes) is list and len(meshes) <= limits.max_meshes, 'Invalid native display mesh catalog')
    descriptors, assets, mesh_map = {}, {}, {}
    total_vertices = total_indices = total_bytes = 0
    for index, mesh in enumerate(meshes):
        _require(type(mesh) is dict and set(mesh) == {'id', 'vertices', 'faces'}
                 and type(mesh['id']) is int and mesh['id'] == index,
                 'Native mesh IDs must retain their compiled order')
        vertices, faces = mesh['vertices'], mesh['faces']
        _require(type(vertices) is list and len(vertices) >= 3 and type(faces) is list and len(faces) >= 1,
                 'Native display mesh has no triangle geometry')
        total_vertices += len(vertices)
        total_indices += len(faces) * 3
        _require(total_vertices <= limits.max_vertices and total_indices <= limits.max_indices,
                 'Native display mesh arrays exceed their resource limit')
        coordinates = []
        for vertex in vertices:
            coordinates.extend(_bounded_vector(vertex, 3, 'compiled vertex'))
        indices = []
        for face in faces:
            _require(type(face) is list and len(face) == 3
                     and all(type(item) is int and 0 <= item < len(vertices) for item in face),
                     'Native display face has an invalid vertex index')
            indices.extend(face)
        positions = _packed(coordinates, 'f')
        triangles = _packed(indices, 'I')
        blob = positions + triangles
        digest = _sha(blob)
        descriptor = dict(sha256=digest, byte_length=len(blob), media_type='application/octet-stream', layout=dict(
            positions=dict(byte_offset=0, byte_length=len(positions), count=len(vertices), item_size=3, component_type='float32'),
            indices=dict(byte_offset=len(positions), byte_length=len(triangles), count=len(indices), item_size=1, component_type='uint32')))
        if digest not in assets:
            total_bytes += len(blob)
            _require(total_bytes <= limits.max_total_asset_bytes, 'Native display assets exceed their byte limit')
            descriptors[digest], assets[digest] = descriptor, blob
        else:
            _require(descriptors[digest] == descriptor, 'A native mesh digest has conflicting array layouts')
        mesh_map[index] = digest
    return [descriptors[key] for key in sorted(descriptors)], mesh_map, assets


class RosGraspDisplay:
    """Immutable native catalog; dynamic poses require an explicit run identity.

    An owner's GraspObservationStore remains responsible for age and guard
    eligibility. Repeated reads return a cached pose without changing source
    timestamps. A different execution requires a new display/run binding.
    """
    def __init__(self, target, *, run_id=None, limits=None):
        _require(target.description().get('artifact_kind') == ARTIFACT,
                 'Native display requires an explicit checked ROS grasp target')
        target.validate()
        self._limits = SceneLimits() if limits is None else limits
        _require(isinstance(self._limits, SceneLimits), 'Expected explicit display resource limits')
        if run_id is not None:
            _require(type(run_id) is str and _RUN.fullmatch(run_id), 'Invalid native display run identity')
        self._run_id = run_id
        self._lock = threading.RLock()
        self._store = None
        self._last_sequence = self._last_digest = self._last_pose = None
        self._description = target.description()
        self._plugin_sha256 = file_sha256(target.root / 'native/grasp_ros2_plugin/src/observation.cpp')
        raw, proof = _native_export(target, self._limits)
        descriptors, mesh_map, self._assets = _mesh_assets(raw, self._limits)
        qpos = _bounded_vector(raw.get('initial_qpos'), 21, 'initial static qpos', maximum=1e6)
        geoms = raw.get('geoms')
        _require(type(geoms) is list and 1 <= len(geoms) <= min(256, self._limits.max_geoms),
                 'Invalid native display geometry catalog')
        geometry, static_pose = [], []
        body_geoms = {}
        for index, geom in enumerate(geoms):
            _require(type(geom) is dict and type(geom.get('id')) is int and geom['id'] == index
                     and type(geom.get('name')) is str and len(geom['name']) <= 128,
                     'Native geometry IDs/names must retain the compiled catalog')
            group, kind = geom.get('group'), geom.get('type')
            _require(type(group) is int and 0 <= group <= 5 and kind in ('mesh', 'box', 'sphere', 'capsule', 'cylinder', 'plane'),
                     'Unsupported native display geometry')
            mesh_id = geom.get('mesh_id')
            _require(type(mesh_id) is int and ((kind == 'mesh' and mesh_id in mesh_map) or (kind != 'mesh' and mesh_id == -1)),
                     'Native geometry references another mesh catalog')
            geometry.append(dict(id=index, name=geom['name'], type=kind, group=group,
                default_visible=group in DEFAULT_VISIBLE_GROUPS, size=_bounded_vector(geom.get('size'), 3, 'geometry size'),
                rgba=_bounded_vector(geom.get('rgba'), 4, 'geometry RGBA', maximum=1),
                asset_sha256=mesh_map[mesh_id] if kind == 'mesh' else None))
            static_pose.append(dict(id=index, position=_bounded_vector(geom.get('initial_position'), 3, 'initial geometry position'),
                rotation=_bounded_vector(geom.get('initial_rotation'), 9, 'initial geometry rotation', maximum=1.000001)))
            body_id = geom.get('body_id')
            _require(type(body_id) is int and body_id >= 0, 'Invalid native geometry body ID')
            body_geoms.setdefault(body_id, []).append(index)
        layout = {row['name']: row['qpos_address'] for row in raw['qpos_layout']}
        home = {name: qpos[layout[name]] for name in target.joints}
        _require(all(abs(home[name] - value) <= 1e-12 for name, value in zip(target.joints, target.home)),
                 'Native display initial arm state differs from the checked target')
        source_sha = _sha(_canonical(dict(model_sha256=proof['model_sha256'],
            execution_manifest_sha256=target.manifest_sha256, exporter_source_sha256=proof['exporter_source_sha256'],
            adapter_source_sha256=file_sha256(Path(__file__)), compiled_sha256=proof['compiled_sha256'])))
        manifest = dict(schema_version=2, export_version=VERSION, model_sha256=proof['model_sha256'],
            provenance=dict(adapter='native-ros-grasp', source_sha256=source_sha,
                            engine=dict(name='mujoco', version=_NATIVE, compiled_sha256=proof['compiled_sha256'])),
            world_frame=deepcopy(WORLD), units=dict(length='m', angle='rad', up_axis='Z', rotation='row-major'),
            joint_names=list(target.joints), home_rad=home, default_visible_groups=list(DEFAULT_VISIBLE_GROUPS),
            geoms=geometry, assets=descriptors)
        manifest['scene_sha256'] = scene_digest(manifest)
        self._manifest = validate_manifest(manifest)
        self._raw = _canonical(self._manifest) + b'\n'
        _require(len(self._raw) <= MAX_MANIFEST_BYTES, 'Native display manifest exceeds its byte limit')
        self._static_preview = validate_pose(dict(schema_version=1, model_sha256=manifest['model_sha256'],
                                                  scene_sha256=manifest['scene_sha256'], geoms=static_pose), manifest)
        self._geom_names = tuple(geom['name'] for geom in geometry)
        bodies = raw.get('bodies')
        _require(type(bodies) is list and 1 <= len(bodies) <= 512, 'Invalid native display body hierarchy')
        for index, body in enumerate(bodies):
            _require(type(body) is dict and body.get('id') == index and type(body.get('id')) is int
                     and type(body.get('parent_id')) is int and 0 <= body['parent_id'] <= index
                     and (index == 0 or body['parent_id'] < index)
                     and type(body.get('name')) is str and len(body['name']) <= 128,
                     'Native body hierarchy is not a bounded compiled tree')
        _require(all(body < len(bodies) for body in body_geoms), 'Native geometry references a missing body')
        self._bodies = [dict(body, geom_ids=body_geoms.get(body['id'], [])) for body in bodies]
        self._metadata = dict(proof, native_version=_NATIVE, physics_steps_during_export=0,
            geoms=len(geometry), meshes=len(raw['meshes']), assets=len(self._assets),
            asset_bytes=sum(map(len, self._assets.values())),
            geometry='Native compiled mesh vertices and triangle faces; compiler mesh offsets are not applied a second time',
            dynamic_pose='Complete geometry world transforms from checked native observations; no FK reconstruction',
            collision_scope='Visual triangle meshes are displayed; native collision uses their compiled convex representation',
            freshness='Owner retains observation receipt/clock ages and guard eligibility',
            state_contract=deepcopy(target.description()['qpos_state_contract']))

    @property
    def manifest(self):
        return deepcopy(self._manifest)

    @property
    def manifest_raw(self):
        return self._raw

    @property
    def assets(self):
        return dict(self._assets)

    @property
    def geom_names(self):
        return self._geom_names

    @property
    def body_hierarchy(self):
        return deepcopy(self._bodies)

    @property
    def static_preview_pose(self):
        """Declared static preview only; this is never an observed live frame."""
        return deepcopy(self._static_preview)

    def metadata(self):
        return deepcopy(self._metadata)

    def asset(self, digest):
        _require(type(digest) is str and _SHA.fullmatch(digest), 'Invalid native display asset identity')
        if digest not in self._assets:
            raise KeyError('Unknown native display asset')
        return self._assets[digest]

    def frame_from_observation(self, observation, *, expected_run_id=None):
        """Copy checked native transforms; return the existing viewer pose shape."""
        _require(isinstance(observation, GraspObservation), 'Expected a checked native GraspObservation')
        with self._lock:
            run_id = self._run_id if expected_run_id is None else expected_run_id
            _require(type(run_id) is str and _RUN.fullmatch(run_id), 'An explicit native display run identity is required')
            _require(self._run_id is None or self._run_id == run_id, 'Native display belongs to another run; create a new binding')
            value = observation.payload
            _require(value.get('run_id') == run_id and value.get('model_sha256') == self._manifest['model_sha256']
                     and value.get('plugin_sha256') == self._plugin_sha256 and value.get('native_version') == _NATIVE,
                     'Native display observation belongs to another run, model, plugin or engine')
            digest = _sha(_canonical(value))
            if value.get('sequence') == self._last_sequence:
                _require(digest == self._last_digest, 'Different native poses share the same sequence')
                return deepcopy(self._last_pose)
            if self._store is None:
                self._store = GraspObservationStore(run_id=run_id, model_sha256=self._manifest['model_sha256'],
                    plugin_sha256=self._plugin_sha256, manifest=self._description, geom_names=self._geom_names)
            accepted = self._store.ingest(value, receipt_sec=observation.receipt_sec)
            _require(accepted.accepted, 'Native display observation rejected: ' + accepted.reason)
            pose = validate_pose(dict(schema_version=1, model_sha256=self._manifest['model_sha256'],
                                      scene_sha256=self._manifest['scene_sha256'], geoms=value['geoms']), self._manifest)
            self._run_id = run_id
            self._last_sequence, self._last_digest, self._last_pose = value['sequence'], digest, pose
            return deepcopy(pose)

    pose = frame_from_observation


def build_scene(target, *, run_id=None, limits=None):
    return RosGraspDisplay(target, run_id=run_id, limits=limits)


def frame_from_observation(scene, observation, *, expected_run_id=None):
    return scene.frame_from_observation(observation, expected_run_id=expected_run_id)
