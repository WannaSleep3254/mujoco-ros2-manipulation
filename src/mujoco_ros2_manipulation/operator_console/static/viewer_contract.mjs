/** Pure scene transport validation; no DOM, Three.js, ROS or physics. */
const SHA256 = /^[a-f0-9]{64}$/;
const ID32 = /^[a-f0-9]{32}$/;
const TYPES = new Set(['mesh', 'box', 'sphere', 'capsule', 'cylinder', 'plane']);
const DEFAULT_GROUPS = [0, 1, 2];
const LIMITS = Object.freeze({manifestBytes: 8 * 1024 * 1024, assetBytes: 48 * 1024 * 1024,
  totalAssetBytes: 128 * 1024 * 1024, geoms: 10000, vertices: 2000000, faces: 2000000});

function requireValue(value, message) {
  if (!value) throw new Error(message);
}

function finite(value, nonnegative = false) {
  return typeof value === 'number' && Number.isFinite(value) && (!nonnegative || value >= 0);
}

function vector(value, size, label) {
  requireValue(Array.isArray(value) && value.length === size && value.every(v => finite(v)), `Invalid ${label}`);
  return value;
}

function integer(value, label, maximum = Number.MAX_SAFE_INTEGER) {
  requireValue(Number.isSafeInteger(value) && value >= 0 && value <= maximum, `Invalid ${label}`);
  return value;
}

/** Only compare server-relative ages with client-local elapsed durations. */
export function poseAgeMs(sourceAgeSec, receivedAtMs, nowMs) {
  requireValue(finite(sourceAgeSec, true) && finite(receivedAtMs, true) && finite(nowMs, true), 'Invalid pose age');
  return sourceAgeSec * 1000 + Math.max(0, nowMs - receivedAtMs);
}

/** Copy the complete application binding; unknown/missing fields are rejected. */
export function validateBinding(bindingID, binding) {
  requireValue(typeof bindingID === 'string' && SHA256.test(bindingID), 'Invalid scene binding ID');
  const keys = ['operator_session_id', 'generation', 'mode', 'source_id', 'job_id', 'run_id', 'motion_session_id', 'request_id'];
  requireValue(binding && typeof binding === 'object' && !Array.isArray(binding) &&
    Object.keys(binding).length === keys.length && keys.every(key => Object.hasOwn(binding, key)), 'Incomplete scene binding');
  requireValue(typeof binding.operator_session_id === 'string' && ID32.test(binding.operator_session_id), 'Invalid operator session');
  requireValue(['live', 'model_preview', 'plan_preview'].includes(binding.mode), 'Invalid scene binding mode');
  requireValue(typeof binding.source_id === 'string' && binding.source_id.length <= 128 &&
    /^(?:(?:workcell|profile|scene):)?[a-z][a-z0-9_-]*$/.test(binding.source_id), 'Invalid scene source ID');
  const copy = {operator_session_id: binding.operator_session_id, generation: integer(binding.generation, 'scene generation'),
    mode: binding.mode, source_id: binding.source_id};
  for (const key of ['job_id', 'motion_session_id', 'request_id']) {
    requireValue(binding[key] === null || (typeof binding[key] === 'string' && ID32.test(binding[key])), `Invalid ${key}`);
    copy[key] = binding[key];
  }
  requireValue(binding.run_id === null || (typeof binding.run_id === 'string' && binding.run_id.length <= 128 &&
    /^[A-Za-z0-9][A-Za-z0-9_.-]*$/.test(binding.run_id)), 'Invalid run ID');
  copy.run_id = binding.run_id;
  requireValue((copy.job_id === null) === (copy.run_id === null), 'Job/run binding must be paired');
  if (copy.mode === 'model_preview') requireValue(['job_id', 'run_id', 'motion_session_id', 'request_id'].every(key => copy[key] === null),
    'Model preview cannot inherit live/request identity');
  if (copy.mode === 'live') requireValue(copy.request_id === null, 'Live binding cannot reference a saved request');
  if (copy.mode === 'plan_preview') requireValue(copy.request_id !== null, 'Plan preview requires a saved request identity');
  return copy;
}

function sameIdentity(left, right) {
  return left !== null && right !== null && Object.keys(left).every(key => left[key] === right[key]);
}

function checkRotation(rotation) {
  vector(rotation, 9, 'row-major geom rotation');
  for (let row = 0; row < 3; row++) {
    for (let other = row; other < 3; other++) {
      let dot = 0;
      for (let col = 0; col < 3; col++) dot += rotation[row * 3 + col] * rotation[other * 3 + col];
      requireValue(Math.abs(dot - Number(row === other)) <= 1e-4, 'Non-orthonormal geom rotation');
    }
  }
  const [a, b, c, d, e, f, g, h, i] = rotation;
  requireValue(Math.abs(a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g) - 1) <= 1e-4,
    'Reflected geom rotation');
  return rotation;
}

/** Validation completes before any existing scene is changed. */
export function validateManifest(manifest, expected = {}) {
  requireValue(manifest && manifest.schema_version === 1 && manifest.export_version === 'compiled-scene-v1', 'Invalid manifest schema');
  for (const key of ['scene_sha256', 'model_sha256', 'compiled_model_sha256']) {
    requireValue(typeof manifest[key] === 'string' && SHA256.test(manifest[key]), `Invalid ${key}`);
    if (expected[key] !== undefined && expected[key] !== null) requireValue(expected[key] === manifest[key], `Manifest ${key} mismatch`);
  }
  requireValue(manifest.units?.length === 'm' && manifest.units?.angle === 'rad' &&
    manifest.units?.up_axis === 'Z' && manifest.units?.rotation === 'row-major', 'Unsupported scene units or rotation convention');
  requireValue(Array.isArray(manifest.default_visible_groups) && JSON.stringify(manifest.default_visible_groups) === JSON.stringify(DEFAULT_GROUPS),
    'Unexpected default geom visibility');
  requireValue(Array.isArray(manifest.geoms) && manifest.geoms.length > 0 && manifest.geoms.length <= LIMITS.geoms,
    'Invalid compiled geom catalog');
  requireValue(Array.isArray(manifest.assets) && manifest.assets.length <= manifest.geoms.length, 'Invalid compiled mesh descriptors');
  const assets = new Map();
  let totalBytes = 0;
  for (const asset of manifest.assets) {
    requireValue(asset && typeof asset.sha256 === 'string' && SHA256.test(asset.sha256) && !assets.has(asset.sha256), 'Duplicate or invalid compiled mesh SHA');
    requireValue(asset.media_type === 'application/octet-stream', 'Unsupported compiled mesh encoding');
    integer(asset.byte_length, 'compiled asset byte length', LIMITS.assetBytes);
    const positions = asset.layout?.positions, indices = asset.layout?.indices;
    requireValue(positions && indices && positions.component_type === 'float32' && positions.item_size === 3 && positions.byte_offset === 0 &&
      indices.component_type === 'uint32' && indices.item_size === 1, 'Invalid compiled mesh buffer layout');
    integer(positions.count, 'compiled vertex count', LIMITS.vertices);
    integer(indices.count, 'compiled index count', LIMITS.faces * 3);
    requireValue(positions.count >= 3 && indices.count >= 3 && indices.count % 3 === 0 && positions.byte_length === positions.count * 12 &&
      indices.byte_offset === positions.byte_length && indices.byte_length === indices.count * 4 &&
      asset.byte_length === positions.byte_length + indices.byte_length, 'Inconsistent compiled mesh buffer ranges');
    totalBytes += asset.byte_length;
    assets.set(asset.sha256, asset);
  }
  requireValue(totalBytes <= LIMITS.totalAssetBytes, 'Compiled scene assets exceed total byte limit');
  const ids = new Set();
  for (const [catalogIndex, geom] of manifest.geoms.entries()) {
    integer(geom.id, 'geom ID');
    requireValue(geom.id === catalogIndex, 'Compiled geom IDs must form the complete ordered catalog');
    requireValue(!ids.has(geom.id), 'Duplicate compiled geom ID');
    ids.add(geom.id);
    requireValue(TYPES.has(geom.type), 'Unsupported compiled geom type');
    integer(geom.group, 'geom group', 5);
    vector(geom.size, 3, 'MuJoCo geom size');
    requireValue(geom.size.every(v => Math.abs(v) <= 1e9), 'Unbounded geom size');
    vector(geom.rgba, 4, 'geom RGBA');
    requireValue(geom.rgba.every(v => v >= 0 && v <= 1), 'Geom RGBA outside 0..1');
    requireValue(geom.size.every(v => v >= 0), 'Negative geom size');
    requireValue(!('mesh_pos' in geom) && !('mesh_quat' in geom), 'Compiled vertices must not be transformed twice');
    if (geom.type === 'mesh') requireValue(typeof geom.asset_sha256 === 'string' && assets.has(geom.asset_sha256), 'Missing compiled mesh asset');
    else {
      requireValue(geom.asset_sha256 === null, 'Primitive unexpectedly references a mesh');
      requireValue(geom.size[0] > 0 || geom.type === 'plane', 'Degenerate primitive size');
      if (geom.type === 'box') requireValue(geom.size.every(v => v > 0), 'Degenerate primitive extents');
      if (geom.type === 'cylinder') requireValue(geom.size[1] > 0, 'Degenerate cylinder height');
    }
    requireValue(typeof geom.default_visible === 'boolean' && geom.default_visible === DEFAULT_GROUPS.includes(geom.group), 'Geom default visibility mismatch');
  }
  return manifest;
}

/** Decode explicit little-endian compiled arrays, without reapplying mesh offsets. */
export function decodeMesh(bytes, descriptor) {
  requireValue(bytes instanceof ArrayBuffer && bytes.byteLength === descriptor.byte_length, 'Compiled mesh byte length mismatch');
  const view = new DataView(bytes);
  const {positions: p, indices: i} = descriptor.layout;
  const positions = new Float32Array(p.count * 3);
  const indices = new Uint32Array(i.count);
  for (let index = 0; index < positions.length; index++) {
    const value = view.getFloat32(p.byte_offset + index * 4, true);
    requireValue(finite(value), 'Nonfinite compiled vertex');
    positions[index] = value;
  }
  for (let index = 0; index < indices.length; index++) {
    const value = view.getUint32(i.byte_offset + index * 4, true);
    requireValue(value < p.count, 'Compiled mesh vertex index out of range');
    indices[index] = value;
  }
  return {positions, indices};
}

export function validatePose(envelope, manifest, bindingID, binding) {
  requireValue(envelope && envelope.schema_version === 1 && envelope.binding_id === bindingID &&
    sameIdentity(validateBinding(envelope.binding_id, envelope.binding), binding), 'Pose binding mismatch');
  integer(envelope.sequence, 'pose sequence');
  requireValue(envelope.sequence >= 1, 'Pose sequence must start at one');
  requireValue(binding.mode === 'live' ? finite(envelope.source_state_age_sec, true) : envelope.source_state_age_sec === null,
    'Invalid source state age for this view mode');
  requireValue(envelope.simulation_sec === null || finite(envelope.simulation_sec, true), 'Invalid source simulation stamp');
  if (binding.mode === 'plan_preview') requireValue(finite(envelope.preview_time_sec, true), 'Missing plan preview time');
  const pose = envelope.pose;
  requireValue(pose && pose.schema_version === 1 && pose.model_sha256 === manifest.model_sha256 && pose.scene_sha256 === manifest.scene_sha256,
    'Pose compiled scene/model mismatch');
  requireValue(Array.isArray(pose.geoms) && pose.geoms.length === manifest.geoms.length, 'Incomplete geom pose catalog');
  const ids = new Set(manifest.geoms.map(g => g.id));
  for (const geom of pose.geoms) {
    requireValue(ids.delete(geom.id), 'Unknown or duplicate geom pose');
    vector(geom.position, 3, 'world geom position');
    requireValue(geom.position.every(v => Math.abs(v) <= 1e9), 'Unbounded world geom position');
    checkRotation(geom.rotation);
  }
  requireValue(ids.size === 0, 'Missing geom poses');
  return envelope;
}

export {SHA256, DEFAULT_GROUPS, LIMITS, requireValue, finite, sameIdentity};
