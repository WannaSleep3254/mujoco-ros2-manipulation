/** Read-only projection of common display data; no robot commands or physics.
 *
 * Imports resolve to reviewed local files through the page's import map. This
 * module never fetches a CDN or interprets a mesh filename as a file-system path.
 * The application owns choosing the current identity and authenticated API routes.
 */
import * as THREE from 'three';
import { OrbitControls } from './vendor/OrbitControls.js';

import {SHA256, DEFAULT_GROUPS, LIMITS, requireValue, finite, sameIdentity,
  validateBinding, validateManifest, decodeMesh, validatePose, poseAgeMs} from './viewer_contract.mjs';

function localURL(value, baseURL) {
  const url = new URL(value, baseURL);
  const base = new URL(baseURL);
  requireValue(['http:', 'https:'].includes(url.protocol) && url.origin === base.origin && !url.username && !url.password,
    'Scene resources must use the current local origin');
  return url.href;
}

function cancelled() {
  return new DOMException('Scene load superseded', 'AbortError');
}

function emptyStats() {
  return {manifest_fetch_ms: null, manifest_parse_ms: null, manifest_verify_ms: null,
    assets_fetch_ms: null, assets_parse_ms: null, assets_verify_ms: null, asset_bytes: 0,
    pose_fetch_ms: null, pose_parse_ms: null, pose_verify_ms: null, pose_apply_ms: null, rendering_fps: 0,
    pose_apply_fps: 0, render_submit_ms: null, source_age_ms: null, sequence: null};
}

/** The class owns only the DOM children it creates inside container. */
export class LocalSceneView {
  constructor(container, options = {}) {
    requireValue(container && typeof container.append === 'function', 'Missing local scene container');
    this.container = container;
    this.options = options;
    this.now = options.now ?? (() => performance.now());
    this.fetch = options.fetch ?? globalThis.fetch.bind(globalThis);
    this.baseURL = options.baseURL ?? globalThis.location.href;
    this.crypto = options.crypto ?? globalThis.crypto;
    this.assetURL = options.assetURL ?? (hash => '/api/view/assets/' + hash);
    requireValue(this.crypto?.subtle, 'SHA256 verification requires a secure local browser context');
    this.staleAfterMs = options.staleAfterMs ?? 500;
    requireValue(finite(this.staleAfterMs) && this.staleAfterMs >= 100 && this.staleAfterMs <= 500, 'Invalid freshness limit');
    this.planeExtentM = options.planeExtentM ?? 10;
    requireValue(finite(this.planeExtentM) && this.planeExtentM > 0, 'Invalid displayed infinite-plane extent');
    this.document = container.ownerDocument;
    this.bindingID = null;
    this.binding = null;
    this.manifest = null;
    this.objects = new Map();
    this.materials = new Set();
    this.geometries = new Set();
    this.disposed = false;
    this.loading = false;
    this.loadGeneration = 0;
    this.loadController = null;
    this.poseController = null;
    this.sequence = -1;
    this.sourceSample = null;
    this.receivedAt = null;
    this.sourceAgeSec = null;
    this.sceneError = null;
    this.manualStaleReason = null;
    this.needsFit = true;
    this.hasPose = false;
    this.sized = false;
    this.collisionVisible = false;
    this.stats = emptyStats();
    this.firstVisibleSubmittedAt = null;
    this.statsAt = this.now();
    this.renderCount = 0;
    this.poseCount = 0;
    this.scene = new THREE.Scene();
    this.scene.background = new THREE.Color(options.background ?? 0x17242a);
    this.world = new THREE.Group();
    this.world.visible = false;
    this.scene.add(this.world);
    this.scene.add(new THREE.HemisphereLight(0xffffff, 0x5a626d, 2.2));
    const light = new THREE.DirectionalLight(0xffffff, 2.8);
    light.position.set(3, -4, 6);
    this.scene.add(light);
    this.camera = new THREE.PerspectiveCamera(45, 1, .01, 1000);
    this.camera.up.set(0, 0, 1);
    this.camera.position.set(2, -2, 1.8);
    this.renderer = new THREE.WebGLRenderer({antialias: true, alpha: false, powerPreference: 'default'});
    this.renderer.setPixelRatio(Math.min(globalThis.devicePixelRatio || 1, 2));
    this.canvas = this.renderer.domElement;
    this.canvas.setAttribute('aria-label', '읽기 전용 로봇 3D 장면. 드래그 회전, 휠 확대, 오른쪽 드래그 이동');
    this.canvas.setAttribute('tabindex', '0');
    this.canvas.className = 'local-scene-canvas';
    this.canvas.style.width = '100%';
    this.canvas.style.height = '100%';
    this.canvas.style.display = 'block';
    this.label = this.document.createElement('div');
    this.label.className = 'local-scene-status';
    this.label.setAttribute('role', 'status');
    this.label.textContent = '장면과 관절 상태 대기';
    container.append(this.canvas, this.label);
    this.controls = new OrbitControls(this.camera, this.canvas);
    this.controls.target.set(0, 0, .6);
    this.controls.enableDamping = true;
    this.controls.screenSpacePanning = true;
    this.controls.minDistance = .05;
    this.controls.maxDistance = 100;
    this.setControlSensitivity(options.controlSensitivity ?? {rotate: 1, zoom: 1, pan: 1});
    this.controls.update();
    this._onContextLost = event => { event.preventDefault(); this.sceneError = '3D 그래픽 컨텍스트를 잃었습니다'; this._refreshStatus(); };
    this.canvas.addEventListener('webglcontextlost', this._onContextLost);
    this._onVisibility = () => {this.statsAt = this.now(); this.renderCount = this.poseCount = 0; this._refreshStatus();};
    this.document.addEventListener('visibilitychange', this._onVisibility);
    this._onResize = () => this.resize();
    this.resizeObserver = typeof ResizeObserver === 'function' ? new ResizeObserver(this._onResize) : null;
    this.resizeObserver?.observe(container);
    globalThis.addEventListener?.('resize', this._onResize);
    this.resize();
    this.renderer.setAnimationLoop(() => this._frame());
    if (options.binding_id !== undefined) this.setExpectedBinding(options.binding_id, options.binding);
  }

  setExpectedBinding(bindingID, binding) {
    requireValue(!this.disposed, 'Scene view is destroyed');
    const next = validateBinding(bindingID, binding);
    if (this.bindingID === bindingID && sameIdentity(this.binding, next)) return false;
    this.loadController?.abort();
    this.poseController?.abort();
    this.loadGeneration++;
    this.bindingID = bindingID;
    this.binding = next;
    this.loading = false;
    this.sequence = -1;
    this.sourceSample = null;
    this.receivedAt = this.sourceAgeSec = null;
    this.stats = emptyStats();
    this.firstVisibleSubmittedAt = null;
    this.statsAt = this.now();
    this.renderCount = this.poseCount = 0;
    this.hasPose = false;
    this.world.visible = false; // A previous job/model must never look current.
    this.sceneError = this.manualStaleReason = null;
    this._refreshStatus();
    return true;
  }

  /** Blank an unavailable epoch without inventing a binding or resetting camera. */
  clearBinding(reason = '장면 준비 대기') {
    requireValue(!this.disposed, 'Scene view is destroyed');
    this.loadController?.abort();
    this.poseController?.abort();
    this.loadController = this.poseController = null;
    this.loadGeneration++;
    this.bindingID = this.binding = null;
    this.loading = false;
    this.sequence = -1;
    this.sourceSample = null;
    this.receivedAt = this.sourceAgeSec = null;
    this.stats = emptyStats();
    this.firstVisibleSubmittedAt = null;
    this.statsAt = this.now();
    this.renderCount = this.poseCount = 0;
    this.hasPose = false;
    this.world.visible = false;
    this.sceneError = null;
    this.manualStaleReason = String(reason);
    this._refreshStatus();
    if (this.label.textContent !== this.manualStaleReason) this.label.textContent = this.manualStaleReason;
  }

  async _bytes(url, {signal, sha256 = null, maximum, expectedBytes = null}) {
    const started = this.now();
    const response = await this.fetch(localURL(url, this.baseURL), {signal, cache: 'no-store', credentials: 'same-origin', redirect: 'error'});
    requireValue(response.ok, `Scene resource request failed (${response.status})`);
    const headerLength = response.headers.get('content-length');
    if (headerLength !== null) requireValue(/^\d+$/.test(headerLength) && Number.isSafeInteger(Number(headerLength)) &&
      Number(headerLength) <= maximum, 'Scene resource exceeds byte limit');
    // Bound chunked responses as they arrive, rather than buffering an unchecked body.
    let bytes;
    if (response.body?.getReader) {
      const reader = response.body.getReader();
      const chunks = [];
      let total = 0;
      try {
        while (true) {
          const {done, value} = await reader.read();
          if (done) break;
          total += value.byteLength;
          requireValue(total <= maximum, 'Scene resource exceeds byte limit');
          chunks.push(value);
        }
      } catch (error) {
        await reader.cancel().catch(() => {});
        throw error;
      } finally {reader.releaseLock();}
      const joined = new Uint8Array(total);
      let offset = 0;
      for (const chunk of chunks) {joined.set(chunk, offset); offset += chunk.byteLength;}
      bytes = joined.buffer;
    } else bytes = await response.arrayBuffer();
    const fetched = this.now();
    requireValue(bytes.byteLength <= maximum, 'Scene resource exceeds byte limit');
    if (expectedBytes !== null) requireValue(bytes.byteLength === expectedBytes, 'Scene resource byte length mismatch');
    const verified = await this.crypto.subtle.digest('SHA-256', bytes);
    const actualSha = Array.from(new Uint8Array(verified), v => v.toString(16).padStart(2, '0')).join('');
    const verifyDone = this.now();
    if (sha256 !== null) requireValue(SHA256.test(sha256) && actualSha === sha256, 'Scene resource SHA256 mismatch');
    return {buffer: bytes, status: response.status, sha256: actualSha, bytes: bytes.byteLength, fetchMs: fetched - started,
      verifyMs: verifyDone - fetched, requestStartedAt: started};
  }

  async _json(url, options) {
    const item = await this._bytes(url, options);
    const started = this.now();
    if (item.status === 204) {item.data = null; item.parseMs = 0; return item;}
    item.data = JSON.parse(new TextDecoder('utf-8', {fatal: true}).decode(item.buffer));
    item.parseMs = this.now() - started;
    return item;
  }

  async loadManifest(url, {binding_id = this.bindingID, binding = this.binding, sha256 = null,
    model_sha256 = null, scene_sha256 = null} = {}) {
    this.setExpectedBinding(binding_id, binding);
    const expectedBinding = {...this.binding};
    const expectedID = this.bindingID;
    const generation = ++this.loadGeneration;
    this.loadController?.abort();
    const controller = this.loadController = new AbortController();
    this.loading = true;
    this.world.visible = false;
    this.stats = emptyStats();
    this.firstVisibleSubmittedAt = null;
    this.statsAt = this.now();
    this.renderCount = this.poseCount = 0;
    this.sceneError = null;
    this._refreshStatus();
    const geometries = new Set();
    const materials = new Set();
    try {
      const downloaded = await this._json(url, {signal: controller.signal, sha256, maximum: LIMITS.manifestBytes});
      const manifest = validateManifest(downloaded.data, {model_sha256, scene_sha256});
      const meshAssets = new Map();
      const assetStats = {fetch: 0, parse: 0, verify: 0, bytes: 0};
      // Sequential, deduplicated fetches bound parse/network bursts during a model switch.
      const descriptors = new Map(manifest.assets.map(item => [item.sha256, item]));
      for (const hash of new Set(manifest.geoms.filter(g => g.type === 'mesh').map(g => g.asset_sha256))) {
        const descriptor = descriptors.get(hash);
        const item = await this._bytes(this.assetURL(hash, expectedID, expectedBinding),
          {signal: controller.signal, sha256: hash, maximum: LIMITS.assetBytes, expectedBytes: descriptor.byte_length});
        const started = this.now();
        const decoded = decodeMesh(item.buffer, descriptor);
        const parseMs = this.now() - started;
        assetStats.bytes += item.bytes;
        requireValue(assetStats.bytes <= LIMITS.totalAssetBytes, 'Compiled scene assets exceed total byte limit');
        assetStats.fetch += item.fetchMs;
        assetStats.parse += parseMs;
        assetStats.verify += item.verifyMs;
        meshAssets.set(hash, decoded);
      }
      if (this.disposed || generation !== this.loadGeneration || expectedID !== this.bindingID || !sameIdentity(expectedBinding, this.binding)) throw cancelled();
      const group = new THREE.Group();
      const objects = new Map();
      const meshGeometries = new Map();
      for (const descriptor of manifest.geoms) {
        let geometry;
        if (descriptor.type === 'mesh' && meshGeometries.has(descriptor.asset_sha256)) geometry = meshGeometries.get(descriptor.asset_sha256);
        else {
          geometry = this._geometry(descriptor, meshAssets.get(descriptor.asset_sha256));
          geometries.add(geometry);
          if (descriptor.type === 'mesh') meshGeometries.set(descriptor.asset_sha256, geometry);
        }
        const [r, g, b, alpha] = descriptor.rgba;
        const material = new THREE.MeshStandardMaterial({color: new THREE.Color().setRGB(r, g, b),
          opacity: alpha, transparent: alpha < 1, depthWrite: alpha >= 1, roughness: .7, metalness: .05, side: THREE.DoubleSide});
        materials.add(material);
        const mesh = new THREE.Mesh(geometry, material);
        mesh.name = descriptor.name ?? `geom-${descriptor.id}`;
        mesh.matrixAutoUpdate = false;
        mesh.visible = alpha > 0 && (descriptor.default_visible || (this.collisionVisible && descriptor.group === 3));
        mesh.userData = {geomId: descriptor.id, geomGroup: descriptor.group, geomType: descriptor.type, alpha};
        group.add(mesh);
        objects.set(descriptor.id, mesh);
      }
      group.visible = false;
      this.needsFit = !this.manifest || this.manifest.scene_sha256 !== manifest.scene_sha256;
      this._disposeModel();
      this.scene.remove(this.world);
      this.world = group;
      this.scene.add(group);
      this.objects = objects;
      this.geometries = geometries;
      this.materials = materials;
      this.manifest = manifest;
      this.manifestSHA256 = downloaded.sha256;
      this.sequence = -1;
      this.receivedAt = this.sourceAgeSec = null;
      this.stats.sequence = this.stats.source_age_ms = null;
      this.hasPose = false;
      this.stats.manifest_fetch_ms = downloaded.fetchMs;
      this.stats.manifest_verify_ms = downloaded.verifyMs;
      this.stats.manifest_parse_ms = downloaded.parseMs;
      this.stats.assets_fetch_ms = assetStats.fetch;
      this.stats.assets_parse_ms = assetStats.parse;
      this.stats.assets_verify_ms = assetStats.verify;
      this.stats.asset_bytes = assetStats.bytes;
      return {model_sha256: manifest.model_sha256, scene_sha256: manifest.scene_sha256, manifest_sha256: downloaded.sha256, geoms: manifest.geoms.length};
    } catch (error) {
      for (const geometry of geometries) geometry.dispose();
      for (const material of materials) material.dispose();
      if (generation === this.loadGeneration && !this.disposed && error.name !== 'AbortError') this.sceneError = '3D 장면을 불러오지 못했습니다';
      throw error;
    } finally {
      if (generation === this.loadGeneration && !this.disposed) {this.loading = false; this._refreshStatus();}
    }
  }

  _geometry(geom, mesh) {
    const [x, y, z] = geom.size;
    switch (geom.type) {
      case 'mesh': {
        const geometry = new THREE.BufferGeometry();
        geometry.setAttribute('position', new THREE.BufferAttribute(mesh.positions, 3));
        geometry.setIndex(new THREE.BufferAttribute(mesh.indices, 1));
        geometry.computeVertexNormals();
        geometry.computeBoundingSphere();
        return geometry; // These are already the compiled, recentered vertices.
      }
      case 'box': return new THREE.BoxGeometry(2 * x, 2 * y, 2 * z);
      case 'sphere': return new THREE.SphereGeometry(x, 24, 16);
      case 'capsule': return new THREE.CapsuleGeometry(x, 2 * y, 8, 24).rotateX(Math.PI / 2);
      case 'cylinder': return new THREE.CylinderGeometry(x, x, 2 * y, 24).rotateX(Math.PI / 2);
      // MuJoCo zero-size planes are infinite; the browser only displays this extent.
      case 'plane': return new THREE.PlaneGeometry(x > 0 ? 2 * x : this.planeExtentM, y > 0 ? 2 * y : this.planeExtentM);
      default: throw new Error('Unsupported compiled geom');
    }
  }

  /** Apply a complete verified binding atomically; camera transforms are untouched. */
  updatePose(envelope, {elapsedRequestMs = 0} = {}) {
    requireValue(!this.disposed, 'Scene view is destroyed');
    if (envelope?.binding_id !== this.bindingID) return false; // Late response for a previous job.
    requireValue(this.manifest && !this.loading, 'Compiled scene is not ready');
    validatePose(envelope, this.manifest, this.bindingID, this.binding);
    if (envelope.sequence <= this.sequence) return false;
    requireValue(!this.sourceSample || envelope.schema_version === 2, 'Source schema changed without a new binding');
    if (envelope.schema_version === 2 && this.sourceSample) {
      const previous = this.sourceSample, current = envelope.source;
      requireValue(current.session_id === previous.session_id && current.clock_domain === previous.clock_domain &&
        current.sequence > previous.sequence && current.stamp_sec >= previous.stamp_sec,
        'Source restarted, rewound or replayed without a new binding');
    }
    requireValue(finite(elapsedRequestMs, true), 'Invalid client request elapsed duration');
    const started = this.now();
    for (const geom of envelope.pose.geoms) {
      const [a, b, c, d, e, f, g, h, i] = geom.rotation;
      const [x, y, z] = geom.position;
      this.objects.get(geom.id).matrix.set(a, b, c, x, d, e, f, y, g, h, i, z, 0, 0, 0, 1);
      this.objects.get(geom.id).matrixWorldNeedsUpdate = true;
    }
    this.sequence = envelope.sequence;
    this.sourceSample = envelope.schema_version === 2 ? {...envelope.source} : null;
    this.receivedAt = this.now();
    // Conservatively include the whole locally observed request duration if supplied.
    this.sourceAgeSec = envelope.source_state_age_sec === null ? null : envelope.source_state_age_sec + elapsedRequestMs / 1000;
    this.hasPose = true;
    this.manualStaleReason = null;
    this.world.visible = true;
    if (this.needsFit) {this._fitCamera(); this.needsFit = false;}
    this.stats.pose_apply_ms = this.now() - started;
    this.stats.sequence = this.sequence;
    this.poseCount++;
    this._refreshStatus();
    return true;
  }

  /** Optional polling helper. At most one request may run; the app owns its cadence. */
  async pollPose(url) {
    if (this.poseController || this.loading || !this.manifest || this.disposed) return false;
    const expected = {...this.binding};
    const expectedID = this.bindingID;
    const controller = this.poseController = new AbortController();
    try {
      const item = await this._json(url, {signal: controller.signal, maximum: LIMITS.manifestBytes});
      if (this.disposed || expectedID !== this.bindingID || !sameIdentity(expected, this.binding)) return false;
      if (item.data === null) {this.markStale(); return false;}
      const applied = this.updatePose(item.data, {elapsedRequestMs: this.now() - item.requestStartedAt});
      this.stats.pose_fetch_ms = item.fetchMs;
      this.stats.pose_parse_ms = item.parseMs;
      this.stats.pose_verify_ms = item.verifyMs;
      return applied;
    } catch (error) {
      if (error.name !== 'AbortError' && !this.disposed && expectedID === this.bindingID && sameIdentity(expected, this.binding)) this.markStale('갱신 대기 · 마지막 수신 자세');
      throw error;
    } finally {
      if (this.poseController === controller) this.poseController = null;
    }
  }

  setCollisionVisible(visible) {
    requireValue(typeof visible === 'boolean', 'Collision visibility must be boolean');
    this.collisionVisible = visible;
    for (const object of this.objects.values()) {
      const {geomGroup, alpha} = object.userData;
      object.visible = alpha > 0 && (DEFAULT_GROUPS.includes(geomGroup) || (visible && geomGroup === 3));
    }
  }

  markStale(reason = '갱신 대기 · 마지막 수신 자세') {
    this.manualStaleReason = String(reason);
    this._refreshStatus();
  }

  _refreshStatus() {
    if (this.disposed) return;
    const live = this.binding?.mode === 'live';
    const age = this.hasPose && live ? poseAgeMs(this.sourceAgeSec, this.receivedAt, this.now()) : null;
    const stale = this.hasPose && (this.manualStaleReason !== null || (live && age > this.staleAfterMs) || Boolean(this.sceneError));
    this.stats.source_age_ms = age;
    const state = this.sceneError ? 'error' : this.loading ? 'loading' : !this.hasPose ? 'waiting' : stale ? 'stale' : live ? 'fresh' : 'preview';
    this.canvas.style.visibility = this.hasPose && this.world.visible ? 'visible' : 'hidden';
    const text = this.sceneError ?? (this.loading ? '3D 장면 준비 중' : !this.hasPose ? (!this.binding ? this.manualStaleReason ?? '관절 상태 대기' : '관절 상태 대기') :
      stale ? this.manualStaleReason ?? '갱신 대기 · 마지막 수신 자세' : live ? '현재 관절 자세 · 읽기 전용' :
        this.binding.mode === 'plan_preview' ? '계획 경로 미리보기 · 읽기 전용' : '모델 미리보기 · 읽기 전용');
    if (this.label.dataset.state !== state) {
      this.canvas.style.opacity = stale ? '.55' : '1';
      this.canvas.style.filter = stale ? 'saturate(.15)' : '';
      this.label.dataset.state = state;
    }
    if (this.label.textContent !== text) this.label.textContent = text;
  }

  /** Local camera only: retain the current target and distance, change direction. */
  cameraPreset(view = 'overview') {
    requireValue(!this.disposed, 'Scene view is destroyed');
    requireValue(['overview', 'front', 'top'].includes(view), 'Unknown camera preset');
    if (!this.hasPose || !this.manifest || this.loading) return false;
    const target = this.controls.target.clone();
    const distance = this.camera.position.clone().sub(target).length();
    const directions = {overview: [1, -1, .7], front: [0, -1, 0], top: [0, -.0001, 1]};
    // A tiny tilt keeps a top view well-defined while the camera's up stays Z.
    this._placeCamera(target, new THREE.Vector3(...directions[view]), distance);
    return true;
  }

  /** Relative local dolly; the same OrbitControls limits apply to wheel/buttons. */
  zoomCamera(direction) {
    requireValue(!this.disposed, 'Scene view is destroyed');
    requireValue(['in', 'out'].includes(direction), 'Unknown zoom direction');
    if (!this.hasPose || !this.manifest || this.loading) return false;
    const target = this.controls.target.clone();
    const offset = this.camera.position.clone().sub(target), distance = offset.length();
    const minimum = this.controls.minDistance ?? .05, maximum = this.controls.maxDistance ?? 100;
    requireValue(finite(distance) && distance > 0 && finite(minimum) && minimum > 0 &&
      finite(maximum) && maximum >= minimum, 'Invalid camera distance limits');
    const scale = Math.pow(1.25, (this.controls.zoomSpeed ?? 1) * (direction === 'in' ? -1 : 1));
    const next = Math.max(minimum, Math.min(maximum, distance * scale));
    if (Math.abs(next - distance) < 1e-12) return false;
    this._placeCamera(target, offset, next);
    return true;
  }

  getControlSensitivity() {
    return {rotate: this.controls.rotateSpeed ?? 1, zoom: this.controls.zoomSpeed ?? 1, pan: this.controls.panSpeed ?? 1};
  }

  setControlSensitivity(value) {
    requireValue(!this.disposed, 'Scene view is destroyed');
    const keys = ['rotate', 'zoom', 'pan'];
    requireValue(value && typeof value === 'object' && !Array.isArray(value) &&
      Object.keys(value).length === keys.length && keys.every(key => Object.hasOwn(value, key) &&
        finite(value[key]) && value[key] >= .25 && value[key] <= 3), 'Camera sensitivity must be 0.25..3');
    this.controls.rotateSpeed = value.rotate;
    this.controls.zoomSpeed = value.zoom;
    this.controls.panSpeed = value.pan;
    return this.getControlSensitivity();
  }

  /** Explicit fit preserves the viewing direction; state updates never call it. */
  fitCamera() {
    requireValue(!this.disposed, 'Scene view is destroyed');
    if (!this.hasPose || !this.manifest || this.loading) return false;
    const direction = this.camera.position.clone().sub(this.controls.target);
    const fitted = this._fitCamera(direction, false);
    if (fitted) this.needsFit = false;
    return fitted;
  }

  _placeCamera(target, direction, distance) {
    requireValue(finite(distance) && distance > 0 && finite(direction.length()) && direction.length() > 0,
      'Invalid camera placement');
    const damping = this.controls.enableDamping;
    try {
      // Drain pending user damping before placing a preset, so it cannot drift
      // back toward the old view on the next animation frame.
      this.controls.enableDamping = false;
      this.controls.update();
      this.controls.target.copy(target);
      this.camera.up.set(0, 0, 1);
      this.camera.position.copy(target).add(direction.clone().normalize().multiplyScalar(distance));
      this.controls.update();
    } finally {this.controls.enableDamping = damping;}
  }

  _fitCamera(direction = new THREE.Vector3(1, -1, .7), preferRobotVisuals = true) {
    this.world.updateMatrixWorld(true);
    const visible = [...this.objects.values()].filter(object => object.visible && object.userData.geomType !== 'plane');
    const robotVisuals = preferRobotVisuals ? visible.filter(object => [1, 2].includes(object.userData.geomGroup)) : [];
    const groupZeroMeshes = preferRobotVisuals && !robotVisuals.length ?
      visible.filter(object => object.userData.geomGroup === 0 && object.userData.geomType === 'mesh') : [];
    // Initial framing favors robot visuals over a large installation/base. The
    // group-0 mesh fallback is only a camera heuristic: CAD meshes can share it.
    // Geometry classification/rendering is unchanged; explicit fit uses all bounds.
    const candidates = robotVisuals.length ? robotVisuals : groupZeroMeshes.length ? groupZeroMeshes : visible;
    const box = new THREE.Box3();
    for (const object of candidates) box.expandByObject(object);
    if (box.isEmpty()) return false;
    const center = box.getCenter(new THREE.Vector3());
    const size = box.getSize(new THREE.Vector3());
    const radius = Math.max(size.length() / 2, .1);
    const angle = THREE.MathUtils.degToRad(this.camera.fov / 2);
    const distance = radius / Math.sin(angle) * 1.2 / Math.min(this.camera.aspect, 1);
    this.camera.near = Math.max(.001, radius / 1000);
    this.camera.far = Math.max(100, radius * 100, distance + radius * 2);
    this.camera.updateProjectionMatrix();
    this._placeCamera(center, direction, distance);
    this.controls.saveState();
    return true;
  }

  resetCamera() {
    requireValue(!this.disposed, 'Scene view is destroyed');
    if (!this.hasPose || !this.manifest || this.loading) return false;
    const fitted = this._fitCamera(new THREE.Vector3(1, -1, .7));
    if (fitted) this.needsFit = false;
    return fitted;
  }

  resize() {
    if (this.disposed) return;
    const {width, height} = this.container.getBoundingClientRect();
    this.sized = width > 0 && height > 0;
    if (!this.sized) return;
    this.camera.aspect = width / height;
    this.camera.updateProjectionMatrix();
    this.renderer.setSize(width, height, false);
  }

  _frame() {
    if (this.disposed) return;
    this._refreshStatus();
    if (this.document.hidden || !this.sized || this.sceneError) return;
    this.controls.update(); // Remains independent of ROS state/polling cadence.
    const started = this.now();
    this.renderer.render(this.scene, this.camera);
    const displayingPose = this.hasPose && this.world.visible;
    this.stats.render_submit_ms = displayingPose ? this.now() - started : null; // CPU submission, not GPU timing.
    // Optional read-only benchmark hook. A render submission is not GPU
    // completion or a physical display paint; consumers must retain that label.
    if (displayingPose && this.firstVisibleSubmittedAt === null) this.firstVisibleSubmittedAt = this.now();
    this.options.onRenderFrame?.({submitted_at_ms: this.now(), sequence: this.sequence,
      displaying_pose: displayingPose, render_submit_ms: this.stats.render_submit_ms,
      first_visible_submitted_at_ms: this.firstVisibleSubmittedAt,
      camera_position: this.camera.position.toArray(), camera_quaternion: this.camera.quaternion.toArray()});
    if (displayingPose) this.renderCount++;
    const now = this.now();
    const windowMs = now - this.statsAt;
    if (windowMs >= 1000) {
      this.stats.rendering_fps = this.renderCount * 1000 / windowMs;
      this.stats.pose_apply_fps = this.poseCount * 1000 / windowMs;
      this.statsAt = now;
      this.renderCount = this.poseCount = 0;
      this.options.onMetrics?.({...this.stats, binding_id: this.bindingID, mode: this.binding?.mode ?? null,
        view_state: this.label.dataset.state, stale: this.label.dataset.state === 'stale',
        note: 'Client FPS counts rendered frames; pose_apply_fps counts unique poses. Fetch/parse/apply are browser-local durations; render_submit_ms is CPU submission, not GPU time.'});
    }
  }

  _disposeModel() {
    for (const geometry of this.geometries) geometry.dispose();
    for (const material of this.materials) material.dispose();
    this.geometries.clear();
    this.materials.clear();
    this.objects.clear();
  }

  destroy() {
    if (this.disposed) return;
    this.disposed = true;
    this.loadGeneration++;
    this.loadController?.abort();
    this.poseController?.abort();
    this.renderer.setAnimationLoop(null);
    this.resizeObserver?.disconnect();
    globalThis.removeEventListener?.('resize', this._onResize);
    this.document.removeEventListener('visibilitychange', this._onVisibility);
    this.canvas.removeEventListener('webglcontextlost', this._onContextLost);
    this.controls.dispose();
    this._disposeModel();
    this.renderer.dispose();
    this.renderer.forceContextLoss();
    this.canvas.remove();
    this.label.remove();
  }
}
