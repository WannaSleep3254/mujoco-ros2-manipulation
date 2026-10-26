#!/usr/bin/env node
/** Chrome process isolation and evidence collection; no active app connection. */
import {createRequire} from 'node:module';
import {spawn} from 'node:child_process';
import {readFile, writeFile, mkdir} from 'node:fs/promises';
import {once} from 'node:events';
import {createHash} from 'node:crypto';
import {fileURLToPath} from 'node:url';
import path from 'node:path';

const options = {};
for (let i = 2; i < process.argv.length; i += 2) options[process.argv[i].replace(/^--/, '')] = process.argv[i + 1];
const require = createRequire(import.meta.url);
const {chromium} = require(options['playwright-module']);
const script = fileURLToPath(new URL('./viewer_display_benchmark.py', import.meta.url));
const output = options.output;
await mkdir(output, {recursive: true});
const delay = ms => new Promise(resolve => setTimeout(resolve, ms));

function summary(values) {
  const a = values.filter(Number.isFinite).sort((a, b) => a - b);
  if (!a.length) return {count: 0, mean: null, median: null, p95: null, max: null};
  return {count: a.length, mean: a.reduce((a, b) => a + b, 0) / a.length,
    median: a.length % 2 ? a[(a.length - 1) / 2] : (a[a.length / 2 - 1] + a[a.length / 2]) / 2,
    p95: a[Math.ceil(a.length * .95) - 1], max: a.at(-1)};
}

async function processes(root) {
  const result = new Map();
  const visit = async pid => {
    try {
      const raw = await readFile(`/proc/${pid}/stat`, 'utf8'), fields = raw.slice(raw.lastIndexOf(')') + 2).split(/\s+/);
      result.set(`${pid}:${fields[19]}`, {pid, cpu_ticks: Number(fields[11]) + Number(fields[12]), rss_pages: Number(fields[21])});
      const children = (await readFile(`/proc/${pid}/task/${pid}/children`, 'utf8')).trim();
      if (children) await Promise.all(children.split(/\s+/).map(Number).map(visit));
    } catch (_) { /* Owned processes can terminate during sampling. */ }
  };
  await visit(root);
  return result;
}

// Linux USER_HZ is queried rather than silently assuming 100 on other kernels.
const getconf = spawn('getconf', ['CLK_TCK']);
let ticksOutput = ''; getconf.stdout.on('data', chunk => {ticksOutput += chunk;});
await once(getconf, 'exit');
const ticksPerSec = Number(ticksOutput.trim());
if (!Number.isFinite(ticksPerSec) || ticksPerSec <= 0) throw new Error('Cannot read CLK_TCK');
const getpages = spawn('getconf', ['PAGESIZE']);
let pagesOutput = ''; getpages.stdout.on('data', chunk => {pagesOutput += chunk;});
await once(getpages, 'exit');
const pageSize = Number(pagesOutput.trim());
if (!Number.isFinite(pageSize) || pageSize <= 0) throw new Error('Cannot read PAGESIZE');

async function sampleResource(pid, prior) {
  const now = performance.now(), current = await processes(pid);
  const elapsed = prior ? (now - prior.at) / 1000 : null;
  let cpuSec = 0, measured = 0, created = 0, rss = 0;
  for (const [key, item] of current) {
    rss += item.rss_pages * pageSize;
    const before = prior?.values.get(key);
    if (before && item.cpu_ticks >= before.cpu_ticks) {cpuSec += (item.cpu_ticks - before.cpu_ticks) / ticksPerSec; measured++;}
    else created++;
  }
  return {at: now, values: current, public: {at_ms: now, process_count: current.size,
    cpu_percent_one_core: measured && elapsed > 0 ? cpuSec / elapsed * 100 : null,
    rss_mib_sum: rss / 1024 / 1024, unmeasured_new_processes: created}};
}

async function launchFixture(mode) {
  const args = [script, '--serve', '--mode', mode, '--model', options.model, '--recording', options.recording,
    '--output', output, '--width', options.width, '--height', options.height, '--poll-ms', options['poll-ms']];
  const process = spawn(options.python, args, {env: {...globalThis.process.env, MUJOCO_GL: 'egl'}, stdio: ['ignore', 'pipe', 'pipe']});
  let stderr = '', stdout = '';
  process.stderr.on('data', data => {stderr += data.toString();});
  const ready = new Promise((resolve, reject) => {
    const timer = setTimeout(() => reject(new Error('Fixture initialization timeout: ' + stderr.slice(-2000))), 60000);
    process.stdout.on('data', chunk => {
      stdout += chunk.toString();
      const line = stdout.split('\n')[0];
      if (stdout.includes('\n')) {
        clearTimeout(timer);
        try {resolve(JSON.parse(line));} catch (error) {reject(error);}
      }
    });
    process.on('exit', code => {clearTimeout(timer); reject(new Error(`Fixture exited ${code}: ${stderr.slice(-2000)}`));});
  });
  try {return {process, ...(await ready)};} catch (error) {process.kill('SIGTERM'); throw error;}
}

const trials = [];
const readSHA = async file => createHash('sha256').update(await readFile(file)).digest('hex');
const expectedModel = await readSHA(options.model), expectedRecording = await readSHA(options.recording);
async function trial(mode, round, duration, soak = false) {
  let fixture, browserServer, browser;
  try {
    fixture = await launchFixture(mode);
    const browserArgs = ['--no-sandbox', '--disable-dev-shm-usage'];
    if (options['hardware-gl'] === 'true') browserArgs.push('--use-gl=angle', '--use-angle=gl', '--disable-software-rasterizer');
    browserServer = await chromium.launchServer({executablePath: options.chrome, headless: true,
      args: browserArgs, timeout: 30000});
    browser = await chromium.connect(browserServer.wsEndpoint());
    const context = await browser.newContext({viewport: {width: Number(options.width), height: Number(options.height)}, deviceScaleFactor: 1});
    const page = await context.newPage(), errors = [];
    page.on('pageerror', error => {errors.push(String(error));});
    const protocol = await context.newCDPSession(page);
    await protocol.send('Performance.enable');
    const navigationStart = performance.now();
    await page.goto(fixture.url, {waitUntil: 'load', timeout: 30000});
    await page.waitForFunction(() => globalThis.benchmark?.ready && globalThis.benchmark?.firstVisibleMs !== null, undefined, {timeout: 30000});
    const readyMs = performance.now() - navigationStart;
    await delay(1000);
    const start = await page.evaluate(() => ({now: performance.now(), frames: benchmark.frames, applied: benchmark.applied,
      resources: performance.getEntriesByType('resource').map(r => ({name: new URL(r.name).pathname, bytes: r.encodedBodySize, start: r.startTime}))}));
    const resources = [], serverResources = [];
    let browserPrior = await sampleResource(browserServer.process().pid), serverPrior = await sampleResource(fixture.process.pid);
    const startWall = performance.now();
    while (performance.now() - startWall < duration * 1000) {
      await delay(500);
      browserPrior = await sampleResource(browserServer.process().pid, browserPrior);
      serverPrior = await sampleResource(fixture.process.pid, serverPrior);
      resources.push(browserPrior.public); serverResources.push(serverPrior.public);
    }
    const end = await page.evaluate(() => ({now: performance.now(), frames: benchmark.frames, applied: benchmark.applied,
      resources: performance.getEntriesByType('resource').map(r => ({name: new URL(r.name).pathname, bytes: r.encodedBodySize, start: r.startTime}))}));
    const perf = (await protocol.send('Performance.getMetrics')).metrics;
    const snapshot = await page.evaluate(() => benchmark.snapshot());
    if (options['hardware-gl'] === 'true' && mode === 'web3d' && (!snapshot.gl?.renderer ||
        /swiftshader|llvmpipe|softpipe|software/i.test(snapshot.gl.renderer)))
      throw new Error('Hardware GL was requested but a software WebGL renderer was selected');
    const elapsed = (end.now - start.now) / 1000;
    const windowResources = end.resources.filter(r => r.start >= start.now && ['/pose', '/frame'].includes(r.name));
    const initialResources = end.resources.filter(r => ['/manifest'].includes(r.name) || r.name.startsWith('/assets/'));
    // Trusted CDP-generated wheel events, after the stationary-camera baseline.
    // These are neither physical mouse events nor input-to-screen measurements.
    await page.mouse.move(Number(options.width) / 2, Number(options.height) / 2);
    for (let i = 0; i < 6; i++) {await page.mouse.wheel(0, i % 2 ? 100 : -100); await delay(500);}
    const input = await page.evaluate(() => benchmark.inputSamples);
    const stability = mode === 'web3d' && soak ? await page.evaluate(() => benchmark.stability()) : null;
    await page.screenshot({path: path.join(output, `${soak ? 'soak' : 'trial'}-${mode}-${round}.png`)});
    const record = {mode, round, soak, duration_sec: elapsed, independent_browser_pid: browserServer.process().pid,
      independent_server_pid: fixture.process.pid, browser_version: browser.version(), browser_args: browserArgs,
      hardware_gl_required: options['hardware-gl'] === 'true', navigation_to_ready_ms: readyMs,
      frames_per_sec: (end.frames - start.frames) / elapsed, unique_pose_or_image_per_sec: (end.applied - start.applied) / elapsed,
      response_body_bytes: windowResources.reduce((sum, r) => sum + r.bytes, 0),
      response_body_bytes_per_sec: windowResources.reduce((sum, r) => sum + r.bytes, 0) / elapsed,
      initial_manifest_asset_bytes: initialResources.reduce((sum, r) => sum + r.bytes, 0),
      browser_cpu_percent: summary(resources.map(r => r.cpu_percent_one_core)),
      browser_rss_mib_sum: summary(resources.map(r => r.rss_mib_sum)),
      fixture_cpu_percent: summary(serverResources.map(r => r.cpu_percent_one_core)),
      fixture_rss_mib_sum: summary(serverResources.map(r => r.rss_mib_sum)),
      browser_resource_samples: resources, fixture_resource_samples: serverResources,
      cdp_metrics: Object.fromEntries(perf.filter(m => ['JSHeapUsedSize', 'JSHeapTotalSize', 'TaskDuration', 'LayoutDuration', 'ScriptDuration'].includes(m.name)).map(m => [m.name, m.value])),
      input_samples: input, stability, snapshot, page_errors: errors};
    if (errors.length || snapshot.errors.length) throw new Error('Browser errors: ' + JSON.stringify({errors, fetch: snapshot.errors}));
    trials.push(record);
    await writeFile(path.join(output, `${soak ? 'soak' : 'trial'}-${mode}-${round}.json`), JSON.stringify(record, null, 2) + '\n');
    console.log(JSON.stringify({mode, round, soak, fps: record.frames_per_sec, pose_hz: record.unique_pose_or_image_per_sec,
      browser_cpu: record.browser_cpu_percent.mean, server_cpu: record.fixture_cpu_percent.mean}));
  } finally {
    await browser?.close().catch(() => {});
    await browserServer?.close().catch(() => {});
    if (fixture?.process.exitCode === null) {fixture.process.kill('SIGTERM'); await once(fixture.process, 'exit').catch(() => {});}
  }
}

for (let round = 1; round <= Number(options.repeats); round++) {
  for (const mode of round % 2 ? ['web3d', 'image'] : ['image', 'web3d']) {
    if (options['resume-trials'] === 'true') {
      const saved = JSON.parse(await readFile(path.join(output, `trial-${mode}-${round}.json`), 'utf8'));
      const c = saved.snapshot.config;
      if (saved.mode !== mode || saved.round !== round || saved.soak || saved.page_errors.length || saved.snapshot.errors.length ||
          c.model_sha256 !== expectedModel || c.record_sha256 !== expectedRecording || c.width !== Number(options.width) ||
          c.height !== Number(options.height) || c.poll_ms !== Number(options['poll-ms']) ||
          Boolean(saved.hardware_gl_required) !== (options['hardware-gl'] === 'true') ||
          saved.duration_sec < Number(options.seconds) || saved.duration_sec > Number(options.seconds) + 2)
        throw new Error('Saved trial does not match the requested conditions');
      trials.push(saved);
      console.log(JSON.stringify({mode, round, reused_completed_trial: true}));
    } else await trial(mode, round, Number(options.seconds));
  }
}
if (options['baseline-only'] === 'true') {
  console.log(JSON.stringify({baseline_complete: true, trial_count: trials.length, next: 'Repeat with --resume-trials to run the separate soak and assemble evidence.'}));
  process.exit(0);
}
await trial('web3d', 1, Number(options['soak-seconds']), true);
const groups = Object.fromEntries(['image', 'web3d'].map(mode => {
  const records = trials.filter(r => r.mode === mode && !r.soak);
  return [mode, Object.fromEntries(['navigation_to_ready_ms', 'frames_per_sec', 'unique_pose_or_image_per_sec',
    'response_body_bytes_per_sec', 'initial_manifest_asset_bytes'].map(key => [key, summary(records.map(r => r[key]))]))];
}));
const hashes = {};
for (const name of ['viewer_display_benchmark.py', 'viewer_display_benchmark.mjs', 'viewer_display_benchmark_page.mjs'])
  hashes[name] = createHash('sha256').update(await readFile(new URL(name, import.meta.url))).digest('hex');
const result = {schema_version: 1, benchmark: 'moving-recorded-display-v1', measured_at: new Date().toISOString(),
  status: 'completed', model_sha256: trials[0].snapshot.config.model_sha256,
  recording_sha256: trials[0].snapshot.config.record_sha256,
  conditions: {width: Number(options.width), height: Number(options.height), dpr: 1, poll_ms: Number(options['poll-ms']),
    repeats: Number(options.repeats), baseline_duration_sec: Number(options.seconds), soak_duration_sec: Number(options['soak-seconds']),
    camera: trials[0].snapshot.config.camera, independent_browser_and_server_each_trial: true,
    hardware_gl_required: options['hardware-gl'] === 'true',
    fresh_temporary_browser_profiles: true, prerecorded_positions_mj_forward_only: true, robot_commands: 0},
  limitations: ['Read-only display harness, not production asynchronous JPEG/ROS/control latency.',
    'JPEG renders+encodes on request; WebGL computes FK on request and renders locally. Lighting/materials differ despite matching geometry/camera/pixels.',
    'WebGL frames include redraws; JPEG rate is decode/draw plus next rAF, not verified physical paint.',
    'Wheel latency endpoints differ: WebGL CPU render submission vs JPEG decode/draw plus next rAF. Neither is GPU completion or input-to-physical-display latency.',
    'Chrome runs headless with its reported GL renderer. No physical mouse, Edge, Ubuntu input dropout, GPU time/VRAM, real robot motion or remote network is measured.',
    'CPU 100% means one core; browser/fixture descendants only, sampling excludes CPU before new processes are first observed. RSS sums duplicate shared pages.',
    'Each fresh browser/server is independent; host GPU driver/filesystem caches and background user workload are not reset.',
    'Initial visible/navigation timing starts after server model/EGL initialization and Chrome launch; it is not complete system cold-start latency.',
    'Soak is the configured bounded duration, not a multi-hour/day reliability guarantee. Source rewind test uses an explicit new owner epoch.'],
  script_sha256: hashes, groups, trials};
result.source_sha256 = Object.fromEntries(await Promise.all(['viewer.js', 'viewer_contract.mjs', 'vendor/three.module.js',
  'vendor/three.core.js', 'vendor/OrbitControls.js'].map(async name => [name, await readSHA(new URL('../src/mujoco_ros2_manipulation/operator_console/static/' + name, import.meta.url))])));
result.resumed_completed_trials = options['resume-trials'] === 'true';
await writeFile(path.join(output, 'evidence.json'), JSON.stringify(result, null, 2) + '\n', {flag: 'wx'});
console.log(path.join(output, 'evidence.json'));
