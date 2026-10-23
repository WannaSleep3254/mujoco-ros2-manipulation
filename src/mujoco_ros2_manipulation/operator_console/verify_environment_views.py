"""Verify local allowlisted environment previews in a fresh owned CPU-only console.

No simulation or motion is started: all start/attach/command/demo calls below are
required preview rejections. Design geometry evidence stays in the new ignored
VerificationReport directory; this tool does not publish or upload it.
"""
import argparse
from copy import deepcopy
import json
from pathlib import Path
import re
import signal
import socket
import subprocess
import sys
import time
from urllib.error import HTTPError, URLError
from urllib.parse import urlencode, urlsplit
from urllib.request import Request, build_opener, HTTPRedirectHandler, ProxyHandler

from ..paths import ROOT
from ..model_build.integrity import file_sha256
from ..verification.reports import VerificationReport, atomic_json, utc_now
from ..verification.runner import group_gone, stop_process
from ..workcells.config import load_workcell
from ..workcells.contracts import describe_workcell
from ..workcells.integrity import check_workcell
from ..workcells.motion_request import command
from .demo_jobs import matching_process
from .environment_sources import _canonical, _entries, _identity, _regular, _sha, _source, MAX_CATALOG_BYTES
from .scene_host import SceneSource, _manifest
from .verify_views import bounded_file, checked_pose, scene_workers
from .view_binding import binding_id

MAX_JSON = 8 * 1024 * 1024
MAX_ASSET = 48 * 1024 * 1024
MAX_TOTAL_ASSETS = 128 * 1024 * 1024


def require(value, message):
    if not value:
        raise RuntimeError(message)


class _NoRedirect(HTTPRedirectHandler):
    def redirect_request(self, request, fp, code, message, headers, newurl):
        raise HTTPError(request.full_url, code, 'Loopback verification does not follow redirects', headers, fp)


def assert_quiet(state, base_source_id):
    require(state['view']['enabled'] and state['view']['display_mode'] == 'web3d'
        and state['view']['config_source_id'] == base_source_id, 'View changed the configured control base or display mode')
    require(not state['can_submit'] and state['active_request_id'] is None and not state['history']
        and not any(state['graph'].values()) and state['positions_rad'] is None
        and state['simulation_sec'] is None and state['server'] is None and state['guard'] is None
        and not state['simulation_verified'], 'Read-only preview acquired ROS state, a command owner or request history')
    require(not state['lifecycle']['owned'] and state['lifecycle']['phase'] == 'idle'
        and state['lifecycle']['last_launch_directory'] is None and state['demo']['current'] is None,
        'Preview started or attached a simulation, command, demo or image child')


def public_view(value, private_paths):
    text = json.dumps(value, ensure_ascii=False, allow_nan=False)
    require(all(path not in text for path in private_paths), 'Public view leaked a local design path')


def failure_record(error):
    """Keep HTTP status/endpoint only, even when an assertion wrapped HTTPError."""
    result = dict(type=type(error).__name__, message='Local environment preview acceptance failed.')
    current = error
    for _ in range(5):
        if current is None:
            break
        route = getattr(current, 'verification_route', None)
        if (isinstance(current, HTTPError) and type(current.code) is int
                and isinstance(route, str) and route.startswith('/api/')
                and len(route) <= 256 and '?' not in route and '#' not in route):
            result.update(http_status=current.code, route=route)
            break
        current = current.__cause__ or current.__context__
    return result


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--catalog', type=Path, required=True, help='Explicit local startup catalog; never sent through HTTP')
    parser.add_argument('--domain', type=int, default=149)
    parser.add_argument('--port', type=int, default=8895)
    parser.add_argument('--workcell', default='fr5_ur5e')
    args = parser.parse_args(argv)
    if not 0 <= args.domain <= 232 or not 1024 <= args.port <= 65535:
        parser.error('Invalid isolated ROS domain or loopback port')
    report = VerificationReport(ROOT, 'environment_views', 'cpu_preview', args.domain)
    process = session = None
    token = None
    failed = False
    cleanup, workers, checks = [], {}, []
    url = f'http://127.0.0.1:{args.port}'
    opener = build_opener(ProxyHandler({}), _NoRedirect())
    base_config = None
    private_paths = set()
    models, frozen = {}, {}

    def http(route, body=None, maximum=MAX_JSON):
        request = Request(url + route) if body is None else Request(url + route,
            data=json.dumps(body, allow_nan=False).encode(), headers={'Content-Type': 'application/json',
                'Origin': url, 'X-Operator-Token': token})
        try:
            with opener.open(request, timeout=5) as response:
                raw = response.read(maximum + 1)
                require(len(raw) <= maximum, 'Loopback response exceeded its byte bound')
                return response.status, response.headers.get_content_type(), raw
        except HTTPError as error:
            error.verification_route = urlsplit(route).path  # No query, token, response body or local path.
            raise

    def api(route, body=None):
        status, mime, raw = http(route, body)
        require(status in (200, 202) and mime == 'application/json', 'Unexpected console JSON response')
        return json.loads(raw)

    def until(predicate, timeout, label):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            require(process is not None and process.poll() is None, 'Owned console exited before ' + label)
            try:
                result = predicate()
                if result:
                    return result
            except HTTPError:
                raise
            except (URLError, TimeoutError):
                pass
            time.sleep(.05)
        raise RuntimeError('Timed out: ' + label)

    def reject(route, body, codes, *, preview=False):
        try:
            api(route, body)
        except HTTPError as error:
            require(error.code in codes, 'Unexpected HTTP rejection status')
            if preview:
                raw = error.read(65537)
                require(len(raw) <= 65536 and '미리보기' in json.loads(raw).get('error', ''),
                    'Mutation was rejected for another reason instead of preview admission')
        else:
            raise RuntimeError('Unexpectedly accepted read-only preview mutation')

    def retired(identifier):
        # Only requests for an explicitly retired binding may race as 400/409.
        for endpoint in ('manifest', 'pose'):
            try:
                http('/api/view/' + endpoint + '?' + urlencode({'binding_id': identifier}))
            except HTTPError as error:
                require(error.code in (400, 409), 'Retired binding failed for another reason')
            else:
                raise RuntimeError('A retired binding still returned a scene or pose')

    def observe_workers():
        current = scene_workers(process.pid, session)
        for value in current:
            workers[(value['pid'], value['start_ticks'])] = value
        return current

    def events():
        return [json.loads(line) for line in bounded_file(session / 'events.jsonl', 16 * 1024 * 1024).splitlines()]

    def no_image(state):
        assert_quiet(state, base_id)
        require(not (session / 'frame.jpg').exists() and not (session / 'frame.json').exists(),
            'Read-only CPU preview generated a JPEG')
        require(not any(row.get('action') == 'process_started' for row in events()),
            'A preview started a simulation, command, demo or image subprocess')
        public_view(state['view'], private_paths)

    def same_base():
        current = api('/api/bootstrap')['config']
        require(current == base_config, 'Environment selection modified base configuration, targets or control contract')
        return current

    def ready(source_id, mode):
        def selected():
            state = api('/api/state')
            no_image(state)
            view = state['view']
            require(not view.get('error') and view.get('status') != 'error', 'CPU scene host reported an error')
            return state if (view.get('status') == 'ready' and view.get('manifest_descriptor')
                and view.get('binding', {}).get('source_id') == source_id and view.get('mode') == mode) else None
        state = until(selected, 45, 'matching CPU preview scene')
        view = state['view']
        identifier, binding, descriptor = view['binding_id'], view['binding'], view['manifest_descriptor']
        require(binding_id(binding) == identifier and binding['operator_session_id'] == state['lifecycle']['session_id']
            and binding['mode'] == mode and binding['source_id'] == source_id,
            'Scene binding belongs to another console, mode or source')
        require(binding['job_id'] is None and binding['run_id'] is None and binding['motion_session_id'] is None
            and binding['request_id'] is None, 'Read-only scene acquired a robot execution identity')
        source = models[source_id]
        status, mime, raw = http('/api/view/manifest?' + urlencode({'binding_id': identifier}))
        require(status == 200 and mime == 'application/json' and _sha(raw) == descriptor['sha256']
            and len(raw) == descriptor['byte_length'], 'Current manifest bytes differ from their descriptor')
        manifest, catalog = _manifest(raw, source, file_sha256(source.xml_path))
        require(all(descriptor[key] == manifest[key] for key in ('model_sha256', 'scene_sha256', 'compiled_model_sha256')),
            'Compiled manifest identity differs from the current scene descriptor')
        public_view(manifest, private_paths)
        total, assets = 0, []
        for sha, item in catalog.items():
            status, mime, blob = http('/api/view/assets/' + sha, maximum=MAX_ASSET)
            require(status == 200 and mime == 'application/octet-stream' and _sha(blob) == sha
                and len(blob) == item['byte_length'], 'Compiled asset bytes differ from the declared SHA/size')
            total += len(blob)
            require(total <= MAX_TOTAL_ASSETS, 'Compiled scene assets exceed their acceptance byte budget')
            assets.append(dict(sha256=sha, bytes=len(blob)))
        current = observe_workers()
        require(len(current) == 1, 'Expected one current direct owned CPU scene worker')
        folder = report.directory / 'private-design-evidence' / f'{len(checks):02d}'
        folder.mkdir(parents=True)
        (folder / 'manifest.json').write_bytes(raw)
        record = dict(source_id=source_id, mode=mode, binding_id=identifier,
            model_sha256=manifest['model_sha256'], scene_sha256=manifest['scene_sha256'],
            compiled_model_sha256=manifest['compiled_model_sha256'], manifest_sha256=_sha(raw),
            geoms=len(manifest['geoms']), asset_bytes=total, assets=assets)
        checks.append(record)
        report.update(view_checks=checks)
        return state, manifest, folder

    def home_pose(state, manifest):
        identifier, binding = state['view']['binding_id'], state['view']['binding']
        def received():
            status, mime, raw = http('/api/view/pose?' + urlencode({'binding_id': identifier}))
            if status == 204:
                require(not raw, '204 CPU pose carried a response body')
                return None
            require(status == 200 and mime == 'application/json', 'Unexpected current CPU pose status')
            pose = checked_pose(json.loads(raw), manifest, binding)
            require(pose['simulation_sec'] is None, 'Model preview invented a simulation clock')
            public_view(pose, private_paths)
            return pose
        pose = until(received, 15, 'complete home preview pose')
        owned = observe_workers()
        require(len(owned) == 1, 'Lost the current owned CPU scene worker')
        directory = Path(owned[0]['directory'])
        value = json.loads(bounded_file(directory / 'input.json', 256 * 1024))
        raw_pose = json.loads(bounded_file(directory / 'pose.json', MAX_JSON))
        require(value == dict(sequence=pose['sequence'], positions=manifest['home_rad'])
            and set(value['positions']) == set(manifest['joint_names'])
            and raw_pose['sequence'] == pose['sequence'] and raw_pose['pose'] == pose['pose'],
            'HTTP home pose differs from the complete named home input or owned CPU output')
        return pose

    try:
        report.update(code_commit=subprocess.check_output(['git', '-C', str(ROOT), 'rev-parse', 'HEAD'], text=True).strip(),
            cpu_preview_requested=True, ros_overlay_execution_requested=False, jpeg_requested=False,
            evidence_scope='Ignored local design evidence; no publication or upload')
        report.stage('unused_domain_port_and_immutable_catalog_preflight')
        from ..workcells.verify_motion_reproduction import require_unused_domain
        require_unused_domain(args.domain)
        with socket.socket() as probe:
            probe.bind(('127.0.0.1', args.port))
        cell = load_workcell(args.workcell)
        metadata = check_workcell(cell)
        contract = describe_workcell(cell, metadata)
        base_id = 'workcell:' + cell.id
        models[base_id] = SceneSource(base_id, cell.label, cell.output / 'cell.xml', tuple(cell.joints),
            dict(zip(cell.joints, cell.home)), lambda: check_workcell(cell))
        catalog = args.catalog.absolute()
        catalog_sha = _sha(_regular(catalog, MAX_CATALOG_BYTES))
        private_paths.add(str(catalog))
        overlay_ids = []
        for directory, sha in _entries(catalog):
            before = _identity(directory, cell, sha)
            source = _source(directory, sha, before, cell)
            require(source.id not in models, 'Duplicate declared environment source ID')
            models[source.id] = source
            frozen[source.id] = (directory, sha, deepcopy(before))
            private_paths.add(str(directory))
            private_paths.update(before['dependencies'])
            overlay_ids.append(source.id)
        require(_sha(_regular(catalog, MAX_CATALOG_BYTES)) == catalog_sha, 'Local catalog changed during preflight')
        report.update(catalog_sha256=catalog_sha, workcell_contract=contract,
            overlay_sources=[dict(source_id=identifier, manifest_sha256=frozen[identifier][1],
                fingerprint=frozen[identifier][2]['fingerprint']) for identifier in overlay_ids])
        report.stage('owned_console_startup_catalog_cpu_checks')
        with report.log_path.open('w') as log:
            process = subprocess.Popen([sys.executable, '-m', 'mujoco_ros2_manipulation.operator_console', '--workcell', cell.id,
                '--domain', str(args.domain), '--port', str(args.port), '--view-scene-catalog', str(catalog)],
                cwd=ROOT, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        def owned_session():
            text = bounded_file(report.log_path, 16 * 1024 * 1024).decode(errors='replace')[-65536:]
            match = re.search(r'OPERATOR_READY http://127\.0\.0\.1:' + str(args.port)
                + r'/ domain=' + str(args.domain) + r' session=([^\n]+)', text)
            if match:
                directory = Path(match.group(1)).resolve()
                require(directory.is_relative_to((ROOT / 'runtime/operator').resolve()), 'Owned console directory is outside runtime')
                return directory
        # The catalog may contain eight sequential 30-second CPU preflights.
        session = until(owned_session, 30 * len(overlay_ids) + 30, 'owned console port reservation and catalog checks')
        require(_sha(_regular(catalog, MAX_CATALOG_BYTES)) == catalog_sha, 'Startup catalog changed before READY')
        bootstrap = api('/api/bootstrap')
        token = bootstrap['token']
        base_config = deepcopy(bootstrap['config'])
        require(base_config['domain'] == args.domain and base_config['workcell'] == cell.id
            and base_config['workcell_contract'] == contract and base_config['home_rad'] == models[base_id].home
            and base_config['targets']['home'] == models[base_id].home, 'Bootstrap changed the requested control base')
        initial, _, _ = ready(base_id, 'live')
        catalog_ids = {item['id'] for item in initial['view']['catalog']}
        require({name for name in catalog_ids if name.startswith('scene:')} == set(overlay_ids),
            'Declared environment catalog is incomplete or contains undeclared scenes')
        old = initial['view']['binding_id']
        status, _, raw = http('/api/view/pose?' + urlencode({'binding_id': old}))
        require(status == 204 and not raw, 'Initial live view fabricated ROS observations')
        report.update(base_config_sha256=_sha(_canonical(base_config)), initial_live_without_ros_checked=True)

        for identifier in overlay_ids:
            report.stage('model_preview_' + identifier.removeprefix('scene:'))
            directory, sha, before = frozen[identifier]
            require(_identity(directory, cell, sha) == before, 'Selected overlay changed after startup')
            api('/api/view/select', dict(mode='model_preview', source_id=identifier))
            selected, manifest, folder = ready(identifier, 'model_preview')
            require(selected['view']['binding_id'] != old, 'Environment preview reused the previous view epoch')
            retired(old)
            pose = home_pose(selected, manifest)
            atomic_json(folder / 'home-pose.json', pose)
            same_base()
            binding = selected['view']['binding_id']
            reject('/api/view/select', dict(mode='live', source_id=identifier), (400,))
            require(api('/api/state')['view']['binding_id'] == binding, 'Rejected overlay live selection changed the epoch')
            for route, payload in (('/api/command', command(base_config['home_rad'])),
                    ('/api/start', dict(native_gui=False, planning_case='none')), ('/api/attach', {}),
                    ('/api/demos/start', dict(scenario='contact_comparison', native_gui=False))):
                reject(route, payload, (409,), preview=True)
            after = api('/api/state')
            no_image(after)
            require(after['view']['binding_id'] == binding and after['view']['preview_active'],
                'Rejected preview mutation changed the selected source or epoch')
            require(_identity(directory, cell, sha) == before, 'Viewing mutated immutable environment artifacts')
            same_base()
            checks[-1].update(home_input_checked=True, preview_mutations_rejected=True,
                live_overlay_rejected=True, control_base_unchanged=True)
            old = binding

        report.stage('restore_base_live_without_overlay_execution')
        api('/api/view/select', dict(mode='live', source_id=base_id))
        restored, _, _ = ready(base_id, 'live')
        require(restored['view']['binding_id'] != old and not restored['view']['preview_active'],
            'Restoring the configured base reused an overlay epoch')
        retired(old)
        status, _, raw = http('/api/view/pose?' + urlencode({'binding_id': restored['view']['binding_id']}))
        require(status == 204 and not raw, 'Restored base live view fabricated a home pose without ROS')
        same_base()
        require(check_workcell(cell) == metadata, 'Environment preview mutated the original base model')
        report.update(environment_previews_checked=True, preview_admission_checked=True,
            control_base_unchanged=True, no_ros_execution_checked=True, no_jpeg_renderer_checked=True,
            restored_base_live_checked=True, view_checks=checks)
    except (Exception, KeyboardInterrupt) as error:
        failed = True
        report.update(error=failure_record(error), failure_stage=report.data['stage'])
    finally:
        token = None
        if process is not None:
            if session is not None:
                try:
                    observe_workers()
                except Exception as error:
                    cleanup.append('worker_observation:' + type(error).__name__)
            try:
                if process.poll() is None:
                    process.send_signal(signal.SIGINT)
                deadline = time.monotonic() + 45
                while process.poll() is None and time.monotonic() < deadline:
                    if session is not None:
                        observe_workers()
                    time.sleep(.05)
                process.wait(timeout=max(.1, deadline - time.monotonic()))
                require(process.returncode == 0 and group_gone(process.pid), 'Owned console shutdown was not clean')
            except Exception as error:
                cleanup.append('console_shutdown:' + type(error).__name__)
                try:
                    stop_process(process)
                except Exception as fallback:
                    cleanup.append('owned_console_fallback:' + type(fallback).__name__)
        if session is not None:
            try:
                deadline = time.monotonic() + 5
                while any(matching_process(row) for row in workers.values()) and time.monotonic() < deadline:
                    time.sleep(.05)
                require(all(not matching_process(row) and group_gone(row['pid']) for row in workers.values()),
                    'A proven owned CPU scene worker remains alive')
                rows = events()
                require(not any(row.get('action') == 'process_started' or row.get('cleanup_errors') for row in rows),
                    'Preview started control children or recorded incomplete cleanup')
            except Exception as error:
                cleanup.append('owned_worker_cleanup:' + type(error).__name__)
        report.update(status='failed' if failed or cleanup else 'passed', stage='complete', completed_utc=utc_now(),
            cleanup_errors=cleanup, shutdown_clean=not cleanup,
            process_group_gone=process is not None and group_gone(process.pid),
            launch_return_code=None if process is None else process.returncode,
            owned_scene_workers=[{key: value for key, value in row.items() if key != 'directory'} for row in workers.values()])
    print(report.data['status'].upper() + ': ' + str(report.path), flush=True)
    return int(failed or bool(cleanup))


if __name__ == '__main__':
    raise SystemExit(main())
