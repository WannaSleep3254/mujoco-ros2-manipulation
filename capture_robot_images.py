"""Render model reference images and capture owned MuJoCo/RViz X11 windows."""

import argparse
import ctypes
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import re
import struct
import subprocess
import time

from robot_config import ROOT, check_generated_model, list_profiles, load_profile


class XButtonEvent(ctypes.Structure):
    _fields_ = [('type', ctypes.c_int), ('serial', ctypes.c_ulong),
                ('send_event', ctypes.c_int), ('display', ctypes.c_void_p),
                ('window', ctypes.c_ulong), ('root', ctypes.c_ulong),
                ('subwindow', ctypes.c_ulong), ('time', ctypes.c_ulong),
                ('x', ctypes.c_int), ('y', ctypes.c_int),
                ('x_root', ctypes.c_int), ('y_root', ctypes.c_int),
                ('state', ctypes.c_uint), ('button', ctypes.c_uint), ('same_screen', ctypes.c_int)]


class XClientData(ctypes.Union):
    _fields_ = [('longs', ctypes.c_long * 5), ('bytes', ctypes.c_char * 20)]


class XClientMessage(ctypes.Structure):
    _fields_ = [('type', ctypes.c_int), ('serial', ctypes.c_ulong),
                ('send_event', ctypes.c_int), ('display', ctypes.c_void_p),
                ('window', ctypes.c_ulong), ('message_type', ctypes.c_ulong),
                ('format', ctypes.c_int), ('data', XClientData)]


class XEvent(ctypes.Union):
    _fields_ = [('button', XButtonEvent), ('client', XClientMessage),
                ('padding', ctypes.c_long * 24)]


def save_window_image(window, destination):
    """Capture a visible client window and encode its unchanged pixels as PNG."""
    from PIL import Image

    dump = subprocess.check_output(['xwd', '-silent', '-nobdrs', '-id', window], timeout=10)
    header = struct.unpack('>25I', dump[:100])
    if header[1:3] != (7, 2) or header[14:17] != (0xff0000, 0xff00, 0xff):
        raise RuntimeError('Unsupported XWD format; expected a TrueColor ZPixmap')
    width, height, bits, stride = header[4], header[5], header[11], header[12]
    modes = {(0, 32): 'BGRX', (1, 32): 'XRGB', (0, 24): 'BGR', (1, 24): 'RGB'}
    mode = modes.get((header[7], bits))
    if mode is None:
        raise RuntimeError(f'Unsupported XWD pixel format: {bits} bits')
    offset = header[0] + header[19] * 12
    pixels = dump[offset:offset + stride * height]
    image = Image.frombytes('RGB', (width, height), pixels, 'raw', mode, stride, 1)
    image.save(destination)
    return [width, height]


def capture_gui_windows(profile, process_group, directory):
    """Capture only processes started by the current verifier's launch group."""
    directory = Path(directory).resolve()
    if not directory.is_relative_to(ROOT):
        raise ValueError('Screenshot output must be inside the project')
    directory.mkdir(parents=True, exist_ok=True)
    processes = {}
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit():
            continue
        try:
            status = (entry / 'stat').read_text().rsplit(')', 1)[1].split()
            if int(status[2]) != process_group:
                continue
            executable = (entry / 'cmdline').read_bytes().split(b'\0')[0].decode()
            name = Path(executable).name
            if name in ['ros2_control_node', 'rviz2']:
                if name in processes:
                    raise RuntimeError(f'Multiple {name} processes in the capture group')
                processes[name] = int(entry.name)
        except (FileNotFoundError, PermissionError, ProcessLookupError):
            continue
    if set(processes) != {'ros2_control_node', 'rviz2'}:
        raise RuntimeError(f'Missing GUI processes in the launch group: {sorted(processes)}')
    windows = {}
    deadline = time.monotonic() + 10
    while time.monotonic() < deadline:
        clients = subprocess.check_output(['xprop', '-root', '_NET_CLIENT_LIST'], text=True, timeout=5)
        for window in re.findall(r'0x[0-9a-fA-F]+', clients):
            pid = subprocess.check_output(['xprop', '-id', window, '_NET_WM_PID'], text=True, timeout=5)
            match = re.search(r'=\s*(\d+)', pid)
            if match:
                for name, owned_pid in processes.items():
                    if int(match.group(1)) == owned_pid:
                        windows[name] = window
        if set(windows) == set(processes):
            break
        time.sleep(0.2)
    else:
        raise RuntimeError('MuJoCo and RViz X11 windows did not become available')

    x11 = ctypes.CDLL('libX11.so.6')
    x11.XOpenDisplay.argtypes = [ctypes.c_char_p]
    x11.XOpenDisplay.restype = ctypes.c_void_p
    x11.XMoveResizeWindow.argtypes = [ctypes.c_void_p, ctypes.c_ulong,
                                     ctypes.c_int, ctypes.c_int, ctypes.c_uint, ctypes.c_uint]
    x11.XRaiseWindow.argtypes = [ctypes.c_void_p, ctypes.c_ulong]
    x11.XMapRaised.argtypes = [ctypes.c_void_p, ctypes.c_ulong]
    x11.XDefaultScreen.argtypes = [ctypes.c_void_p]
    x11.XDefaultScreen.restype = ctypes.c_int
    x11.XIconifyWindow.argtypes = [ctypes.c_void_p, ctypes.c_ulong, ctypes.c_int]
    x11.XInternAtom.argtypes = [ctypes.c_void_p, ctypes.c_char_p, ctypes.c_int]
    x11.XInternAtom.restype = ctypes.c_ulong
    x11.XSync.argtypes = [ctypes.c_void_p, ctypes.c_int]
    x11.XSetErrorHandler.argtypes = [ctypes.c_void_p]
    x11.XSetErrorHandler.restype = ctypes.c_void_p
    x11.XFlush.argtypes = [ctypes.c_void_p]
    x11.XCloseDisplay.argtypes = [ctypes.c_void_p]
    x11.XDefaultRootWindow.argtypes = [ctypes.c_void_p]
    x11.XDefaultRootWindow.restype = ctypes.c_ulong
    x11.XWarpPointer.argtypes = [ctypes.c_void_p, ctypes.c_ulong, ctypes.c_ulong,
                                ctypes.c_int, ctypes.c_int, ctypes.c_uint, ctypes.c_uint,
                                ctypes.c_int, ctypes.c_int]
    x11.XSendEvent.argtypes = [ctypes.c_void_p, ctypes.c_ulong, ctypes.c_int,
                              ctypes.c_long, ctypes.POINTER(XEvent)]
    x11.XQueryPointer.argtypes = [ctypes.c_void_p, ctypes.c_ulong,
                                 ctypes.POINTER(ctypes.c_ulong), ctypes.POINTER(ctypes.c_ulong),
                                 ctypes.POINTER(ctypes.c_int), ctypes.POINTER(ctypes.c_int),
                                 ctypes.POINTER(ctypes.c_int), ctypes.POINTER(ctypes.c_int),
                                 ctypes.POINTER(ctypes.c_uint)]
    display = x11.XOpenDisplay(None)
    if not display:
        raise RuntimeError('Cannot open the X11 desktop display')
    errors = []
    error_callback_type = ctypes.CFUNCTYPE(ctypes.c_int, ctypes.c_void_p, ctypes.c_void_p)
    @error_callback_type
    def record_x_error(_display, _error):
        errors.append('X11 window request failed')
        return 0
    old_error_handler = x11.XSetErrorHandler(ctypes.cast(record_x_error, ctypes.c_void_p))
    root = x11.XDefaultRootWindow(display)
    active_atom = x11.XInternAtom(display, b'_NET_ACTIVE_WINDOW', 0)
    root_return, child_return = ctypes.c_ulong(), ctypes.c_ulong()
    pointer_x, pointer_y, local_x, local_y = (ctypes.c_int() for _ in range(4))
    mask = ctypes.c_uint()
    pointer_saved = x11.XQueryPointer(display, root, ctypes.byref(root_return),
        ctypes.byref(child_return), ctypes.byref(pointer_x), ctypes.byref(pointer_y),
        ctypes.byref(local_x), ctypes.byref(local_y), ctypes.byref(mask))
    screenshots = []
    try:
        # Keep this run's RViz from obscuring the native simulation capture.
        x11.XIconifyWindow(display, int(windows['rviz2'], 16), x11.XDefaultScreen(display))
        x11.XFlush(display)
        time.sleep(0.5)
        for name, filename in [('ros2_control_node', 'mujoco_gui.png'), ('rviz2', 'rviz.png')]:
            window = windows[name]
            # Resize and raise only this verification run's windows.
            x11.XMoveResizeWindow(display, int(window, 16), 40, 60, 1850, 950)
            x11.XMapRaised(display, int(window, 16))
            active = XEvent()
            active.client = XClientMessage(type=33, send_event=1, display=display,
                window=int(window, 16), message_type=active_atom, format=32)
            active.client.data.longs[0] = 2  # EWMH pager source permits activation.
            x11.XSendEvent(display, root, 0, (1 << 19) | (1 << 20), ctypes.byref(active))
            x11.XFlush(display)
            time.sleep(1.0)
            zoom_steps = 0
            if name == 'ros2_control_node':
                # MuJoCo's wheel-up zooms out; it changes only the viewer camera.
                zoom_steps = 8 if profile.id == 'fr5' else 0
                x11.XWarpPointer(display, 0, int(window, 16), 0, 0, 0, 0, 925, 475)
                x11.XFlush(display)
                time.sleep(0.1)
                for _ in range(zoom_steps):
                    event = XEvent()
                    event.button = XButtonEvent(type=4, send_event=1, display=display,
                        window=int(window, 16), root=root, x=925, y=475, button=4, same_screen=1)
                    x11.XSendEvent(display, int(window, 16), 0, 1 << 2, ctypes.byref(event))
                    event.button.type = 5
                    x11.XSendEvent(display, int(window, 16), 0, 1 << 3, ctypes.byref(event))
                    x11.XFlush(display)
                    time.sleep(0.03)
                time.sleep(0.3)
            x11.XSync(display, 0)
            if errors:
                raise RuntimeError(errors[0])
            path = directory / filename
            size = save_window_image(window, path)
            screenshots.append({'file': str(path.relative_to(ROOT)), 'size_px': size,
                                'application': name, 'kind': 'X11 client-window screenshot',
                                'sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
                                'camera_wheel_up_steps': zoom_steps})
    finally:
        x11.XMapRaised(display, int(windows['rviz2'], 16))
        if pointer_saved:
            x11.XWarpPointer(display, 0, root, 0, 0, 0, 0, pointer_x.value, pointer_y.value)
            x11.XFlush(display)
        x11.XCloseDisplay(display)
        x11.XSetErrorHandler(old_error_handler)
    return {'captured_utc': datetime.now(timezone.utc).isoformat(),
            'robot_id': profile.id, 'robot_profile_sha256': profile.fingerprint,
            'pose': 'After verified MoveIt trajectory execution', 'images': screenshots}


def render_reference(profile, directory):
    """Render the configured keyframe; this image is not a ROS GUI screenshot."""
    os.environ.setdefault('MUJOCO_GL', 'egl')
    import mujoco
    import numpy as np
    from PIL import Image

    check_generated_model(profile)
    directory.mkdir(parents=True, exist_ok=True)
    model = mujoco.MjModel.from_xml_path(str(profile.scene_path))
    model.vis.global_.offwidth = 1000
    model.vis.global_.offheight = 750
    state = mujoco.MjData(model)
    mujoco.mj_resetDataKeyframe(model, state, 0)
    mujoco.mj_forward(model, state)
    camera = mujoco.MjvCamera()
    vertices = []
    for geom_id in range(model.ngeom):
        if model.geom_group[geom_id] != 2 or model.geom_type[geom_id] != mujoco.mjtGeom.mjGEOM_MESH:
            continue
        mesh_id = model.geom_dataid[geom_id]
        start, count = model.mesh_vertadr[mesh_id], model.mesh_vertnum[mesh_id]
        points = model.mesh_vert[start:start+count]
        vertices.append(points @ state.geom_xmat[geom_id].reshape(3, 3).T + state.geom_xpos[geom_id])
    points = np.vstack(vertices)
    minimum, maximum = points.min(0), points.max(0)
    camera.lookat[:] = (minimum + maximum) / 2
    camera.distance = np.linalg.norm(maximum - minimum) * 1.8
    camera.azimuth = 135
    camera.elevation = -25
    path = directory / 'home.png'
    with mujoco.Renderer(model, height=750, width=1000) as renderer:
        renderer.update_scene(state, camera)
        renderer.scene.flags[mujoco.mjtRndFlag.mjRND_SHADOW] = False
        Image.fromarray(renderer.render()).save(path)
    metadata = {'captured_utc': datetime.now(timezone.utc).isoformat(),
                'robot_id': profile.id, 'robot_profile_sha256': profile.fingerprint,
                'kind': 'MuJoCo EGL offscreen reference rendering',
                'mujoco_python_version': mujoco.__version__,
                'file': str(path.relative_to(ROOT)), 'size_px': [1000, 750],
                'sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
                'joint_names': profile.joints, 'joint_positions_rad': profile.home,
                'pose': 'Configured home keyframe; no physics settling or ROS nodes',
                'shadows_enabled': False,
                'camera': {'lookat_m': camera.lookat.tolist(), 'distance_m': camera.distance,
                           'azimuth_deg': camera.azimuth, 'elevation_deg': camera.elevation}}
    (directory.parent / 'reference_image.json').write_text(json.dumps(metadata, indent=2) + '\n')
    print(f'Rendered {profile.id}: {path.relative_to(ROOT)}', flush=True)


def collect_assets(profile):
    """Freeze a successful capture report and its matching image provenance."""
    from PIL import Image

    directory = ROOT / 'docs' / 'robots' / profile.id
    report = json.loads((ROOT / 'runtime' / 'validation' / f'{profile.id}_gui.json').read_text())
    if not all(report.get(key) is True for key in ['motion_checks_passed', 'shutdown_clean', 'process_group_gone']):
        raise ValueError(f'{profile.id}: capture verification did not pass')
    if report.get('launch_return_code') != 0 or report['robot_profile_sha256'] != profile.fingerprint:
        raise ValueError(f'{profile.id}: capture has an unsuccessful exit or a stale profile')
    reference = json.loads((directory / 'reference_image.json').read_text())
    captured = report['screenshots']
    if any(record['robot_profile_sha256'] != profile.fingerprint for record in [reference, captured]):
        raise ValueError(f'{profile.id}: image provenance uses a stale profile')
    images = [dict(reference, provenance='reference_image.json')]
    for record in captured['images']:
        images.append(dict(record, captured_utc=captured['captured_utc'], pose=captured['pose'],
                           provenance='gui_capture_validation.json'))
    if {Path(record['file']).name for record in images} != {'home.png', 'mujoco_gui.png', 'rviz.png'}:
        raise ValueError(f'{profile.id}: expected the three model images')
    for record in images:
        path = profile.path(record['file'])
        if hashlib.sha256(path.read_bytes()).hexdigest() != record['sha256']:
            raise ValueError(f'{profile.id}: image differs from its capture provenance: {path.name}')
        with Image.open(path) as image:
            if list(image.size) != record['size_px']:
                raise ValueError(f'{profile.id}: image size differs from its capture provenance')
            image.verify()
        record['file'] = str(path.relative_to(directory))
        record['bytes'] = path.stat().st_size
    (directory / 'gui_capture_validation.json').write_text(json.dumps(report, indent=2) + '\n')
    manifest = {'schema_version': 1, 'robot_id': profile.id, 'robot': profile.label,
                'robot_profile_sha256': profile.fingerprint, 'images': images,
                'manufacturer_source_repository': profile.data['source']['repository_url'],
                'manufacturer_source_commit': profile.data['source']['commit'],
                'capture_script_sha256': hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
                'verification_report': 'gui_capture_validation.json'}
    (directory / 'assets.json').write_text(json.dumps(manifest, indent=2) + '\n')
    print(f'Collected {profile.id}: 3 images and successful GUI verification', flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--robot', choices=['all'] + [profile.id for profile in list_profiles()],
                        default='all')
    parser.add_argument('--collect', action='store_true', help='collect successful GUI reports and image manifests')
    args = parser.parse_args()
    profiles = list_profiles() if args.robot == 'all' else [load_profile(args.robot)]
    for profile in profiles:
        if args.collect:
            collect_assets(profile)
        else:
            render_reference(profile, ROOT / 'docs' / 'robots' / profile.id / 'images')


if __name__ == '__main__':
    main()
