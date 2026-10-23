"""Owned native 3.12 compile/FK check for installed local environment geometry."""
import json
import math
import os
from pathlib import Path
import signal
import subprocess

from .environment_assets import bbox, rpy_matrix, sha, stl_vertices, transformed, require, validate_bundle
from .environment_overlay import check_overlay_identity


def _owned(argv, *, timeout, stdout, stderr):
    process = subprocess.Popen(argv, stdin=subprocess.DEVNULL, stdout=stdout, stderr=stderr, start_new_session=True)
    try:
        code = process.wait(timeout=timeout)
    except BaseException:
        # Only the newly owned, unreaped process group can be stopped here.
        try:
            if process.poll() is None:
                try: os.killpg(process.pid, signal.SIGTERM)
                except ProcessLookupError: pass
                try: process.wait(timeout=3)
                except BaseException:
                    try: os.killpg(process.pid, signal.SIGKILL)
                    except ProcessLookupError: pass
                    process.wait(timeout=3)
        except BaseException:
            pass  # Preserve the triggering timeout/interrupt.
        raise
    if code:
        raise subprocess.CalledProcessError(code, argv)


def check_native_overlay(directory, base_cell, report, work_directory):
    """No physics steps or arm/environment path-clearance claims are made here."""
    directory = Path(directory).resolve()
    work = Path(work_directory).resolve()
    require(work.is_dir() and not list(work.iterdir()), 'Native check requires a new empty owned directory')
    require(check_overlay_identity(directory, base_cell) == report, 'Overlay changed before native check')
    vendor = base_cell.root / 'runtime/root/opt/ros/humble/opt/mujoco_vendor'
    library = vendor / 'lib/libmujoco.so.3.12.0'
    source = base_cell.root / 'native/environment_geometry_probe.cpp'
    header = vendor / 'include/mujoco/mujoco.h'
    require(source.is_file() and library.is_file() and header.is_file(), 'Pinned native MuJoCo runtime is unavailable')
    source_hash, library_hash, header_hash = [sha(path.read_bytes()) for path in (source, library, header)]
    executable = work / 'geometry-probe'
    compile_args = ['c++', '-std=c++17', '-O2', '-Wall', '-Wextra', '-Werror', str(source),
        '-I' + str(vendor / 'include'), str(library), '-Wl,-rpath,' + str(vendor / 'lib'), '-o', str(executable)]
    with (work / 'compile.log').open('wb') as log:
        _owned(compile_args, timeout=60, stdout=log, stderr=subprocess.STDOUT)
    state_path = work / 'home.txt'
    state_path.write_text(str(len(base_cell.joints)) + '\n' + ''.join(
        f'{name} {value:.17g}\n' for name, value in zip(base_cell.joints, base_cell.home)))
    result_path = work / 'geometry.json'
    with result_path.open('wb') as output, (work / 'geometry.log').open('wb') as log:
        _owned([str(executable), str(directory / 'cell.xml'), str(state_path)],
            timeout=30, stdout=output, stderr=log)
    require(result_path.stat().st_size <= 128 * 1024, 'Native geometry receipt is too large')
    measured = json.loads(result_path.read_text())
    require(measured['engine'] == 'MuJoCo 3.12.0' and measured['physics_steps'] == 0
        and measured['home_contacts'] == 0, 'Native home geometry is invalid')
    geoms = {geom['name']: geom for geom in measured['geoms']}
    require(len(geoms) == 2 * len(report['installations']) == len(measured['geoms']), 'Native environment geometry coverage differs')
    error = 0.
    def compare(left, right):
        nonlocal error
        require(len(left) == len(right), 'Native geometry vector size differs')
        require(all(type(value) in (int, float) and math.isfinite(value) for value in left + right),
            'Native geometry values must be finite numbers')
        error = max(error, *(abs(a-b) for a,b in zip(left,right)))
    for entry in report['installations']:
        name = 'environment_' + entry['installation']['name']
        visual, collision = geoms[name + '_visual'], geoms[name + '_collision']
        require(visual['type'] == 7 and visual['group'] == 0 and visual['contype'] == visual['conaffinity'] == 0
            and collision['type'] == 6 and collision['group'] == 3 and collision['contype'] == collision['conaffinity'] == 1,
            'Native visual/collision roles differ')
        bundle = directory / entry['bundle_path']
        asset = validate_bundle(bundle, expected_sha256=entry['manifest_sha256'])
        matrix, offset = rpy_matrix(entry['installation']['rpy_rad']), entry['installation']['xyz_m']
        low, high = bbox([transformed(point, matrix, offset) for point in stl_vertices((bundle / 'visual.stl').read_bytes())])
        compare(visual['world_bbox_min_m'] + visual['world_bbox_max_m'], low + high)
        compare(collision['position'], transformed(asset['collision']['center_m'], matrix, offset))
        compare(collision['half_size'], [v / 2 for v in asset['collision']['size_m']])
        compare(collision['rotation'], matrix)
    require(error <= 1e-6, 'Native installed geometry differs from the checked metre asset/URDF')
    require([sha(path.read_bytes()) for path in (source, library, header)] == [source_hash, library_hash, header_hash]
        and check_overlay_identity(directory, base_cell) == report, 'Native inputs changed during verification')
    return dict(engine=measured['engine'], physics_steps=0, home_contacts=0,
        installed_geometry_error_m=error, source_sha256=source_hash, library_sha256=library_hash,
        header_sha256=header_hash, executable_sha256=sha(executable.read_bytes()),
        scope='Installed visual bounds and conservative box transforms at home; not environment path clearance')
