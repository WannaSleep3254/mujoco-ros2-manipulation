"""Admit only the isolated, current native implementation of the grasp target.

A library existing at the expected path does not prove pluginlib will load it:
all visible registries must name exactly one provider, and that provider must
map the fixed class to the library covered by control-plugin lifetime pinning.
"""
import hashlib
import os
from pathlib import Path, PurePosixPath
import xml.etree.ElementTree as ET


_PACKAGE = 'manipulation_grasp_plugin'
_LOOKUP = 'manipulation_grasp/ObservationPlugin'
_TYPE = 'manipulation_grasp::ObservationPlugin'
_BASE = 'mujoco_ros2_control_plugins::MuJoCoROS2ControlPluginBase'
_LIBRARY = 'mujoco_ros2_control_grasp_observer'
_REGISTRY = 'share/ament_index/resource_index/mujoco_ros2_control_plugins__pluginlib__plugin'
_DESCRIPTOR = f'share/{_PACKAGE}/plugin.xml'


def _read_bounded(path, maximum, label):
    try:
        if not path.is_file() or path.stat().st_size > maximum:
            raise ValueError(f'Invalid or oversized {label}')
        return path.read_bytes()
    except OSError as error:
        raise ValueError(f'Cannot read {label}') from error


def _document(raw):
    try:
        return ET.fromstring(raw)
    except ET.ParseError as error:
        raise ValueError('Invalid native plugin descriptor XML') from error


def _confined_descriptor(prefix, relative):
    # Registry entries are package-relative POSIX paths, never absolute paths,
    # traversal or symlinks into a different build/install prefix.
    parts = PurePosixPath(relative)
    if not relative or parts.is_absolute() or any(part in ('.', '..') for part in relative.split('/')):
        raise ValueError('Native plugin descriptor must stay in its installation prefix')
    path = prefix / relative
    if not path.resolve().is_relative_to(prefix.resolve()):
        raise ValueError('Native plugin descriptor escapes its installation prefix')
    return path


def _visible_prefixes(root):
    install = root / 'ros_ws/install'
    try:
        prefixes = [prefix for prefix in install.iterdir() if prefix.is_dir()]
    except OSError as error:
        raise ValueError('Build the isolated ROS grasp native runtime before launch') from error
    # The launch environment also contains the bundled runtime and can contain
    # explicit overlays. A second class there is just as ambiguous as a stale
    # package in this worktree, even if the expected .so matches its source.
    prefixes.append(root / 'runtime/root/opt/ros/humble')
    prefixes.extend(Path(item) for item in os.environ.get('AMENT_PREFIX_PATH', '').split(os.pathsep) if item)
    return sorted({prefix.resolve() for prefix in prefixes})


def _check_provider(root):
    expected_prefix = (root / 'ros_ws/install' / _PACKAGE).resolve()
    source = _read_bounded(root / 'native/grasp_ros2_plugin/plugin.xml', 65536, 'source plugin descriptor')
    canonical = _document(source)
    classes = list(canonical.iter('class'))
    if (canonical.tag != 'library' or canonical.attrib != {'path': _LIBRARY}
            or len(classes) != 1 or classes[0].attrib != {
                'name': _LOOKUP, 'type': _TYPE, 'base_class_type': _BASE}):
        raise ValueError('The grasp source descriptor must use the fixed pinned observation class and library')
    found = []
    for prefix in _visible_prefixes(root):
        registry = prefix / _REGISTRY
        if not registry.is_dir():
            continue
        for resource in sorted(registry.iterdir()):
            try:
                entries = _read_bounded(resource, 4096, 'native plugin registry').decode('utf-8').splitlines()
            except UnicodeDecodeError as error:
                raise ValueError('Invalid native plugin registry encoding') from error
            for relative in entries:
                path = _confined_descriptor(prefix, relative)
                raw = _read_bounded(path, 65536, 'native plugin descriptor')
                document = _document(raw)
                for item in document.iter('class'):
                    if item.get('name') != _LOOKUP:
                        continue
                    found.append(prefix)
                    if (prefix != expected_prefix or resource.name != _PACKAGE
                            or relative != _DESCRIPTOR or raw != source):
                        raise ValueError('The grasp observation provider or library mapping differs from the current isolated build')
    if found != [expected_prefix]:
        raise ValueError('The grasp observation class must have exactly one installed provider; archive stale builds first')


def check_grasp_runtime(root):
    """Verify binaries and the class pluginlib will select; no ROS imports."""
    root = Path(root).resolve()
    _check_provider(root)
    targets = (
        (root / 'ros_ws/install/mujoco_ros2_control/lib/libmujoco_ros2_control.so',
         (root / 'native/step_monitor.hpp', root / 'native/grasp_policy.hpp')),
        (root / f'ros_ws/install/{_PACKAGE}/lib/lib{_LIBRARY}.so',
         (root / 'native/grasp_ros2_plugin/src/observation.cpp',
          root / 'native/grasp_ros2_plugin/src/ordered_fifo.hpp')),
        (root / 'ros_ws/install/mujoco_ros2_control/lib/mujoco_ros2_control/ros2_control_node',
         (root / 'native/control_loop_shutdown.hpp',)),
    )
    for binary, sources in targets:
        if not binary.is_file():
            raise ValueError('Build the isolated ROS grasp native runtime before launch')
        raw = binary.read_bytes()
        for source in sources:
            if hashlib.sha256(source.read_bytes()).hexdigest().encode() not in raw:
                raise ValueError('ROS grasp native binary does not match its current source; rebuild it')
