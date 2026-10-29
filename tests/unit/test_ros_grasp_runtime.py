"""The selected observation class must match its checked native implementation."""
import hashlib
import os
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from mujoco_ros2_manipulation.grasping.ros_runtime import check_grasp_runtime


PACKAGE = 'manipulation_grasp_plugin'
REGISTRY = 'share/ament_index/resource_index/mujoco_ros2_control_plugins__pluginlib__plugin'
DESCRIPTOR = f'share/{PACKAGE}/plugin.xml'
SOURCE_XML = b'''<library path="mujoco_ros2_control_grasp_observer">
<class name="manipulation_grasp/ObservationPlugin"
 type="manipulation_grasp::ObservationPlugin"
 base_class_type="mujoco_ros2_control_plugins::MuJoCoROS2ControlPluginBase"/>
</library>\n'''
SOURCES = {
    'native/step_monitor.hpp': b'monitor current source\n',
    'native/grasp_policy.hpp': b'grasp policy current source\n',
    'native/grasp_ros2_plugin/src/observation.cpp': b'observation current source\n',
    'native/grasp_ros2_plugin/src/ordered_fifo.hpp': b'ordered FIFO current source\n',
    'native/control_loop_shutdown.hpp': b'shutdown current source\n',
}
BINARIES = {
    'ros_ws/install/mujoco_ros2_control/lib/libmujoco_ros2_control.so':
        ('native/step_monitor.hpp', 'native/grasp_policy.hpp'),
    f'ros_ws/install/{PACKAGE}/lib/libmujoco_ros2_control_grasp_observer.so':
        ('native/grasp_ros2_plugin/src/observation.cpp', 'native/grasp_ros2_plugin/src/ordered_fifo.hpp'),
    'ros_ws/install/mujoco_ros2_control/lib/mujoco_ros2_control/ros2_control_node':
        ('native/control_loop_shutdown.hpp',),
}


def write(root, relative, data):
    path = root / relative
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_bytes(data)
    return path


def provider(prefix, package=PACKAGE, descriptor=DESCRIPTOR, xml=SOURCE_XML):
    registry = write(prefix, f'{REGISTRY}/{package}', (descriptor + '\n').encode())
    path = write(prefix, descriptor, xml)
    return registry, path


def fixture(root):
    for source, data in SOURCES.items():
        write(root, source, data)
    for binary, sources in BINARIES.items():
        write(root, binary, b'fake binary\x00' + b'\x00'.join(
            hashlib.sha256(SOURCES[source]).hexdigest().encode() for source in sources))
    write(root, 'native/grasp_ros2_plugin/plugin.xml', SOURCE_XML)
    return provider(root / 'ros_ws/install' / PACKAGE)


class RosGraspRuntimeTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.addCleanup(self.temporary.cleanup)
        self.root = Path(self.temporary.name)
        self.registry, self.descriptor = fixture(self.root)
        environment = patch.dict(os.environ, {'AMENT_PREFIX_PATH': ''})
        environment.start()
        self.addCleanup(environment.stop)

    def test_one_current_class_and_all_source_bound_binaries_are_admitted(self):
        self.assertIsNone(check_grasp_runtime(self.root))
        with patch.dict(os.environ, {'AMENT_PREFIX_PATH': str(self.root / 'ros_ws/install' / PACKAGE)}):
            self.assertIsNone(check_grasp_runtime(self.root))

    def test_missing_binaries_or_stale_monitor_policy_observer_and_shutdown_are_rejected(self):
        for binary in BINARIES:
            path = self.root / binary
            original = path.read_bytes()
            with self.subTest(binary=binary):
                path.unlink()
                with self.assertRaises(ValueError):
                    check_grasp_runtime(self.root)
                path.write_bytes(original)
        for source in SOURCES:
            path = self.root / source
            original = path.read_bytes()
            with self.subTest(source=source):
                path.write_bytes(original + b'changed')
                with self.assertRaisesRegex(ValueError, 'does not match'):
                    check_grasp_runtime(self.root)
                path.write_bytes(original)

    def test_old_package_exporting_same_class_is_rejected_even_with_valid_expected_binary(self):
        provider(self.root / 'ros_ws/install/stale_observer', package='stale_observer',
                 descriptor='share/stale_observer/plugin.xml',
                 xml=SOURCE_XML.replace(b'mujoco_ros2_control_grasp_observer', b'manipulation_grasp_observer'))
        with self.assertRaisesRegex(ValueError, 'provider or library mapping'):
            check_grasp_runtime(self.root)

    def test_external_ament_overlay_and_bundled_runtime_cannot_hide_a_second_provider(self):
        for prefix in (self.root / 'overlay', self.root / 'runtime/root/opt/ros/humble'):
            with self.subTest(prefix=prefix):
                registry, descriptor = provider(prefix)
                with patch.dict(os.environ, {'AMENT_PREFIX_PATH': str(prefix)}):
                    with self.assertRaisesRegex(ValueError, 'provider or library mapping'):
                        check_grasp_runtime(self.root)
                registry.unlink()
                descriptor.unlink()

    def test_missing_and_duplicate_registry_entries_are_rejected(self):
        original = self.registry.read_bytes()
        for content in (b'', original + original):
            with self.subTest(content=content):
                self.registry.write_bytes(content)
                with self.assertRaisesRegex(ValueError, 'exactly one'):
                    check_grasp_runtime(self.root)

    def test_installed_descriptor_must_match_current_class_base_type_library_and_bytes(self):
        mutations = (
            SOURCE_XML.replace(b'manipulation_grasp/ObservationPlugin', b'other/Class'),
            SOURCE_XML.replace(b'manipulation_grasp::ObservationPlugin', b'other::Class'),
            SOURCE_XML.replace(b'mujoco_ros2_control_plugins::MuJoCoROS2ControlPluginBase', b'other::Base'),
            SOURCE_XML.replace(b'mujoco_ros2_control_grasp_observer', b'unpinned_observer'),
            SOURCE_XML + b'\n',
        )
        for content in mutations:
            with self.subTest(content=content):
                self.descriptor.write_bytes(content)
                with self.assertRaises(ValueError):
                    check_grasp_runtime(self.root)

    def test_changed_source_mapping_to_unpinned_library_or_other_class_is_rejected(self):
        source = self.root / 'native/grasp_ros2_plugin/plugin.xml'
        for old, new in ((b'mujoco_ros2_control_grasp_observer', b'unpinned_observer'),
                         (b'manipulation_grasp::ObservationPlugin', b'other::Class'),
                         (b'base_class_type=', b'other_attribute=')):
            content = SOURCE_XML.replace(old, new)
            source.write_bytes(content)
            self.descriptor.write_bytes(content)
            with self.subTest(change=new), self.assertRaisesRegex(ValueError, 'fixed pinned'):
                check_grasp_runtime(self.root)

    def test_unexpected_package_resource_name_and_descriptor_path_are_rejected(self):
        original = self.registry.read_bytes()
        renamed = self.registry.with_name('alias_package')
        self.registry.rename(renamed)
        with self.assertRaisesRegex(ValueError, 'provider or library mapping'):
            check_grasp_runtime(self.root)
        renamed.rename(self.registry)
        other = write(self.descriptor.parent, 'other.xml', SOURCE_XML)
        self.registry.write_bytes((f'share/{PACKAGE}/other.xml\n').encode())
        with self.assertRaisesRegex(ValueError, 'provider or library mapping'):
            check_grasp_runtime(self.root)
        self.assertTrue(other.is_file())
        self.registry.write_bytes(original)

    def test_absolute_traversal_and_symlink_descriptor_escape_are_rejected(self):
        outside = write(self.root, 'outside.xml', SOURCE_XML)
        prefix = self.root / 'ros_ws/install' / PACKAGE
        for entry in (str(outside), '../../../outside.xml', './share/plugin.xml', 'share//plugin.xml'):
            self.registry.write_text(entry + '\n')
            with self.subTest(entry=entry), self.assertRaises(ValueError):
                check_grasp_runtime(self.root)
        self.descriptor.unlink()
        self.descriptor.symlink_to(outside)
        self.registry.write_text(DESCRIPTOR + '\n')
        with self.assertRaisesRegex(ValueError, 'escapes'):
            check_grasp_runtime(self.root)
        self.assertTrue(outside.is_file())

    def test_unreadable_oversized_and_malformed_registry_or_descriptor_fail_as_value_errors(self):
        for raw in (b'\xff', b'x' * 4097):
            self.registry.write_bytes(raw)
            with self.subTest(registry=raw[:10]), self.assertRaises(ValueError):
                check_grasp_runtime(self.root)
        self.registry.write_text(DESCRIPTOR + '\n')
        for raw in (b'<library', b'x' * 65537):
            self.descriptor.write_bytes(raw)
            with self.subTest(descriptor=raw[:10]), self.assertRaises(ValueError):
                check_grasp_runtime(self.root)


if __name__ == '__main__':
    unittest.main()
