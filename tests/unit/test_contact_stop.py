"""Latch persistence, controller ownership, test isolation and stale-runtime rejection."""
import hashlib
from pathlib import Path
from types import SimpleNamespace
import tempfile
import unittest
from mujoco_ros2_manipulation.workcells.contact_policy import (
    ContactPolicy,
    contact_test_model,
    check_contact_runtime,
)


class ContactPolicyTests(unittest.TestCase):
    def test_false_status_reset_cannot_release_latched_stop(self):
        policy = ContactPolicy(["left", "right"])
        states = {"left": "active", "right": "active", "gripper": "active"}
        self.assertEqual(policy.phase(states), "waiting")
        policy.observe(False)
        self.assertEqual(policy.phase(states), "ready")
        self.assertEqual(policy.targets(states), [])
        policy.observe(True)
        policy.observe(False)
        self.assertEqual(policy.targets(states), ["left", "right"])
        self.assertEqual(policy.phase(states), "stopping")
        self.assertEqual(policy.phase({"left": "inactive"}), "stopping")
        states.update(left="inactive", right="inactive")
        self.assertEqual(policy.phase(states), "stopped")
        states["right"] = "active"
        self.assertEqual(policy.targets(states), ["right"])
        self.assertEqual(policy.phase(states), "stopping")

    def test_invalid_controller_ownership_rejected(self):
        for names in [[], ["left"], ["left", "left"], ["left", "right", "extra"]]:
            with self.assertRaises(ValueError):
                ContactPolicy(names)

    def test_fixture_is_separate_and_restricted_to_run_directory(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            output = root / "generated"
            output.mkdir()
            source = '<mujoco><worldbody><body name="left_forearm_link"/></worldbody></mujoco>'
            (output / "cell.xml").write_text(source)
            cell = SimpleNamespace(root=root, output=output)
            with self.assertRaises(ValueError):
                contact_test_model(cell, root)
            directory = root / "runtime/validation/runs/test"
            directory.mkdir(parents=True)
            model = contact_test_model(cell, directory)
            self.assertIn('mocap="true"', model.read_text())
            self.assertIn('pos="100 100 100"', model.read_text())
            self.assertEqual((output / "cell.xml").read_text(), source)
            with self.assertRaises(ValueError):
                contact_test_model(cell, directory)

    def test_outdated_runtime_cannot_start_protected_workcell(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            (root / "native").mkdir()
            header = root / "native/step_monitor.hpp"
            header.write_text("new")
            with self.assertRaises(ValueError):
                check_contact_runtime(root)
            binary = (
                root
                / "ros_ws/install/mujoco_ros2_control/lib/libmujoco_ros2_control.so"
            )
            binary.parent.mkdir(parents=True)
            binary.write_bytes(b"old")
            with self.assertRaises(ValueError):
                check_contact_runtime(root)
            binary.write_bytes(hashlib.sha256(header.read_bytes()).hexdigest().encode())
            check_contact_runtime(root)
