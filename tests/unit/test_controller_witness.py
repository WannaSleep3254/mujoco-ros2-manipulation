"""Passive evidence preserves clock/delivery limits and survives malformed input."""
from copy import deepcopy
import json
import os
from pathlib import Path
import signal
import stat
import subprocess
import sys
from tempfile import TemporaryDirectory
from types import ModuleType, SimpleNamespace
import unittest
from unittest.mock import patch

from mujoco_ros2_manipulation.workcells.controller_witness import (
    ControllerWitness, MAX_CAPACITY, MAX_JOINTS, MAX_STATUSES, main, ros_controller, ros_statuses,
)

TOPIC = "/left_controller/controller_state"
STATUS = "/left_controller/follow_joint_trajectory/_action/status"


def stamp(sec=10, nanosec=0):
    return dict(sec=sec, nanosec=nanosec)


def point(values=(.1, .2)):
    return dict(positions=list(values), velocities=[0.] * len(values), accelerations=[], effort=[],
                time_from_start=stamp(1, 500_000_000))


def controller(sec=10):
    return dict(header=dict(stamp=stamp(sec), frame_id="world"), joint_names=["left_j1", "left_j2"],
                reference=point(), feedback=point((.09, .18)), output=point(), error=point((.01, .02)),
                desired=point(), actual=point((.09, .18)))


def statuses():
    return dict(status_list=[dict(goal_uuid="a" * 32, goal_stamp=stamp(9), status=2)])


class ControllerWitnessTests(unittest.TestCase):
    def setUp(self):
        self.now = 1_000_000_000
        self.recorder = ControllerWitness(clock=lambda: self.now, session_id="f" * 32, domain=202)

    def test_modern_and_legacy_points_keep_name_order_and_independent_clock_values(self):
        self.assertTrue(self.recorder.observe("clock", "/clock", stamp(9)))
        original = controller()
        before = deepcopy(original)
        self.now += 10
        self.assertTrue(self.recorder.observe("controller", TOPIC, original))
        record = self.recorder.snapshot()["events"][-1]
        self.assertEqual(record["source_stamp_ns"], 10_000_000_000)
        self.assertEqual(record["observed_clock_ns"], 9_000_000_000)
        self.assertEqual(record["receipt_monotonic_ns"], 1_000_000_010)
        self.assertEqual(record["joint_names"], ["left_j1", "left_j2"])
        self.assertEqual(set(record["points"]), {"reference", "feedback", "output", "error", "desired", "actual"})
        self.assertEqual(record["points"]["reference"]["time_from_start_ns"], 1_500_000_000)
        self.assertEqual(original, before)
        original["reference"]["positions"][0] = 999
        record["points"]["feedback"]["positions"][0] = 999
        saved = self.recorder.snapshot()["events"][-1]["points"]
        self.assertEqual(saved["reference"]["positions"], [.1, .2])
        self.assertEqual(saved["feedback"]["positions"], [.09, .18])

    def test_legacy_only_positions_and_empty_optional_vectors_are_kept(self):
        value = controller()
        del value["reference"], value["feedback"], value["output"]
        value["error"] = point(())
        value["error"]["time_from_start"] = stamp(-1, 500_000_000)
        self.assertTrue(self.recorder.observe("controller", TOPIC, value))
        self.assertEqual(self.recorder.snapshot()["events"][0]["points"]["error"]["time_from_start_ns"], -500_000_000)

    def test_malformed_vectors_names_stamps_and_nonfinite_values_are_counted_without_loss_of_next_good_sample(self):
        invalid = []
        value = controller(); value["reference"]["positions"] = [0.]; invalid.append(value)
        value = controller(); value["feedback"]["velocities"] = [float("nan"), 0.]; invalid.append(value)
        value = controller(); value["output"]["effort"] = [float("inf"), 0.]; invalid.append(value)
        value = controller(); value["joint_names"] = ["left_j1", "left_j1"]; invalid.append(value)
        value = controller(); value["joint_names"] = ["x"] * (MAX_JOINTS + 1); invalid.append(value)
        value = controller(); value["header"]["stamp"]["nanosec"] = 1_000_000_000; invalid.append(value)
        value = controller(); value["header"]["frame_id"] = "x" * 257; invalid.append(value)
        value = controller(); value["reference"]["positions"] = [True, 0.]; invalid.append(value)
        value = controller(); value["actual"]["positions"] = []; value["feedback"]["positions"] = []; invalid.append(value)
        for value in invalid:
            with self.subTest(value=value):
                self.assertFalse(self.recorder.observe("controller", TOPIC, value))
        self.assertTrue(self.recorder.observe("controller", TOPIC, controller()))
        report = self.recorder.snapshot()
        self.assertEqual(report["counters"]["invalid"], len(invalid))
        self.assertEqual(report["counters"]["accepted"], 1)
        self.assertEqual(report["retained_count"], 1)

    def test_unknown_types_and_topics_have_bounded_error_keys(self):
        for kind, topic, payload in (("secret", TOPIC, {}), ("controller", "/other", {}),
                                    ("controller", [], {}), ("controller", TOPIC, None)):
            self.assertFalse(self.recorder.observe(kind, topic, payload))
        self.recorder.reject("untrusted long diagnostic" * 10000)
        report = self.recorder.snapshot()
        self.assertEqual(report["counters"]["invalid"], 5)
        self.assertNotIn("untrusted", json.dumps(report))
        self.assertEqual(set(report["invalid_reasons"]), {"invalid_kind", "unregistered_topic", "invalid_structure", "invalid_controller", "invalid_ros_message"})

    def test_backward_controller_stamp_is_rejected_until_observed_clock_reset_new_epoch(self):
        self.recorder.observe("clock", "/clock", stamp(10))
        self.recorder.observe("controller", TOPIC, controller(10))
        self.assertFalse(self.recorder.observe("controller", TOPIC, controller(9)))
        self.recorder.observe("clock", "/clock", stamp(1))
        self.assertTrue(self.recorder.observe("controller", TOPIC, controller(1)))
        report = self.recorder.snapshot()
        self.assertEqual(report["clock"]["observed_resets"], 1)
        self.assertEqual([record["clock_epoch"] for record in report["events"]], [0, 0, 1, 1])
        self.assertEqual(report["invalid_reasons"]["controller_stamp_backwards"], 1)
        self.assertEqual(report["session_id"], "f" * 32)

    def test_names_cannot_change_even_after_reset_and_receipt_regression_does_not_advance_state(self):
        self.recorder.observe("controller", TOPIC, controller(10), received_ns=100)
        self.assertFalse(self.recorder.observe("clock", "/clock", stamp(1), received_ns=99))
        self.recorder.observe("clock", "/clock", stamp(10), received_ns=101)
        self.recorder.observe("clock", "/clock", stamp(1), received_ns=102)
        changed = controller(1); changed["joint_names"].reverse()
        self.assertFalse(self.recorder.observe("controller", TOPIC, changed, received_ns=103))
        self.assertTrue(self.recorder.observe("controller", TOPIC, controller(1), received_ns=103))
        self.assertEqual(self.recorder.snapshot()["invalid_reasons"], {"receipt_clock_backwards": 1, "joint_names_changed": 1})

    def test_ring_evicts_oldest_with_explicit_counts_and_gap_preserving_indexes(self):
        recorder = ControllerWitness(capacity=2)
        for value in range(5):
            recorder.observe("clock", "/clock", stamp(value), received_ns=value)
        report = recorder.snapshot()
        self.assertTrue(report["truncated"])
        self.assertEqual(report["retained_count"], 2)
        self.assertEqual(report["counters"]["evicted_by_capacity"], 3)
        self.assertEqual(report["topic_counts"]["/clock"], 5)
        self.assertEqual([record["index"] for record in report["events"]], [3, 4])

    def test_goal_uuid_and_status_array_are_preserved_without_inferred_request_or_state_stamp(self):
        self.recorder.observe("goal_status", STATUS, statuses())
        self.recorder.observe("goal_status", STATUS, dict(status_list=[]))
        record = self.recorder.snapshot()["events"][0]
        self.assertEqual(record["status_list"], [dict(goal_uuid="a" * 32, goal_stamp_ns=9_000_000_000, status=2)])
        self.assertIsNone(record["observed_clock_ns"])
        self.assertNotIn("request_id", record)
        self.assertNotIn("source_stamp_ns", record)

    def test_bad_goal_statuses_are_rejected_and_status_count_bounded(self):
        invalid = []
        value = statuses(); value["status_list"][0]["goal_uuid"] = "X" * 32; invalid.append(value)
        value = statuses(); value["status_list"][0]["status"] = 7; invalid.append(value)
        value = statuses(); value["status_list"][0]["status"] = True; invalid.append(value)
        value = statuses(); value["status_list"] *= 2; invalid.append(value)
        value = statuses(); value["status_list"] *= MAX_STATUSES + 1; invalid.append(value)
        for value in invalid:
            self.assertFalse(self.recorder.observe("goal_status", STATUS, value))
        self.assertEqual(self.recorder.snapshot()["counters"]["goal_status"], 0)

    def test_configuration_limits_reject_before_any_capture(self):
        for options in (dict(capacity=0), dict(capacity=True), dict(capacity=MAX_CAPACITY+1),
                        dict(controllers=[]), dict(controllers=["left", "left"]),
                        dict(controllers=["../controller"]), dict(session_id="bad"), dict(domain=233)):
            with self.subTest(options=options), self.assertRaises(ValueError):
                ControllerWitness(**options)

    def test_final_json_is_atomic_owner_only_and_existing_file_or_symlink_never_replaced(self):
        self.recorder.observe("controller", TOPIC, controller())
        self.recorder.finish("signal", signal_number=signal.SIGTERM)
        self.recorder.finish("error", error_type="RuntimeError")
        self.assertFalse(self.recorder.observe("clock", "/clock", stamp()))
        with TemporaryDirectory() as directory:
            path = Path(directory) / "witness.json"
            self.recorder.save(path)
            value = json.loads(path.read_text(), parse_constant=lambda value: self.fail("nonfinite JSON"))
            self.assertEqual(value["completion"]["reason"], "signal")
            self.assertEqual(value["completion"]["signal_number"], 15)
            self.assertEqual(value["ros_domain_id"], 202)
            self.assertEqual(stat.S_IMODE(path.stat().st_mode), 0o600)
            before = path.read_bytes()
            with self.assertRaises(FileExistsError):
                self.recorder.save(path)
            link = Path(directory) / "link.json"; link.symlink_to(path)
            with self.assertRaises(FileExistsError):
                self.recorder.save(link)
            self.assertEqual(path.read_bytes(), before)
            self.assertEqual(sorted(p.name for p in Path(directory).iterdir()), ["link.json", "witness.json"])

    def test_failed_write_does_not_leave_partial_published_file_or_temporary(self):
        with TemporaryDirectory() as directory:
            path = Path(directory) / "witness.json"
            with patch("mujoco_ros2_manipulation.workcells.controller_witness.os.link", side_effect=OSError("disk")):
                with self.assertRaises(OSError):
                    self.recorder.save(path)
            self.assertEqual(list(Path(directory).iterdir()), [])

    def test_pure_module_does_not_import_ros_numpy_or_mujoco(self):
        code = """
import sys
class Deny:
    def find_spec(self, name, path=None, target=None):
        if name.split('.')[0] in ('rclpy','control_msgs','action_msgs','rosgraph_msgs','mujoco','numpy'):
            raise RuntimeError('forbidden import ' + name)
sys.meta_path.insert(0,Deny())
from mujoco_ros2_manipulation.workcells.controller_witness import ControllerWitness
assert ControllerWitness(capacity=1).snapshot()['passive_subscribe_only']
"""
        subprocess.run([sys.executable, "-c", code], check=True, env=dict(os.environ,
            PYTHONPATH=str(Path(__file__).resolve().parents[2]/"src")), capture_output=True, text=True)

    def test_ros_converters_bound_arrays_before_copy_and_keep_goal_ids(self):
        def namespace(value):
            return SimpleNamespace(**{name: namespace(item) if isinstance(item, dict) else item for name, item in value.items()})
        message = namespace(controller())
        message.multi_dof_joint_names = []
        self.assertEqual(ros_controller(message), controller())
        message.reference.positions = range(MAX_JOINTS+1)
        with self.assertRaises(ValueError):
            ros_controller(message)
        goal = SimpleNamespace(goal_info=SimpleNamespace(goal_id=SimpleNamespace(uuid=[170]*16), stamp=namespace(stamp(9))), status=2)
        self.assertEqual(ros_statuses(SimpleNamespace(status_list=[goal])), statuses())
        with self.assertRaises(ValueError):
            ros_statuses(SimpleNamespace(status_list=[goal]*(MAX_STATUSES+1)))

    def test_runtime_signal_saves_and_cleans_up_using_subscriptions_only(self):
        handlers, subscriptions, destroyed, shutdown, infrastructure_removed = {}, [], [], [], []
        def module(name, **fields):
            value = ModuleType(name); value.__dict__.update(fields); return value
        class Node:
            def __init__(self, *args, **kwargs):
                self.assertions = kwargs
                self.publishers = ["parameter_events"]
            def destroy_publisher(self, publisher):
                self.publishers.remove(publisher)
                infrastructure_removed.append(publisher)
            def create_subscription(self, kind, topic, callback, qos):
                subscriptions.append(topic)
            def destroy_node(self):
                destroyed.append(True)
        def set_signal(sig, handler):
            previous = handlers.get(sig, "original")
            handlers[sig] = handler
            return previous
        fake = {
            "rclpy": module("rclpy", init=lambda **kwargs: None, ok=lambda: True,
                shutdown=lambda: shutdown.append(True), spin_once=lambda *args, **kwargs: handlers[signal.SIGTERM](15, None)),
            "rclpy.node": module("rclpy.node", Node=Node),
            "rclpy.signals": module("rclpy.signals", SignalHandlerOptions=SimpleNamespace(NO=0)),
            "rclpy.qos": module("rclpy.qos", QoSProfile=lambda **kwargs: kwargs,
                ReliabilityPolicy=SimpleNamespace(BEST_EFFORT=1, RELIABLE=2), DurabilityPolicy=SimpleNamespace(TRANSIENT_LOCAL=1)),
            "control_msgs": module("control_msgs"),
            "control_msgs.msg": module("control_msgs.msg", JointTrajectoryControllerState=object),
            "action_msgs": module("action_msgs"),
            "action_msgs.msg": module("action_msgs.msg", GoalStatusArray=object),
            "rosgraph_msgs": module("rosgraph_msgs"),
            "rosgraph_msgs.msg": module("rosgraph_msgs.msg", Clock=object),
        }
        with TemporaryDirectory() as directory, patch.dict(sys.modules, fake), patch.dict(os.environ), patch("signal.signal", set_signal):
            path = Path(directory) / "witness.json"
            self.assertEqual(main(["--domain", "202", "--output", str(path), "--duration", "10"]), 0)
            report = json.loads(path.read_text())
        self.assertEqual(report["completion"]["reason"], "signal")
        self.assertEqual(report["completion"]["signal_number"], 15)
        self.assertEqual(len(subscriptions), 5)
        self.assertEqual(destroyed, [True])
        self.assertEqual(shutdown, [True])
        self.assertEqual(infrastructure_removed, ["parameter_events"])
        self.assertEqual(set(handlers.values()), {"original"})

    def test_runtime_import_failure_still_saves_failure_completion(self):
        with TemporaryDirectory() as directory, patch.dict(sys.modules, {"rclpy": None}), patch.dict(os.environ):
            path = Path(directory) / "witness.json"
            self.assertEqual(main(["--domain", "202", "--output", str(path)]), 1)
            value = json.loads(path.read_text())
            self.assertEqual(value["completion"]["reason"], "error")
            self.assertEqual(value["completion"]["error_type"], "ModuleNotFoundError")
            self.assertEqual(value["retained_count"], 0)


if __name__ == "__main__":
    unittest.main()
