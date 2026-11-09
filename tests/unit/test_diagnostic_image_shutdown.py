"""The read-only diagnostic source closes its own ROS resources after SIGINT."""
from contextlib import redirect_stdout
import io
import json
import sys
import types
import unittest
from unittest.mock import Mock, patch

from mujoco_ros2_manipulation.operator_console import diagnostic_image


class ExternalShutdown(Exception):
    pass


class DiagnosticImageShutdownTests(unittest.TestCase):
    def run_interruption(self, exception, context_ok):
        node = Mock()
        ros = types.ModuleType("rclpy")
        ros.init = Mock()
        ros.ok = Mock(return_value=context_ok)
        ros.spin_once = Mock(side_effect=exception)
        ros.shutdown = Mock()
        node_module = types.ModuleType("rclpy.node")
        node_module.Node = Mock(return_value=node)
        executor_module = types.ModuleType("rclpy.executors")
        executor_module.ExternalShutdownException = ExternalShutdown
        qos_module = types.ModuleType("rclpy.qos")
        qos_module.QoSProfile = Mock()
        qos_module.ReliabilityPolicy = types.SimpleNamespace(BEST_EFFORT=1)
        image_module = types.ModuleType("sensor_msgs.msg")
        image_module.Image = Mock()
        clock_module = types.ModuleType("rosgraph_msgs.msg")
        clock_module.Clock = Mock()
        modules = {"rclpy": ros, "rclpy.node": node_module,
                   "rclpy.executors": executor_module, "rclpy.qos": qos_module,
                   "sensor_msgs.msg": image_module, "rosgraph_msgs.msg": clock_module}
        output = io.StringIO()
        with patch.dict(sys.modules, modules), patch.object(sys, "argv", [
                "diagnostic_image", "--domain", "222", "--duration", "1"]), \
                patch.object(diagnostic_image.time, "monotonic", return_value=0), \
                redirect_stdout(output):
            self.assertEqual(diagnostic_image.main(), 0)
        node.destroy_node.assert_called_once_with()
        self.assertEqual(ros.shutdown.call_count, int(context_ok))
        node.create_publisher.return_value.publish.assert_not_called()
        self.assertEqual(json.loads(output.getvalue())["control_commands"], 0)

    def test_keyboard_interrupt_closes_live_context(self):
        self.run_interruption(KeyboardInterrupt(), True)

    def test_external_shutdown_keeps_already_closed_context(self):
        self.run_interruption(ExternalShutdown(), False)


if __name__ == "__main__":
    unittest.main()
