"""ROS-free environment policy tests, including the highest supported DDS domain."""
import os
from pathlib import Path
import subprocess
import unittest
import xml.etree.ElementTree as ET

ROOT = Path(__file__).resolve().parents[2]


class MiddlewareEnvironmentTests(unittest.TestCase):
    def environment(self, **overrides):
        env = {k: v for k, v in os.environ.items()
               if k not in ("CYCLONEDDS_URI", "MANIPULATION_RMW_IMPLEMENTATION", "RMW_IMPLEMENTATION")}
        env.update(overrides)
        output = subprocess.check_output([
            "bash", "-c", 'source "$1"; env -0', "middleware-test",
            str(ROOT/"scripts/ros_middleware_env.sh")], env=env, cwd="/tmp")
        result = dict(item.decode().split("=",1) for item in output.split(b"\0") if item)
        return {key: result.get(key) for key in ("RMW_IMPLEMENTATION", "CYCLONEDDS_URI")}

    def test_default_resolves_workspace_config_outside_checkout(self):
        result = self.environment()
        self.assertEqual(result["RMW_IMPLEMENTATION"], "rmw_cyclonedds_cpp")
        self.assertEqual(result["CYCLONEDDS_URI"], (ROOT/"config/ros/cyclonedds.xml").as_uri())

    def test_explicit_dds_configuration_is_preserved(self):
        for uri in ("file:///custom/ros.xml", "<CycloneDDS><Domain /></CycloneDDS>"):
            with self.subTest(uri=uri):
                self.assertEqual(self.environment(CYCLONEDDS_URI=uri)["CYCLONEDDS_URI"], uri)

    def test_alternative_middleware_does_not_receive_cyclone_defaults(self):
        result = self.environment(MANIPULATION_RMW_IMPLEMENTATION="rmw_fastrtps_cpp")
        self.assertEqual(result["RMW_IMPLEMENTATION"], "rmw_fastrtps_cpp")
        self.assertIsNone(result["CYCLONEDDS_URI"])

    def test_capacity_has_headroom_and_stays_in_supported_domain_port_range(self):
        root = ET.parse(ROOT/"config/ros/cyclonedds.xml").getroot()
        ns = {"c": "https://cdds.io/config"}
        discovery = root.find("c:Domain/c:Discovery", ns)
        self.assertEqual(discovery.findtext("c:ParticipantIndex", namespaces=ns), "auto")
        maximum = int(discovery.findtext("c:MaxAutoParticipantIndex", namespaces=ns))
        self.assertGreaterEqual(maximum+1, 32)
        # DDSI default PB=7400, DG=250, PG=2, d1=10, d3=11.
        self.assertLessEqual(7400+250*232+2*maximum+11, 65535)
        self.assertLess(2*maximum+11, 250)
