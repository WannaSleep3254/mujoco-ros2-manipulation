import argparse
import unittest

from mujoco_ros2_manipulation.operator_console.__main__ import local_grasp_viewer_url


class RosGraspNavigationTests(unittest.TestCase):
    def test_explicit_local_root_is_canonicalized(self):
        self.assertEqual(local_grasp_viewer_url("http://127.0.0.1:8930"), "http://127.0.0.1:8930/")

    def test_navigation_cannot_include_remote_hosts_credentials_or_command_paths(self):
        for value in ("https://127.0.0.1:8930/", "http://example.com:8930/", "http://localhost:8930/",
                      "http://127.0.0.1/", "http://127.0.0.1:80/", "http://127.0.0.1:70000/",
                      "http://user:secret@127.0.0.1:8930/", "http://127.0.0.1:8930/api/start",
                      "http://127.0.0.1:8930/?run=1", "http://127.0.0.1:8930/#start", None):
            with self.subTest(value=value), self.assertRaises(argparse.ArgumentTypeError):
                local_grasp_viewer_url(value)
