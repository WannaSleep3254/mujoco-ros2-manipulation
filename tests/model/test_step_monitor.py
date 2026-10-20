"""Compile the same monitor against the actual pinned C++ MuJoCo runtime."""
import hashlib
import json
import os
from pathlib import Path
import subprocess
import tempfile
import unittest

from mujoco_ros2_manipulation.paths import ROOT
from mujoco_ros2_manipulation.verification.diagnostics import check_physics


class NativeMonitorTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.vendor = ROOT / "runtime/root/opt/ros/humble/opt/mujoco_vendor"
        if not (cls.vendor / "include/mujoco/mujoco.h").exists():
            raise unittest.SkipTest("Pinned native MuJoCo runtime not built")
        cls.temporary = tempfile.TemporaryDirectory()
        cls.addClassCleanup(cls.temporary.cleanup)
        cls.directory = Path(cls.temporary.name)
        cls.digest = hashlib.sha256(
            (ROOT / "native/step_monitor.hpp").read_bytes()
        ).hexdigest()
        cls.binary = cls.directory / "probe"
        subprocess.run(
            [
                "g++",
                "-std=c++17",
                "-O2",
                "-Wall",
                "-Wextra",
                "-Werror",
                "-Wconversion",
                '-DMANIPULATION_MONITOR_SHA256="' + cls.digest + '"',
                "-I" + str(ROOT / "native"),
                "-I" + str(cls.vendor / "include"),
                str(ROOT / "native/step_monitor_probe.cpp"),
                str(cls.vendor / "lib/libmujoco.so.3.12.0"),
                "-Wl,-rpath," + str(cls.vendor / "lib"),
                "-o",
                str(cls.binary),
            ],
            check=True,
        )

    def run_probe(self, mode):
        model = self.directory / "probe.xml"
        model.write_text(
            """<mujoco><option timestep="0.002" gravity="0 0 0" integrator="implicitfast"/>
          <worldbody><geom name="floor" type="plane" size="2 2 .1"/>
          <body name="ball" pos="0 0 1"><freejoint/><geom name="ball_geom" type="sphere" size=".1"/>
          </body></worldbody></mujoco>"""
        )
        output = self.directory / (mode + ".json")
        subprocess.run(
            [str(self.binary), str(model), mode],
            check=True,
            env=dict(
                os.environ,
                MANIPULATION_DIAGNOSTICS_PATH=str(output),
                MANIPULATION_DIAGNOSTICS_RUN_ID='probe"\\id',
            ),
        )
        return json.loads(output.read_text())

    def test_no_contact_and_complete_step_accounting(self):
        data = self.run_probe("clean")
        check_physics(
            data, run_id='probe"\\id', source_sha256=self.digest, timestep=0.002
        )
        self.assertEqual(data["steps"], 24)
        self.assertEqual(data["mujoco_version"], "3.12.0")

    def test_single_step_transient_contact_is_detected_and_fails_policy(self):
        data = self.run_probe("transient")
        self.assertEqual(data["steps"], 24)
        self.assertEqual(data["contact_steps"], 1)
        self.assertEqual(data["active_contacts"], 1)
        (pair,) = data["pairs"]
        self.assertEqual(set(pair["geoms"]), {"floor", "ball_geom"})
        self.assertAlmostEqual(pair["first_sec"], 11 * 0.002)
        self.assertEqual(pair["first_sec"], pair["last_sec"])
        self.assertLess(pair["min_distance_m"], 0)
        self.assertGreater(pair["max_normal_force_N"], 0)
        with self.assertRaisesRegex(ValueError, "Unexpected MuJoCo contacts"):
            check_physics(
                data, run_id='probe"\\id', source_sha256=self.digest, timestep=0.002
            )

    def test_workcell_inter_arm_contact_is_reported_without_commanding_ros(self):
        from mujoco_ros2_manipulation.workcells.config import load_workcell
        from mujoco_ros2_manipulation.workcells.integrity import check_workcell

        cell = load_workcell()
        if not (cell.output / "source.json").exists():
            self.skipTest("Shared workcell not generated in this matrix job")
        check_workcell(cell)
        model = self.directory / "cell.xml"
        model.write_bytes((cell.output / "cell.xml").read_bytes())
        pose = {
            name: value
            for inst in cell.instances
            for name, value in zip(
                inst.joints, inst.profile.data["verification"]["moveit_target_rad"]
            )
        }
        Path(str(model) + ".pose").write_text(
            "\n".join(f"{name} {value}" for name, value in pose.items())
        )
        output = self.directory / "workcell_contact.json"
        subprocess.run(
            [str(self.binary), str(model), "workcell_contact"],
            check=True,
            env=dict(
                os.environ,
                MANIPULATION_DIAGNOSTICS_PATH=str(output),
                MANIPULATION_DIAGNOSTICS_RUN_ID="workcell_probe",
            ),
        )
        data = json.loads(output.read_text())
        self.assertEqual(data["contact_steps"], 1)
        self.assertTrue(
            any(
                (a.startswith("left_") and b.startswith("right_"))
                or (a.startswith("right_") and b.startswith("left_"))
                for a, b in [p["bodies"] for p in data["pairs"]]
            )
        )
        with self.assertRaisesRegex(ValueError, "Unexpected MuJoCo contacts"):
            check_physics(
                data, run_id="workcell_probe", source_sha256=self.digest, timestep=0.002
            )
