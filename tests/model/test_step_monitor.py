"""Compile the same monitor against the actual pinned C++ MuJoCo runtime."""
import hashlib
import json
import os
from pathlib import Path
import subprocess
import tempfile
import textwrap
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

        # Longer real-engine fixture for wraparound and model-lifetime checks;
        # it does not require a ROS process, controller or simulator server.
        source = cls.directory / "witness_probe.cpp"
        source.write_text(textwrap.dedent(r'''
            #include <memory>
            #include <limits>
            #include "step_monitor.hpp"
            int main(int argc, char **argv) {
              if (argc != 4) return 2;
              char error[1024]{};
              std::unique_ptr<mjModel, decltype(&mj_deleteModel)> model(
                  mj_loadXML(argv[1], nullptr, error, sizeof(error)), mj_deleteModel);
              if (!model) return 3;
              std::unique_ptr<mjData, decltype(&mj_deleteData)> data(
                  mj_makeData(model.get()), mj_deleteData);
              const int count = std::stoi(argv[2]);
              const std::string mode = argv[3];
              manipulation::StepMonitor monitor;
              uint64_t integrated = 0;
              for (int j = 0; j < count; ++j) {
                if (mode == "reload" && j == 300) {
                  // Allocate the new model before freeing the old model, so its
                  // identity is unambiguous even with allocator address reuse.
                  decltype(model) next(mj_loadXML(argv[1], nullptr, error, sizeof(error)),
                                       mj_deleteModel);
                  if (!next) return 4;
                  data.reset(mj_makeData(next.get()));
                  model = std::move(next);
                }
                if (model->nu) {
                  data->ctrl[0] = static_cast<double>(j % 7) / 10.0;
                  if (mode == "nonfinite" && j == 3)
                    data->ctrl[0] = std::numeric_limits<double>::quiet_NaN();
                }
                if (monitor.step(model.get(), data.get())) ++integrated;
              }
              monitor.finish(integrated);
              std::printf("{\"qpos\":%.17g,\"qvel\":%.17g,\"integrated\":%llu}\n",
                          model->nq ? data->qpos[0] : 0.0,
                          model->nv ? data->qvel[0] : 0.0,
                          static_cast<unsigned long long>(integrated));
            }
        '''))
        cls.witness_binary = cls.directory / "witness-probe"
        subprocess.run(
            ["g++", "-std=c++17", "-O2", "-Wall", "-Wextra", "-Werror",
             "-Wconversion", '-DMANIPULATION_MONITOR_SHA256="' + cls.digest + '"',
             "-I" + str(ROOT / "native"), "-I" + str(cls.vendor / "include"),
             str(source), str(cls.vendor / "lib/libmujoco.so.3.12.0"),
             "-Wl,-rpath," + str(cls.vendor / "lib"), "-o", str(cls.witness_binary)],
            check=True,
        )

    def run_probe(self, mode, *, witness=None):
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
                MANIPULATION_CONTACT_STOP="1" if mode.startswith("stop_") else "0",
                MANIPULATION_STEP_WITNESS_PATH=str(witness) if witness else "",
            ),
        )
        return json.loads(output.read_text())

    def run_witness_fixture(self, *, count=600, mode="clean", model=None,
                            witness=True, stop=False):
        path = self.directory / (mode + "-witness-model.xml")
        path.write_text(model or '''<mujoco>
          <option timestep="0.002" gravity="0 0 0" integrator="implicitfast"/>
          <worldbody><body name="arm"><joint name="hinge" type="hinge"/>
            <geom name="arm_geom" type="sphere" size=".1" contype="0" conaffinity="0"/>
          </body></worldbody><actuator><motor name="motor" joint="hinge"/></actuator>
          </mujoco>''')
        output = self.directory / (mode + "-witness.json")
        aggregate = self.directory / (mode + "-witness-aggregate.json")
        result = subprocess.run(
            [str(self.witness_binary), str(path), str(count), mode],
            check=True, capture_output=True, text=True,
            env=dict(os.environ, MANIPULATION_DIAGNOSTICS_PATH=str(aggregate),
                     MANIPULATION_DIAGNOSTICS_RUN_ID='witness"\\id',
                     MANIPULATION_CONTACT_STOP="1" if stop else "0",
                     MANIPULATION_STEP_WITNESS_PATH=str(output) if witness else ""),
        )
        final = json.loads(result.stdout.splitlines()[-1])
        return final, json.loads(output.read_text()) if witness else None

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

    def test_latched_contact_stop_blocks_all_later_physics_steps(self):
        from mujoco_ros2_manipulation.verification.diagnostics import check_stop_physics

        data = self.run_probe("stop_transient")
        check_stop_physics(
            data, run_id='probe"\\id', source_sha256=self.digest, timestep=0.002
        )
        self.assertEqual(data["steps"], 12)
        self.assertEqual(data["engine_steps"], 12)
        self.assertEqual(data["hold_ticks"], 12)
        self.assertEqual(data["stop_step"], 12)
        self.assertAlmostEqual(data["hold_time_sec"], 0.048)

    def test_native_guard_freezes_all_twelve_joints_without_ros(self):
        from mujoco_ros2_manipulation.verification.diagnostics import check_stop_physics

        # Reuse the contact-pose preparation from the diagnostic regression.
        self.test_workcell_inter_arm_contact_is_reported_without_commanding_ros()
        model = self.directory / "cell.xml"
        output = self.directory / "stopped_workcell.json"
        subprocess.run(
            [str(self.binary), str(model), "stop_workcell_contact"],
            check=True,
            env=dict(
                os.environ,
                MANIPULATION_DIAGNOSTICS_PATH=str(output),
                MANIPULATION_DIAGNOSTICS_RUN_ID="native_stop",
                MANIPULATION_CONTACT_STOP="1",
            ),
        )
        data = json.loads(output.read_text())
        check_stop_physics(
            data, run_id="native_stop", source_sha256=self.digest, timestep=0.002
        )
        self.assertEqual(len(data["stopped_qpos"]), 12)
        self.assertEqual(data["steps"], 1)
        self.assertEqual(data["hold_ticks"], 23)

    def test_clock_deadlines_progress_pause_and_startup(self):
        self.run_probe("clean")  # Writes the small model consumed by the policy probe.
        subprocess.run(
            [str(self.binary), str(self.directory / "probe.xml"), "clock_policy"],
            check=True,
        )

    def test_clock_stop_without_contact_freezes_physics_and_keeps_first_reason(self):
        from mujoco_ros2_manipulation.verification.diagnostics import (
            check_clock_stop_physics,
        )

        data = self.run_probe("stop_clock")
        check_clock_stop_physics(
            data, run_id='probe"\\id', source_sha256=self.digest, timestep=0.002
        )
        self.assertEqual(data["steps"], 11)
        self.assertEqual(data["hold_ticks"], 13)
        self.assertEqual(data["stop_reason"], "clock_stall")

    def test_witness_keeps_bounded_contiguous_steps_and_exact_applied_control(self):
        plain, _ = self.run_witness_fixture(witness=False)
        final, data = self.run_witness_fixture()
        self.assertEqual(final, plain, "Observation must not change physical state")
        self.assertEqual(data["schema_version"], 1)
        self.assertEqual(data["monitor_source_sha256"], self.digest)
        self.assertEqual(data["run_id"], 'witness"\\id')
        self.assertTrue(data["complete"])
        self.assertIsNone(data["error"])
        self.assertEqual(data["engine_steps"], 600)
        self.assertEqual(data["captured_steps"], 600)
        self.assertEqual(data["overwritten_steps"], 88)
        frames = data["frames"]
        self.assertEqual(len(frames), data["frame_capacity"])
        self.assertEqual([f["step_id"] for f in frames], list(range(89, 601)))
        self.assertIsNone(data["trigger"])
        self.assertIsNone(data["post_latch"])
        self.assertEqual(data["dimensions"], dict(nq=1, nv=1, nu=1))
        self.assertEqual(data["joints"][0]["name"], "hinge")
        self.assertEqual(data["actuators"][0]["name"], "motor")
        self.assertEqual(data["actuators"][0]["transmission_ids"][0], 0)
        for index, frame in enumerate(frames):
            self.assertEqual(frame["contact_count"], 0)
            self.assertAlmostEqual(frame["post"]["simulation_sec"] -
                                   frame["pre"]["simulation_sec"], 0.002)
            self.assertLessEqual(frame["pre"]["steady_ns"], frame["post"]["steady_ns"])
            expected = ((frame["step_id"] - 1) % 7) / 10
            self.assertEqual(frame["pre"]["ctrl"], [expected])
            self.assertEqual(frame["post"]["ctrl"], [expected])
            if index:
                previous = frames[index - 1]["post"]
                self.assertEqual(frame["pre"]["qpos"], previous["qpos"])
                self.assertEqual(frame["pre"]["qvel"], previous["qvel"])
                self.assertEqual(frame["pre"]["simulation_sec"], previous["simulation_sec"])

    def test_witness_distinguishes_contact_evaluation_post_step_and_latched_state(self):
        output = self.directory / "stop-transient-witness.json"
        aggregate = self.run_probe("stop_transient", witness=output)
        data = json.loads(output.read_text())
        self.assertEqual(aggregate["steps"], 12)
        self.assertEqual(aggregate["hold_ticks"], 12)
        self.assertEqual(len(data["frames"]), 12)
        self.assertEqual(data["trigger"]["reason"], "contact")
        self.assertEqual(data["trigger"]["step_id"], 12)
        self.assertAlmostEqual(data["trigger"]["contact_evaluation_time_sec"], .022)
        frame = data["frames"][-1]
        self.assertEqual(frame["contact_count"], 1)
        self.assertAlmostEqual(frame["pre"]["qpos"][2], .05)
        self.assertEqual(frame["pre"]["qvel"], [0] * 6)
        self.assertNotEqual(frame["post"]["qvel"], [0] * 6)
        self.assertLess(data["trigger_contacts"][0]["distance_m"], 0)
        stopped = data["post_latch"]
        self.assertEqual(stopped["phase"], "post_latch_no_integration")
        self.assertEqual(stopped["step_id"], frame["step_id"])
        self.assertEqual(stopped["state"]["qpos"], frame["post"]["qpos"])
        self.assertEqual(stopped["state"]["qvel"], [0] * 6)
        self.assertEqual(stopped["state"]["simulation_sec"], frame["post"]["simulation_sec"])
        self.assertLessEqual(frame["post"]["steady_ns"], stopped["state"]["steady_ns"])

    def test_witness_freezes_at_first_contact_even_when_interlock_is_disabled(self):
        output = self.directory / "transient-witness.json"
        aggregate = self.run_probe("transient", witness=output)
        data = json.loads(output.read_text())
        self.assertEqual(aggregate["steps"], 24)
        self.assertFalse(aggregate["stop_latched"])
        self.assertEqual(data["captured_steps"], 12)
        self.assertEqual(data["frames"][-1]["step_id"], 12)
        self.assertIsNone(data["post_latch"])

    def test_clock_stop_freezes_prior_window_without_inventing_a_contact(self):
        output = self.directory / "clock-witness.json"
        aggregate = self.run_probe("stop_clock", witness=output)
        data = json.loads(output.read_text())
        self.assertEqual(aggregate["hold_ticks"], 13)
        self.assertEqual(data["trigger"]["reason"], "clock_stall")
        self.assertEqual(data["trigger"]["step_id"], 11)
        self.assertEqual(data["trigger_contacts"], [])
        self.assertEqual(data["post_latch"]["reason"], "clock_stall")
        self.assertEqual(len(data["frames"]), 11)
        self.assertTrue(all(frame["contact_count"] == 0 for frame in data["frames"]))

    def test_unwritable_witness_does_not_change_contact_stop(self):
        plain = self.run_probe("stop_transient")
        bad = self.run_probe("stop_transient", witness=self.directory / "missing" / "witness.json")
        for field in ("steps", "engine_steps", "hold_ticks", "stopped_qpos",
                      "stop_latched", "stop_step", "stop_time_sec", "pairs"):
            self.assertEqual(bad[field], plain[field])

    def test_witness_cannot_overwrite_aggregate_using_a_path_alias(self):
        alias = self.directory / "alias"
        alias.mkdir(exist_ok=True)
        aggregate = self.run_probe("stop_transient", witness=alias / ".." / "stop_transient.json")
        self.assertTrue(aggregate["stop_latched"])
        self.assertNotIn("frames", aggregate)

    def test_reloaded_model_disables_witness_and_preserves_initial_metadata(self):
        final, data = self.run_witness_fixture(mode="reload")
        self.assertEqual(final["integrated"], 600)
        self.assertTrue(data["complete"])
        self.assertEqual(data["error"], "model_changed")
        self.assertEqual(data["captured_steps"], 300)
        self.assertEqual(len(data["frames"]), 300)
        self.assertEqual(data["joints"][0]["name"], "hinge")

    def test_witness_handles_zero_state_and_nonfinite_values_as_valid_json(self):
        final, empty = self.run_witness_fixture(count=3, model="<mujoco/>")
        self.assertEqual(final["integrated"], 3)
        self.assertEqual(empty["dimensions"], dict(nq=0, nv=0, nu=0))
        self.assertEqual(empty["frames"][-1]["post"]["ctrl"], [])
        _, bad = self.run_witness_fixture(count=8, mode="nonfinite")
        self.assertIsNone(bad["frames"][3]["pre"]["ctrl"][0])

    def test_witness_bounds_contact_details_and_disables_oversized_state(self):
        def spheres(count):
            bodies = "".join(f'<body pos="{j * .3} 0 .05"><freejoint/>'
                             '<geom type="sphere" size=".1"/></body>'
                             for j in range(count))
            return ('<mujoco><option timestep=".002" integrator="implicitfast"/>'
                    '<worldbody><geom type="plane" size="100 100 .1"/>' +
                    bodies + '</worldbody></mujoco>')

        _, contacts = self.run_witness_fixture(count=2, model=spheres(40), stop=True)
        self.assertEqual(contacts["trigger"]["step_id"], 1)
        self.assertEqual(contacts["frames"][0]["contact_count"], 40)
        self.assertEqual(len(contacts["trigger_contacts"]), contacts["contact_capacity"])
        self.assertIsNotNone(contacts["post_latch"])
        plain, _ = self.run_witness_fixture(count=2, model=spheres(80), stop=True, witness=False)
        oversized, disabled = self.run_witness_fixture(count=2, model=spheres(80), stop=True)
        self.assertEqual(oversized, plain)
        self.assertEqual(disabled["error"], "state_dimensions_exceed_limit")
        self.assertEqual(disabled["frames"], [])
        self.assertIsNone(disabled["post_latch"])
