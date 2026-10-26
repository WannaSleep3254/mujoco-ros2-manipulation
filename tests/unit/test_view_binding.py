"""View identity is strict and detached; no ROS, MuJoCo, browser, or GL needed."""
from copy import deepcopy
import hashlib
import json
import math
import unittest

from mujoco_ros2_manipulation.operator_console.view_binding import make_binding, binding_id, envelope


OPERATOR = "a"*32
JOB = "b"*32
MOTION = "c"*32
REQUEST = "d"*32


def pose():
    return dict(schema_version=1, scene_sha256="1"*64, model_sha256="2"*64,
                geoms=[dict(id=0, position=[1., 2., 3.], rotation=[0., -1., 0., 1., 0., 0., 0., 0., 1.])])


class ViewBindingTests(unittest.TestCase):
    def binding(self, mode="live", **kwargs):
        return make_binding(OPERATOR, 3, "workcell:fr5_ur5e", mode, **kwargs)

    def test_canonical_identity_is_order_independent_and_changes_with_owner_generation(self):
        binding = self.binding()
        self.assertEqual(set(binding), {"operator_session_id", "generation", "source_id", "mode",
                                        "job_id", "run_id", "motion_session_id", "request_id"})
        expected = hashlib.sha256(json.dumps(binding, sort_keys=True, separators=(",", ":"),
                                             ensure_ascii=True, allow_nan=False).encode("ascii")).hexdigest()
        self.assertEqual(binding_id(binding), expected)
        self.assertEqual(binding_id(dict(reversed(list(binding.items())))), expected)
        for key, value in (("operator_session_id", "f"*32), ("generation", 4), ("source_id", "profile:fr5")):
            changed = dict(binding, **{key:value})
            self.assertNotEqual(binding_id(changed), expected)

    def test_exact_fields_and_safe_generation_are_required(self):
        original = self.binding()
        for value in (dict(original, unknown="ignored"), {k:v for k,v in original.items() if k != "run_id"}):
            with self.subTest(value=value), self.assertRaises(ValueError):
                binding_id(value)
        for value in (True, 1.0, -1, 2**53):
            with self.subTest(value=value), self.assertRaises(ValueError):
                make_binding(OPERATOR, value, "fr5", "live")
        self.assertEqual(make_binding(OPERATOR, 2**53-1, "fr5", "live")["generation"], 2**53-1)

    def test_source_and_run_names_are_identifiers_not_paths_or_urls(self):
        for value in ("/absolute/cell.xml", "../cell", "file:cell", "https://host/scene", "user scene", "fr5.xml", "é", "a"*129):
            with self.subTest(value=value), self.assertRaises(ValueError):
                make_binding(OPERATOR, 0, value, "live")
        for value in ("../run", "/absolute/run", "a/b", "a\\b", ".", "..", "run with spaces"):
            with self.subTest(value=value), self.assertRaises(ValueError):
                self.binding(job_id=JOB, run_id=value)
        accepted = self.binding(job_id=JOB, run_id="20261023T132013.123456Z-01fe22aa")
        self.assertEqual(accepted["job_id"], JOB)

    def test_mode_and_execution_id_pair_rules(self):
        self.assertEqual(self.binding()["request_id"], None)
        for kwargs in (dict(job_id=JOB), dict(run_id="run-1"), dict(request_id=REQUEST),
                       dict(job_id=True, run_id="run-1"), dict(motion_session_id="private-server")):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                self.binding(**kwargs)
        for kwargs in (dict(job_id=JOB, run_id="run-1"), dict(motion_session_id=MOTION), dict(request_id=REQUEST)):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                self.binding("model_preview", **kwargs)
        with self.assertRaisesRegex(ValueError, "saved request"):
            self.binding("plan_preview")
        self.assertEqual(self.binding("plan_preview", request_id=REQUEST)["request_id"], REQUEST)
        self.assertEqual(self.binding("model_preview")["mode"], "model_preview")

    def test_live_envelope_preserves_stale_age_and_simulation_clock_without_invented_freshness(self):
        binding, compiled = self.binding(motion_session_id=MOTION), pose()
        before = deepcopy((binding, compiled))
        result = envelope(binding, compiled, 8, source_state_age_sec=1.25, simulation_sec=0.)
        self.assertEqual(result["binding_id"], binding_id(binding))
        self.assertEqual(result["source_state_age_sec"], 1.25)
        self.assertEqual(result["simulation_sec"], 0.)
        self.assertNotIn("preview_time_sec", result)
        binding["generation"] = 99
        compiled["geoms"][0]["position"][0] = 999
        self.assertEqual(result["binding"], before[0])
        self.assertEqual(result["pose"], before[1])
        result["pose"]["geoms"][0]["position"][0] = -99
        self.assertEqual(before[1]["geoms"][0]["position"][0], 1.)

    def test_preview_envelopes_use_null_live_age_and_explicit_plan_playback_time(self):
        model = envelope(self.binding("model_preview"), pose(), 1)
        self.assertIsNone(model["source_state_age_sec"])
        self.assertIsNone(model["simulation_sec"])
        self.assertNotIn("preview_time_sec", model)
        plan = self.binding("plan_preview", request_id=REQUEST, motion_session_id=MOTION)
        with self.assertRaises(ValueError):
            envelope(plan, pose(), 1)
        result = envelope(plan, pose(), 1, preview_time_sec=0.)
        self.assertEqual(result["preview_time_sec"], 0.)
        for binding, kwargs in ((self.binding(), {}), (plan, dict(preview_time_sec=1,source_state_age_sec=0)),
                                (self.binding("model_preview"), dict(preview_time_sec=0))):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                envelope(binding, pose(), 1, **kwargs)

    def test_pose_catalog_schema_and_private_extra_fields_fail_closed(self):
        source = pose()
        changes = [dict(source, schema_version=True), dict(source, model_sha256="A"*64),
                   dict(source, source_path="/absolute/private.xml"), dict(source, geoms=[dict(source["geoms"][0], id=1)]),
                   dict(source, geoms=source["geoms"]*2),
                   dict(source, geoms=[dict(source["geoms"][0], error="private")])]
        for changed in changes:
            with self.subTest(changed=changed), self.assertRaises(ValueError):
                envelope(self.binding(), changed, 1, source_state_age_sec=0)
        empty = dict(source, geoms=[])
        self.assertEqual(envelope(self.binding(), empty, 1, source_state_age_sec=0)["pose"]["geoms"], [])

    def test_nonfinite_unbounded_and_improper_rotations_are_rejected(self):
        for invalid in ([True, 0, 0], [math.nan, 0, 0], [math.inf, 0, 0], [1e10, 0, 0], [0, 0]):
            bad = pose(); bad["geoms"][0]["position"] = invalid
            with self.subTest(invalid=invalid), self.assertRaises(ValueError):
                envelope(self.binding(), bad, 1, source_state_age_sec=0)
        for invalid in ([0.]*9, [1.,0,0,0,1.,0,0,0,-1.], [1.,0,0,.1,1.,0,0,0,1.], [math.nan]*9):
            bad = pose(); bad["geoms"][0]["rotation"] = invalid
            with self.subTest(invalid=invalid), self.assertRaises(ValueError):
                envelope(self.binding(), bad, 1, source_state_age_sec=0)

    def test_sequence_and_clock_numbers_are_strict_finite_and_bounded(self):
        for sequence in (0, -1, True, 1., 2**53):
            with self.subTest(sequence=sequence), self.assertRaises(ValueError):
                envelope(self.binding(), pose(), sequence, source_state_age_sec=0)
        for key in ("source_state_age_sec", "simulation_sec"):
            for value in (True, -1, math.nan, math.inf, 1e10, 10**1000):
                args = dict(source_state_age_sec=0, simulation_sec=1); args[key] = value
                with self.subTest(key=key, value=value), self.assertRaises(ValueError):
                    envelope(self.binding(), pose(), 1, **args)


if __name__ == "__main__":
    unittest.main()
