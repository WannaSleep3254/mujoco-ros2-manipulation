"""Hold exactness, unequal horizons, search ordering and analytic path metrics."""
from copy import deepcopy
from types import SimpleNamespace as NS
import unittest

from mujoco_ros2_manipulation.workcells.interference import Spline
from mujoco_ros2_manipulation.workcells.timing import delayed_curve, choose_delay
from mujoco_ros2_manipulation.workcells.path_quality import (
    sample_schedule,
    evaluate_path,
)


def curve(name, derivative=False):
    return Spline(
        [name],
        [
            dict(
                t=0.0,
                p=[0.0],
                v=[0.0] if derivative else [],
                a=[0.0] if derivative else [],
            ),
            dict(
                t=2.0,
                p=[1.0],
                v=[0.0] if derivative else [],
                a=[-0.1] if derivative else [],
            ),
        ],
    )


def cell():
    return NS(
        joints=["l", "r"],
        instances=[NS(id="left", joints=["l"]), NS(id="right", joints=["r"])],
    )


class AnalyticDistance:
    def at(self, state):
        l, r = state["l"], state["r"]
        return dict(
            distance_m=((l - 0.5) ** 2 + (r - 0.5) ** 2) ** 0.5 - 0.2,
            bodies=["left", "right"],
            nearest_points_m=[[l, 0, 0], [0, r, 0]],
            tips_m=[[l, 0, 0], [0, r, 0]],
        )


class TimingTests(unittest.TestCase):
    def test_delay_preserves_moving_polynomial_and_derivatives_without_endpoint_hold_overshoot(
        self,
    ):
        source = curve("l", True)
        original = deepcopy(source.record())
        shift = delayed_curve(source, 0.8)
        self.assertEqual(source.record(), original)
        self.assertEqual(shift.points[1]["p"], source.points[0]["p"])
        self.assertEqual(shift.points[-1]["a"], [-0.1])
        for t in (0, 0.1, 0.79):
            self.assertEqual(shift.position(t), [0.0])
        for t in (0, 0.2, 0.7, 1.6, 2):
            self.assertAlmostEqual(
                shift.position(t + 0.8)[0], source.position(t)[0], places=12
            )
        # The completed arm is held by JTC after the end; no artificial spline segment is appended.
        states = list(
            sample_schedule(
                [source, delayed_curve(curve("r", True), 0.8)], {"l": 0.0, "r": 0.0}
            )
        )
        self.assertTrue(all(s["l"] == 1 for t, s in states if t >= 2))
        for delay in (-0.1, float("nan"), 34):
            with self.assertRaises(ValueError):
                delayed_curve(source, delay)
        moving = deepcopy(source)
        moving.points[0]["v"] = [0.2]
        with self.assertRaises(ValueError):
            delayed_curve(moving, 0.8)

    def test_search_selects_smallest_feasible_grid_delay_and_fails_closed_when_budget_insufficient(
        self,
    ):
        curves = [curve("l"), curve("r")]
        initial = {"l": 0.0, "r": 0.0}
        policy = dict(
            sample_dt_sec=0.01,
            min_inter_arm_distance_m=0.05,
            tracking_reserve_m=0.0,
            delay_step_sec=0.2,
            max_delay_sec=1.2,
        )
        result = choose_delay(curves, initial, cell(), AnalyticDistance(), policy)
        self.assertEqual(result["delayed_arm"], "left")
        self.assertAlmostEqual(result["delay_sec"], 0.8)
        self.assertGreaterEqual(result["evaluation"]["minimum"]["distance_m"], 0.05)
        self.assertTrue(any(not c["complete"] for c in result["candidates"]))
        policy["max_delay_sec"] = 0.4
        with self.assertRaisesRegex(ValueError, "No departure-only solution"):
            choose_delay(curves, initial, cell(), AnalyticDistance(), policy)
        # A long source curve must exhaust the admissible horizon cleanly, not lose failure evidence.
        long_curves = [curve("l"), curve("r")]
        for c in long_curves:
            c.points[-1]["t"] = 34.9
            c.times[-1] = 34.9
        with self.assertRaisesRegex(ValueError, "No departure-only solution"):
            choose_delay(long_curves, initial, cell(), AnalyticDistance(), policy)

    def test_path_metrics_are_geometry_preserving_and_detect_close_approach(self):
        initial = {"l": 0.0, "r": 0.0}
        curves = [curve("l"), curve("r")]
        reference = evaluate_path(curves, initial, cell(), AnalyticDistance())
        timed = evaluate_path(
            [delayed_curve(curves[0], 0.8), curves[1]],
            initial,
            cell(),
            AnalyticDistance(),
        )
        self.assertAlmostEqual(reference["minimum"]["distance_m"], -0.2)
        for result in (reference, timed):
            self.assertAlmostEqual(result["joint_total_variation_rad"], 2)
            self.assertAlmostEqual(result["tip_frame_path_m"]["left"], 1)
            self.assertAlmostEqual(result["tip_frame_path_m"]["right"], 1)
        self.assertGreater(timed["minimum"]["distance_m"], 0.05)
        with self.assertRaises(ValueError):
            list(sample_schedule([curves[0], curves[0]], initial))


class TimingPlanIntegrityTests(unittest.TestCase):
    def test_mutated_plan_geometry_stale_policy_and_different_evaluator_are_rejected(
        self,
    ):
        import json
        import tempfile
        from pathlib import Path
        from unittest.mock import patch
        from mujoco_ros2_manipulation.workcells.timing import load_timing
        from mujoco_ros2_manipulation.model_build.integrity import file_sha256

        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            bundle_path = root / "bundle.json"
            bundle_path.write_text("{}")
            policy_path = root / "policy.yaml"
            policy_path.write_text("locked")
            bundle = dict(case_sha256="case", model_manifest_sha256="model")
            curves = dict(left=curve("l"), right=curve("r"))
            policy = dict(max_delay_sec=2.0, min_inter_arm_distance_m=0.02)
            evaluator = NS(metadata=lambda: dict(engine="native"))
            record = dict(
                schema_version=1,
                bundle_sha256=file_sha256(bundle_path),
                case_sha256="case",
                model_manifest_sha256="model",
                policy=policy,
                policy_sha256=file_sha256(policy_path),
                evaluator=evaluator.metadata(),
                delayed_arm="left",
                delay_sec=0.8,
                curves=[
                    delayed_curve(curves["left"], 0.8).record(),
                    curves["right"].record(),
                ],
            )
            path = root / "plan.json"
            with patch(
                "mujoco_ros2_manipulation.workcells.timing.load_bundle",
                return_value=(bundle, curves),
            ), patch(
                "mujoco_ros2_manipulation.workcells.timing.load_policy",
                return_value=(policy_path, policy),
            ):
                path.write_text(json.dumps(record))
                load_timing(path, bundle_path, cell(), evaluator)
                for change in ("shape", "evaluator", "policy", "source", "delay"):
                    bad = deepcopy(record)
                    if change == "shape":
                        bad["curves"][0]["points"][1]["p"] = [0.1]
                    if change == "evaluator":
                        bad["evaluator"]["engine"] = "different"
                    if change == "policy":
                        bad["policy"]["min_inter_arm_distance_m"] = 0.001
                    if change == "source":
                        bad["bundle_sha256"] = "stale"
                    if change == "delay":
                        bad["delay_sec"] = 3
                    path.write_text(json.dumps(bad))
                    with self.subTest(change=change), self.assertRaises(ValueError):
                        load_timing(path, bundle_path, cell(), evaluator)


class TimingComparisonTests(unittest.TestCase):
    def test_reports_must_match_and_cannot_hide_clearance_violations_or_nonfinite_metrics(
        self,
    ):
        import json, tempfile
        from pathlib import Path
        from mujoco_ros2_manipulation.model_build.integrity import file_sha256
        from mujoco_ros2_manipulation.workcells.timing_comparison import compare_timing

        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            plan = root / "plan.json"
            policy = dict(min_inter_arm_distance_m=0.02, tracking_reserve_m=0.01)
            plan.write_text(
                json.dumps(
                    dict(
                        bundle_sha256="source",
                        policy=policy,
                        reference=dict(
                            joint_total_variation_rad=2.0,
                            tip_frame_path_m=dict(left=1.0, right=1.0),
                        ),
                        delay_sec=1.0,
                        delayed_arm="left",
                    )
                )
            )

            def metrics(travel, gap):
                return dict(
                    complete=True,
                    samples=100,
                    minimum=dict(distance_m=gap),
                    joint_total_variation_rad=travel,
                    duration_sec=10.0,
                    tip_frame_path_m=dict(left=travel / 2, right=travel / 2),
                )

            records = []
            for mode in ("baseline", "timing"):
                records.append(
                    dict(
                        timing_mode=mode,
                        run_id=mode,
                        bundle_sha256="source",
                        timing_plan_sha256=file_sha256(plan),
                        model_manifest_sha256="model",
                        case_file_sha256="case",
                        policy=policy,
                        policy_sha256="policy",
                        evaluator={},
                        initial_commanded_rad={},
                        goal_constraint_rad={},
                        comparison_duration_sec=11.0,
                        status="passed",
                        motion_checks_passed=True,
                        physics_checks_passed=True,
                        shutdown_clean=True,
                        process_group_gone=True,
                        goal_terminal_statuses=[4, 4],
                        physics=dict(contacts=0, stop_latched=False, steps=100),
                        contact_injection_requests=0,
                        verifier_cancel_requests=0,
                        max_goal_error_rad=0.004,
                        planned=metrics(10.0 if mode == "baseline" else 2.0, 0.04),
                        observed=metrics(10.0 if mode == "baseline" else 2.0, 0.035),
                        motion_duration_sec=10.0 if mode == "baseline" else 3.0,
                    )
                )
            paths = [root / "baseline.json", root / "timing.json"]

            def write(data):
                for p, r in zip(paths, data):
                    p.write_text(json.dumps(r))

            write(records)
            self.assertTrue(compare_timing(paths, plan)["matched"])
            for case in ("clearance", "nan", "source", "failed", "geometry"):
                bad = deepcopy(records)
                if case == "clearance":
                    bad[1]["observed"]["minimum"]["distance_m"] = 0.019
                if case == "nan":
                    bad[1]["planned"]["minimum"]["distance_m"] = float("nan")
                if case == "source":
                    bad[1]["bundle_sha256"] = "other"
                if case == "failed":
                    bad[1]["status"] = "failed"
                if case == "geometry":
                    bad[1]["planned"]["joint_total_variation_rad"] = 3.0
                write(bad)
                with self.subTest(case=case), self.assertRaises(ValueError):
                    compare_timing(paths, plan)


class TimingFailureEvidenceTests(unittest.TestCase):
    def test_infeasible_search_preserves_candidates_and_does_not_overwrite_failure(
        self,
    ):
        import json, tempfile
        from pathlib import Path
        from unittest.mock import patch
        from mujoco_ros2_manipulation.workcells.timing import (
            prepare_timing,
            TimingRejected,
        )

        with tempfile.TemporaryDirectory() as tmp:
            root = Path(tmp)
            bundle = root / "bundle.json"
            bundle.write_text("{}")
            policy = root / "policy.yaml"
            policy.write_text("policy")
            output = root / "runtime" / "plan.json"
            candidate = dict(delay_sec=0.1, minimum=dict(distance_m=-0.02))
            source = dict(case_id="case", initial_rad=dict(l=0.0, r=0.0))
            with patch(
                "mujoco_ros2_manipulation.workcells.timing.load_bundle",
                return_value=(source, dict(left=curve("l"), right=curve("r"))),
            ), patch(
                "mujoco_ros2_manipulation.workcells.timing.load_policy",
                return_value=(policy, dict(case="case")),
            ), patch(
                "mujoco_ros2_manipulation.workcells.timing.choose_delay",
                side_effect=TimingRejected([candidate]),
            ):
                with self.assertRaises(TimingRejected):
                    prepare_timing(
                        bundle, output, NS(root=root), NS(metadata=lambda: {})
                    )
                failure = output.with_suffix(".failure.json")
                original = failure.read_bytes()
                self.assertEqual(json.loads(original)["candidates"], [candidate])
                self.assertFalse(output.exists())
                with self.assertRaisesRegex(ValueError, "new runtime"):
                    prepare_timing(
                        bundle, output, NS(root=root), NS(metadata=lambda: {})
                    )
                self.assertEqual(failure.read_bytes(), original)
