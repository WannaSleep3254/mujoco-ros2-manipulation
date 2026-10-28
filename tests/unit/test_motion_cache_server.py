"""Real cache/approval orchestration with ROS transport replaced, no dispatch hardware."""
from copy import deepcopy
from types import SimpleNamespace
import unittest
from unittest.mock import Mock

from tests.unit import test_motion_review_server as review_fixture
from mujoco_ros2_manipulation.workcells.interference import Spline
from mujoco_ros2_manipulation.workcells.motion_request import command
from mujoco_ros2_manipulation.workcells.planning_budget import PlanningBudgetExceeded


class MotionCacheServerTests(unittest.TestCase):
    send = review_fixture.ReviewServerTests.send
    make_review = review_fixture.ReviewServerTests.make_review

    def setUp(self):
        review_fixture.ReviewServerTests.setUp(self)
        n = self.node
        n.max_plan_candidates = 1
        n.goal_tolerance_rad = .003
        for row in n.metadata["joint_limits"].values(): row["velocity"] = 1.
        n.configure_planning(trajectory_cache=True)
        n.planning_source_sha256 = "a"*64
        n._planning_source_sha256 = Mock(return_value=n.planning_source_sha256)
        n._native_cache_provenance = Mock(return_value=dict(engine="MuJoCo 3.12.0", library_sha256="b"*64))
        n._planner_configuration_evidence = Mock(return_value=dict(
            requested_planner_id=n.planner_id, observed_type="geometric::RRTConnect", selected_planner_id=None))
        self.curve = Spline(n.joint_names, [dict(t=0.,p=[0.,0.],v=[0.,0.],a=[0.,0.]),
            dict(t=1.,p=[.1,.1],v=[0.,0.],a=[0.,0.])])
        n.coordinated_path.return_value = self.curve

    def cold(self):
        return self.make_review()

    def test_cold_store_then_warm_hit_gets_new_review_and_all_native_moveit_checks(self):
        first, cold, _ = self.cold()
        cache = self.node.trajectory_cache
        cache.store = Mock(wraps=cache.store)
        self.node.coordinated_path.reset_mock()
        self.node.validity_of.reset_mock()
        second, warm, _ = self.make_review()
        self.assertFalse(cold["trajectory_cache"]["lookup"]["hit"])
        self.assertTrue(cold["trajectory_cache"]["store"]["stored"])
        self.assertTrue(warm["trajectory_cache"]["lookup"]["hit"])
        self.assertGreater(warm["trajectory_cache"]["full_revalidation_wall_sec"], 0)
        self.assertGreater(self.node.validity_of.call_count, 2)
        self.assertNotEqual(first["request_id"], second["request_id"])
        self.assertNotEqual(cold["review"]["plan_sha256"], warm["review"]["plan_sha256"])
        self.assertEqual(warm["review_payload"]["trajectories"], [self.curve.record()])
        self.node.coordinated_path.assert_not_called()
        cache.store.assert_not_called()
        self.node.dispatch.assert_not_called()

    def test_invalid_cached_path_is_evicted_then_freshly_planned_with_same_budget(self):
        self.cold()
        self.node.coordinated_path.reset_mock()
        original = self.node.validate_motion_path
        budget_ids = []
        rejected = [False]
        def check(curves, initial, trace):
            budget_ids.append(id(self.node.compute_budget))
            if not rejected[0]:
                rejected[0] = True
                raise review_fixture.Server.validate_motion_path.__globals__["MotionPathRejected"]("Injected path rejection")
            return original(curves, initial, trace)
        self.node.validate_motion_path = check
        _, trace, _ = self.make_review()
        record = trace["trajectory_cache"]
        self.assertEqual(record["reason"], "validation_rejected_fallback")
        self.assertTrue(record["fallback"])
        self.assertGreater(record["fallback_planning_wall_sec"], 0)
        self.assertEqual(len(set(budget_ids)), 1)
        self.node.coordinated_path.assert_called_once()
        self.node.dispatch.assert_not_called()

    def test_cancellation_budget_environment_and_transport_failures_never_fallback(self):
        globals_ = review_fixture.Server.validate_motion_path.__globals__
        for error in (globals_["CommandCancelled"]("cancel"), PlanningBudgetExceeded("budget"),
                      globals_["MotionEnvironmentChanged"]("changed"), RuntimeError("ROS transport failed"),
                      ValueError("Transport callback failed")):
            with self.subTest(error=type(error).__name__):
                self.setUp()
                self.cold()
                self.node.coordinated_path.reset_mock()
                cache = self.node.trajectory_cache
                cache.evict = Mock(wraps=cache.evict)
                cache.store = Mock(wraps=cache.store)
                self.node.validate_motion_path = Mock(side_effect=error)
                payload = command(dict(left=.1,right=.1))
                self.send(payload); self.node.process_next()
                result = self.node.replies[payload["request_id"]]
                self.assertNotEqual(result["status"], "succeeded")
                self.assertEqual(result["error"]["type"], type(error).__name__)
                self.node.coordinated_path.assert_not_called()
                self.node.dispatch.assert_not_called()
                cache.evict.assert_not_called(); cache.store.assert_not_called()

    def test_anchor_checkpoint_valueerror_is_not_a_path_rejection_or_fallback(self):
        self.cold()
        self.node.coordinated_path.reset_mock()
        cache = self.node.trajectory_cache
        cache.evict = Mock(wraps=cache.evict)
        original = self.node._validate_cached_anchor
        def interrupt(curves, initial, target):
            checkpoint = self.node.compute_checkpoint
            self.node.compute_checkpoint = Mock(side_effect=ValueError("ROS callback failed"))
            try: return original(curves, initial, target)
            finally: self.node.compute_checkpoint = checkpoint
        self.node._validate_cached_anchor = interrupt
        payload = command(dict(left=.1,right=.1))
        self.send(payload); self.node.process_next()
        result = self.node.replies[payload["request_id"]]
        self.assertEqual(result["status"], "failed")
        self.assertEqual(result["error"]["type"], "ValueError")
        cache.evict.assert_not_called(); self.node.coordinated_path.assert_not_called()
        self.node.dispatch.assert_not_called()

    def test_actual_native_or_moveit_rejection_performs_fresh_fallback(self):
        for kind in ("native", "moveit"):
            with self.subTest(kind=kind):
                self.setUp(); self.cold()
                self.node.coordinated_path.reset_mock()
                rejected = [False]
                if kind == "native":
                    baseline = deepcopy(self.node.evaluator.at.return_value)
                    def evaluate(state):
                        value = deepcopy(baseline)
                        if not rejected[0] and .03 < state["left"] < .07:
                            value["distance_m"] = .01
                            rejected[0] = True
                        return value
                    self.node.evaluator.at.side_effect = evaluate
                else:
                    def valid(state):
                        if not rejected[0] and .03 < state["left"] < .07:
                            rejected[0] = True
                            return SimpleNamespace(valid=False)
                        return SimpleNamespace(valid=True)
                    self.node.validity_of.side_effect = valid
                _, trace, _ = self.make_review()
                self.assertTrue(rejected[0])
                self.assertTrue(trace["trajectory_cache"]["fallback"])
                self.node.coordinated_path.assert_called_once()
                self.node.dispatch.assert_not_called()

    def test_scene_or_source_change_after_cache_lookup_fails_without_fallback(self):
        for mutation in ("scene", "source", "state"):
            with self.subTest(mutation=mutation):
                self.setUp(); self.cold()
                self.node.coordinated_path.reset_mock()
                cache = self.node.trajectory_cache
                cache.evict = Mock(wraps=cache.evict)
                original = self.node.validate_motion_path
                def change(curves, initial, trace):
                    original(curves, initial, trace)
                    if mutation == "scene":
                        self.node._review_runtime_identity.return_value = dict(self.identity, planning_scene_sha256="c"*64)
                    elif mutation == "source": self.node._planning_source_sha256.return_value = "b"*64
                    else: self.node.actual["left"] = .002
                self.node.validate_motion_path = change
                payload = command(dict(left=.1,right=.1))
                self.send(payload); self.node.process_next()
                self.assertEqual(self.node.replies[payload["request_id"]]["status"], "failed")
                self.node.coordinated_path.assert_not_called()
                self.node.dispatch.assert_not_called(); cache.evict.assert_not_called()

    def test_approved_execution_uses_the_exact_review_without_a_cache_lookup(self):
        _, _, approval = self.cold()
        cache = self.node.trajectory_cache
        cache.lookup = Mock(wraps=cache.lookup)
        self.node.coordinated_path.reset_mock()
        self.send(approval, "approve"); self.node.process_next()
        self.assertEqual(self.node.replies[approval["request_id"]]["status"], "succeeded")
        cache.lookup.assert_not_called()
        self.node.coordinated_path.assert_not_called()
        self.node.dispatch.assert_called_once()

    def test_cached_auto_command_still_rechecks_identity_before_dispatch(self):
        self.cold()
        self.node.coordinated_path.reset_mock()
        payload = command(dict(left=.1,right=.1))
        self.send(payload, "command"); self.node.process_next()
        result = self.node.replies[payload["request_id"]]
        self.assertEqual(result["status"], "succeeded", result.get("error"))
        self.node.coordinated_path.assert_not_called()
        self.node.dispatch.assert_called_once()
        self.assertGreaterEqual(self.node._review_runtime_identity.call_count, 6)

    def test_reanchored_first_segment_dynamics_rejection_falls_back(self):
        _, cold, _ = self.cold()
        bad = Spline(self.node.joint_names, [dict(t=0.,p=[0.,0.],v=[0.,0.],a=[0.,0.]),
            dict(t=.2,p=[.1,.1],v=[0.,0.],a=[0.,0.])])
        self.node.trajectory_cache.store(cold["trajectory_cache"]["identity"],
            dict(left=0.,right=0.), dict(left=.1,right=.1), [bad.record()])
        self.node.coordinated_path.reset_mock()
        _, trace, _ = self.make_review()
        self.assertEqual(trace["trajectory_cache"]["rejected_candidate"]["type"], "CacheDynamicsRejected")
        self.assertTrue(trace["trajectory_cache"]["fallback"])
        self.node.coordinated_path.assert_called_once()

    def test_idle_reconfiguration_validates_and_starts_a_new_cold_cache(self):
        original = self.node.trajectory_cache
        self.node.configure_planning(planner_id="RRTkConfigDefault", trajectory_cache=True,
                                     cache_start_tolerance_rad=.0001)
        self.assertIsNot(original, self.node.trajectory_cache)
        self.assertEqual(self.node.planner_id, "RRTkConfigDefault")
        self.node.active = object()
        with self.assertRaisesRegex(RuntimeError, "idle"):
            self.node.configure_planning()

    def test_default_cache_off_keeps_final_review_query_outside_finished_budget(self):
        self.node.configure_planning()
        observed = []
        def identity():
            if self.node.compute_budget is not None:
                observed.append(self.node.compute_budget.finished)
            return deepcopy(self.identity)
        self.node._review_runtime_identity.side_effect = identity
        _, trace, _ = self.make_review()
        self.assertEqual(len(observed), 1)
        self.assertIsNotNone(observed[0])
        self.assertFalse(trace["planning_configuration"]["budget_includes_review"])
        self.assertEqual(trace["planning_configuration"]["compute_scope"],
                         "computation_started_before_final_review_identity")

    def test_experiment_cache_off_includes_final_review_query_and_issues_once_after_budget_finish(self):
        self.node.configure_planning()
        self.node.review_in_compute_budget = True
        observed = []
        def identity():
            if self.node.compute_budget is not None:
                observed.append(self.node.compute_budget.finished)
            return deepcopy(self.identity)
        self.node._review_runtime_identity.side_effect = identity
        self.node.review_store.issue = Mock(wraps=self.node.review_store.issue)
        _, trace, _ = self.make_review()
        self.assertEqual(observed, [None])
        self.node.review_store.issue.assert_called_once()
        self.assertTrue(trace["planning_configuration"]["budget_includes_review"])
        self.assertEqual(trace["planning_configuration"]["compute_scope"],
                         "computation_started_through_final_review_identity")
        events = [event["name"] for event in trace["events"]]
        self.assertLess(events.index("computation_done"), events.index("plan_ready"))
        self.node.dispatch.assert_not_called()

    def test_experiment_final_review_budget_or_cancel_failure_cannot_issue_token(self):
        for failure in ("budget", "cancel"):
            with self.subTest(failure=failure):
                self.setUp()
                self.node.configure_planning()
                self.node.review_in_compute_budget = True
                def identity():
                    budget = self.node.compute_budget
                    if budget is not None and budget.finished is None:
                        if failure == "budget":
                            budget.clock = lambda: budget.started + round((budget.limit + .001) * 1e9)
                        else: self.node.cancel_requested = True
                    return deepcopy(self.identity)
                self.node._review_runtime_identity.side_effect = identity
                self.node.review_store.issue = Mock(wraps=self.node.review_store.issue)
                payload = command(dict(left=.1,right=.1))
                self.send(payload); self.node.process_next()
                result = self.node.replies[payload["request_id"]]
                self.assertEqual(result["failure_kind"], "budget_exhausted" if failure == "budget" else "cancelled")
                self.node.review_store.issue.assert_not_called()
                self.node.dispatch.assert_not_called()

    def test_review_budget_scope_is_strict_idle_only_and_cache_optin_always_includes_it(self):
        for value in (1, "yes", None):
            with self.subTest(value=value), self.assertRaises(ValueError):
                self.node.review_in_compute_budget = value
        self.assertFalse(self.node.review_in_compute_budget)
        _, trace, _ = self.cold()
        self.assertTrue(trace["planning_configuration"]["budget_includes_review"])
        self.node.review_in_compute_budget = True
        self.node.configure_planning()
        self.assertTrue(self.node.review_in_compute_budget)
        self.node.active = object()
        with self.assertRaisesRegex(RuntimeError, "idle"):
            self.node.review_in_compute_budget = False

    def test_native_opened_provenance_rejects_current_source_or_binary_replacement(self):
        from mujoco_ros2_manipulation.model_build.integrity import file_sha256
        root = self.node.directory
        self.node.cell.root = root
        names = ("native/path_distance.cpp", "native/path_distance_environment.cpp", "runtime/native/path_distance.so")
        for name in names:
            path = root/name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(name.encode())
        opened = dict(engine="MuJoCo 3.12.0", source_sha256=file_sha256(root/names[0]),
            library_sha256=file_sha256(root/names[2]),
            environment=dict(source_sha256=file_sha256(root/names[1])))
        self.node.evaluator.metadata = Mock(return_value=opened)
        check = review_fixture.Server._native_cache_provenance
        self.assertEqual(check(self.node)["opened"], opened)
        for name in names:
            with self.subTest(name=name):
                (root/name).write_bytes(b"changed")
                with self.assertRaisesRegex(review_fixture.ReviewRejected, "Native evaluator"):
                    check(self.node)
                (root/name).write_bytes(name.encode())


if __name__ == "__main__": unittest.main()
