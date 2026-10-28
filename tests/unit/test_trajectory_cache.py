"""Structural reuse gates without ROS, physics, files or approval authority."""
from concurrent.futures import ThreadPoolExecutor
from copy import deepcopy
from dataclasses import replace
import hashlib
import json
import unittest

from mujoco_ros2_manipulation.workcells.interference import Spline
from mujoco_ros2_manipulation.workcells.trajectory_cache import (
    IDENTITY_FIELDS, MAX_ENTRY_BYTES, MISS_REASONS, TrajectoryCache,
)


def identity():
    return dict(zip(IDENTITY_FIELDS, (str(index) * 64 for index in range(6))))


def curve(start=None, target=None):
    start = dict(left=0., right=.2) if start is None else start
    target = dict(left=.1, right=.3) if target is None else target
    return [dict(names=["left", "right"], points=[
        dict(t=0., p=[start["left"], start["right"]], v=[0., 0.], a=[0., 0.]),
        dict(t=1., p=[target["left"], target["right"]], v=[0., 0.], a=[.1, -.1]),
    ])]


class CacheTests(unittest.TestCase):
    def setUp(self):
        self.now = 10_000_000_000
        self.cache = TrajectoryCache(["left", "right"], clock=lambda: self.now)
        self.identity = identity()
        self.start = dict(left=0., right=.2)
        self.target = dict(left=.1, right=.3)
        self.path = curve()

    def store(self, **changes):
        args = dict(identity=self.identity, initial_rad=self.start,
                    target_rad=self.target, trajectories=self.path)
        args.update(changes)
        return self.cache.store(**args)

    def lookup(self, **changes):
        args = dict(identity=self.identity, initial_rad=self.start, target_rad=self.target)
        args.update(changes)
        return self.cache.lookup(**args)

    def test_cold_hit_and_detached_immutable_storage_and_new_lookup(self):
        self.assertEqual(self.lookup()["reason"], "cold")
        receipt = self.store()
        self.path[0]["points"][1]["p"][0] = 999
        self.identity["model_sha256"] = "f" * 64
        hit = self.lookup(identity=identity())
        self.assertTrue(hit["hit"])
        self.assertEqual(hit["entry_sha256"], receipt["entry_sha256"])
        self.assertEqual(hit["trajectories"][0]["points"][1]["p"], [.1, .3])
        self.assertTrue(hit["full_revalidation_required"])
        hit["trajectories"][0]["points"].clear()
        again = self.lookup(identity=identity())
        self.assertEqual(again["trajectories"], curve())
        self.assertNotIn("session_id", again)
        self.assertNotIn("plan_sha256", again)
        self.assertNotIn("approval", again)

    def test_named_dictionary_order_is_normalized_without_rounding(self):
        receipt = self.store()
        hit = self.lookup(initial_rad=dict(right=.2, left=0), target_rad=dict(right=.3, left=.1))
        self.assertTrue(hit["hit"])
        self.assertEqual(hit["key_sha256"], receipt["key_sha256"])
        miss = self.lookup(initial_rad=dict(left=1e-15, right=.2))
        self.assertFalse(miss["hit"])
        self.assertEqual(miss["reason"], "start_mismatch")
        self.assertIsNone(miss["trajectories"])
        self.assertTrue(self.lookup()["hit"])

    def test_opt_in_nearest_start_reanchors_only_a_detached_first_knot(self):
        self.cache = TrajectoryCache(["left", "right"], start_tolerance_rad=.0001, clock=lambda: self.now)
        first = self.store()
        near = dict(left=.00008, right=.2)
        nearer = self.store(initial_rad=near, trajectories=curve(start=near))
        source_entry = self.cache._entries[nearer["key_sha256"]]
        current = dict(left=.00007, right=.2)
        hit = self.lookup(initial_rad=current)
        self.assertTrue(hit["hit"])
        self.assertTrue(hit["reanchored"])
        self.assertEqual(hit["key_sha256"], nearer["key_sha256"])
        self.assertNotEqual(hit["key_sha256"], hit["request_key_sha256"])
        self.assertEqual(hit["source_entry_sha256"], nearer["entry_sha256"])
        self.assertEqual(hit["source_initial_rad"], near)
        self.assertEqual(hit["current_initial_rad"], current)
        self.assertAlmostEqual(hit["start_max_error_rad"], .00001)
        expected = curve(start=near)
        expected[0]["points"][0]["p"] = [.00007, .2]
        self.assertEqual(hit["trajectories"], expected)
        self.assertNotEqual(hit["candidate_sha256"], hit["source_entry_sha256"])
        self.assertEqual(self.cache._entries[nearer["key_sha256"]], source_entry)
        self.assertIn(first["key_sha256"], self.cache._entries)
        observed = Spline(**hit["trajectories"][0])
        source = Spline(**curve(start=near)[0])
        self.assertNotEqual(observed.position(.5), source.position(.5))
        self.assertEqual(observed.position(1.), source.position(1.))
        hit["source_initial_rad"]["left"] = 999
        hit["current_initial_rad"]["left"] = 999
        hit["trajectories"][0]["points"][0]["p"][0] = 999
        self.assertEqual(self.cache._entries[nearer["key_sha256"]], source_entry)
        self.assertEqual(current, dict(left=.00007, right=.2))

    def test_opt_in_hits_cannot_accumulate_start_drift_or_extend_source_ttl(self):
        self.cache = TrajectoryCache(["left", "right"], start_tolerance_rad=.0001,
                                     ttl_sec=1., clock=lambda: self.now)
        stored = self.store()
        source = self.cache._entries[stored["key_sha256"]]
        self.now += 900_000_000
        hit = self.lookup(initial_rad=dict(left=.00009, right=.2))
        self.assertTrue(hit["hit"])
        self.assertEqual(hit["entry_age_wall_sec"], .9)
        self.assertEqual(self.cache._entries[stored["key_sha256"]], source)
        # This is near the last returned candidate, but outside the original
        # anchor. A hit must never become the next anchor or extend its TTL.
        drift = self.lookup(initial_rad=dict(left=.00018, right=.2))
        self.assertFalse(drift["hit"])
        self.assertEqual(drift["reason"], "start_mismatch")
        self.now += 100_000_000
        expired = self.lookup()
        self.assertEqual(expired["reason"], "expired")

    def test_opt_in_start_match_still_requires_exact_goal_identity_and_bound(self):
        self.cache = TrajectoryCache(["left", "right"], start_tolerance_rad=.0001, clock=lambda: self.now)
        self.store()
        self.assertFalse(self.lookup(initial_rad=dict(left=.000100001, right=.2))["hit"])
        self.assertFalse(self.lookup(initial_rad=dict(left=.00009, right=.2),
                                     target_rad=dict(left=.100000000000001, right=.3))["hit"])
        self.assertFalse(self.lookup(initial_rad=dict(left=.00009, right=.2),
                                     identity=dict(self.identity, environment_sha256="f" * 64))["hit"])
        maximum = self.lookup(initial_rad=dict(left=.0001, right=.2))
        self.assertTrue(maximum["hit"])
        self.assertEqual(maximum["start_max_error_rad"], .0001)
        exact = self.lookup()
        self.assertFalse(exact["reanchored"])
        self.assertEqual(exact["candidate_sha256"], exact["source_entry_sha256"])
        exact_cache = TrajectoryCache(["left", "right"], clock=lambda: self.now)
        exact_store = exact_cache.store(self.identity, self.start, self.target, curve())
        self.assertNotEqual(exact["key_sha256"], exact_store["key_sha256"])

    def test_every_source_identity_change_is_a_miss_not_approximate_reuse(self):
        self.store()
        for field in IDENTITY_FIELDS:
            with self.subTest(field=field):
                changed = dict(self.identity, **{field: "f" * 64})
                miss = self.lookup(identity=changed)
                self.assertEqual(miss["reason"], "identity_mismatch")
                self.assertEqual(miss["mismatch_fields"], [field])
                self.assertIsNone(miss["trajectories"])
        no_tool = dict(self.identity, tool_sha256=None)
        self.store(identity=no_tool)
        self.assertTrue(self.lookup(identity=no_tool)["hit"])
        self.assertTrue(self.lookup()["hit"])

    def test_malformed_caller_identity_and_state_raise_instead_of_miss(self):
        self.store()
        invalid_identity = [None, {}, dict(self.identity, unknown="a" * 64),
                            dict(self.identity, source_sha256=None), dict(self.identity, model_sha256="A" * 64),
                            dict(self.identity, policy_sha256={"unhashed": "object"})]
        for item in invalid_identity:
            with self.subTest(identity=item), self.assertRaises(ValueError):
                self.lookup(identity=item)
        invalid_state = [{}, dict(left=0.), dict(left=False, right=.2),
                         dict(left=float("nan"), right=.2), dict(left=10 ** 1000, right=.2),
                         dict(left=0., right=float("inf")), dict(left=0., right=.2, unknown=0.)]
        for item in invalid_state:
            with self.subTest(state=str(item)[:100]):
                with self.assertRaises(ValueError):
                    self.lookup(initial_rad=item)
                with self.assertRaises(ValueError):
                    self.lookup(target_rad=item)
        self.assertTrue(self.lookup()["hit"])

    def test_goal_and_combined_start_goal_mismatches_do_not_remove_old_path(self):
        self.store()
        self.assertEqual(self.lookup(target_rad=dict(left=.15, right=.3))["reason"], "goal_mismatch")
        self.assertEqual(self.lookup(initial_rad=dict(left=.01, right=.2),
                                    target_rad=dict(left=.15, right=.3))["reason"], "start_and_goal_mismatch")
        self.assertTrue(self.lookup()["hit"])

    def test_changed_goal_miss_reason_uses_declared_start_tolerance_only_for_diagnostics(self):
        self.cache = TrajectoryCache(["left", "right"], start_tolerance_rad=.0001, clock=lambda: self.now)
        stored = self.store()
        original = self.cache._entries[stored["key_sha256"]]
        changed_goal = dict(left=.15, right=.3)
        for delta in (2.4225e-19, .0001):
            with self.subTest(delta=delta):
                missed = self.lookup(initial_rad=dict(left=delta, right=.2), target_rad=changed_goal)
                self.assertFalse(missed["hit"])
                self.assertEqual(missed["reason"], "goal_mismatch")
                self.assertIsNone(missed["trajectories"])
                self.assertFalse(missed["reanchored"])
                self.assertEqual(self.cache._entries[stored["key_sha256"]], original)
        outside = self.lookup(initial_rad=dict(left=.000100001, right=.2), target_rad=changed_goal)
        self.assertFalse(outside["hit"])
        self.assertEqual(outside["reason"], "start_and_goal_mismatch")
        # The exact default continues to treat any nonzero start difference as
        # a mismatch. No search, reanchor or validation threshold changed.
        exact = TrajectoryCache(["left", "right"], clock=lambda: self.now)
        exact.store(self.identity, self.start, self.target, curve())
        missed = exact.lookup(self.identity, dict(left=2.4225e-19, right=.2), changed_goal)
        self.assertEqual(missed["reason"], "start_and_goal_mismatch")

    def test_whole_path_structure_anchor_order_and_endpoint_gates(self):
        changes = [lambda p: p.append(deepcopy(p[0])),
                   lambda p: p[0]["names"].reverse(),
                   lambda p: p[0]["names"].pop(),
                   lambda p: p[0].update(approval="not allowed"),
                   lambda p: p[0]["points"].pop(),
                   lambda p: p[0]["points"][0].update(t=.01),
                   lambda p: p[0]["points"][1].update(t=0.),
                   lambda p: p[0]["points"][1].update(t=35.001),
                   lambda p: p[0]["points"][0]["p"].__setitem__(0, 1e-15),
                   lambda p: p[0]["points"][0]["v"].__setitem__(0, .01),
                   lambda p: p[0]["points"][0]["a"].__setitem__(0, .01),
                   lambda p: p[0]["points"][1]["p"].__setitem__(0, .104),
                   lambda p: p[0]["points"][1]["v"].__setitem__(0, .001),
                   lambda p: p[0]["points"][1]["a"].__setitem__(0, float("nan")),
                   lambda p: p[0]["points"][0]["p"].__setitem__(0, False),
                   lambda p: p[0]["points"][1].update(v=[]),
                   lambda p: p[0]["points"][1].update(a=[0.]),
                   lambda p: p[0]["points"][1].update(extra=1)]
        for index, mutate in enumerate(changes):
            with self.subTest(index=index):
                path = curve()
                mutate(path)
                with self.assertRaises(ValueError):
                    self.store(trajectories=path)
        self.assertEqual(self.lookup()["reason"], "cold")
        # TOTG can return nonzero terminal acceleration; retain it exactly.
        self.store()
        spline = Spline(**self.lookup()["trajectories"][0])
        self.assertEqual(spline.points[-1]["a"], [.1, -.1])

    def test_linear_cubic_and_quintic_records_remain_compatible(self):
        for derivatives in (0, 1, 2):
            path = curve()
            for point in path[0]["points"]:
                if derivatives < 1:
                    point["v"] = []
                if derivatives < 2:
                    point["a"] = []
            with self.subTest(derivatives=derivatives):
                self.store(trajectories=path)
                record = self.lookup()["trajectories"][0]
                self.assertEqual(record, path[0])
                self.assertEqual(Spline(**record).position(0.), [0., .2])

    def test_point_and_numeric_resource_bounds_are_enforced_before_storage(self):
        path = curve()
        path[0]["points"] = [dict(t=index / 100., p=[0., .2], v=[], a=[])
                                for index in range(2001)]
        with self.assertRaises(ValueError):
            self.store(trajectories=path)
        for value in (10 ** 1000, 1e7, float("inf")):
            path = curve()
            path[0]["points"][-1]["a"][0] = value
            with self.subTest(value=str(value)[:100]), self.assertRaises(ValueError):
                self.store(trajectories=path)
        self.assertEqual(self.lookup()["entries"], 0)

    def test_declared_endpoint_tolerance_is_enforced_and_bound_into_the_key(self):
        loose = self.store()
        strict = TrajectoryCache(["left", "right"], goal_tolerance_rad=0, clock=lambda: self.now)
        exact = strict.store(self.identity, self.start, self.target, curve())
        self.assertNotEqual(loose["key_sha256"], exact["key_sha256"])
        path = curve()
        path[0]["points"][-1]["p"][0] += .002
        self.store(trajectories=path)
        with self.assertRaises(ValueError):
            strict.store(self.identity, self.start, self.target, path)
        self.assertTrue(strict.lookup(self.identity, self.start, self.target)["hit"])

    def test_expiry_is_based_on_creation_and_a_hit_cannot_extend_ttl(self):
        self.cache = TrajectoryCache(["left", "right"], ttl_sec=1, clock=lambda: self.now)
        self.store()
        self.now += 900_000_000
        self.assertTrue(self.lookup()["hit"])
        self.now += 100_000_000
        expired = self.lookup()
        self.assertEqual(expired["reason"], "expired")
        self.assertEqual(expired["evicted_expired"], 1)
        self.assertEqual(expired["entries"], 0)
        self.assertEqual(self.lookup()["reason"], "cold")

    def test_expired_miss_diagnosis_matches_only_the_declared_start_goal_and_identity_policy(self):
        cases = [dict(tolerance=0., delta=0., reason="expired"),
                 dict(tolerance=0., delta=2.4225e-19, reason="cold"),
                 dict(tolerance=.0001, delta=2.4225e-19, reason="expired"),
                 dict(tolerance=.0001, delta=.0001, reason="expired"),
                 dict(tolerance=.0001, delta=.000100001, reason="cold"),
                 dict(tolerance=.0001, delta=0., goal=dict(left=.15, right=.3), reason="cold"),
                 dict(tolerance=.0001, delta=0., identity=dict(self.identity, environment_sha256="f" * 64), reason="cold")]
        for case in cases:
            with self.subTest(case=case):
                self.cache = TrajectoryCache(["left", "right"], ttl_sec=1.,
                    start_tolerance_rad=case["tolerance"], clock=lambda: self.now)
                self.store()
                self.now += 1_000_000_000
                result = self.lookup(initial_rad=dict(left=case["delta"], right=.2),
                    target_rad=case.get("goal", self.target), identity=case.get("identity", self.identity))
                self.assertFalse(result["hit"])
                self.assertIsNone(result["trajectories"])
                self.assertFalse(result["reanchored"])
                self.assertEqual(result["reason"], case["reason"])
                self.assertEqual(result["evicted_expired"], 1)
                self.assertEqual(result["entries"], 0)
                self.assertEqual(result["stored_bytes"], 0)
                self.assertEqual(self.lookup()["reason"], "cold")

    def test_lru_capacity_access_and_replacement(self):
        self.cache = TrajectoryCache(["left", "right"], max_entries=2, clock=lambda: self.now)
        target2, target3 = dict(left=.2, right=.3), dict(left=.3, right=.3)
        first = self.store()
        second = self.store(target_rad=target2, trajectories=curve(target=target2))
        self.assertTrue(self.lookup()["hit"])
        third = self.store(target_rad=target3, trajectories=curve(target=target3))
        self.assertEqual(third["evicted_lru"], 1)
        self.assertEqual(set(self.cache._entries), {first["key_sha256"], third["key_sha256"]})
        self.assertNotIn(second["key_sha256"], self.cache._entries)
        replaced = self.store()
        self.assertTrue(replaced["replaced"])
        self.assertEqual(replaced["entries"], 2)

    def test_total_byte_capacity_and_oversize_rejection_preserve_valid_entries(self):
        size = self.store()["entry_bytes"]
        self.cache = TrajectoryCache(["left", "right"], max_total_bytes=size, clock=lambda: self.now)
        self.store()
        same_size_identity = dict(self.identity, environment_sha256="f" * 64)
        receipt = self.store(identity=same_size_identity)
        self.assertEqual(receipt["evicted_lru"], 1)
        self.assertLessEqual(receipt["stored_bytes"], size)
        path = curve()
        path[0]["points"].insert(1, dict(t=.5, p=[.05, .25], v=[0., 0.], a=[0., 0.]))
        with self.assertRaises(ValueError):
            self.store(identity=same_size_identity, trajectories=path)
        self.assertTrue(self.lookup(identity=same_size_identity)["hit"])

    def test_corrupt_json_checksum_envelope_and_resigned_invalid_path_are_evicted(self):
        variants = [lambda entry: replace(entry, raw=b"not json"),
                    lambda entry: replace(entry, sha256="0" * 64),
                    lambda entry: replace(entry, created_ns=self.now + 1),
                    lambda entry: replace(entry, raw=b"x" * (MAX_ENTRY_BYTES + 1))]
        for index, corrupt in enumerate(variants):
            with self.subTest(index=index):
                receipt = self.store()
                key = receipt["key_sha256"]
                self.cache._entries[key] = corrupt(self.cache._entries[key])
                result = self.lookup()
                self.assertEqual(result["reason"], "invalid_entry")
                self.assertEqual(result["evicted_invalid"], 1)
                self.assertIsNone(result["trajectories"])
        receipt = self.store()
        key = receipt["key_sha256"]
        entry = self.cache._entries[key]
        payload = json.loads(entry.raw)
        payload["trajectories"][0]["points"][0]["p"][0] = .001
        raw = json.dumps(payload, sort_keys=True, separators=(",", ":")).encode()
        self.cache._entries[key] = replace(entry, raw=raw, sha256=hashlib.sha256(raw).hexdigest())
        self.assertEqual(self.lookup()["reason"], "invalid_entry")

    def test_resigned_wrong_request_digest_is_evicted_not_relocated(self):
        key = self.store()["key_sha256"]
        entry = self.cache._entries[key]
        payload = json.loads(entry.raw)
        payload["request"]["identity"]["source_sha256"] = "f" * 64
        raw = json.dumps(payload, sort_keys=True, separators=(",", ":")).encode()
        self.cache._entries[key] = replace(entry, raw=raw, sha256=hashlib.sha256(raw).hexdigest())
        result = self.lookup()
        self.assertEqual(result["reason"], "invalid_entry")
        self.assertEqual(result["stored_bytes"], 0)

    def test_resigned_malformed_duplicate_or_deep_json_is_evicted(self):
        malformed = [b"not json", b'{"request":{},"request":{},"trajectories":[]}',
                     ("[" * 1500 + "0" + "]" * 1500).encode()]
        for raw in malformed:
            with self.subTest(kind=raw[:20]):
                key = self.store()["key_sha256"]
                entry = self.cache._entries[key]
                self.cache._entries[key] = replace(entry, raw=raw, sha256=hashlib.sha256(raw).hexdigest())
                result = self.lookup()
                self.assertEqual(result["reason"], "invalid_entry")
                self.assertEqual(result["evicted_invalid"], 1)

    def test_explicit_validator_rejection_evicts_only_the_exact_candidate(self):
        first = self.store()
        other = dict(self.identity, source_sha256="f" * 64)
        second = self.store(identity=other)
        receipt = self.cache.evict(first["key_sha256"], "validation_rejected")
        self.assertTrue(receipt["evicted"])
        self.assertTrue(self.lookup(identity=other)["hit"])
        self.assertEqual(set(self.cache._entries), {second["key_sha256"]})
        self.assertFalse(self.cache.evict(first["key_sha256"])["evicted"])
        with self.assertRaises(ValueError):
            self.cache.evict(second["key_sha256"], "arbitrary unbounded reason")

    def test_measured_lookup_cost_includes_structural_revalidation(self):
        self.store()
        def clock():
            self.now += 1_000_000
            return self.now
        self.cache.clock = clock
        result = self.lookup()
        self.assertTrue(result["hit"])
        self.assertGreater(result["lookup_wall_sec"], 0)
        self.assertGreaterEqual(result["elapsed_wall_sec"], result["lookup_wall_sec"])
        self.assertEqual(result["inspected_entries"], 1)
        self.assertEqual(result["mismatch_fields"], [])

    def test_concurrent_store_lookup_and_eviction_are_bounded_and_detached(self):
        cache = TrajectoryCache(["left", "right"], max_entries=4)
        def operation(index):
            source = dict(self.identity, source_sha256=f"{index:064x}")
            receipt = cache.store(source, self.start, self.target, curve())
            result = cache.lookup(source, self.start, self.target)
            if result["hit"]:
                result["trajectories"].clear()
            cache.evict(receipt["key_sha256"])
            return result["hit"] or result["reason"] in MISS_REASONS
        with ThreadPoolExecutor(max_workers=8) as pool:
            self.assertTrue(all(pool.map(operation, range(32))))
        self.assertLessEqual(len(cache._entries), 4)

    def test_bad_configuration_and_clock_failures_do_not_become_cache_misses(self):
        for changes in (dict(max_entries=True), dict(max_entries=129), dict(max_total_bytes=0),
                        dict(ttl_sec=float("nan")), dict(ttl_sec=True), dict(goal_tolerance_rad=.02),
                        dict(goal_tolerance_rad=-.1), dict(start_tolerance_rad=.00101),
                        dict(start_tolerance_rad=-.001), dict(start_tolerance_rad=True), dict(clock=0)):
            with self.subTest(changes=changes), self.assertRaises(ValueError):
                TrajectoryCache(["left", "right"], **changes)
        for names in ([], ["left", "left"], ["left", "bad/name"], [True], ["x" * 129]):
            with self.subTest(names=names), self.assertRaises(ValueError):
                TrajectoryCache(names)
        self.store()
        self.now -= 1
        with self.assertRaises(ValueError):
            self.lookup()
        def interrupted_clock():
            raise KeyboardInterrupt()
        self.cache.clock = interrupted_clock
        with self.assertRaises(KeyboardInterrupt):
            self.lookup()


if __name__ == "__main__":
    unittest.main()
