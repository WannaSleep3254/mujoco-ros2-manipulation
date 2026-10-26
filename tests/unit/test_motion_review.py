"""Review identity, expiry and single-use ownership without ROS or simulators."""
from concurrent.futures import ThreadPoolExecutor
import threading
import unittest

from mujoco_ros2_manipulation.workcells.motion_request import command, validate_approval, validate_command
from mujoco_ros2_manipulation.workcells.motion_review import PlanReviewStore, ReviewRejected


class ReviewTests(unittest.TestCase):
    def setUp(self):
        self.now = 100.0
        self.store = PlanReviewStore("a"*32, clock=lambda: self.now)
        self.identity = dict(model_sha256="b"*64, environment_sha256="c"*64, policy_sha256="d"*64)
        self.initial = dict(left=0.0, right=0.0)
        self.curves = [dict(names=["left", "right"], points=[
            dict(t=0.0,p=[0.0,0.0],v=[],a=[]), dict(t=1.0,p=[.1,.1],v=[],a=[])])]
        self.issue()

    def issue(self):
        self.summary = self.store.issue("e"*32, identity=self.identity, initial_state_rad=self.initial,
            simulation_sec=10.0, command=command(dict(left=.1,right=.1),identifier="e"*32), trajectories=self.curves)
        self.approval = dict(schema_version=1, request_id="f"*32, plan_request_id="e"*32,
            session_id="a"*32, plan_sha256=self.summary["plan_sha256"])

    def claim(self, **changes):
        args = dict(identity=self.identity, state_rad=self.initial, simulation_sec=10.1)
        args.update(changes)
        return self.store.claim(self.approval, **args)

    def test_additive_approval_contract_does_not_relax_command_v1(self):
        self.assertEqual(validate_approval(self.approval), self.approval)
        for mutation in (dict(extra=True),dict(schema_version=True),dict(plan_sha256="A"*64),
                         dict(session_id="wrong"),dict(request_id="e"*32)):
            with self.subTest(mutation=mutation), self.assertRaises(ValueError):
                validate_approval(dict(self.approval,**mutation))
        with self.assertRaises(ValueError):
            validate_command(dict(command({"left":0.0,"right":0.0}), mode="plan_only"),
                {n:dict(lower=-1,upper=1) for n in self.initial})

    def test_review_payload_and_claim_are_detached_and_digest_is_stable(self):
        payload, summary = self.claim()
        self.assertEqual(summary["state"], "consumed")
        self.assertEqual(summary["execution_request_id"], "f"*32)
        self.identity["model_sha256"] = "0"*64
        self.curves[0]["points"][-1]["p"][0] = 99
        self.assertEqual(payload["identity"]["model_sha256"], "b"*64)
        self.assertEqual(payload["trajectories"][0]["points"][-1]["p"][0], .1)
        payload["trajectories"].clear()
        self.assertTrue(self.store.record["payload"]["trajectories"])
        with self.assertRaises(ReviewRejected):
            self.claim()

    def test_wall_time_expiry_cannot_be_extended_by_a_new_simulation_stamp(self):
        self.now = 160.0
        self.assertEqual(self.store.summary()["state"], "expired")
        with self.assertRaises(ReviewRejected):
            self.claim(simulation_sec=20000.0)

    def test_current_model_environment_policy_and_initial_state_invalidate_review(self):
        cases = [dict(identity=dict(self.identity, model_sha256="0"*64)),
                 dict(identity=dict(self.identity, environment_sha256="0"*64)),
                 dict(identity=dict(self.identity, policy_sha256="0"*64)),
                 dict(state_rad=dict(left=.002,right=0.0)), dict(state_rad=dict(left=0.0)),
                 dict(simulation_sec=9.999)]
        for change in cases:
            with self.subTest(change=change):
                self.setUp()
                with self.assertRaises(ReviewRejected):
                    self.claim(**change)
                self.assertEqual(self.store.summary()["state"], "invalidated")

    def test_wrong_session_plan_or_digest_cannot_consume_a_different_review(self):
        for change in (dict(session_id="0"*32),dict(plan_request_id="0"*32),dict(plan_sha256="0"*64)):
            with self.subTest(change=change), self.assertRaises(ReviewRejected):
                self.store.claim(dict(self.approval,**change), identity=self.identity,
                    state_rad=self.initial, simulation_sec=10.1)
            self.assertEqual(self.store.summary()["state"], "awaiting_approval")

    def test_concurrent_approvals_have_exactly_one_owner(self):
        barrier = threading.Barrier(8)
        def take(index):
            barrier.wait()
            try:
                payload, _ = self.store.claim(dict(self.approval,request_id=f"{index:032x}"),
                    identity=self.identity,state_rad=self.initial,simulation_sec=10.1)
                return payload
            except ReviewRejected:
                return None
        with ThreadPoolExecutor(max_workers=8) as executor:
            results = list(executor.map(take, range(8)))
        self.assertEqual(sum(result is not None for result in results), 1)

    def test_expiry_or_state_change_after_claim_still_refuses_dispatch(self):
        for change in (dict(now=160.0), dict(state_rad=dict(left=.002,right=0.0)),
                       dict(identity=dict(self.identity,environment_sha256="0"*64)),dict(simulation_sec=9.9)):
            with self.subTest(change=change):
                self.setUp()
                payload, _ = self.claim()
                self.now = change.get("now",self.now)
                with self.assertRaises(ReviewRejected):
                    self.store.recheck_claim(payload,identity=change.get("identity",self.identity),
                        state_rad=change.get("state_rad",self.initial),simulation_sec=change.get("simulation_sec",10.2))
                with self.assertRaises(ReviewRejected):
                    self.claim()

    def test_new_plan_or_normal_command_revokes_previous_review(self):
        self.store.invalidate("another motion owns the workcell")
        self.assertEqual(self.store.summary()["state"], "invalidated")
        with self.assertRaises(ReviewRejected):
            self.claim()
        self.store.issue("1"*32,identity=self.identity,initial_state_rad=self.initial,
            simulation_sec=11.0,command=command(self.initial,identifier="1"*32),trajectories=[],no_motion_required=True)
        self.assertEqual(self.store.summary()["plan_request_id"], "1"*32)
        with self.assertRaises(ReviewRejected):
            self.claim()

    def test_nonfinite_partial_and_boolean_states_never_enter_a_review(self):
        for initial in ({},dict(joint=True),dict(joint=float("nan")),dict(joint=float("inf"))):
            with self.subTest(initial=initial), self.assertRaises(ValueError):
                self.store.issue("2"*32,identity=self.identity,initial_state_rad=initial,
                    simulation_sec=10.0,command={},trajectories=[])


if __name__ == "__main__":
    unittest.main()
