"""Independent self geometry must gate clear cross-arm/MoveIt paths and reviews."""
from copy import deepcopy
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock

from mujoco_ros2_manipulation.paths import ROOT
from mujoco_ros2_manipulation.workcells.self_clearance import (
    require_self_result, require_self_clearance, self_policy_snapshot, validate_self_policy,
    SelfClearanceRejected,
)
from mujoco_ros2_manipulation.workcells.interference import Spline
from mujoco_ros2_manipulation.workcells.path_quality import evaluate_path
from mujoco_ros2_manipulation.workcells.motion_diagnostics import measure_endpoints,path_clearance_diagnostic
from mujoco_ros2_manipulation.workcells.plan_selection import select_clearance_plan,PlanCandidatesExhausted
from mujoco_ros2_manipulation.workcells.timing import choose_delay,TimingRejected
from tests.unit.test_environment_path_gate import ARM_POLICY
from tests.unit.test_spatial_proposal_contract import trajectory
from tests.unit import test_motion_review_server as review_tests

POLICY=dict(schema_version=1,planning_min_self_distance_m=.01,observed_start_min_self_distance_m=.001,sample_dt_sec=.01)
def own(distance):
    return dict(distance_m=distance,geom_ids=[7,13],bodies=['left_forearm_link','left_wrist3_link'],
        nearest_points_m=[[0.,0.,0.],[distance,0.,0.]],pair_count=24,scope='within_each_arm')

def evaluator(distance):
    def at(q):
        gap=distance(q) if callable(distance) else distance
        return dict(distance_m=.2,bodies=['left','right'],tips_m=[[q['left'],0.,0.],[q['right'],1.,0.]],self_collision=own(gap))
    return NS(self_collision_enabled=True,at=Mock(side_effect=at))

class SelfPathTests(unittest.TestCase):
    def setUp(self):
        self.initial=dict(left=0.,right=0.)
        self.cell=NS(joints=list(self.initial),instances=[NS(id=x,joints=[x]) for x in self.initial])
        self.curve=Spline(['left','right'],[dict(t=0.,p=[0.,0.],v=[],a=[]),dict(t=1.,p=[.1,.1],v=[],a=[])])

    def test_policy_is_separate_exact_bound_snapshot(self):
        path,value,digest=self_policy_snapshot(NS(root=ROOT))
        self.assertEqual(value,POLICY)
        self.assertEqual(path,ROOT/'config/planning/self_clearance.yaml')
        self.assertEqual(len(digest),64)
        for key,value in [('schema_version',True),('planning_min_self_distance_m',float('nan')),
                          ('observed_start_min_self_distance_m',0),('sample_dt_sec',.02)]:
            with self.subTest(key=key),self.assertRaises(ValueError):validate_self_policy(dict(POLICY,**{key:value}))
        with self.assertRaises(ValueError):validate_self_policy(dict(POLICY,unknown=True))

    def test_negative_overlap_is_valid_evidence_but_never_an_admission(self):
        self.assertEqual(require_self_result(own(-.002))['distance_m'],-.002)
        with self.assertRaises(SelfClearanceRejected):require_self_clearance(evaluator(-.002),self.initial,.001)
        self.assertEqual(require_self_clearance(evaluator(.001),self.initial,.001)['distance_m'],.001)
        disabled=evaluator(.1);disabled.self_collision_enabled=False
        with self.assertRaises(SelfClearanceRejected):require_self_clearance(disabled,self.initial,.001)

    def test_clear_cross_and_endpoints_cannot_hide_intermediate_self_overlap(self):
        ev=evaluator(lambda q:-.002 if .03<q['left']<.07 else .02)
        ends=measure_endpoints(self.initial,dict(left=.1,right=.1),ev,.03,self_threshold_m=.01)
        self.assertEqual(ends['endpoint_violations'],[])
        result=evaluate_path([self.curve],self.initial,self.cell,ev,self_reject_below=.01)
        self.assertFalse(result['complete']);self.assertEqual(result['rejection_kind'],'self_clearance')
        self.assertEqual(result['minimum']['distance_m'],.2)
        self.assertEqual(result['self_minimum']['distance_m'],-.002)
        self.assertEqual(path_clearance_diagnostic(ends,result,1.)['witness']['geom_ids'],[7,13])
        self.assertEqual(path_clearance_diagnostic(ends,result,1.)['violation_location'],'intermediate')

    def test_missing_or_malformed_native_self_data_fails_before_any_acceptance(self):
        for bad in (None,dict(own(.02),pair_count=0),dict(own(.02),geom_ids=[7,7]),
                    dict(own(.02),scope='cross_arm'),dict(own(.02),distance_m=True),
                    dict(own(.02),nearest_points_m=[[0,0,0],[float('nan'),0,0]])):
            ev=evaluator(.02);value=ev.at(self.initial);value['self_collision']=bad;ev.at=Mock(return_value=value)
            with self.subTest(bad=bad),self.assertRaises(ValueError):
                evaluate_path([self.curve],self.initial,self.cell,ev,self_reject_below=.01)

    def test_selector_preserves_bad_native_witness_then_selects_safe_candidate(self):
        phase=[0];saved=[];paths=[trajectory(),trajectory()]
        def propose():phase[0]+=1;return 1,paths[phase[0]-1]
        ev=evaluator(lambda q:-.002 if phase[0]==1 else .02)
        moveit=Mock(return_value=dict(invalid_samples=0))
        result=select_clearance_plan(propose,self.initial,self.cell,ev,ARM_POLICY,moveit,
            lambda c:saved.append(deepcopy(c)),self_policy=POLICY,max_candidates=2)
        self.assertIs(result,paths[1]);self.assertEqual(saved[-1][0]['rejection_kind'],'self_clearance')
        self.assertEqual(saved[-1][0]['native']['self_minimum']['distance_m'],-.002)
        self.assertNotIn('moveit',saved[-1][0]);moveit.assert_called_once()
        with self.assertRaises(PlanCandidatesExhausted):
            select_clearance_plan(lambda:(1,trajectory()),self.initial,self.cell,evaluator(-.002),ARM_POLICY,
                moveit,lambda c:None,self_policy=POLICY,max_candidates=1)

    def test_legacy_metrics_default_and_timing_optin_remain_separate(self):
        ev=evaluator(.02);value=ev.at(self.initial);value.pop('self_collision');ev.at=Mock(return_value=value)
        old=evaluate_path([self.curve],self.initial,self.cell,ev)
        self.assertTrue(old['complete']);self.assertNotIn('self_minimum',old)
        curves=[Spline([x],[dict(t=0.,p=[0.],v=[],a=[]),dict(t=1.,p=[.1],v=[],a=[])]) for x in self.initial]
        policy=dict(ARM_POLICY,delay_step_sec=.1,max_delay_sec=.1)
        with self.assertRaises(TimingRejected):choose_delay(curves,self.initial,self.cell,evaluator(-.002),policy,self_policy=POLICY)

class SelfProductionTests(unittest.TestCase):
    def setUp(self):
        self.fixture=review_tests.ReviewServerTests();self.fixture.setUp();self.addCleanup(self.fixture.doCleanups)
        self.node=self.fixture.node

    def command(self,target=None):
        import json
        from mujoco_ros2_manipulation.workcells.motion_request import command
        payload=command(target or dict(left=.1,right=.1));self.fixture.send(payload,'command');self.node.process_next()
        return json.loads((self.node.directory/(payload['request_id']+'.json')).read_text())

    def test_fresh_and_noop_cannot_dispatch_without_self_coverage(self):
        self.node.evaluator.at.return_value.pop('self_collision')
        for target in (dict(left=.1,right=.1),dict(left=0.,right=0.)):
            result=self.command(target);self.assertEqual(result['dispatched_goals'],0);self.node.dispatch.assert_not_called()

    def test_final_gate_keeps_ten_ms_self_samples_with_a_coarser_timing_policy(self):
        self.node.policy['sample_dt_sec'] = .02
        baseline = deepcopy(self.node.evaluator.at.return_value)
        def at(q):
            result = deepcopy(baseline)
            if .045 < q['left'] < .055:
                result['self_collision']['distance_m'] = -.002
            return result
        self.node.evaluator.at.side_effect = at
        result = self.command()
        self.assertEqual(result['failure_kind'], 'self_clearance')
        self.assertEqual(result['planned_clearance']['self_minimum']['distance_m'], -.002)
        self.node.dispatch.assert_not_called()

    def test_pre_submit_hook_rechecks_late_policy_mutation_and_start_reservation(self):
        self.node.execution_ledger = NS(attempts=[NS(submission_started=False)])
        self.node.active = NS(data={'initial_state_rad':dict(self.node.actual)})
        self.node.motion_start = self.node.clock_last+1.
        self.node._dispatch_self_abort()
        self.node.self_policy_path.write_text('changed after goal observer')
        with self.assertRaisesRegex(RuntimeError,'policy changed'):
            self.node._dispatch_self_abort()
        self.node.self_policy_path.write_text('test self policy snapshot')
        self.node.motion_start = self.node.clock_last+.05
        with self.assertRaisesRegex(RuntimeError,'reservation'):
            self.node._dispatch_self_abort()
        self.node.dispatch.assert_not_called()

    def test_slow_identity_or_geometry_cannot_use_precheck_recency(self):
        identity = deepcopy(self.node._native_cache_provenance.return_value)
        original_geometry = self.node.evaluator.at.return_value
        self.node.execution_ledger = NS(attempts=[NS(submission_started=False)])
        self.node.active = NS(data={'initial_state_rad':dict(self.node.actual)})
        self.node.motion_start = self.node.clock_last+1.
        for stage in ('identity','geometry'):
            with self.subTest(stage=stage):
                self.node.state_received_ns = self.node.clock_received_ns = self.fixture.wall_ns
                def slow_identity():
                    self.fixture.wall_ns += 2_000_000_000
                    return identity
                def slow_geometry(_):
                    self.fixture.wall_ns += 2_000_000_000
                    return original_geometry
                self.node._native_cache_provenance.side_effect = slow_identity if stage == 'identity' else None
                self.node.evaluator.at.side_effect = slow_geometry if stage == 'geometry' else None
                with self.assertRaisesRegex(RuntimeError, 'stale'):
                    self.node._dispatch_self_abort()
                self.node.dispatch.assert_not_called()

    def test_fresh_but_aged_clock_cannot_overstate_future_start_margin(self):
        self.node.execution_ledger = NS(attempts=[NS(submission_started=False)])
        self.node.active = NS(data={'initial_state_rad':dict(self.node.actual)})
        self.node.motion_start = self.node.clock_last+.25
        self.fixture.wall_ns += 200_000_000  # still within the 300-ms recency gate
        with self.assertRaisesRegex(RuntimeError, 'reservation'):
            self.node._dispatch_self_abort()
        self.node.dispatch.assert_not_called()

    def test_native_and_policy_identity_change_after_planning_never_dispatches(self):
        for change in ('policy','native'):
            with self.subTest(change=change):
                def plan(*a):
                    if change=='policy':self.node.self_policy_path.write_text('changed')
                    else:self.node._native_cache_provenance.return_value={'new_component':'changed'}
                    return self.fixture.curve
                self.node.coordinated_path.side_effect=plan
                result=self.command();self.assertEqual(result['dispatched_goals'],0);self.node.dispatch.assert_not_called()
                self.node.self_policy_path.write_text('test self policy snapshot')
                self.node._native_cache_provenance.return_value=dict(engine='MuJoCo 3.12.0',self_collision='test-native-component')

    def test_start_state_self_clearance_rechecks_after_planning(self):
        def plan(*a):
            self.node.evaluator.at.return_value['self_collision']['distance_m']=.0005
            return self.fixture.curve
        self.node.coordinated_path.side_effect=plan
        result=self.command();self.assertEqual(result['failure_kind'],'self_clearance')
        self.assertEqual(result['dispatched_goals'],0);self.node.dispatch.assert_not_called()

    def test_approved_exact_path_is_rejected_if_self_geometry_gate_changes(self):
        _,_,approval=self.fixture.make_review()
        self.node.evaluator.at.return_value['self_collision']['distance_m']=-.001
        self.fixture.send(approval,'approve');self.node.process_next()
        self.node.dispatch.assert_not_called()
