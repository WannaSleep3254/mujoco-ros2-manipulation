"""Read-only saved-plan replay rejects cross-session/model or partial paths."""
from copy import deepcopy
import ast
from importlib.util import find_spec
import json
from pathlib import Path
from types import SimpleNamespace
import unittest
from mujoco_ros2_manipulation.operator_console.view_plan import SavedPlan, ReviewPlan
from mujoco_ros2_manipulation.workcells.motion_review import PlanReviewStore
from mujoco_ros2_manipulation.workcells.request_trace import RequestTrace


def fixture():
    limits={'joint':dict(lower=-2.,upper=2.)}
    manifest={'model':'test','joints':['joint'],'joint_limits':deepcopy(limits)}
    trace = dict(schema_version=1,request_id='a'*32, status='succeeded',error=None,
        events=[dict(name='terminal',wall_sec=2.)],initial_state_rad={'joint':0.0},
        command=dict(schema_version=1,request_id='a'*32,strategy='coordinated',target_rad={'joint':1.}),
        planned_clearance=dict(complete=True, minimum=dict(distance_m=.04),duration_sec=1.,samples=101),
        trajectories=[dict(names=['joint'], points=[dict(t=0.,p=[0.],v=[],a=[]),dict(t=1.,p=[1.],v=[],a=[])])])
    trace['planned_trajectories']=deepcopy(trace['trajectories'])
    session = dict(schema_version=1,session_id='b'*32, model_manifest=deepcopy(manifest),
        policy=dict(schema_version=1,min_inter_arm_distance_m=.02,tracking_reserve_m=.01))
    options = dict(request_id='a'*32,motion_session_id='b'*32,model_manifest=manifest,
        joint_names=['joint'],joint_limits=limits)
    return trace,session,options


def build(trace, session, options):
    return SavedPlan(json.dumps(trace).encode(),json.dumps(session).encode(),**options)


def production_review_fixture():
    """Use the server's real issuer and store, without constructing a ROS node."""
    trace_data,session,options=fixture()
    source=Path(find_spec('mujoco_ros2_manipulation.workcells.motion_server').origin)
    tree=ast.parse(source.read_text())
    server=next(node for node in tree.body if isinstance(node,ast.ClassDef) and node.name=='MotionCommandServer')
    method=deepcopy(next(node for node in server.body if isinstance(node,ast.FunctionDef) and node.name=='_issue_review'))
    def require(condition,message):
        if not condition:raise ValueError(message)
    namespace={'require':require}
    exec(compile(ast.Module(body=[method],type_ignores=[]),str(source),'exec'),namespace)
    store=PlanReviewStore(options['motion_session_id'],clock=lambda:3.)
    identity={'build':{'model':'test'},'planning_scene_sha256':'f'*64}
    owner=SimpleNamespace(_review_runtime_identity=lambda:deepcopy(identity),
        confirm_start_unchanged=lambda state:None,review_store=store,clock_last=7.)
    trace=RequestTrace(options['request_id'],clock=lambda:1_000_000_000)
    trace.data.update({key:deepcopy(value) for key,value in trace_data.items()
                       if key not in ('events','status','error')})
    trace.data.update(mode='plan_only',dispatched_goals=0)
    namespace['_issue_review'](owner,trace,trace.data['initial_state_rad'],identity)
    return trace.data,session,options,store.summary()


def build_review(trace,session,options,review):
    return ReviewPlan(json.dumps(trace).encode(),json.dumps(session).encode(),review=review,**options)


class ViewPlanTests(unittest.TestCase):
    def test_detached_interpolation_and_end_hold(self):
        trace,session,options=fixture();plan=build(trace,session,options)
        self.assertEqual(plan.duration,1.)
        self.assertEqual(plan.positions(.5),{'joint':.5})
        trace['trajectories'][0]['points'][1]['p'][0]=2.
        self.assertEqual(plan.positions(1.),{'joint':1.})

    def test_failed_or_partial_not_accepted(self):
        for mutate in (lambda t:t.update(status='failed'), lambda t:t['planned_clearance'].update(complete=False),
                       lambda t:t['planned_clearance']['minimum'].update(distance_m=.029), lambda t:t.update(trajectories=[])):
            trace,session,options=fixture();mutate(trace)
            with self.assertRaises(ValueError):build(trace,session,options)

    def test_request_session_and_model_must_match(self):
        for key,value in [('request_id','c'*32),('motion_session_id','c'*32),('model_manifest',{'model':'other'})]:
            trace,session,options=fixture();options[key]=value
            with self.assertRaises(ValueError):build(trace,session,options)

    def test_joint_coverage_and_limits(self):
        for mutate in (lambda t:t['trajectories'][0].update(names=['other']),
                       lambda t:t['trajectories'].append(deepcopy(t['trajectories'][0])),
                       lambda t:t['trajectories'][0]['points'][1].update(p=[3.])):
            trace,session,options=fixture();mutate(trace)
            with self.assertRaises(ValueError):build(trace,session,options)

    def test_time_finite_and_in_bounds(self):
        trace,session,options=fixture();plan=build(trace,session,options)
        for stamp in (-.1,1.1,float('nan'),float('inf'),True):
            with self.assertRaises(ValueError):plan.positions(stamp)

    def test_duplicate_and_nonfinite_json(self):
        trace,session,options=fixture()
        for raw in (b'{"request_id":"x","request_id":"y"}',b'{"x":NaN}'):
            with self.assertRaises(ValueError):SavedPlan(raw,json.dumps(session).encode(),**options)
        raw=(json.dumps(trace)[:-1]+',"unused":1e999}').encode()
        with self.assertRaisesRegex(ValueError,'Nonfinite'):
            SavedPlan(raw,json.dumps(session).encode(),**options)

    def test_success_is_terminal_and_command_uses_the_same_request(self):
        for mutate in (lambda t:t.update(schema_version=True),lambda t:t.update(events=[]),
            lambda t:t['events'][-1].update(name='running'),lambda t:t['events'][-1].update(wall_sec=-1.),
            lambda t:t.update(error={'message':'unexpected'}),lambda t:t['command'].update(request_id='c'*32),
            lambda t:t['command'].update(strategy='unknown')):
            trace,session,options=fixture();mutate(trace)
            with self.assertRaises(ValueError):build(trace,session,options)
        trace,session,options=fixture();session['schema_version']=2
        with self.assertRaises(ValueError):build(trace,session,options)

    def test_execution_curve_must_be_the_path_with_the_complete_quality_record(self):
        trace,session,options=fixture()
        trace['trajectories'][0]['points'][1]['p']=[1.5]
        with self.assertRaisesRegex(ValueError,'differs from the fully checked'):
            build(trace,session,options)
        for mutate in (lambda t:t['planned_clearance'].update(duration_sec=.9),
                       lambda t:t['planned_clearance'].update(samples=1)):
            trace,session,options=fixture();mutate(trace)
            with self.assertRaises(ValueError):build(trace,session,options)

    def test_missing_negative_or_nonfinite_policy_is_not_defaulted_to_zero(self):
        for mutate in (lambda p:p.pop('tracking_reserve_m'),lambda p:p.update(tracking_reserve_m=-.01),
            lambda p:p.update(min_inter_arm_distance_m=True),lambda p:p.update(min_inter_arm_distance_m=float('inf'))):
            trace,session,options=fixture();mutate(session['policy'])
            with self.assertRaises(ValueError):build(trace,session,options)

    def test_model_joint_limit_contract_is_exact_and_finite(self):
        for mutate in (lambda o:o['joint_limits']['joint'].update(upper=float('inf')),
            lambda o:o['joint_limits']['joint'].update(lower=3.),lambda o:o.update(joint_names='joint'),
            lambda o:o['model_manifest'].update(joints=['other'])):
            trace,session,options=fixture();mutate(options)
            with self.assertRaises(ValueError):build(trace,session,options)

    def test_start_and_every_knot_are_checked_even_when_endpoints_are_in_bounds(self):
        for points in ([dict(t=0.,p=[.02],v=[],a=[]),dict(t=1.,p=[1.],v=[],a=[])],
            [dict(t=0.,p=[0.],v=[],a=[]),dict(t=.5,p=[2.5],v=[],a=[]),dict(t=1.,p=[1.],v=[],a=[])]):
            trace,session,options=fixture();trace['trajectories'][0]['points']=points
            trace['planned_trajectories']=deepcopy(trace['trajectories'])
            with self.assertRaises(ValueError):build(trace,session,options)

    def test_cubic_and_quintic_interpolation_overshoot_never_returns_an_invalid_pose(self):
        for points in ([dict(t=0.,p=[0.],v=[10.],a=[]),dict(t=1.,p=[0.],v=[-10.],a=[])],
            [dict(t=0.,p=[0.],v=[0.],a=[100.]),dict(t=1.,p=[0.],v=[0.],a=[100.])]):
            trace,session,options=fixture();trace['trajectories'][0]['points']=points
            trace['planned_trajectories']=deepcopy(trace['trajectories'])
            trace['command']['target_rad']['joint']=0.
            plan=build(trace,session,options)
            self.assertEqual(plan.positions(0.),{'joint':0.})
            self.assertEqual(plan.positions(1.),{'joint':0.})
            with self.assertRaisesRegex(ValueError,'exceeds model limits'):plan.positions(.5)

    def test_two_disjoint_curves_cover_the_model_and_hold_the_shorter_endpoint(self):
        trace,session,options=fixture()
        options['joint_names'].append('other');options['joint_limits']['other']=dict(lower=-2.,upper=2.)
        options['model_manifest'].update(joints=list(options['joint_names']),joint_limits=deepcopy(options['joint_limits']))
        session['model_manifest']=deepcopy(options['model_manifest'])
        trace['initial_state_rad']['other']=0.;trace['command']['target_rad']['other']=-.5
        trace['trajectories'].append(dict(names=['other'],points=[dict(t=0.,p=[0.],v=[],a=[]),dict(t=.5,p=[-.5],v=[],a=[])]))
        trace['planned_trajectories']=deepcopy(trace['trajectories'])
        plan=build(trace,session,options)
        self.assertEqual(plan.positions(.75),{'joint':.75,'other':-.5})
        options['joint_limits']['other']['lower']=0.
        self.assertEqual(plan.positions(.75)['other'],-.5)

    def test_malformed_derivative_arrays_and_overflowing_numeric_values_fail_closed(self):
        for mutate in (lambda p:p.update(v={}),lambda p:p.update(p=[10**400]),lambda p:p.update(v=[True])):
            trace,session,options=fixture();mutate(trace['trajectories'][0]['points'][0])
            trace['planned_trajectories']=deepcopy(trace['trajectories'])
            with self.assertRaises(ValueError):build(trace,session,options)
        trace,session,options=fixture();plan=build(trace,session,options)
        with self.assertRaises(ValueError):plan.positions(10**400)


class ReviewPlanTests(unittest.TestCase):
    def test_real_issuer_finishes_terminal_after_plan_ready_and_replays_exact_store_payload(self):
        trace,session,options,review=production_review_fixture()
        self.assertEqual([item['name'] for item in trace['events'][-2:]],['plan_ready','terminal'])
        self.assertEqual(trace['status'],'succeeded')
        self.assertEqual(trace['dispatched_goals'],0)
        plan=build_review(trace,session,options,review)
        self.assertEqual(plan.positions(.5),{'joint':.5})
        self.assertEqual(plan.positions(1.),{'joint':1.})

    def test_changing_trace_path_or_anchor_cannot_keep_the_store_issued_digest(self):
        def changed_path(trace):
            trace['trajectories'][0]['points'][1]['p']=[1.5]
            trace['planned_trajectories']=deepcopy(trace['trajectories'])
        for mutate in (changed_path,
                       lambda trace:trace['initial_state_rad'].update(joint=.001),
                       lambda trace:trace['command']['target_rad'].update(joint=1.5)):
            with self.subTest(mutate=mutate):
                trace,session,options,review=production_review_fixture();mutate(trace)
                with self.assertRaisesRegex(ValueError,'exact pending plan digest'):
                    build_review(trace,session,options,review)

    def test_changing_the_detached_payload_or_digest_is_rejected_before_interpolation(self):
        for mutate in (
            lambda trace:trace['review_payload']['identity'].update(planning_scene_sha256='e'*64),
            lambda trace:trace['review_payload'].update(simulation_sec=8.),
            lambda trace:trace['review_payload']['trajectories'][0]['points'][1].update(p=[1.5]),
            lambda trace:trace['review'].update(plan_sha256='e'*64),
            lambda trace:trace.pop('review_payload'),
        ):
            with self.subTest(mutate=mutate):
                trace,session,options,review=production_review_fixture();mutate(trace)
                with self.assertRaises(ValueError):build_review(trace,session,options,review)

    def test_expired_consumed_or_invalidated_pending_review_is_not_a_valid_preview(self):
        for change in ({'remaining_sec':0.},{'remaining_sec':-1.},{'remaining_sec':float('nan')},
                       {'state':'expired'},{'state':'consumed'},{'state':'invalidated'}):
            with self.subTest(change=change):
                trace,session,options,review=production_review_fixture();review.update(change)
                with self.assertRaises(ValueError):build_review(trace,session,options,review)

    def test_pending_preview_and_executed_replay_cannot_be_substituted_for_each_other(self):
        trace,session,options,review=production_review_fixture()
        with self.assertRaisesRegex(ValueError,'pending-plan preview'):build(trace,session,options)
        for mode in ('automatic','approved_plan',None):
            with self.subTest(mode=mode):
                changed=deepcopy(trace);changed['mode']=mode
                with self.assertRaises(ValueError):build_review(changed,session,options,review)
        trace['dispatched_goals']=1
        with self.assertRaises(ValueError):build_review(trace,session,options,review)
        executed,session,options=fixture();executed['mode']='approved_plan'
        self.assertEqual(build(executed,session,options).positions(1.),{'joint':1.})


if __name__=='__main__':unittest.main()
