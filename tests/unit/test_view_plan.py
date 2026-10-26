"""Read-only saved-plan replay rejects cross-session/model or partial paths."""
from copy import deepcopy
import json
import unittest
from mujoco_ros2_manipulation.operator_console.view_plan import SavedPlan


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


if __name__=='__main__':unittest.main()
