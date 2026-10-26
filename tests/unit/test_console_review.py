"""Plan review never publishes motion before an explicit, current approval."""
import ast
from copy import deepcopy
from email.message import Message
import io
import json
from pathlib import Path
import queue
import tempfile
from types import SimpleNamespace as NS
import unittest
from unittest.mock import Mock

from mujoco_ros2_manipulation.operator_console.application import ConsoleApplication
from mujoco_ros2_manipulation.operator_console.http_server import Handler
from mujoco_ros2_manipulation.operator_console.state import ConsoleState
from mujoco_ros2_manipulation.workcells.motion_request import command


SESSION='a'*32
PLAN='b'*32
DIGEST='c'*64


def fixture():
    now=[0.]
    config=dict(home_rad=dict(l=0.,r=0.),joint_limits={name:dict(lower=-1.,upper=1.) for name in ('l','r')})
    state=ConsoleState(config,clock=lambda:now[0]);now[0]=3.
    state.graph=dict(clock=1,joints=1,status=1,commands=1,guards=1,plans=1,approvals=1)
    state.identity=True;state.guard=dict(latched=False,phase='ready')
    state.joints(['l','r'],[0.,0.],[0.,0.]);state.tick(1.)
    state.status(dict(session_id=SESSION,request_id=None,status='ready',review=None))
    return state,now


def planned(state):
    payload=command(dict(l=.1,r=.2),identifier=PLAN)
    state.reserve_plan(payload)
    review=dict(state='awaiting_approval',plan_request_id=PLAN,session_id=SESSION,
                plan_sha256=DIGEST,expires_after_sec=60.,remaining_sec=60.)
    state.status(dict(session_id=SESSION,request_id=PLAN,status='succeeded',stage='plan_ready',
        mode='plan_only',review=review,trace_path='/trusted/runtime/'+PLAN+'.json'))
    state.status(dict(session_id=SESSION,request_id=None,status='ready',review=review))
    return dict(schema_version=1,request_id='d'*32,plan_request_id=PLAN,plan_sha256=DIGEST,session_id=SESSION)


class ReviewStateTests(unittest.TestCase):
    def test_planning_finishes_without_motion_reservation_and_expiry_uses_receiver_clock(self):
        state,now=fixture();planned(state)
        self.assertIsNone(state.active)
        self.assertEqual(state.history[PLAN]['mode'],'plan_only')
        self.assertTrue(state.snapshot()['can_plan'])
        now[0]+=61
        self.assertEqual(state.review_snapshot()['state'],'expired')
        self.assertEqual(state.review_snapshot()['remaining_sec'],0)

    def test_foreign_and_malformed_reviews_never_create_an_approvable_plan(self):
        for change in ('unowned','hash','remaining','state','session'):
            with self.subTest(change=change):
                state,_=fixture();approval=planned(state)
                review=state.review_snapshot()
                if change=='unowned':review['plan_request_id']='e'*32
                if change=='hash':review['plan_sha256']='bad'
                if change=='remaining':review['remaining_sec']=float('nan')
                if change=='state':review.pop('state')
                if change=='session':review['session_id']='f'*32
                state.status(dict(session_id=SESSION,request_id=None,status='ready',review=review))
                self.assertIsNone(state.pending_plan)
                with self.assertRaises(RuntimeError):state.reserve_approval(approval)

    def test_approval_requires_current_exact_token_and_new_identity(self):
        for key,value in (('plan_sha256','f'*64),('session_id','f'*32),('plan_request_id','f'*32)):
            state,_=fixture();approval=planned(state);approval[key]=value
            with self.subTest(key=key),self.assertRaises(RuntimeError):state.reserve_approval(approval)
            self.assertIsNone(state.active)
        state,_=fixture();approval=planned(state)
        self.assertTrue(state.reserve_approval(approval))
        self.assertFalse(state.reserve_approval(approval))
        self.assertEqual(state.active,approval['request_id'])
        self.assertEqual(state.pending_plan['state'],'approval_queued')
        reserved=state.snapshot()['history'][0]
        self.assertEqual(reserved['approval'],approval)
        self.assertEqual(reserved['plan_request_id'],PLAN)
        self.assertEqual(reserved['execution_request_id'],approval['request_id'])
        state.status(dict(session_id=SESSION,request_id=approval['request_id'],status='running',stage='safety_checks'))
        self.assertEqual(state.snapshot()['history'][0]['approval'],approval)
        changed=dict(approval,request_id='e'*32)
        with self.assertRaises(RuntimeError):state.reserve_approval(changed)

    def test_server_replacement_and_new_command_invalidate_pending_approval(self):
        state,_=fixture();approval=planned(state)
        state.status(dict(session_id='e'*32,request_id=None,status='ready',review=None))
        self.assertIsNone(state.pending_plan)
        with self.assertRaises(RuntimeError):state.reserve_approval(approval)
        state,_=fixture();planned(state)
        state.reserve(command(dict(l=.3,r=.3)))
        self.assertEqual(state.pending_plan['state'],'invalidated')


class ReviewApplicationTests(unittest.TestCase):
    def test_request_details_projects_read_only_review_and_exact_execution_link(self):
        state,_=fixture();approval=planned(state)
        with tempfile.TemporaryDirectory() as temp:
            root=Path(temp);runtime=root/'runtime';runtime.mkdir()
            app=ConsoleApplication(NS(root=root),state,NS(outbox=queue.Queue()),root/'console',render=False)
            record=dict(request_id=PLAN,mode='plan_only',command=command(dict(l=.1,r=.2),identifier=PLAN),
                        initial_state_rad=dict(l=0.,r=0.),
                        planned_clearance=dict(complete=True,duration_sec=2.,
                            minimum=dict(distance_m=.04,time_sec=.75,bodies=['left','right'])))
            path=runtime/(PLAN+'.json');path.write_text(json.dumps(record))
            state.history[PLAN]['trace_path']=str(path)
            before=deepcopy(state.pending_plan)
            details=app.request_details(PLAN)
            self.assertEqual(details['review_summary']['target_delta_rad'],dict(l=.1,r=.2))
            self.assertEqual(details['review_summary']['minimum'],dict(distance_m=.04,time_sec=.75,pair=['left','right']))
            self.assertEqual(state.pending_plan,before)
            self.assertNotIn('review_payload',details)
            details['review_summary']['target_delta_rad']['l']=99.
            self.assertEqual(app.request_details(PLAN)['review_summary']['target_delta_rad']['l'],.1)
            state.reserve_approval(approval)
            execution=runtime/(approval['request_id']+'.json')
            execution.write_text(json.dumps(dict(record,request_id=approval['request_id'],mode='approved_plan',approval=approval)))
            state.status(dict(session_id=SESSION,request_id=approval['request_id'],status='failed',trace_path=str(execution)))
            details=app.request_details(approval['request_id'])
            self.assertEqual(details['approval'],approval)
            self.assertEqual(details['plan_request_id'],PLAN)
            self.assertEqual(details['execution_request_id'],approval['request_id'])
            app.close()

    def test_plan_only_approval_and_discard_use_separate_bridge_messages(self):
        state,_=fixture()
        with tempfile.TemporaryDirectory() as temp:
            bridge=NS(outbox=queue.Queue())
            app=ConsoleApplication(NS(root=Path(temp)),state,bridge,Path(temp)/'console',render=False)
            app.views.approve_review=Mock()
            payload=command(dict(l=.1,r=.2),identifier=PLAN)
            app.create_plan(payload)
            self.assertEqual(bridge.outbox.get_nowait(),('plan',payload))
            # There is no command publication or execution while planning.
            self.assertTrue(bridge.outbox.empty())
            approval=planned(state)
            app.execute_plan(approval)
            app.views.approve_review.assert_called_once_with(PLAN)
            self.assertEqual(bridge.outbox.get_nowait(),('approve',approval))
            state.status(dict(session_id=SESSION,request_id=approval['request_id'],status='cancelled'))
            app.close()
        state,_=fixture()
        with tempfile.TemporaryDirectory() as temp:
            bridge=NS(outbox=queue.Queue());app=ConsoleApplication(NS(root=Path(temp)),state,bridge,Path(temp)/'console',render=False)
            planned(state);app.discard_plan(PLAN)
            self.assertEqual(bridge.outbox.get_nowait(),('cancel',dict(request_id=PLAN)))
            self.assertEqual(state.pending_plan['state'],'invalidated')
            app.close()

    def test_expired_or_changed_plan_never_enters_view_approval_or_queue(self):
        state,now=fixture()
        with tempfile.TemporaryDirectory() as temp:
            bridge=NS(outbox=queue.Queue());app=ConsoleApplication(NS(root=Path(temp)),state,bridge,Path(temp)/'console',render=False)
            app.views.approve_review=Mock();approval=planned(state)
            state.pending_plan['state']='invalidated'
            with self.assertRaises(RuntimeError):app.execute_plan(approval)
            app.views.approve_review.assert_not_called();self.assertTrue(bridge.outbox.empty())
            app.close()

    def test_empty_approval_and_preview_without_a_plan_cannot_enqueue_anything(self):
        state,_=fixture()
        with tempfile.TemporaryDirectory() as temp:
            bridge=NS(outbox=queue.Queue());app=ConsoleApplication(NS(root=Path(temp)),state,bridge,Path(temp)/'console',render=False)
            app.views.approve_review=Mock()
            self.assertFalse(app.snapshot()['can_approve'])
            with self.assertRaises(ValueError):app.execute_plan({})
            approval=planned(state)
            self.assertTrue(app.snapshot()['can_approve'])
            app.views.mode='plan_preview'
            # Review display blocks a new goal but permits an explicit approval.
            self.assertFalse(app.snapshot()['can_plan'])
            self.assertTrue(app.snapshot()['can_approve'])
            state.pending_plan['state']='expired'
            self.assertFalse(app.snapshot()['can_approve'])
            with self.assertRaises(RuntimeError):app.execute_plan(approval)
            app.views.approve_review.assert_not_called();self.assertTrue(bridge.outbox.empty())
            app.close()


class ReviewBridgeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        source=Path('src/mujoco_ros2_manipulation/operator_console/ros_adapter.py')
        tree=ast.parse(source.read_text())
        bridge=next(node for node in tree.body if isinstance(node,ast.ClassDef) and node.name=='ConsoleBridge')
        method=deepcopy(next(node for node in bridge.body if isinstance(node,ast.FunctionDef) and node.name=='send'))
        namespace=dict(queue=queue,json=json,String=lambda **kwargs:NS(**kwargs))
        exec(compile(ast.Module(body=[method],type_ignores=[]),str(source),'exec'),namespace)
        cls.send=staticmethod(namespace['send'])

    def bridge(self,state):
        return NS(state=state,outbox=queue.Queue(),commands=NS(publish=Mock()),plans=NS(publish=Mock()),
                  approvals=NS(publish=Mock()),cancels=NS(publish=Mock()))

    def test_plan_and_approval_publish_only_their_owned_topic_once(self):
        state,_=fixture();payload=command(dict(l=.1,r=.2),identifier=PLAN);state.reserve_plan(payload)
        bridge=self.bridge(state);bridge.outbox.put(('plan',payload));self.send(bridge);self.send(bridge)
        bridge.plans.publish.assert_called_once();bridge.commands.publish.assert_not_called();bridge.approvals.publish.assert_not_called()
        approval=planned(state);state.reserve_approval(approval);bridge.outbox.put(('approve',approval))
        self.send(bridge);bridge.approvals.publish.assert_called_once();bridge.commands.publish.assert_not_called()

    def test_restarted_server_cannot_receive_a_stale_approval_from_queue(self):
        state,_=fixture();approval=planned(state);state.reserve_approval(approval)
        bridge=self.bridge(state);bridge.outbox.put(('approve',approval))
        state.status(dict(session_id='e'*32,request_id=None,status='ready',review=None))
        self.send(bridge);bridge.approvals.publish.assert_not_called()
        self.assertEqual(state.history[approval['request_id']]['status'],'unknown')


class ReviewHTTPTests(unittest.TestCase):
    def request(self,path,payload,token='secret'):
        value=object.__new__(Handler);value.path=path;value.headers=Message()
        value.headers['Host']='127.0.0.1:8890';value.headers['Origin']='http://127.0.0.1:8890'
        value.headers['X-Operator-Token']=token;value.headers['Content-Type']='application/json'
        raw=json.dumps(payload).encode();value.headers['Content-Length']=str(len(raw));value.rfile=io.BytesIO(raw)
        app=NS(create_plan=Mock(return_value={}),execute_plan=Mock(return_value={}),discard_plan=Mock(return_value={}),
               views=NS(select_review_plan=Mock(return_value={})))
        value.server=NS(application=app,server_address=('127.0.0.1',8890),token='secret')
        value.respond=Mock();value.do_POST();return value,app

    def test_new_routes_require_the_same_origin_token_and_exact_preview_discard_shape(self):
        for route,method in (('/api/plan','create_plan'),('/api/plan/execute','execute_plan')):
            value,app=self.request(route,{},token='wrong');self.assertEqual(value.respond.call_args.args[1],403)
            getattr(app,method).assert_not_called()
            value,app=self.request(route,{});getattr(app,method).assert_called_once_with({})
        for route,method in (('/api/view/review','select_review_plan'),('/api/plan/discard','discard_plan')):
            value,app=self.request(route,dict(request_id=PLAN,extra=1));self.assertEqual(value.respond.call_args.args[1],404)
            target=getattr(app.views,method) if route.endswith('review') else getattr(app,method);target.assert_not_called()
            value,app=self.request(route,dict(request_id=PLAN))
            target=getattr(app.views,method) if route.endswith('review') else getattr(app,method);target.assert_called_once_with(PLAN)


if __name__=='__main__':unittest.main()
