"""Admission, read-only reconnect and loopback HTTP mutation boundaries."""
from copy import deepcopy
from pathlib import Path
import hashlib
import json
import queue
import tempfile
import threading
import unittest
from types import SimpleNamespace as NS
from unittest.mock import patch
from urllib.request import Request, urlopen
from urllib.error import HTTPError
from urllib.parse import quote

from mujoco_ros2_manipulation.operator_console.state import ConsoleState
from mujoco_ros2_manipulation.operator_console.application import ConsoleApplication
from mujoco_ros2_manipulation.operator_console.http_server import ConsoleHTTPServer, Handler
from mujoco_ros2_manipulation.workcells.motion_request import command


def fixture():
    now=[0.0]
    config=dict(home_rad=dict(l=0.0,r=0.0),joint_limits={n:dict(lower=-1.0,upper=1.0) for n in ('l','r')})
    state=ConsoleState(config,clock=lambda:now[0]);now[0]=3
    state.graph=dict(clock=1,joints=1,status=1,commands=1,guards=1)
    state.identity=True;state.guard=dict(latched=False,phase='ready')
    state.joints(['l','r'],[0.0,0.0],[0.0,0.0]);state.tick(1.0)
    state.status(dict(session_id='session-a',request_id=None,status='ready'))
    return state,now


class ConsoleAdmissionTests(unittest.TestCase):
    def test_stale_state_repeated_clock_and_duplicate_owners_block_dispatch(self):
        for mode in ('state','clock','duplicate','identity','latched','blocked'):
            state,now=fixture();self.assertTrue(state.snapshot()['can_submit'])
            if mode=='state':now[0]+=0.51;state.tick(1.5)
            if mode=='clock':now[0]+=0.51;state.joints(['l','r'],[0,0],[0,0]);state.tick(1.0)
            if mode=='duplicate':state.graph['status']=2
            if mode=='identity':state.identity=False
            if mode=='latched':state.guard.update(latched=True,phase='stopped')
            if mode=='blocked':state.status(dict(session_id='session-a',request_id=None,status='blocked'))
            with self.subTest(mode=mode),self.assertRaises(RuntimeError):state.reserve(command(dict(l=0.2,r=0.2)))
            self.assertIsNone(state.active)
        state,_=fixture();state.graph['commands']=2 # Passive topic monitors do not own the controller.
        self.assertTrue(state.snapshot()['can_submit'])

    def test_reservation_is_immediate_idempotent_and_does_not_replay_changed_targets(self):
        state,_=fixture();payload=command(dict(l=0.2,r=0.2))
        self.assertTrue(state.reserve(payload));self.assertFalse(state.reserve(payload))
        with self.assertRaises(RuntimeError):state.reserve(command(dict(l=0.1,r=0.1)))
        changed=deepcopy(payload);changed['target_rad']['l']=0.5
        with self.assertRaises(ValueError):state.reserve(changed)
        state.status(dict(session_id='session-a',request_id=payload['request_id'],status='cancelled',stage='terminal',metrics={}))
        self.assertIsNone(state.active);self.assertFalse(state.snapshot()['can_submit'])
        state.status(dict(session_id='session-a',request_id=None,status='ready'))
        self.assertTrue(state.snapshot()['can_submit'])

    def test_partial_nonfinite_states_and_malformed_status_do_not_refresh_readiness(self):
        state,now=fixture();now[0]+=0.51;state.tick(1.5)
        for names,positions in ((['l'],[0]),(['l','l'],[0,0]),(['l','r'],[float('nan'),0])):
            state.joints(names,positions,[0,0])
        state.status(dict(session_id='session-a',request_id='not-an-id',status='running'))
        state.status(dict(session_id='session-a',status='unexpected'))
        self.assertFalse(state.snapshot()['can_submit']);self.assertIsNone(state.active)

    def test_server_restart_preserves_unknown_outcome_instead_of_success(self):
        state,_=fixture();payload=command(dict(l=.2,r=.2));state.reserve(payload)
        state.status(dict(session_id='session-b',request_id=None,status='ready'))
        self.assertEqual(state.history[payload['request_id']]['status'],'unknown')
        self.assertIsNone(state.active)


    def test_active_request_survives_many_external_rejections(self):
        state,_=fixture();payload=command(dict(l=.2,r=.2));state.reserve(payload)
        for i in range(100):
            state.status(dict(session_id='session-a',request_id=f'{i:032x}',status='rejected'))
        self.assertIn(payload['request_id'],state.history)
        self.assertEqual(state.active,payload['request_id'])
        state.status(dict(session_id='session-b',request_id=None,status='ready'))
        self.assertEqual(state.history[payload['request_id']]['status'],'unknown')


class ConsoleOwnershipTests(unittest.TestCase):
    def test_close_never_cancels_foreign_request_or_stops_unowned_process(self):
        state,_=fixture();foreign='f'*32
        state.status(dict(session_id='session-a',request_id=foreign,status='running'))
        with tempfile.TemporaryDirectory() as temp:
            bridge=NS(outbox=queue.Queue());app=ConsoleApplication(NS(root=Path(temp)),state,bridge,Path(temp)/'console',render=False)
            with patch('mujoco_ros2_manipulation.operator_console.application.stop_process') as stop:
                app.close();stop.assert_not_called()
            self.assertTrue(bridge.outbox.empty());self.assertEqual(state.active,foreign)

    def test_stop_drops_unpublished_command_and_suspends_future_dispatch(self):
        state,_=fixture();payload=command(dict(l=.2,r=.2));state.reserve(payload)
        with tempfile.TemporaryDirectory() as temp:
            bridge=NS(outbox=queue.Queue());bridge.outbox.put(('command',payload))
            app=ConsoleApplication(NS(root=Path(temp)),state,bridge,Path(temp)/'console',render=False)
            app.close()
            self.assertTrue(bridge.outbox.empty());self.assertIsNone(state.active)
            self.assertEqual(state.history[payload['request_id']]['status'],'cancelled')
            self.assertFalse(state.snapshot()['can_submit'])

    def test_cancelled_start_worker_cannot_spawn_after_stop(self):
        state,_=fixture()
        with tempfile.TemporaryDirectory() as temp:
            app=ConsoleApplication(NS(root=Path(temp)),state,NS(outbox=queue.Queue()),Path(temp)/'console',render=False)
            app.phase='idle';app.generation=2
            with patch.object(app,'spawn') as spawn:
                app.start_worker(dict(native_gui=False,planning_case='none'),Path(temp),False,1)
                spawn.assert_not_called()
            app.close()

    def test_startup_failure_reports_dds_cause_from_simulation_log(self):
        state,_=fixture()
        with tempfile.TemporaryDirectory() as temp:
            folder=Path(temp)
            (folder/'simulation.log').write_text('Failed to find a free participant index for domain 144\n')
            (folder/'commands.log').write_text('RuntimeError: Timeout: contact guard ready\n')
            app=ConsoleApplication(NS(root=folder),state,NS(outbox=queue.Queue()),folder/'console',render=False)
            app.last_launch=folder
            app.children=dict(commands=NS(poll=lambda:1))
            error=app.process_failure(preparing=True)
            self.assertIn('DDS 참가자 슬롯 부족',error)
            self.assertIn('commands (exit=1)',error)
            self.assertIn(str(folder),error)
            app.children.clear();app.close()

    def test_renderer_exit_does_not_masquerade_as_core_process_failure(self):
        state,_=fixture()
        with tempfile.TemporaryDirectory() as temp:
            app=ConsoleApplication(NS(root=Path(temp)),state,NS(outbox=queue.Queue()),Path(temp)/'console',render=False)
            app.children=dict(simulation=NS(poll=lambda:None),renderer=NS(poll=lambda:1))
            self.assertIsNone(app.process_failure())
            app.children['commands']=NS(poll=lambda:2)
            self.assertIn('commands (exit=2)',app.process_failure())
            app.children.clear();app.close()

    def test_stop_orders_owned_command_owner_before_simulation(self):
        state,_=fixture()
        with tempfile.TemporaryDirectory() as temp:
            app=ConsoleApplication(NS(root=Path(temp)),state,NS(outbox=queue.Queue()),Path(temp)/'console',render=False)
            commands=NS(returncode=0);simulation=NS(returncode=0);renderer=NS(returncode=0)
            app.children=dict(simulation=simulation,commands=commands,renderer=renderer)
            with patch('mujoco_ros2_manipulation.operator_console.application.stop_process',return_value=False) as stop:
                app.close();self.assertEqual([c.args[0] for c in stop.call_args_list],[commands,simulation,renderer])


class ConsoleHTTPTests(unittest.TestCase):
    def setUp(self):
        self.temp=tempfile.TemporaryDirectory();root=Path(self.temp.name)
        (root/'docs/reports').mkdir(parents=True)
        (root/'docs/reports/catalog.json').write_text('{"reports":[]}')
        (root/'docs/reports/site-manifest.json').write_text('{"files":{}}')
        self.called=[]
        self.app=NS(cell=NS(root=root),directory=root,state=NS(config=dict(joint_limits={n:dict(lower=-1,upper=1) for n in ('l','r')})),
            start=lambda payload:self.called.append(payload) or dict(phase='starting'),snapshot=lambda:dict(can_submit=False),files=[])
        self.server=ConsoleHTTPServer(('127.0.0.1',0),Handler);self.server.application=self.app;self.server.token='test-token'
        self.thread=threading.Thread(target=self.server.serve_forever,daemon=True);self.thread.start()
        self.url='http://127.0.0.1:'+str(self.server.server_address[1])
    def tearDown(self):
        self.server.shutdown();self.server.server_close();self.thread.join();self.temp.cleanup()
    def post(self,path,body,**headers):
        return urlopen(Request(self.url+path,data=body,headers={'Content-Type':'application/json','Origin':self.url,'X-Operator-Token':'test-token',**headers}),timeout=3)

    def publication_fixture(self):
        root=self.app.cell.root
        assets={
            'index.html':b'<a href="operator/demo/index.html">Demo</a>',
            'catalog.json':b'{"reports":[]}',
            'sample/index.html':b'<a href="../index.html">Report catalog</a>',
            'operator/demo/index.html':b'<img src="poster.png"><video src="motion.mp4"></video>',
            'operator/demo/poster.png':b'\x89PNG\r\n\x1a\nHTTP transport fixture',
            'operator/demo/motion.mp4':b'\x00\x00\x00\x18ftypisom\x00\x00\x02\x00isommp41',
            'operator/demo/\ud55c\uae00 \uc774\ubbf8\uc9c0.png':b'\x89PNG\r\n\x1a\nEncoded path fixture',
        }
        for name,content in assets.items():
            if name.startswith('operator/'):
                destination=root/'docs'/name
            else:
                destination=root/'docs/reports'/name
            destination.parent.mkdir(parents=True,exist_ok=True)
            destination.write_bytes(content)
        metadata={name:dict(bytes=len(content),sha256=hashlib.sha256(content).hexdigest())
                  for name,content in assets.items()}
        (root/'docs/reports/site-manifest.json').write_text(json.dumps(dict(files=metadata)))
        return assets

    def test_report_and_operator_publication_assets_keep_their_original_urls(self):
        assets=self.publication_fixture()
        for name,content in assets.items():
            with self.subTest(name=name),urlopen(self.url+'/reports/'+quote(name,safe='/'),timeout=3) as response:
                self.assertEqual(response.status,200)
                self.assertEqual(response.read(),content)
                if name.endswith('.png'):self.assertEqual(response.headers.get_content_type(),'image/png')
                if name.endswith('.mp4'):self.assertEqual(response.headers.get_content_type(),'video/mp4')
        for route,name in (('/reports/','index.html'),('/reports/sample/','sample/index.html'),
                           ('/reports/operator/demo/','operator/demo/index.html')):
            with self.subTest(route=route),urlopen(self.url+route,timeout=3) as response:
                self.assertEqual(response.read(),assets[name])
        with urlopen(self.url+'/reports/site-manifest.json',timeout=3) as response:
            self.assertEqual(response.read(),(self.app.cell.root/'docs/reports/site-manifest.json').read_bytes())

    def test_measurement_report_named_operator_keeps_its_existing_source(self):
        self.publication_fixture()
        root=self.app.cell.root
        assets={'operator/index.html':b'Measurement report named operator',
                'operator/figures/chart.png':b'\x89PNG\r\n\x1a\nMeasurement evidence'}
        manifest_path=root/'docs/reports/site-manifest.json'
        manifest=json.loads(manifest_path.read_text())
        for name,content in assets.items():
            source=root/'docs/reports'/name
            source.parent.mkdir(parents=True,exist_ok=True);source.write_bytes(content)
            manifest['files'][name]=dict(bytes=len(content),sha256=hashlib.sha256(content).hexdigest())
        manifest_path.write_text(json.dumps(manifest))
        (root/'docs/operator/index.html').write_bytes(b'Do not shadow the existing measurement route')
        for name,content in assets.items():
            with self.subTest(name=name),urlopen(self.url+'/reports/'+name,timeout=3) as response:
                self.assertEqual(response.read(),content)
        with urlopen(self.url+'/reports/operator/',timeout=3) as response:
            self.assertEqual(response.read(),assets['operator/index.html'])

    def test_report_routes_reject_unregistered_files_and_encoded_path_escapes(self):
        self.publication_fixture()
        root=self.app.cell.root
        for path in (root/'private.env',root/'docs/private.env',root/'docs/operator/demo/private.env'):
            path.write_bytes(b'NEVER SERVE THIS FILE')
        manifest_path=root/'docs/reports/site-manifest.json'
        manifest=json.loads(manifest_path.read_text())
        # Even a malformed allowlist must not make a noncanonical path readable.
        for name in ('../private.env','operator/demo/../../private.env','operator/demo/./index.html',
                     'operator/demo//index.html','operator\\demo\\index.html',
                     'operator/demo/%2e%2e/private.env',str(root/'private.env')):
            manifest['files'][name]={}
        manifest_path.write_text(json.dumps(manifest))
        routes=('/reports/operator/demo/private.env','/reports/../private.env','/reports/%2e%2e/private.env',
                '/reports/operator/demo/%2e%2e/%2e%2e/private.env',
                '/reports/operator/demo/%252e%252e/private.env','/reports/operator%5cdemo%5cindex.html',
                '/reports/operator/demo/./index.html','/reports/operator/demo//index.html',
                '/reports/'+str(root/'private.env'))
        for route in routes:
            with self.subTest(route=route),self.assertRaises(HTTPError) as caught:
                urlopen(self.url+route,timeout=3)
            self.assertEqual(caught.exception.code,404)
            self.assertNotIn(b'NEVER SERVE THIS FILE',caught.exception.read())

    def test_report_routes_reject_symlinked_assets_and_publication_directories(self):
        self.publication_fixture()
        root=self.app.cell.root
        private=root/'private.env';private.write_bytes(b'NEVER SERVE THIS FILE')
        linked=root/'docs/operator/demo/linked.png';linked.symlink_to(private)
        linked_directory=root/'docs/operator/demo/linked';linked_directory.symlink_to(root,target_is_directory=True)
        manifest_path=root/'docs/reports/site-manifest.json'
        manifest=json.loads(manifest_path.read_text())
        manifest['files'].update({'operator/demo/linked.png':{},'operator/demo/linked/private.env':{}})
        manifest_path.write_text(json.dumps(manifest))
        for route in ('/reports/operator/demo/linked.png','/reports/operator/demo/linked/private.env'):
            with self.subTest(route=route),self.assertRaises(HTTPError) as caught:
                urlopen(self.url+route,timeout=3)
            self.assertEqual(caught.exception.code,404)
        for path,route in ((root/'docs','/reports/index.html'),
                           (root/'docs/reports','/reports/index.html'),
                           (root/'docs/operator','/reports/operator/demo/index.html'),
                           (root/'docs/operator/demo','/reports/operator/demo/index.html'),
                           (root/'docs/reports/sample','/reports/sample/index.html'),
                           (manifest_path,'/reports/operator/demo/index.html')):
            with self.subTest(path=path):
                saved=path.with_name(path.name+'-saved')
                is_directory=path.is_dir()
                path.rename(saved)
                path.symlink_to(saved,target_is_directory=is_directory)
                try:
                    with self.assertRaises(HTTPError) as caught:urlopen(self.url+route,timeout=3)
                    self.assertEqual(caught.exception.code,404)
                finally:
                    path.unlink();saved.rename(path)

    def test_closed_port_can_restart_but_running_listener_cannot_be_shared(self):
        address=self.server.server_address
        with self.assertRaises(OSError):ConsoleHTTPServer(address,Handler)
        with urlopen(self.url+'/api/state') as response:response.read()
        self.server.shutdown();self.server.server_close();self.thread.join()
        replacement=ConsoleHTTPServer(address,Handler)
        replacement.server_close()

    def test_remote_origin_missing_token_and_rebinding_host_cannot_mutate(self):
        for headers in ({'Origin':'https://example.com'},{'X-Operator-Token':''},{'Host':'attacker.example'}):
            with self.subTest(headers=headers),self.assertRaises(HTTPError) as caught:self.post('/api/start',b'{}',**headers)
            self.assertEqual(caught.exception.code,403)
        self.assertEqual(self.called,[])
        with self.post('/api/start',b'{}') as response:self.assertEqual(response.status,202)
        self.assertEqual(self.called,[{}])
    def test_strict_target_import_rejects_duplicates_nonfinite_and_incomplete_mapping(self):
        for raw in ('{"l":0,"l":1,"r":0}','{"l":NaN,"r":0}','{"l":0}'):
            with self.subTest(raw=raw),self.assertRaises(HTTPError) as caught:self.post('/api/target',json.dumps(dict(text=raw)).encode())
            self.assertEqual(caught.exception.code,400)
        with self.post('/api/target',json.dumps(dict(text='{"l":0.2,"r":-0.2}')).encode()) as response:
            self.assertEqual(json.load(response)['target_rad'],dict(l=.2,r=-.2))
    def test_large_body_unknown_files_and_traversal_are_rejected(self):
        with self.assertRaises(HTTPError) as caught:self.post('/api/start',b'x'*65537)
        self.assertEqual(caught.exception.code,413)
        for route in ('/reports/../../pyproject.toml','/reports/%2e%2e/%2e%2e/pyproject.toml','/static/not-present.js'):
            with self.subTest(route=route),self.assertRaises(HTTPError) as caught:urlopen(self.url+route)
            self.assertEqual(caught.exception.code,404)
        with urlopen(self.url+'/') as response:self.assertIn(b'ROS',response.read())
