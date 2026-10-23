"""Startup-only trusted overlay sources never become ROS control targets."""
from copy import deepcopy
from importlib.util import find_spec,module_from_spec,spec_from_file_location
import io
import os
from pathlib import Path
import sys
import tempfile
from types import ModuleType,SimpleNamespace as NS
import unittest
from unittest.mock import Mock,patch

from mujoco_ros2_manipulation.operator_console.scene_host import SceneSource
from mujoco_ros2_manipulation.operator_console.application import ConsoleApplication
from tests.unit import test_console_views as fixtures
from tests.unit.test_console_views import FakeHost,console_class


class EnvironmentViewRegistryTests(unittest.TestCase):
    setUp=fixtures.ConsoleViewsTests.setUp

    def source(self,identifier='scene:installed_fixture',*,names=('q',),home=None,validate=None):
        return SceneSource(identifier,'installed fixture',self.cell.root/'private_overlay/cell.xml',
            names,{'q':0.} if home is None else home,validate or (lambda:{'manifest_sha256':'d'*64}))

    def create(self,sources,**kwargs):
        return console_class()(self.app,host_factory=kwargs.pop('host_factory',FakeHost),extra_sources=sources,**kwargs)

    def test_only_declared_id_can_select_overlay_and_live_stays_on_the_base(self):
        source=self.source();owner=self.create((source,))
        self.assertIn(source.id,owner.registry)
        self.assertIsNot(owner.registry[source.id],source,'Caller cannot mutate the registered contract')
        self.assertEqual(owner.catalog[-1],dict(id=source.id,label=source.label,available=True))
        previous=deepcopy(owner.catalog)
        for identifier in ('scene:unknown','/private/scene.xml','../fixture'):
            with self.subTest(identifier=identifier),self.assertRaises(ValueError):
                owner.select(dict(mode='model_preview',source_id=identifier))
        with self.assertRaisesRegex(ValueError,'현재 작업장면'):
            owner.select(dict(mode='live',source_id=source.id))
        self.assertEqual(owner.mode,'live')
        self.assertEqual(owner.catalog,previous)
        owner.select(dict(mode='model_preview',source_id=source.id))
        self.assertTrue(owner.preview)
        self.assertEqual(owner.source.id,source.id)
        self.assertEqual(self.app.state.positions,{'q':.1})
        self.assertTrue(self.app.state.admission_suspended)
        self.assertEqual(owner.base.xml_path,self.cell.output/'cell.xml')

    def test_invalid_duplicate_or_unbounded_sources_fail_before_host_start(self):
        source=self.source()
        cases=(None,iter((source,)),[object()],[self.source('profile:fr5')],
            [source,self.source()], [self.source('scene:fixture_'+str(i)) for i in range(9)])
        for sources in cases:
            host=Mock()
            with self.subTest(sources=type(sources).__name__),self.assertRaises(ValueError):
                self.create(sources,host_factory=host)
            host.assert_not_called()

    def test_overlay_must_keep_exact_named_joints_and_home(self):
        for source in (self.source(names=('other',),home={'other':0.}),self.source(home={'q':.01})):
            host=Mock()
            with self.subTest(source=source.home),self.assertRaisesRegex(ValueError,'관절'):
                self.create([source],host_factory=host)
            host.assert_not_called()
        source=self.source();source.home['q']=False
        with self.assertRaises(ValueError):self.create([source])

    def test_startup_validation_is_outside_locks_and_before_worker_creation(self):
        host=Mock();events=[]
        def invalid():
            self.assertFalse(self.app.lock._is_owned())
            self.assertFalse(self.app.state.lock._is_owned())
            host.assert_not_called();events.append('validation')
            raise ValueError('changed overlay')
        with self.assertRaisesRegex(ValueError,'changed overlay'):
            self.create([self.source(validate=invalid)],host_factory=host)
        self.assertEqual(events,['validation'])

    def test_changed_validator_token_or_home_cannot_replace_current_live_binding(self):
        token={'manifest_sha256':'d'*64};source=self.source(validate=lambda:deepcopy(token))
        owner=self.create([source]);owner.tick()
        before=owner.host.snapshot();token['manifest_sha256']='e'*64
        with self.assertRaisesRegex(ValueError,'장면이 바뀌었습니다'):
            owner.select(dict(mode='model_preview',source_id=source.id))
        self.assertEqual(owner.host.snapshot(),before)
        self.assertEqual(owner.mode,'live')
        token['manifest_sha256']='d'*64;source.home['q']=.1
        with self.assertRaisesRegex(ValueError,'모델 계약'):
            owner.select(dict(mode='model_preview',source_id=source.id))
        self.assertEqual(owner.host.snapshot(),before)
        self.assertTrue(self.app.state.admission_suspended)

    def test_disabled_display_rejects_extra_sources_without_touching_assets(self):
        host=Mock()
        def forbidden(cell):raise AssertionError('Disabled registry inspected generated assets')
        definition=console_class(check_workcell=forbidden)
        with self.assertRaisesRegex(ValueError,'비활성화'):
            definition(self.app,enabled=False,host_factory=host,extra_sources=[self.source()])
        host.assert_not_called()
        disabled=definition(NS(cell=NS()),enabled=False,host_factory=host)
        self.assertIsNone(disabled.host)


class EnvironmentApplicationRegistryTests(unittest.TestCase):
    def test_no_render_registry_is_rejected_before_directory_or_thread_creation(self):
        with tempfile.TemporaryDirectory() as temporary:
            path=Path(temporary)/'must-not-create'
            with patch('mujoco_ros2_manipulation.operator_console.application.threading.Thread') as thread:
                with self.assertRaises(ValueError):
                    ConsoleApplication(NS(),NS(),NS(),path,render=False,view_sources=(object(),))
                thread.assert_not_called()
            self.assertFalse(path.exists())


class EnvironmentCatalogCliTests(unittest.TestCase):
    def load_cli(self):
        source=find_spec('mujoco_ros2_manipulation.operator_console.__main__').origin
        spec=spec_from_file_location('mujoco_ros2_manipulation.operator_console._catalog_cli_test',source)
        module=module_from_spec(spec);spec.loader.exec_module(module)
        return module

    def test_no_render_catalog_combination_fails_before_any_catalog_or_ros_import(self):
        cli=self.load_cli()
        import builtins
        original=builtins.__import__
        def boundary(name,*args,**kwargs):
            if name=='rclpy' or name.endswith(('workcells.config','environment_sources')):
                raise AssertionError('Early rejection imported runtime: '+name)
            return original(name,*args,**kwargs)
        with patch.object(sys,'argv',['operator','--no-render','--view-scene-catalog','/private/catalog.json']), \
                patch('builtins.__import__',side_effect=boundary), \
                patch('sys.stderr',new_callable=io.StringIO):
            with self.assertRaises(SystemExit) as caught:cli.main()
        self.assertEqual(caught.exception.code,2)

    def test_catalog_preflight_failure_precedes_listening_or_ros_initialization(self):
        cli=self.load_cli();loader=Mock(side_effect=ValueError('frozen overlay changed'))
        catalog=ModuleType('mujoco_ros2_manipulation.operator_console.environment_sources')
        catalog.load_scene_catalog=loader;cell=NS(id='base')
        with patch.object(sys,'argv',['operator','--view-scene-catalog','/private/catalog.json']), \
                patch.dict(os.environ),patch.dict(sys.modules,{catalog.__name__:catalog}), \
                patch('mujoco_ros2_manipulation.workcells.config.load_workcell',return_value=cell), \
                patch('mujoco_ros2_manipulation.workcells.integrity.check_workcell',return_value={}), \
                patch('mujoco_ros2_manipulation.operator_console.http_server.ConsoleHTTPServer') as listen:
            with self.assertRaisesRegex(ValueError,'frozen overlay changed'):cli.main()
            listen.assert_not_called()
        loader.assert_called_once_with(Path('/private/catalog.json'),cell)


if __name__=='__main__':unittest.main()
