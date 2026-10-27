"""Console execution selection keeps base contracts and checked target identity."""
from copy import deepcopy
from dataclasses import dataclass
import json
from pathlib import Path
import queue
import tempfile
from types import SimpleNamespace as Namespace
import unittest
from unittest.mock import Mock, patch

from mujoco_ros2_manipulation.operator_console.application import ConsoleApplication
from mujoco_ros2_manipulation.operator_console.console_views import ConsoleViews
from mujoco_ros2_manipulation.operator_console.state import ConsoleState
from mujoco_ros2_manipulation.workcells.motion_request import command


@dataclass(frozen=True)
class CheckedTarget:
    output: Path
    manifest_sha256: str
    description_json: bytes
    validator: object

    @property
    def id(self): return json.loads(self.description_json)["environment_id"]
    @property
    def model_path(self): return self.output/"cell.xml"
    def description(self): return json.loads(self.description_json)
    def validate(self):
        self.validator()
        return self.description()


class QuietHost:
    """No process/model loading: only the actual view-owner registry is tested."""
    def __init__(self,*args,**kwargs): self.closed=False
    def snapshot(self): return dict(binding_id=None,binding=None,status="waiting")
    def close(self): self.closed=True


MISSING = object()


class ExecutionConsoleTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        self.names = ("fr5_joint1","ur5e_joint1")
        self.cell = Namespace(id="fr5_ur5e",label="FR5 + UR5e",root=self.root,
            output=self.root/"base",joints=self.names,home=(0.,0.),instances=(
                Namespace(id="fr5",prefix="fr5_",profile=Namespace(ros={"base_frame":"base_link"})),
                Namespace(id="ur5e",prefix="ur5e_",profile=Namespace(ros={"base_frame":"base"}))))
        self.target_probe = Mock()
        description = dict(schema_version=1,artifact_kind="environment-execution-target-v1",
            environment_id="test_installation",manifest_sha256="e"*64,
            base_workcell_id=self.cell.id,outputs_sha256={"cell.xml":"c"*64})
        self.target = CheckedTarget(self.root/"installation","e"*64,
            json.dumps(description,sort_keys=True).encode(),self.target_probe)
        self.apps=[]
        self.addCleanup(self.close_apps)

    def close_apps(self):
        for app in reversed(self.apps): app.close()

    def application(self, *, target=True):
        now = [0.]
        config = dict(workcell=self.cell.id,domain=207,home_rad=dict(zip(self.names,self.cell.home)),
            joint_limits={name:dict(lower=-1.,upper=1.) for name in self.names})
        if target: config["execution_target"] = self.target.description()
        state = ConsoleState(config,clock=lambda:now[0])
        now[0] = 3.
        bridge = Namespace(outbox=queue.Queue(maxsize=8))
        app = ConsoleApplication(self.cell,state,bridge,self.root/("console-"+str(len(self.apps))),
            render=False,execution_target=self.target if target else None)
        self.apps.append(app)
        self.ready(app)
        return app

    def ready(self, app, *, server_target=MISSING):
        state = app.state
        state.graph.update(clock=1,joints=1,status=1,commands=1,guards=1,plans=1,approvals=1)
        state.identity = True
        state.guard = dict(latched=False,phase="ready")
        state.joints(list(self.names),[0.,0.],[0.,0.],source_stamp=20.)
        state.tick(21.)
        record = dict(session_id="a"*32,request_id=None,status="ready")
        if server_target is MISSING:
            if app.execution_target is not None: record["execution_target"] = app.execution_target.description()
        elif server_target is not None:
            record["execution_target"] = deepcopy(server_target)
        state.status(record)

    def payload(self): return command(dict(zip(self.names,(.1,.1))))

    def test_exact_configured_target_and_server_heartbeat_identity_admit_one_request(self):
        app = self.application()
        self.assertTrue(app.state.snapshot()["can_submit"])
        payload = self.payload()
        self.assertEqual(app.submit(payload),dict(request_id=payload["request_id"]))
        self.assertEqual(list(app.bridge.outbox.queue),[("command",payload)])
        self.assertEqual(app.state.active,payload["request_id"])
        self.assertEqual(app.state.config["workcell"],self.cell.id)
        self.assertEqual(app.state.config["execution_target"],self.target.description())
        self.target_probe.assert_called_once_with()

    def test_missing_or_changed_server_target_blocks_reserve_and_outbox(self):
        app = self.application()
        changed_id = self.target.description();changed_id["environment_id"]="different"
        changed_hash = self.target.description();changed_hash["outputs_sha256"]["cell.xml"]="f"*64
        changed_manifest = self.target.description();changed_manifest["manifest_sha256"]="0"*64
        unexpected = self.target.description();unexpected["unreviewed"]="extra"
        for description in (None,changed_id,changed_hash,changed_manifest,unexpected):
            self.ready(app,server_target=description)
            with self.subTest(description=description):
                self.assertFalse(app.state.snapshot()["can_submit"])
                self.assertIn("설치 환경",app.state.reason())
                with self.assertRaisesRegex(RuntimeError,"설치 환경"): app.state.reserve(self.payload())
                with self.assertRaisesRegex(RuntimeError,"설치 환경"): app.submit(self.payload())
                self.assertTrue(app.bridge.outbox.empty())
                self.assertIsNone(app.state.active)
                self.assertEqual(len(app.state.history),0)
        self.ready(app)
        self.assertTrue(app.state.snapshot()["can_submit"])

    def test_changed_checked_target_is_rejected_before_reservation_or_queueing(self):
        app = self.application()
        self.target_probe.side_effect = ValueError("Selected installation changed")
        for submit in (app.submit,app.create_plan):
            with self.subTest(submit=submit.__name__),self.assertRaisesRegex(ValueError,"installation changed"):
                submit(self.payload())
            self.assertTrue(app.bridge.outbox.empty())
            self.assertEqual(len(app.state.history),0)
            self.assertIsNone(app.state.active)

    def test_changed_checked_target_cannot_start_a_process(self):
        app = self.application()
        app.state.graph.update(clock=0,joints=0,status=0,guards=0)
        self.target_probe.side_effect = ValueError("Installation output changed")
        with patch.object(app,"spawn") as spawn,patch.object(app,"start_worker") as worker:
            with self.assertRaisesRegex(ValueError,"output changed"):
                app.start(dict(native_gui=False,planning_case="none"))
            spawn.assert_not_called();worker.assert_not_called()
        self.assertEqual(app.phase,"idle")
        self.assertFalse(app.children)
        self.assertTrue(app.bridge.outbox.empty())
        self.assertEqual(list(app.directory.glob("launch-*")),[])

    def start_without_processes(self, app, *, attach=False):
        calls=[]
        def spawn(kind,args,folder,**environment):
            calls.append((kind,list(args),folder,dict(environment)))
            if kind=="commands": self.ready(app)
            return Namespace(poll=lambda:None,returncode=0)
        if attach:
            app.state.graph["status"] = 0
        else:
            app.state.graph.update(clock=0,joints=0,status=0,guards=0)
        with patch.object(app,"spawn",side_effect=spawn):
            app.start(dict(native_gui=False,planning_case="none"),attach=attach)
            app.worker.join(2)
        self.assertFalse(app.worker.is_alive(),"Pure startup fixture did not finish")
        self.assertEqual(app.phase,"running",app.error)
        return calls

    def test_start_passes_exact_target_cli_pair_to_both_owners_and_keeps_base_id(self):
        app = self.application()
        calls = self.start_without_processes(app)
        self.assertEqual([value[0] for value in calls],["simulation","commands"])
        simulation,commands = calls[0][1],calls[1][1]
        self.assertIn("workcell:="+self.cell.id,simulation)
        self.assertEqual(simulation[-2:],["execution_environment:="+str(self.target.output),
            "execution_manifest_sha256:="+self.target.manifest_sha256])
        self.assertEqual(commands[commands.index("--workcell")+1],self.cell.id)
        self.assertEqual(commands[-4:],["--execution-environment",str(self.target.output),
            "--execution-manifest-sha256",self.target.manifest_sha256])
        self.assertEqual(self.cell.id,"fr5_ur5e")
        self.assertEqual(app.state.config["workcell"],"fr5_ur5e")
        self.assertEqual(app.snapshot()["execution_target"],self.target.description())
        self.assertTrue(app.bridge.outbox.empty())

    def test_attach_launches_only_command_owner_with_the_selected_target_pair(self):
        app = self.application()
        calls = self.start_without_processes(app,attach=True)
        self.assertEqual([value[0] for value in calls],["commands"])
        self.assertEqual(calls[0][1][-4:],["--execution-environment",str(self.target.output),
            "--execution-manifest-sha256",self.target.manifest_sha256])
        self.assertEqual(app.state.config["execution_target"],self.target.description())

    def test_selected_installation_refuses_base_demo_and_server_jpeg_before_side_effects(self):
        app = self.application()
        with patch.object(app,"spawn") as spawn:
            with self.assertRaisesRegex(RuntimeError,"설치 환경"):
                app.set_display("image")
            for scenario in ("contact_comparison","timing_comparison"):
                with self.subTest(scenario=scenario),self.assertRaisesRegex(RuntimeError,"설치 환경"):
                    app.demos.start(dict(scenario=scenario,native_gui=False))
            spawn.assert_not_called()
        self.assertEqual(app.display_mode,"web3d")
        self.assertIsNone(app.demos.current)
        self.assertFalse((app.directory/"demos").exists())
        self.assertTrue(app.bridge.outbox.empty())
        self.assertEqual(len(app.state.history),0)

    def test_target_scene_registry_uses_exact_model_joints_home_and_robot_roots(self):
        app = self.application()
        metadata = dict(workcell_sha256="d"*64)
        with patch("mujoco_ros2_manipulation.operator_console.console_views.check_workcell",return_value=metadata), \
             patch("mujoco_ros2_manipulation.operator_console.console_views.load_profile",side_effect=ValueError("No fixture profile")):
            views = ConsoleViews(app,enabled=True,host_factory=QuietHost)
        self.addCleanup(views.close)
        source = views.base
        self.assertEqual(source.id,"scene:"+self.target.id)
        self.assertEqual(source.xml_path,self.target.model_path)
        self.assertEqual(source.joint_names,self.names)
        self.assertEqual(source.home,dict(zip(self.names,self.cell.home)))
        self.assertEqual(source.robot_roots,(("fr5","fr5_base_link"),("ur5e","ur5e_base")))
        self.assertIs(views.registry[source.id],source)
        self.assertNotIn("workcell:"+self.cell.id,views.registry)
        self.assertEqual(source.validate(),self.target.description())
        self.target_probe.side_effect = ValueError("Selected model changed")
        with self.assertRaisesRegex(ValueError,"model changed"): source.validate()
        self.assertEqual(app.state.config["workcell"],self.cell.id)
        self.assertTrue(app.bridge.outbox.empty())

    def test_legacy_start_and_commands_have_no_target_requirements_or_cli_flags(self):
        app = self.application(target=False)
        self.ready(app,server_target=dict(unrelated="ignored by legacy contract"))
        self.assertTrue(app.state.snapshot()["can_submit"])
        calls = self.start_without_processes(app)
        for _,args,_,_ in calls:
            self.assertFalse(any("execution-environment" in arg or "execution_environment:=" in arg
                                 or "execution-manifest-sha256" in arg or "execution_manifest_sha256:=" in arg
                                 for arg in args))
        payload = self.payload()
        app.submit(payload)
        self.assertEqual(list(app.bridge.outbox.queue),[("command",payload)])
        self.assertIsNone(app.snapshot()["execution_target"])
        source = app.views._cell_source()
        self.assertEqual(source.id,"workcell:"+self.cell.id)
        self.assertEqual(source.xml_path,self.cell.output/"cell.xml")
        self.target_probe.assert_not_called()


if __name__ == "__main__": unittest.main()
