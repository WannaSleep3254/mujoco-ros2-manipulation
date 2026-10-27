"""Actual console/HTTP image reads preserve observation and command boundaries."""
from copy import deepcopy
import json
from pathlib import Path
import queue
import tempfile
import threading
from types import SimpleNamespace as Namespace
import unittest
from urllib.error import HTTPError
from urllib.request import Request, urlopen

from mujoco_ros2_manipulation.operator_console.application import ConsoleApplication
from mujoco_ros2_manipulation.operator_console.http_server import ConsoleHTTPServer, Handler
from mujoco_ros2_manipulation.operator_console.image_source import ImageSource
from mujoco_ros2_manipulation.operator_console.state import ConsoleState


JPEG = b"\xff\xd8test-image-console\xff\xd9"


class ImageConsoleTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)
        reports = self.root/"docs"/"reports"
        reports.mkdir(parents=True)
        (reports/"catalog.json").write_text('{"reports":[]}')
        (reports/"site-manifest.json").write_text('{"files":{}}')
        self.now = [0.]
        config = dict(workcell="test_cell", home_rad=dict(left=0., right=0.),
            joint_limits={name:dict(lower=-1.,upper=1.) for name in ("left", "right")})
        self.state = ConsoleState(config, clock=lambda:self.now[0])
        self.now[0] = 3.
        self.state.graph.update(clock=1,joints=1,status=1,commands=1,guards=1,plans=1,approvals=1)
        self.state.identity = True
        self.state.guard = dict(latched=False,phase="ready")
        self.state.joints(["left", "right"],[0.,0.],[0.,0.],source_stamp=900.25)
        self.state.tick(901.)
        self.state.status(dict(session_id="session-a",request_id=None,status="ready"))
        self.bridge = Namespace(outbox=queue.Queue(maxsize=8),image_source=None)
        self.app = ConsoleApplication(Namespace(root=self.root),self.state,self.bridge,
                                      self.root/"console",render=False)
        self.addCleanup(self.close_application)
        self.server = ConsoleHTTPServer(("127.0.0.1",0),Handler)
        self.addCleanup(self.server.server_close)
        self.server.application,self.server.token = self.app,"image-test-token"
        self.thread = threading.Thread(target=lambda:self.server.serve_forever(poll_interval=.01),daemon=True)
        self.thread.start()
        self.addCleanup(self.stop_http)
        self.base = "http://127.0.0.1:"+str(self.server.server_address[1])

    def stop_http(self):
        self.server.shutdown()
        self.thread.join(2)

    def close_application(self):
        self.app.close()
        if self.bridge.image_source is not None:
            self.bridge.image_source.close()

    def get(self,path):
        with urlopen(self.base+path,timeout=3) as response:
            return response.status,response.headers,response.read()

    def json(self,path):
        return json.loads(self.get(path)[2])

    def image(self, *, clock_domain="unknown"):
        source = ImageSource(clock=lambda:self.now[0],encoder=lambda *args:JPEG,
                             clock_domain=clock_domain)
        source.set_publishers(["a"*64])
        self.bridge.image_source = source
        return source

    def receive(self, source, stamp=7_125_000_000):
        return source.receive(width=2,height=1,encoding="rgb8",step=6,data=b"\x01\x02\x03"*2,
            stamp_sec=stamp//1_000_000_000,stamp_nanosec=stamp%1_000_000_000,
            frame_id="camera_optical",generation=source.generation,is_bigendian=0)

    def frame_path(self,generation,sequence=1):
        return f"/api/image-source/frame?generation={generation}&sequence={sequence}"

    def assert_http_status(self,path,status):
        with self.assertRaises(HTTPError) as caught:
            self.get(path)
        self.assertEqual(caught.exception.code,status)
        self.assertEqual(caught.exception.headers.get("Cache-Control"),"no-store")
        return json.loads(caught.exception.read())

    def terminal(self):
        identifier = "b"*32
        self.state.status(dict(session_id="session-a",request_id=identifier,
            status="succeeded",stage="terminal",metrics={}))
        return identifier

    def test_sensor_source_is_opt_in_and_no_source_api_is_null(self):
        self.assertIsNone(self.json("/api/image-source"))
        self.assertIsNone(self.json("/api/state")["image_source"])
        self.assert_http_status(self.frame_path("a"*32),410)
        self.assertTrue(self.bridge.outbox.empty())

    def test_current_generation_exact_frame_returns_immutable_jpeg_with_no_store(self):
        source = self.image()
        self.assertTrue(self.receive(source))
        source.process_once()
        snapshot = self.json("/api/image-source")
        status,headers,body = self.get(self.frame_path(snapshot["generation"],snapshot["latest"]["sequence"]))
        self.assertEqual(status,200)
        self.assertEqual(headers.get_content_type(),"image/jpeg")
        self.assertEqual(headers.get("Content-Length"),str(len(JPEG)))
        self.assertEqual(headers.get("Cache-Control"),"no-store")
        self.assertEqual(headers.get("X-Content-Type-Options"),"nosniff")
        self.assertEqual(body,JPEG)
        self.assertEqual(snapshot["latest"]["jpeg_bytes"],len(body))
        self.assertEqual(self.json("/api/state")["image_source"]["generation"],snapshot["generation"])

    def test_epoch_reset_and_publisher_replacement_make_old_frame_gone(self):
        source = self.image()
        self.receive(source)
        source.process_once()
        old = source.generation
        source.new_epoch()
        self.assert_http_status(self.frame_path(old),410)
        self.assert_http_status(self.frame_path(source.generation),410)
        self.now[0] += .21
        self.receive(source,8_000_000_000)
        source.process_once()
        current = source.generation
        self.assertEqual(self.get(self.frame_path(current,2))[2],JPEG)
        source.set_publishers(["c"*64])
        self.assert_http_status(self.frame_path(current,2),410)
        snapshot = self.json("/api/image-source")
        self.assertIsNone(snapshot["latest"])
        self.assertIsNone(snapshot["latest_received"])
        self.assertTrue(all(not item["image_available"] for item in snapshot["history"]))

    def test_stale_frame_is_explicitly_retained_without_changing_command_admission(self):
        source = self.image()
        self.receive(source)
        source.process_once()
        self.assertTrue(self.json("/api/state")["can_submit"])
        self.now[0] += .6
        # Joint/clock control telemetry remains current independently of camera.
        self.state.joints(["left","right"],[0.,0.],[0.,0.],source_stamp=900.85)
        self.state.tick(901.6)
        self.state.status(dict(session_id="session-a",request_id=None,status="ready"))
        snapshot = self.json("/api/state")
        self.assertEqual(snapshot["image_source"]["status"],"stale")
        self.assertAlmostEqual(snapshot["image_source"]["age_sec"],.6)
        self.assertTrue(snapshot["can_submit"])
        self.assertEqual(self.get(self.frame_path(source.generation))[2],JPEG)
        self.assertTrue(self.bridge.outbox.empty())

    def test_repeated_or_backwards_stamp_cannot_be_refreshed_by_http_reads(self):
        source = self.image()
        self.receive(source)
        source.process_once()
        self.now[0] += .6
        self.assertFalse(self.receive(source))
        for _ in range(3):
            snapshot = self.json("/api/image-source")
            self.assertEqual(snapshot["status"],"stale")
            self.assertAlmostEqual(snapshot["latest_received"]["age_sec"],.6)
        old = source.generation
        self.assertFalse(self.receive(source,7_000_000_000))
        self.assertEqual(self.json("/api/image-source")["status"],"invalid")
        self.assert_http_status(self.frame_path(old),410)

    def test_invalid_query_shapes_are_rejected_without_latest_fallback(self):
        source = self.image()
        self.receive(source)
        source.process_once()
        generation = source.generation
        paths = ["/api/image-source/frame", "/api/image-source/frame?generation="+generation,
            self.frame_path(generation)+"&extra=1",self.frame_path(generation)+"&sequence=2",
            self.frame_path(generation)+"&generation="+generation,
            self.frame_path(generation,"0"),self.frame_path(generation,"01"),
            self.frame_path(generation,"-1"),self.frame_path(generation,"1.0"),
            self.frame_path(generation,"true"),self.frame_path(generation,"1e3"),
            self.frame_path(generation,"9"*17),self.frame_path("A"*32),
            self.frame_path("a"*31),self.frame_path("../private"),
            self.frame_path(generation,"")]
        for path in paths:
            with self.subTest(path=path): self.assert_http_status(path,400)
        self.assert_http_status(self.frame_path(generation,999),410)
        self.assert_http_status(self.frame_path("f"*32,1),410)
        self.assertEqual(source.snapshot()["counters"]["received"],1)
        self.assertTrue(self.bridge.outbox.empty())

    def test_analysis_uses_declared_image_clock_and_receiver_age_without_alignment(self):
        source = self.image(clock_domain="ros_simulation")
        self.receive(source)
        source.process_once()
        identifier = self.terminal()
        self.now[0] += .25
        analysis = self.json("/api/analysis/"+identifier)
        image = analysis["sources"]["image"]
        self.assertEqual(image["stamp_sec"],7.125)
        self.assertEqual(image["clock_domain"],"ros_simulation")
        self.assertAlmostEqual(image["age_sec"],.25)
        self.assertEqual(image["age_clock_domain"],"console_receiver_elapsed_monotonic")
        self.assertEqual(image["sequence"],1)
        self.assertEqual(image["freshness"],"fresh")
        self.assertFalse(analysis["timeline"]["cross_domain_durations_valid"])
        self.assertFalse(analysis["timeline"]["interpolation"])
        self.assertTrue(analysis["read_only"])
        self.assertFalse(analysis["command_authority"])
        self.assertNotIn("received_monotonic_sec",image)
        self.assertNotIn("topic",image)
        self.assertNotIn("history",image)

    def test_analysis_unknown_clock_and_unconnected_source_do_not_become_fresh(self):
        identifier = self.terminal()
        absent = self.json("/api/analysis/"+identifier)["sources"]["image"]
        self.assertFalse(absent["available"])
        source = self.image()
        waiting = self.json("/api/analysis/"+identifier)["sources"]["image"]
        self.assertIsNone(waiting["stamp_sec"])
        self.assertEqual(waiting["freshness"],"unknown")
        self.receive(source)
        source.process_once()
        fresh = self.json("/api/analysis/"+identifier)["sources"]["image"]
        self.assertEqual(fresh["clock_domain"],"unknown")
        self.assertEqual(fresh["status"],"ready")
        source.set_publishers(["a"*64,"b"*64])
        ambiguous = self.json("/api/analysis/"+identifier)["sources"]["image"]
        self.assertEqual(ambiguous["status"],"ambiguous")
        self.assertEqual(ambiguous["freshness"],"unknown")

    def test_all_image_and_analysis_gets_leave_queue_history_and_audit_unchanged(self):
        source = self.image()
        self.receive(source)
        source.process_once()
        identifier = self.terminal()
        self.bridge.outbox.put(("sentinel",dict(untouched=True)))
        history = deepcopy(self.state.history)
        ui_ids = self.state.ui_ids.copy()
        audit = (self.app.directory/"events.jsonl").read_bytes()
        before = source.snapshot()
        for path in ("/api/bootstrap","/api/state","/api/image-source",
                     self.frame_path(source.generation),"/api/analysis/"+identifier):
            self.get(path)
        self.assertEqual(list(self.bridge.outbox.queue),[("sentinel",dict(untouched=True))])
        self.assertEqual(self.state.history,history)
        self.assertEqual(self.state.ui_ids,ui_ids)
        self.assertEqual((self.app.directory/"events.jsonl").read_bytes(),audit)
        self.assertEqual(source.snapshot(),before)

    def test_console_observation_reset_invalidates_image_generation_and_ages(self):
        source = self.image()
        self.receive(source)
        source.process_once()
        old = source.generation
        self.app.reset_observation()
        snapshot = self.json("/api/state")
        self.assertNotEqual(snapshot["image_source"]["generation"],old)
        self.assertEqual(snapshot["image_source"]["status"],"waiting_frame")
        self.assertIsNone(snapshot["image_source"]["age_sec"])
        self.assertIsNone(snapshot["joint_state_stamp_simulation_sec"])
        self.assertIsNone(snapshot["simulation_sec"])
        self.assertFalse(snapshot["can_submit"])
        self.assert_http_status(self.frame_path(old),410)


if __name__ == "__main__": unittest.main()
