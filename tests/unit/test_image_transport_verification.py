"""Diagnostic producer continuity and retained HTTP refusal evidence."""
from copy import deepcopy
import io
import json
import threading
import time
import unittest
from unittest.mock import patch
from urllib.error import HTTPError

from mujoco_ros2_manipulation.operator_console import verify_image_transport as verification


class ImageTransportVerificationTests(unittest.TestCase):
    @staticmethod
    def live_state():
        return dict(lifecycle=dict(phase="running"),server=dict(session_id="a"*32),
            view=dict(status="ready",pose_ready=True,mode="live",binding_id="b"*64,
                binding=dict(mode="live",motion_session_id="a"*32,job_id=None)))

    def test_capture_readiness_requires_current_live_owner_even_when_pose_is_already_ready(self):
        state=self.live_state()
        catalog=dict(capture=dict(can_start=True))
        self.assertTrue(verification.capture_readiness(state,catalog)["ready"])
        variants=[]
        for server in (None,{},dict(session_id=""),dict(session_id=True),dict(session_id="c"*32)):
            value=deepcopy(state);value["server"]=server;variants.append(value)
        for binding in (None,{},dict(mode="live",motion_session_id=None,job_id=None),
                dict(mode="live",motion_session_id="c"*32,job_id=None),
                dict(mode="live",motion_session_id="a"*32,job_id="job"),
                dict(mode="model_preview",motion_session_id="a"*32,job_id=None)):
            value=deepcopy(state);value["view"]["binding"]=binding;variants.append(value)
        for field,bad in (("status","loading"),("pose_ready",False),("pose_ready",1),("mode","model_preview")):
            value=deepcopy(state);value["view"][field]=bad;variants.append(value)
        value=deepcopy(state);value["lifecycle"]["phase"]="starting";variants.append(value)
        for index,value in enumerate(variants):
            with self.subTest(index=index):
                self.assertFalse(verification.capture_readiness(value,catalog)["ready"])
        for unavailable in (None,{},dict(capture=None),dict(capture=dict(can_start=False)),
                dict(capture=dict(can_start=1))):
            with self.subTest(catalog=unavailable):
                self.assertFalse(verification.capture_readiness(state,unavailable)["ready"])

    def test_capture_waits_through_old_ready_binding_and_loading_then_sends_one_start(self):
        old=self.live_state();old["view"]["binding"]["motion_session_id"]=None
        loading=self.live_state();loading["view"].update(status="loading",pose_ready=False)
        snapshots=[old,loading,self.live_state(),self.live_state()]
        availability=[True,True,False,True]
        calls=[];poll=[-1];pumped=[];result={}
        def http(path,payload=None):
            calls.append((path,payload))
            if path=="/api/state":
                poll[0]+=1
                return snapshots[poll[0]]
            if path=="/api/sync-recordings":
                return dict(capture=dict(can_start=availability[poll[0]],
                    availability_reason=None if availability[poll[0]] else "waiting for current source"))
            self.assertEqual(path,"/api/sync-recordings/start")
            self.assertEqual(poll[0],3)
            return {"id":"capture"}
        def wait(predicate,label,seconds,*,frames):
            self.assertTrue(frames)
            for _ in range(1000):
                pumped.append("frame")
                value=predicate()
                if value:return value
                time.sleep(.001)
            raise TimeoutError(label)
        self.assertEqual(verification.start_synchronized_capture(http,wait,result),"capture")
        self.assertEqual([path for path,_ in calls].count("/api/sync-recordings/start"),1)
        self.assertEqual([path for path,_ in calls].count("/api/state"),4)
        evidence=result["synchronized_capture_readiness"]
        self.assertEqual(evidence["poll_count"],4)
        self.assertEqual(evidence["start_requests"],1)
        self.assertTrue(evidence["last_observed"]["owner_ready"])
        self.assertTrue(evidence["last_observed"]["can_start"])
        self.assertGreaterEqual(len(pumped),5)

    def test_capture_readiness_timeout_retains_last_reason_without_starting(self):
        calls=[];result={}
        def http(path,payload=None):
            calls.append(path)
            self.assertIsNone(payload)
            if path=="/api/state":return self.live_state()
            self.assertEqual(path,"/api/sync-recordings")
            return dict(capture=dict(can_start=False,availability_reason="source not ready"))
        def wait(predicate,label,seconds,*,frames):
            self.assertTrue(frames)
            for _ in range(3):self.assertFalse(predicate())
            raise TimeoutError(label)
        with self.assertRaises(TimeoutError):
            verification.start_synchronized_capture(http,wait,result)
        self.assertNotIn("/api/sync-recordings/start",calls)
        evidence=result["synchronized_capture_readiness"]
        self.assertEqual(evidence["poll_count"],3)
        self.assertEqual(evidence["start_requests"],0)
        self.assertEqual(evidence["last_observed"]["availability_reason"],"source not ready")

    def test_capture_start_refusal_after_readiness_is_retained_and_never_retried(self):
        failure=HTTPError("http://127.0.0.1/start",400,"Bad Request",{},None)
        calls=[];result={}
        def http(path,payload=None):
            calls.append(path)
            if path=="/api/state":return self.live_state()
            if path=="/api/sync-recordings":return dict(capture=dict(can_start=True))
            self.assertEqual(path,"/api/sync-recordings/start")
            raise failure
        def wait(predicate,label,seconds,*,frames):
            self.assertTrue(frames)
            deadline=time.monotonic()+2
            while time.monotonic()<deadline:
                value=predicate()
                if value:return value
                time.sleep(.001)
            raise TimeoutError(label)
        with self.assertRaises(HTTPError) as caught:
            verification.start_synchronized_capture(http,wait,result)
        self.assertIs(caught.exception,failure)
        self.assertEqual(calls.count("/api/sync-recordings/start"),1)
        self.assertEqual(result["synchronized_capture_readiness"]["start_requests"],1)

    def test_capture_readiness_diagnostics_are_bounded(self):
        state=self.live_state()
        state["server"]["session_id"]="s"*9000
        state["view"]["binding"]["motion_session_id"]="s"*9000
        state["view"]["binding_id"]="b"*9000
        observed=verification.capture_readiness(state,
            dict(capture=dict(can_start=False,availability_reason="r"*9000)))
        self.assertLess(len(json.dumps(observed)),2048)
        self.assertTrue(all(len(value)<=256 for value in observed.values() if isinstance(value,str)))

    def test_blocking_start_keeps_the_main_thread_pumping_and_sends_once(self):
        main_thread=threading.get_ident()
        published=[]
        calls=[]
        ready=threading.Event()
        def operation():
            calls.append(threading.get_ident())
            if not ready.wait(2):
                raise TimeoutError("Diagnostic producer stopped during HTTP preparation")
            return {"id":"capture"}
        def wait(predicate,label,seconds,*,frames):
            self.assertEqual(threading.get_ident(),main_thread)
            self.assertTrue(frames)
            deadline=time.monotonic()+2
            while time.monotonic()<deadline:
                published.append("fresh_frame")
                if len(published)>=2:ready.set()
                value=predicate()
                if value:return value
                time.sleep(.001)
            raise TimeoutError(label)
        result=verification.request_while_pumping(operation,wait,"capture")
        self.assertEqual(result,{"id":"capture"})
        self.assertGreaterEqual(len(published),2)
        self.assertEqual(len(calls),1)
        self.assertNotEqual(calls[0],main_thread)

    def test_failed_start_propagates_after_completion_without_retry(self):
        failure=HTTPError("http://127.0.0.1/start",400,"Bad Request",{},None)
        calls=[]
        def operation():
            calls.append(1)
            raise failure
        def wait(predicate,*_,**__):
            deadline=time.monotonic()+2
            while time.monotonic()<deadline:
                value=predicate()
                if value:return value
                time.sleep(.001)
            raise TimeoutError("request did not complete")
        with self.assertRaises(HTTPError) as caught:
            verification.request_while_pumping(operation,wait,"capture")
        self.assertIs(caught.exception,failure)
        self.assertEqual(calls,[1])

    def test_wait_timeout_drains_the_one_request_without_retry(self):
        release=threading.Event()
        finished=threading.Event()
        calls=[]
        def operation():
            calls.append(1)
            release.wait(2)
            finished.set()
        def wait(*_,**__):
            release.set()
            raise TimeoutError("capture start deadline")
        with self.assertRaisesRegex(TimeoutError,"capture start deadline"):
            verification.request_while_pumping(operation,wait,"capture")
        self.assertTrue(finished.is_set())
        self.assertEqual(calls,[1])

    def test_http_error_keeps_status_path_and_bounded_refusal_body(self):
        body=json.dumps({"error":"fresh RGB8 input required"}).encode()
        for raw in (body,b"x"*9000):
            with self.subTest(size=len(raw)):
                failure=HTTPError("http://127.0.0.1:8934/api/sync-recordings/start",
                    400,"Bad Request",{},io.BytesIO(raw))
                with patch.object(verification,"urlopen",side_effect=failure) as opener:
                    with self.assertRaises(HTTPError) as caught:
                        verification.http_request("http://127.0.0.1:8934","/api/sync-recordings/start",{},token="local")
                self.assertIs(caught.exception,failure)
                detail=failure.verification_detail
                self.assertEqual(detail["path"],"/api/sync-recordings/start")
                self.assertEqual(detail["status"],400)
                self.assertEqual(detail["response"],raw[:8192].decode())
                self.assertEqual(detail["response_truncated"],len(raw)>8192)
                self.assertEqual(opener.call_count,1)
                self.assertEqual(opener.call_args.kwargs["timeout"],3)


if __name__=="__main__":unittest.main()
