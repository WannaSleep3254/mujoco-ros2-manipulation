"""Diagnostic producer continuity and retained HTTP refusal evidence."""
import io
import json
import threading
import time
import unittest
from unittest.mock import patch
from urllib.error import HTTPError

from mujoco_ros2_manipulation.operator_console import verify_image_transport as verification


class ImageTransportVerificationTests(unittest.TestCase):
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
