"""RGB acquisition is bounded, passive, and honest about receiver/source clocks."""
from array import array
import hashlib
import io
import json
import subprocess
import sys
import threading
import time
from types import SimpleNamespace as Namespace
import unittest
from unittest.mock import patch

from mujoco_ros2_manipulation.operator_console.image_source import (
    ImageSource, MAX_CACHE_BYTES, MAX_FRAMES, MAX_HISTORY, MAX_JPEG_BYTES, MAX_RAW_BYTES)
from mujoco_ros2_manipulation.operator_console.sensor_ros import SensorSubscriptions, _endpoint_id


JPEG = b"\xff\xd8test\xff\xd9"
ENDPOINT = "a" * 64


class Clock:
    def __init__(self): self.now = 100.
    def __call__(self): return self.now
    def advance(self, seconds=.21): self.now += seconds


class ImageSourceTests(unittest.TestCase):
    def source(self, **kwargs):
        clock = kwargs.pop("clock", Clock())
        source = ImageSource(clock=clock, encoder=kwargs.pop("encoder", lambda *args: JPEG), **kwargs)
        self.addCleanup(source.close)
        source.set_publishers([ENDPOINT])
        return source, clock

    def receive(self, source, stamp=1_000_000_000, **kwargs):
        value = dict(width=2, height=1, encoding="rgb8", step=6,
            data=b"\xff\x00\x00\x00\xff\x00", stamp_sec=stamp//1_000_000_000,
            stamp_nanosec=stamp%1_000_000_000, frame_id="camera_optical",
            generation=source.generation, is_bigendian=0)
        value.update(kwargs)
        return source.receive(**value)

    def test_ros_epoch_stamp_and_exact_nanoseconds_are_not_limited_to_simulation_age(self):
        source, _ = self.source()
        stamp = 1_793_100_000_123_456_789
        self.assertTrue(self.receive(source, stamp=stamp))
        source.process_once()
        metadata = source.snapshot()["latest"]
        self.assertEqual(metadata["source_stamp_ns"], str(stamp))
        self.assertEqual(metadata["clock_domain"], "unknown")
        source.new_epoch()
        self.assertTrue(self.receive(source, stamp=(2**31-1)*10**9+999_999_999))
        source.new_epoch()
        self.assertFalse(self.receive(source, stamp=(2**31)*10**9))

    def test_real_pillow_encoder_accepts_bounded_padded_rgb8(self):
        from PIL import Image
        source, _ = self.source(encoder=None, clock_domain="ros_simulation")
        self.assertTrue(self.receive(source, width=8, height=2, step=27,
            data=(b"\xff\x00\x00"*8+b"\x00\x00\x00")*2))
        self.assertTrue(source.process_once())
        snapshot = source.snapshot()
        metadata, jpeg = source.frame(snapshot["generation"], snapshot["latest"]["sequence"])
        self.assertEqual(metadata["clock_domain"], "ros_simulation")
        self.assertEqual(metadata["stamp_sec"], 1.)
        self.assertEqual(metadata["age_sec"], 0.)
        self.assertEqual(metadata["source_stamp_ns"], "1000000000")
        self.assertEqual(metadata["jpeg_sha256"], hashlib.sha256(jpeg).hexdigest())
        with Image.open(io.BytesIO(jpeg)) as image:
            self.assertEqual(image.size, (8, 2))
            self.assertGreater(image.getpixel((0, 0))[0], 245)
        self.assertEqual(snapshot["status"], "ready")
        self.assertIsNone(metadata["calibration"])
        self.assertIsNone(metadata["transform_to_world"])

    def test_source_clock_is_unknown_without_explicit_declaration(self):
        source, _ = self.source()
        source.observe_clock(50.)
        self.assertTrue(self.receive(source))
        self.assertTrue(source.process_once())
        self.assertEqual(source.snapshot()["latest"]["clock_domain"], "unknown")
        self.assertEqual(source.snapshot()["clock_domain"], "unknown")

    def test_invalid_messages_count_and_do_not_replace_last_good_frame(self):
        source, _ = self.source()
        self.assertTrue(self.receive(source))
        source.process_once()
        cases = [dict(encoding="bgr8"), dict(width=641), dict(width=True), dict(height=481),
                 dict(step=5), dict(step=1921), dict(data=b"bad"), dict(data=[0]*6),
                 dict(is_bigendian=True), dict(stamp_nanosec=1_000_000_000),
                 dict(stamp_sec=-1), dict(frame_id="other_frame"),
                 dict(frame_id="camera_"+"x"*128),
                 dict(data=memoryview(array("I", [1]*6))),
                 dict(data=memoryview(bytearray(12))[::2])]
        for invalid in cases:
            with self.subTest(invalid=invalid):
                self.assertFalse(self.receive(source, stamp=2_000_000_000, **invalid))
        result = source.snapshot()
        self.assertEqual(result["counters"]["received"], 1)
        self.assertEqual(result["counters"]["rejected"], len(cases))
        self.assertEqual(result["latest"]["sequence"], 1)
        self.assertEqual(result["latest_received"]["source_stamp_sec"], 1.)
        self.assertEqual(result["memory"]["pending_raw_bytes"], 0)
        json.dumps(result, allow_nan=False)

    def test_ros_attributes_and_array_unsigned_byte_data(self):
        source, _ = self.source()
        message = Namespace(width=1, height=1, encoding="rgb8", step=3,
            data=array("B", [1, 2, 3]), is_bigendian=0,
            header=Namespace(stamp=Namespace(sec=2, nanosec=4), frame_id="camera_optical"))
        self.assertTrue(source.receive_message(message, generation=source.generation))
        self.assertFalse(source.receive_message(Namespace(), generation=source.generation))
        source.process_once()
        self.assertEqual(source.snapshot()["latest"]["source_stamp_ns"], "2000000004")
        self.assertEqual(source.snapshot()["invalid_reasons"]["invalid_message"], 1)

    def test_duplicate_stamp_does_not_refresh_age_or_sequence(self):
        source, clock = self.source()
        self.receive(source)
        source.process_once()
        clock.advance(.6)
        self.assertFalse(self.receive(source))
        result = source.snapshot()
        self.assertEqual(result["status"], "stale")
        self.assertAlmostEqual(result["latest_received"]["age_sec"], .6)
        self.assertEqual(result["latest_received"]["sequence"], 1)
        self.assertEqual(result["sequence"], 1)
        self.assertEqual(result["stamp_sec"], 1.)
        self.assertAlmostEqual(result["age_sec"], .6)
        self.assertEqual(result["age_clock_domain"], "console_receiver_elapsed_monotonic")
        self.assertEqual(result["invalid_reasons"]["duplicate_stamp"], 1)

    def test_image_clock_reversal_latches_until_explicit_new_epoch(self):
        source, _ = self.source()
        self.receive(source, stamp=2_000_000_000)
        source.process_once()
        old = source.generation
        self.assertFalse(self.receive(source, stamp=1_000_000_000))
        self.assertNotEqual(old, source.generation)
        self.assertEqual(source.snapshot()["status"], "invalid")
        self.assertIsNone(source.frame(old, 1))
        self.assertFalse(self.receive(source, stamp=3_000_000_000))
        source.set_publishers([ENDPOINT])
        self.assertEqual(source.snapshot()["status"], "invalid")
        source.new_epoch()
        self.assertTrue(self.receive(source, stamp=0))
        self.assertEqual(source.snapshot()["latest_received"]["sequence"], 2)

    def test_observed_clock_reversal_and_publisher_epoch(self):
        source, _ = self.source()
        self.assertTrue(source.observe_clock(10.))
        self.receive(source)
        self.assertFalse(source.observe_clock(9.))
        self.assertEqual(source.snapshot()["error"], "ros_clock_backwards")
        self.assertFalse(source.observe_clock(12.))
        self.assertFalse(self.receive(source, stamp=2_000_000_000))
        source.set_publishers(["b"*64])
        self.assertFalse(source.snapshot()["clock_invalid"])
        self.assertTrue(self.receive(source, stamp=0))

    def test_integer_clock_retains_epoch_precision_and_detects_one_ns_reversal(self):
        source, _ = self.source()
        self.assertTrue(source.observe_clock_stamp(1_793_100_000,123_456_789))
        self.assertEqual(source.snapshot()["observed_clock_ns"],"1793100000123456789")
        generation=source.generation
        self.assertFalse(source.observe_clock_stamp(1_793_100_000,123_456_788))
        self.assertNotEqual(source.generation,generation)
        self.assertTrue(source.snapshot()["clock_invalid"])
        self.assertEqual(source.snapshot()["error"],"ros_clock_backwards")
        self.assertFalse(source.observe_clock_stamp(1_793_100_000,123_456_790))
        source.new_epoch()
        self.assertTrue(source.observe_clock_stamp(2**31-1,999_999_999))
        self.assertEqual(source.snapshot()["observed_clock_ns"],"2147483647999999999")

    def test_invalid_integer_clock_preserves_last_observation(self):
        source, _ = self.source()
        self.assertTrue(source.observe_clock_stamp(0,0))
        for seconds,nanoseconds in ((-1,0),(2**31,0),(True,0),(0,True),
                                    (0,-1),(0,10**9),(1.,0),(0,1.),(None,None)):
            with self.subTest(seconds=seconds,nanoseconds=nanoseconds):
                self.assertFalse(source.observe_clock_stamp(seconds,nanoseconds))
        self.assertEqual(source.snapshot()["observed_clock_ns"],"0")
        self.assertFalse(source.snapshot()["clock_invalid"])

    def test_monotonic_receipt_reversal_cannot_create_fresh_frame(self):
        source, clock = self.source()
        self.receive(source)
        clock.now = 99.
        self.assertFalse(self.receive(source, stamp=2_000_000_000))
        self.assertEqual(source.snapshot()["status"], "invalid")
        self.assertEqual(source.snapshot()["error"], "receipt_clock_backwards")

    def test_publisher_absence_ambiguity_replacement_and_generation_binding(self):
        source = ImageSource(encoder=lambda *args: JPEG)
        self.addCleanup(source.close)
        self.assertEqual(source.snapshot()["status"], "waiting_source")
        self.assertFalse(self.receive(source))
        source.set_publishers([ENDPOINT, "b"*64])
        self.assertEqual(source.snapshot()["status"], "ambiguous")
        self.assertFalse(self.receive(source))
        source.set_publishers([ENDPOINT])
        old = source.generation
        self.assertTrue(self.receive(source))
        source.process_once()
        source.set_publishers(["b"*64])
        self.assertIsNone(source.frame(old, 1))
        self.assertIsNone(source.snapshot()["latest"])
        self.assertIsNone(source.snapshot()["latest_received"])
        self.assertFalse(source.snapshot()["history"][0]["image_available"])
        self.assertFalse(self.receive(source, stamp=2_000_000_000, generation=old))
        self.assertEqual(source.snapshot()["invalid_reasons"]["old_generation"], 1)
        source.set_publishers([])
        self.assertEqual(source.snapshot()["status"], "disconnected")

    def test_over_cap_catalog_is_explicit_lower_bound_and_can_disconnect(self):
        source, _ = self.source()
        source.discovery_ambiguous()
        generation = source.generation
        source.discovery_ambiguous()
        result = source.snapshot()
        self.assertEqual(result["status"], "ambiguous")
        self.assertEqual(result["generation"], generation)
        self.assertEqual(result["publisher_count"], 9)
        self.assertTrue(result["publisher_count_is_lower_bound"])
        self.assertIsNone(result["publisher_endpoint_sha256"])
        source.set_publishers([])
        self.assertNotEqual(source.generation, generation)
        self.assertEqual(source.snapshot()["status"], "disconnected")

    def test_latest_only_input_overwrite_and_rate_limit(self):
        calls = []
        source, clock = self.source(encoder=lambda *args: calls.append(args) or JPEG)
        for stamp in range(1, 21):
            self.assertTrue(self.receive(source, stamp=stamp, data=bytes([stamp])*6))
        self.assertEqual(source.snapshot()["memory"]["pending_raw_bytes"], 6)
        self.assertTrue(source.process_once())
        self.assertEqual(calls[0][-1], bytes([20])*6)
        self.assertEqual(source.snapshot()["counters"]["raw_overwrites"], 19)
        self.assertEqual(source.snapshot()["history"][0]["processing"], "overwritten_before_encoding")
        self.receive(source, stamp=21)
        self.assertFalse(source.process_once())
        clock.advance(.199)
        self.assertFalse(source.process_once())
        clock.advance(.002)
        self.assertTrue(source.process_once())
        self.assertEqual(len(calls), 2)

    def test_encoded_frame_and_metadata_count_limits(self):
        source, clock = self.source()
        for stamp in range(MAX_HISTORY+14):
            self.assertTrue(self.receive(source, stamp=stamp))
            self.assertTrue(source.process_once())
            clock.advance()
        result = source.snapshot()
        self.assertEqual(result["memory"]["encoded_frames"], MAX_FRAMES)
        self.assertEqual(result["memory"]["metadata"], MAX_HISTORY)
        self.assertEqual(result["counters"]["frame_evictions"], MAX_HISTORY+14-MAX_FRAMES)
        self.assertEqual(result["counters"]["metadata_evictions"], 14)
        self.assertIsNone(source.frame(source.generation, 15))
        self.assertFalse(result["history"][0]["image_available"])
        self.assertTrue(result["history"][-1]["image_available"])

    def test_encoded_byte_budget_is_independent_of_frame_count(self):
        jpeg = b"\xff\xd8"+b"x"*(MAX_JPEG_BYTES-4)+b"\xff\xd9"
        source, clock = self.source(encoder=lambda *args: jpeg)
        for stamp in range(20):
            self.receive(source, stamp=stamp)
            source.process_once()
            clock.advance()
        result = source.snapshot()
        self.assertEqual(result["memory"]["encoded_bytes"], MAX_CACHE_BYTES)
        self.assertEqual(result["memory"]["encoded_frames"], 16)
        self.assertIsNone(source.frame(source.generation, 4))
        self.assertIsNotNone(source.frame(source.generation, 5))

    def test_maximum_raw_sample_is_bounded_in_pending_and_worker(self):
        source, _ = self.source()
        self.assertTrue(self.receive(source, width=640, height=480, step=1920,
            data=b"\x00"*MAX_RAW_BYTES))
        self.assertEqual(source.snapshot()["memory"]["pending_raw_bytes"], MAX_RAW_BYTES)
        self.assertFalse(self.receive(source, stamp=2, width=640, height=480, step=1920,
            data=b"\x00"*(MAX_RAW_BYTES+1)))
        source.process_once()
        self.assertEqual(source.snapshot()["memory"]["pending_raw_bytes"], 0)

    def test_codec_failure_is_fixed_bounded_error_and_can_recover(self):
        def failed(*args): raise RuntimeError("secret host path /home/private/source")
        source, clock = self.source(encoder=failed)
        self.receive(source)
        source.process_once()
        result = source.snapshot()
        self.assertEqual(result["status"], "error")
        self.assertEqual(result["error"], "jpeg_encode_failed")
        self.assertNotIn("private", json.dumps(result))
        self.assertEqual(result["history"][0]["processing"], "codec_failed")
        source.encoder = lambda *args: JPEG
        clock.advance()
        self.receive(source, stamp=2_000_000_000)
        source.process_once()
        self.assertEqual(source.snapshot()["status"], "ready")

    def test_codec_output_requires_bounded_immutable_jpeg_markers(self):
        for invalid in (b"not jpeg", bytearray(JPEG), b"\xff\xd8"+b"x"*MAX_JPEG_BYTES+b"\xff\xd9"):
            with self.subTest(size=len(invalid)):
                source, _ = self.source(encoder=lambda *args: invalid)
                self.receive(source)
                source.process_once()
                self.assertEqual(source.snapshot()["counters"]["codec_errors"], 1)
                self.assertIsNone(source.snapshot()["latest"])

    def test_slow_encoder_cannot_publish_obsolete_generation(self):
        entered, release = threading.Event(), threading.Event()
        def slow(*args):
            entered.set()
            if not release.wait(2): raise RuntimeError("Test worker release timed out")
            return JPEG
        source, clock = self.source(encoder=slow)
        self.receive(source)
        old = source.generation
        worker = threading.Thread(target=source.process_once)
        worker.start()
        self.addCleanup(lambda: (release.set(), worker.join(2)))
        self.assertTrue(entered.wait(1))
        self.assertEqual(source.snapshot()["memory"]["inflight_raw_bytes"], 6)
        source.set_publishers(["b"*64])
        self.receive(source, stamp=2_000_000_000)
        release.set()
        worker.join(1)
        self.assertFalse(worker.is_alive())
        self.assertIsNone(source.frame(old, 1))
        self.assertIsNone(source.snapshot()["latest"])
        self.assertEqual(source.snapshot()["counters"]["obsolete_encodes"], 1)
        self.assertEqual(source.snapshot()["memory"]["pending_raw_bytes"], 6)
        clock.advance()
        self.assertTrue(source.process_once())
        self.assertEqual(source.snapshot()["latest"]["sequence"], 2)

    def test_close_invalidates_pending_cached_and_late_worker_result(self):
        entered, release = threading.Event(), threading.Event()
        def slow(*args):
            entered.set()
            if not release.wait(2): raise RuntimeError("Test worker release timed out")
            return JPEG
        source, _ = self.source(encoder=slow)
        self.receive(source)
        worker = threading.Thread(target=source.process_once)
        worker.start()
        self.addCleanup(lambda: (release.set(), worker.join(2)))
        self.assertTrue(entered.wait(1))
        self.assertTrue(source.close())
        release.set()
        worker.join(1)
        self.assertEqual(source.snapshot()["status"], "closed")
        self.assertIsNone(source.snapshot()["latest"])
        self.assertEqual(source.snapshot()["memory"]["encoded_bytes"], 0)
        self.assertEqual(source.snapshot()["counters"]["obsolete_encodes"], 1)
        self.assertFalse(self.receive(source))

    def test_owned_worker_runs_codec_outside_receiver_lock_and_closes(self):
        entered, release, finished = threading.Event(), threading.Event(), threading.Event()
        def slow(*args):
            entered.set()
            if not release.wait(2): raise RuntimeError("Test worker release timed out")
            finished.set()
            return JPEG
        source, _ = self.source(encoder=slow, clock=time.monotonic)
        self.addCleanup(release.set)
        source.start()
        source.start()
        self.receive(source)
        self.assertTrue(entered.wait(1))
        # These acquire the store lock while the codec is deliberately blocked.
        # New acquisition and metadata polling must remain responsive.
        self.assertTrue(self.receive(source, stamp=2_000_000_000))
        self.assertTrue(source.snapshot()["worker_alive"])
        self.assertEqual(source.snapshot()["memory"]["pending_raw_bytes"], 6)
        release.set()
        self.assertTrue(finished.wait(1))
        self.assertTrue(source.close())
        self.assertFalse(source.snapshot()["worker_alive"])
        self.assertIsNone(source.snapshot()["latest"])

    def test_frame_lookup_is_exact_and_metadata_is_detached(self):
        source, _ = self.source()
        self.receive(source)
        source.process_once()
        generation = source.generation
        first, jpeg = source.frame(generation, 1)
        first["source_id"] = "mutated"
        snapshot = source.snapshot()
        snapshot["history"][0]["source_id"] = "mutated"
        second, again = source.frame(generation, 1)
        self.assertEqual(second["source_id"], "camera0")
        self.assertIs(jpeg, again)
        for bad_generation, sequence in (("f"*32, 1), (True, 1), (generation, True), (generation, 999)):
            self.assertIsNone(source.frame(bad_generation, sequence))
        self.assertRegex(generation, r"^[0-9a-f]{32}$")

    def test_constructor_and_discovery_validation(self):
        for kwargs in (dict(source_id="../bad"), dict(topic="relative"), dict(topic="/camera//image"),
                       dict(frame_id="/absolute"), dict(frame_id="x"*129),
                       dict(stale_after_sec=float("nan")),
                       dict(frame_stale_after_sec=0), dict(clock_domain="ros_wall")):
            with self.subTest(kwargs=kwargs), self.assertRaises(ValueError):
                ImageSource(**kwargs)
        source, _ = self.source()
        for invalid in (["a"], [ENDPOINT]*2, ["a"*64]*9, "a"*64):
            with self.assertRaises(ValueError): source.set_publishers(invalid)

    def test_topic_requires_bounded_absolute_ros_tokens_before_runtime_creation(self):
        for topic in ("/", "/camera/", "/camera//image", "/camera/123", "/123/image",
                      "/camera/1image", "camera/image", "~/image", "/camera/~image",
                      "/{camera}/image", "/camera/${image}", "/camera/이미지", "/"+"a"*256):
            with self.subTest(topic=topic),self.assertRaisesRegex(ValueError,"invalid_topic"):
                ImageSource(topic=topic)
        for topic in ("/camera/image", "/camera0/image_raw", "/_camera/_image0", "/"+"a"*255):
            with self.subTest(topic=topic):
                source=ImageSource(topic=topic)
                self.addCleanup(source.close)
                self.assertEqual(source.snapshot()["topic"],topic)
                self.assertFalse(source.snapshot()["worker_alive"])

    def test_import_does_not_load_optional_ros_or_codec_dependencies(self):
        code = """import sys
from mujoco_ros2_manipulation.operator_console.image_source import ImageSource
from mujoco_ros2_manipulation.operator_console.sensor_ros import SensorSubscriptions
assert not any(name.split('.')[0] in ('rclpy','sensor_msgs','rosgraph_msgs','PIL','numpy','mujoco') for name in sys.modules)
"""
        result = subprocess.run([sys.executable, "-c", code], check=False, capture_output=True, text=True)
        self.assertEqual(result.returncode, 0, result.stderr)


class FakeNode:
    def __init__(self):
        self.publishers = []
        self.created, self.destroyed = [], []
        self.fail_destroy = False
    def get_publishers_info_by_topic(self, topic): return self.publishers
    def create_subscription(self, message_type, topic, callback, qos):
        subscription = Namespace(topic=topic, callback=callback)
        self.created.append(subscription)
        return subscription
    def destroy_subscription(self, subscription):
        self.destroyed.append(subscription)
        return not self.fail_destroy


class SensorSubscriptionTests(unittest.TestCase):
    def helper(self):
        source = ImageSource(encoder=lambda *args: JPEG)
        node = FakeNode()
        with patch.object(source, "start") as start:
            helper = SensorSubscriptions(node, source, object, object, object())
            start.assert_called_once_with()
        self.addCleanup(helper.close)
        return helper, source, node

    def endpoint(self, byte=1): return Namespace(endpoint_gid=array("B", [byte]*16))

    def message(self):
        return Namespace(width=1,height=1,encoding="rgb8",step=3,data=array("B",[1,2,3]),
            is_bigendian=0,header=Namespace(stamp=Namespace(sec=1,nanosec=0),frame_id="camera_optical"))

    def test_one_argument_callback_and_endpoint_replacement_reject_old_callbacks(self):
        helper, source, node = self.helper()
        helper.discover()
        self.assertEqual([s.topic for s in node.created], ["/clock"])
        node.publishers = [self.endpoint()]
        helper.discover()
        old = helper.subscription
        old.callback(self.message())
        self.assertEqual(source.snapshot()["counters"]["received"], 1)
        helper.discover()
        self.assertIs(helper.subscription, old)
        node.publishers = [self.endpoint(2)]
        helper.discover()
        self.assertIn(old, node.destroyed)
        self.assertIsNot(helper.subscription, old)
        old.callback(self.message())
        self.assertEqual(source.snapshot()["invalid_reasons"]["old_generation"], 1)
        helper.subscription.callback(self.message())
        self.assertEqual(source.snapshot()["counters"]["received"], 2)
        node.publishers = [self.endpoint(2), self.endpoint(3)]
        helper.discover()
        self.assertIsNone(helper.subscription)
        self.assertEqual(source.snapshot()["status"], "ambiguous")

    def test_clock_reversal_rebinds_subscription_without_releasing_invalid_latch(self):
        helper, source, node = self.helper()
        node.publishers = [self.endpoint()]
        helper.discover()
        old = helper.subscription
        helper._clock(Namespace(clock=Namespace(sec=5,nanosec=0)))
        helper._clock(Namespace(clock=Namespace(sec=4,nanosec=0)))
        helper.discover()
        self.assertIsNot(helper.subscription, old)
        self.assertEqual(source.snapshot()["status"], "invalid")
        helper.subscription.callback(self.message())
        self.assertEqual(source.snapshot()["counters"]["received"], 0)
        source.new_epoch()
        helper.discover()
        helper.subscription.callback(self.message())
        self.assertEqual(source.snapshot()["counters"]["received"], 1)

    def test_invalid_clock_components_are_dropped_without_normalizing(self):
        helper, source, _ = self.helper()
        for clock in (Namespace(sec=0,nanosec=1_000_000_000), Namespace(sec=True,nanosec=0),
                      Namespace(sec=1.,nanosec=0), Namespace(sec=-1,nanosec=0)):
            helper._clock(Namespace(clock=clock))
        self.assertEqual(source.snapshot()["invalid_reasons"]["invalid_clock"], 4)
        self.assertIsNone(source.snapshot()["observed_clock_ns"])

    def test_ros_clock_callback_keeps_integer_components_at_epoch_scale(self):
        helper,source,_ = self.helper()
        helper._clock(Namespace(clock=Namespace(sec=1_793_100_000,nanosec=123_456_789)))
        self.assertEqual(source.snapshot()["observed_clock_ns"],"1793100000123456789")
        helper._clock(Namespace(clock=Namespace(sec=1_793_100_000,nanosec=123_456_788)))
        self.assertTrue(source.snapshot()["clock_invalid"])
        self.assertEqual(source.snapshot()["error"],"ros_clock_backwards")

    def test_over_cap_discovery_does_not_copy_unbounded_endpoints(self):
        helper, source, node = self.helper()
        node.publishers = [Namespace(endpoint_gid=object())]*9
        helper.discover()
        self.assertEqual(source.snapshot()["status"], "ambiguous")
        self.assertTrue(source.snapshot()["publisher_count_is_lower_bound"])
        self.assertIsNone(helper.subscription)
        node.publishers = []
        helper.discover()
        self.assertEqual(source.snapshot()["status"], "disconnected")

    def test_failed_cleanup_stops_new_subscriptions_and_remains_reported(self):
        helper, source, node = self.helper()
        node.publishers = [self.endpoint()]
        helper.discover()
        node.fail_destroy = True
        node.publishers = [self.endpoint(2)]
        helper.discover()
        created = len(node.created)
        helper.discover()
        self.assertTrue(helper.faulted)
        self.assertEqual(len(node.created), created)
        self.assertEqual(source.snapshot()["status"], "disconnected")
        self.assertFalse(helper.close())
        self.assertFalse(helper.close())

    def test_endpoint_unsigned_integrals_and_bad_types(self):
        expected = hashlib.sha256(bytes([2])*16).hexdigest()
        self.assertEqual(_endpoint_id(self.endpoint(2)), expected)
        for invalid in ([True]*16, [2.]*16, [-1]*16, [256]*16, [2]*15):
            with self.assertRaises(ValueError): _endpoint_id(Namespace(endpoint_gid=invalid))
        try:
            import numpy as np
        except ImportError:
            return
        self.assertEqual(_endpoint_id(Namespace(endpoint_gid=np.array([2]*16,dtype=np.uint8))), expected)


if __name__ == "__main__":
    unittest.main()
