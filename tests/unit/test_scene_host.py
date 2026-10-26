"""Fake-process file-protocol tests: no ROS, MuJoCo, GL, ports or subprocesses."""
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import subprocess
import tempfile
import unittest

from mujoco_ros2_manipulation.operator_console.scene_host import SceneHost, SceneSource, _canonical, _manifest
from mujoco_ros2_manipulation.operator_console.view_binding import make_binding


class Clock:
    def __init__(self):
        self.value = 100.
    def __call__(self):
        return self.value
    def advance(self, seconds):
        self.value += seconds


def binding(generation=1, mode="live"):
    return make_binding("a" * 32, generation, "workcell:test", mode,
                        request_id="d" * 32 if mode == "plan_preview" else None)


def write_json(path, value):
    path.write_bytes(_canonical(value) + b"\n")


def make_manifest(xml, names, home, *, asset=None):
    data = dict(schema_version=1, export_version="compiled-scene-v1",
        model_sha256=hashlib.sha256(xml.read_bytes()).hexdigest(), compiled_model_sha256="c" * 64,
        mujoco_version="3.14.0", units=dict(length="m", angle="rad", up_axis="Z", rotation="row-major"),
        joint_names=names, home_rad=home, default_visible_groups=[0, 1, 2],
        geoms=[dict(id=0, name="fixture", type="box", group=0, default_visible=True,
                    size=[1., 1., 1.], rgba=[.5, .5, .5, 1.], asset_sha256=None)], assets=[])
    if asset is not None:
        descriptor, blob = asset
        data["geoms"][0].update(type="mesh", asset_sha256=descriptor["sha256"])
        data["assets"] = [descriptor]
    data["scene_sha256"] = hashlib.sha256(_canonical(data)).hexdigest()
    return data


class FakeProcess:
    def __init__(self, argv, kwargs, factory):
        self.argv, self.kwargs, self.factory = argv, kwargs, factory
        self.returncode = None
        self.terminated = self.killed = 0
        def option(name):
            return argv[argv.index(name) + 1]
        self.directory = Path(option("--directory"))
        self.manifest = make_manifest(Path(option("--model")), json.loads(option("--joint-names-json")),
            json.loads(option("--home-json")), asset=factory.asset)
        if factory.publish_manifest:
            assets = self.directory / "assets"
            assets.mkdir()
            if factory.asset is not None:
                descriptor, blob = factory.asset
                (assets / (descriptor["sha256"] + ".bin")).write_bytes(blob)
            write_json(self.directory / "manifest.json", self.manifest)
        factory.alive_at_spawn.append(sum(p.poll() is None for p in factory.processes))
    def poll(self):
        return self.returncode
    def terminate(self):
        self.terminated += 1
        if not self.factory.stubborn:
            self.returncode = -15
    def kill(self):
        self.killed += 1
        if not self.factory.stubborn:
            self.returncode = -9
    def wait(self, timeout=None):
        if self.returncode is None:
            raise subprocess.TimeoutExpired("fake worker", timeout)
        return self.returncode
    def reply(self, sequence, *, processed, model_sha=None):
        pose = dict(schema_version=1, scene_sha256=self.manifest["scene_sha256"],
            model_sha256=model_sha or self.manifest["model_sha256"],
            geoms=[dict(id=0, position=[1., 2., 3.], rotation=[1.,0.,0.,0.,1.,0.,0.,0.,1.])])
        write_json(self.directory / "pose.json", dict(sequence=sequence, pose=pose, processed_monotonic=processed))


class Factory:
    def __init__(self):
        self.processes, self.alive_at_spawn = [], []
        self.publish_manifest = True
        self.stubborn = False
        self.asset = None
    def __call__(self, argv, **kwargs):
        process = FakeProcess(argv, kwargs, self)
        self.processes.append(process)
        return process


class SceneHostTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory()
        self.base = Path(self.temporary.name)
        self.xml = self.base / "model.xml"
        self.xml.write_text("<mujoco/>")
        self.clock, self.factory = Clock(), Factory()
        self.validations = 0
        def validate():
            self.validations += 1
            return {"input_digest": "pinned"}
        self.source = SceneSource("workcell:test", "Small fixture", self.xml, ("j1",), {"j1": 0.}, validate)
        self.host = SceneHost(self.base / "host", self.base / ".venv/bin/python", clock=self.clock,
            process_factory=self.factory, start_thread=False, max_pending=2)
    def tearDown(self):
        self.factory.stubborn = False
        if not self.host.closed:
            self.host.close()
        elif self.host._epoch is not None:
            self.host._stop_epoch()
        self.temporary.cleanup()
    def ready(self, mode="live"):
        identifier = self.host.activate(binding(mode=mode), self.source)
        self.host._step()
        self.assertEqual(self.host.snapshot()["status"], "ready")
        return identifier, self.factory.processes[-1]
    def input_sequence(self, process):
        return json.loads((process.directory / "input.json").read_text())["sequence"]

    def test_model_preview_is_explicit_home_and_live_never_synthesizes_a_state(self):
        identifier, process = self.ready()
        self.assertFalse((process.directory / "input.json").exists())
        self.assertIsNone(self.host.pose_envelope(identifier))
        second = self.host.activate(binding(2, "model_preview"), self.source)
        self.host._step()
        self.assertEqual(len(self.factory.processes), 2)
        preview = self.factory.processes[-1]
        self.assertEqual(self.input_sequence(preview), 1)
        preview.reply(1, processed=self.clock())
        self.host._step()
        envelope = self.host.pose_envelope(second)
        self.assertEqual(envelope["binding"]["mode"], "model_preview")
        self.assertIsNone(envelope["source_state_age_sec"])
        self.assertIsNone(envelope["simulation_sec"])

    def test_original_input_receive_time_not_worker_completion_determines_live_age(self):
        identifier, process = self.ready()
        self.clock.advance(1)
        sequence = self.host.submit({"j1": .2}, source_received_monotonic=100.5, simulation_sec=12.)
        self.host._step()
        self.assertEqual(self.input_sequence(process), sequence)
        self.clock.advance(5)
        process.reply(sequence, processed=self.clock())
        self.host._step()
        value = self.host.pose_envelope(identifier)
        self.assertEqual(value["source_state_age_sec"], 5.5)
        self.assertEqual(value["simulation_sec"], 12.)
        self.assertNotIn("processed_monotonic", value)
        self.assertNotIn("source_received_monotonic", value)
        self.clock.advance(1)
        self.assertEqual(self.host.pose_envelope(identifier)["source_state_age_sec"], 6.5)

    def test_binding_change_clears_every_cache_before_old_process_is_stopped(self):
        old_id, old = self.ready()
        seq = self.host.submit({"j1": .1}, source_received_monotonic=99.)
        self.host._step(); old.reply(seq, processed=self.clock()); self.host._step()
        self.assertIsNotNone(self.host.pose_envelope(old_id))
        new_id = self.host.activate(binding(2), self.source)
        self.assertIsNone(self.host.snapshot()["manifest_descriptor"])
        self.assertFalse(self.host.snapshot()["pose_ready"])
        self.assertIsNone(old.poll(), "activate does not block on worker cleanup")
        with self.assertRaises(ValueError):
            self.host.manifest_bytes(old_id)
        self.host._step()
        self.assertEqual(old.terminated, 1)
        self.assertEqual(self.factory.alive_at_spawn, [0, 0])
        self.assertEqual(self.host.snapshot()["binding_id"], new_id)
        self.assertIsNone(self.host.pose_envelope(new_id))

    def test_invalidate_blanks_without_launching_and_same_placeholder_binding_can_activate(self):
        old_id, process = self.ready()
        placeholder = binding(2)
        placeholder_id = self.host.invalidate(placeholder)
        self.assertEqual(self.host.snapshot()["status"], "waiting")
        self.assertIsNone(self.host.snapshot()["source"])
        self.host._step()
        self.assertEqual(process.terminated, 1)
        self.assertEqual(len(self.factory.processes), 1)
        self.assertEqual(self.host.activate(placeholder, self.source), placeholder_id)
        self.host._step()
        self.assertEqual(len(self.factory.processes), 2)
        self.host.invalidate()
        self.assertIsNone(self.host.snapshot()["binding_id"])
        self.host._step()
        self.assertEqual(len(self.factory.processes), 2)

    def test_input_is_bounded_complete_finite_and_duplicates_do_not_refresh_sequence(self):
        identifier, process = self.ready()
        seq = self.host.submit({"j1": .2}, source_received_monotonic=99.)
        self.assertEqual(self.host.submit({"j1": .2}, source_received_monotonic=99.), seq)
        for positions, received in [({}, 99.1), ({"j1": float("nan")}, 99.1), ({"j1": 1e7}, 99.1),
                                    ({"j1": .2}, None), ({"j1": .2}, 101.), ({"j1": .3}, 98.)]:
            with self.subTest(positions=positions, received=received), self.assertRaises(ValueError):
                self.host.submit(positions, source_received_monotonic=received)
        self.assertEqual(self.host._sequence, seq)
        self.host._step()
        self.assertEqual(self.input_sequence(process), seq)

    def test_evicted_or_replayed_worker_result_never_receives_a_new_freshness_timestamp(self):
        identifier, process = self.ready()
        for index in range(3):
            self.host.submit({"j1": index / 10}, source_received_monotonic=98. + index / 10)
        self.host._step()
        self.assertEqual(self.input_sequence(process), 3, "Only the latest pending state is written")
        process.reply(1, processed=self.clock()); self.host._step()
        self.assertIsNone(self.host.pose_envelope(identifier), "No original sample metadata remains for seq1")
        process.reply(3, processed=self.clock()); self.host._step()
        original = self.host.pose_envelope(identifier)
        self.assertEqual(original["sequence"], 3)
        process.reply(2, processed=self.clock()); self.host._step()
        self.assertEqual(self.host.pose_envelope(identifier)["sequence"], 3)

    def test_plan_playback_keeps_preview_time_separate_from_simulation_and_live_age(self):
        identifier, process = self.ready("plan_preview")
        self.assertFalse((process.directory / "input.json").exists())
        seq = self.host.submit({"j1": .3}, preview_time_sec=1.25)
        self.host._step(); process.reply(seq, processed=self.clock()); self.host._step()
        value = self.host.pose_envelope(identifier)
        self.assertEqual(value["preview_time_sec"], 1.25)
        self.assertIsNone(value["source_state_age_sec"])
        self.assertIsNone(value["simulation_sec"])

    def test_sources_are_validated_before_after_load_and_on_change_without_auto_reload(self):
        identifier, process = self.ready()
        self.assertEqual(self.validations, 2)
        self.xml.write_text("<mujoco><!-- changed --></mujoco>")
        self.clock.advance(1.1); self.host._step()
        self.assertEqual(self.host.snapshot()["status"], "error")
        self.assertFalse(self.host.snapshot()["pose_ready"])
        self.assertIsNone(self.host.snapshot()["manifest_descriptor"])
        self.host._step()
        self.assertEqual(process.terminated, 1)
        self.assertEqual(len(self.factory.processes), 1)
        self.host.activate(binding(), self.source); self.host._step()
        self.assertEqual(len(self.factory.processes), 1, "Same failed binding is never automatically retried")

    def test_manifest_model_mismatch_or_startup_timeout_cannot_become_ready(self):
        self.factory.publish_manifest = False
        self.host.activate(binding(), self.source)
        self.host._step()
        self.assertEqual(self.host.snapshot()["status"], "preparing")
        self.clock.advance(31); self.host._step()
        self.assertEqual(self.host.snapshot()["status"], "error")
        self.host._step()
        self.assertEqual(len(self.factory.processes), 1)

    def test_malformed_worker_pose_and_private_error_are_generic_and_invalidate_cache(self):
        identifier, process = self.ready()
        seq = self.host.submit({"j1": .1}, source_received_monotonic=99.)
        self.host._step(); process.reply(seq, processed=self.clock(), model_sha="0" * 64); self.host._step()
        self.assertEqual(self.host.snapshot()["status"], "error")
        self.assertEqual(self.host.snapshot()["error"]["code"], "invalid_worker_output")
        self.host.activate(binding(2), self.source); self.host._step()
        process = self.factory.processes[-1]
        private_path = str(self.base / "private-scene.xml")
        write_json(process.directory / "error.json", dict(kind="pose_failed", error=dict(message=private_path + " secret")))
        self.host._step()
        snapshot = json.dumps(self.host.snapshot())
        self.assertNotIn(private_path, snapshot)
        self.assertNotIn(str(self.base), snapshot)
        self.assertNotIn("secret", snapshot)

    def test_unstoppable_owned_worker_blocks_new_worker_instead_of_running_two(self):
        identifier, old = self.ready()
        self.factory.stubborn = True
        self.host.activate(binding(2), self.source); self.host._step()
        self.assertEqual(len(self.factory.processes), 1)
        self.assertEqual(self.host.snapshot()["status"], "error")
        self.assertEqual(self.host.snapshot()["error"]["code"], "worker_cleanup_failed")
        self.assertEqual(old.terminated, 1); self.assertEqual(old.killed, 1)
        self.factory.stubborn = False
        self.host._step()
        self.assertEqual(len(self.factory.processes), 1, "Cleanup failure is latched until a new binding generation")
        self.host.activate(binding(3), self.source); self.host._step()
        self.assertEqual(len(self.factory.processes), 2)

    def test_unsubmitted_future_worker_sequence_is_an_error_not_a_fresh_pose(self):
        identifier, process = self.ready()
        process.reply(42, processed=self.clock())
        self.host._step()
        self.assertEqual(self.host.snapshot()["status"], "error")
        self.assertEqual(self.host.snapshot()["error"]["code"], "invalid_worker_output")

    def test_frozen_asset_bytes_are_hash_checked_and_restricted_to_current_catalog(self):
        import struct
        blob = struct.pack("<9f3I", 0,0,0,1,0,0,0,1,0,0,1,2)
        digest = hashlib.sha256(blob).hexdigest()
        descriptor = dict(sha256=digest, byte_length=len(blob), media_type="application/octet-stream",
            layout=dict(positions=dict(byte_offset=0,byte_length=36,count=3,item_size=3,component_type="float32"),
                        indices=dict(byte_offset=36,byte_length=12,count=3,item_size=1,component_type="uint32")))
        self.factory.asset = descriptor, blob
        identifier, process = self.ready()
        self.assertEqual(self.host.asset_bytes(digest, identifier), blob)
        (process.directory / "assets" / (digest + ".bin")).write_bytes(b"modified")
        self.assertEqual(self.host.asset_bytes(digest, identifier), blob, "Published asset bytes are immutable snapshots")
        with self.assertRaises(ValueError):
            self.host.asset_bytes("f" * 64, identifier)
        self.host.invalidate(binding(2))
        with self.assertRaises(ValueError):
            self.host.asset_bytes(digest, identifier)

    def test_close_is_idempotent_and_only_stops_its_own_process(self):
        identifier, process = self.ready()
        self.assertTrue(process.kwargs["start_new_session"])
        self.assertEqual(process.kwargs["stdin"], subprocess.DEVNULL)
        self.host.close(); self.host.close()
        self.assertEqual(process.terminated, 1)
        self.assertEqual(self.host.snapshot()["status"], "closed")
        self.assertFalse(process.directory.exists())


if __name__ == "__main__":
    unittest.main()
