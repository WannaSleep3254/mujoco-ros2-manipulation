"""Historical provenance and geometric witness replay, without ROS/native imports."""
from copy import deepcopy
import hashlib
import gzip
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest

from mujoco_ros2_manipulation.paths import ROOT
from mujoco_ros2_manipulation.workcells.reproduction import load_reproduction, replay_witness


def sha(raw):
    return hashlib.sha256(raw).hexdigest()


class ReproductionTests(unittest.TestCase):
    def setUp(self):
        self.directory = tempfile.TemporaryDirectory()
        self.addCleanup(self.directory.cleanup)
        self.root = Path(self.directory.name)
        self.fixture = json.loads((ROOT / "config/reproductions/home_return_clearance.json").read_text())
        sequence_sources = [step["source"] for step in self.fixture.get("setup", {}).get("sequence", [])]
        for source in [*self.fixture["source"]["traces"].values(), self.fixture["source"]["session"], *sequence_sources]:
            destination = self.root / source["path"]
            destination.parent.mkdir(parents=True, exist_ok=True)
            destination.write_bytes((ROOT / source["path"]).read_bytes())
        self.session = json.loads((self.root / self.fixture["source"]["session"]["path"]).read_text())
        self.manifest = deepcopy(self.session["model_manifest"])
        source = self.root / "native/path_distance.cpp"
        source.parent.mkdir(parents=True)
        source.write_bytes((ROOT / "native/path_distance.cpp").read_bytes())
        geometry = self.fixture["source"]["geometry"]
        for name in geometry["config_sha256"]:
            destination = self.root / name
            destination.parent.mkdir(parents=True, exist_ok=True)
            destination.write_bytes((ROOT / name).read_bytes())
        # A minimal pinned asset exercises relocation and tampering without native MuJoCo.
        asset = self.root / "test-assets/shape.stl"
        asset.parent.mkdir()
        asset.write_bytes(b"pinned mesh bytes used by the fake evaluator")
        output = self.root / "generated"
        output.mkdir()
        xml = f'<mujoco><compiler meshdir="."/><asset><mesh name="shape" file="{asset}"/></asset></mujoco>'.encode()
        (output / "cell.xml").write_bytes(xml)
        self.manifest["outputs"]["cell.xml"] = sha(xml)
        geometry["cell_xml_canonical_sha256"] = sha(xml.decode().replace(str(self.root), "{REPOSITORY_ROOT}").encode())
        geometry["asset_sha256"] = {asset.relative_to(self.root).as_posix(): sha(asset.read_bytes())}
        self.cell = SimpleNamespace(root=self.root, id="fr5_ur5e", joints=self.manifest["joints"],
                                    fingerprint=self.manifest["workcell_sha256"], output=output)
        self.path = self.root / "fixture.json"
        self.save()

    def save(self):
        self.path.write_text(json.dumps(self.fixture))

    def load(self):
        return load_reproduction(self.path, self.cell, self.manifest)

    def evaluator(self, fixture, *, distance_error=0.0, wrong_bodies=False, library=None, source=None):
        metadata = deepcopy(self.session["evaluator"])
        if library is not None:
            metadata["library_sha256"] = library
        if source is not None:
            metadata["source_sha256"] = source
        def at(state):
            if state == fixture["initial_rad"]:
                distance = 0.42114959720096734
            elif state == fixture["target_rad"]:
                distance = 0.1862102510763399
            else:
                self.assertEqual(state, fixture["witness"]["state_rad"])
                distance = fixture["witness"]["distance_m"] + distance_error
            return dict(distance_m=distance,
                        bodies=["changed", "body"] if wrong_bodies else fixture["witness"]["bodies"][:],
                        nearest_points_m=[[0, 0, 0], [1, 0, 0]], tips_m=[[0, 0, 0], [1, 0, 0]])
        return SimpleNamespace(metadata=lambda: deepcopy(metadata), at=at)

    def test_portable_model_identity_and_detached_witness_replay(self):
        fixture = self.load()
        self.assertFalse(fixture["_model_comparison"]["raw_cell_xml_sha256"]["matches"])
        self.assertTrue(fixture["_model_comparison"]["portable_geometry_matches"])
        result = replay_witness(fixture, self.evaluator(fixture), 0.03)
        self.assertTrue(result["reproduced"])
        self.assertEqual(result["endpoint_clearance"]["endpoint_violations"], [])
        self.assertEqual(result["witness"]["distance_error_m"], 0.0)
        self.assertIn("sample only", result["scope"])
        fixture["target_rad"][fixture["_historical_session"]["model_manifest"]["joints"][0]] = 1
        self.assertNotEqual(fixture["target_rad"], self.load()["target_rad"])
        self.assertEqual(self.manifest["joint_limits"], self.session["model_manifest"]["joint_limits"])

    def test_frozen_setup_trajectories_preserve_index_source_identity_and_detached_data(self):
        fixture = self.load()
        self.assertEqual(len(fixture["_setup_trajectories"]), len(fixture["setup"]["sequence"]))
        for frozen, step in zip(fixture["_setup_trajectories"], fixture["setup"]["sequence"]):
            self.assertEqual(frozen["source"], dict(request_id=step["source_request_id"],
                trace_file=step["source"]["path"], trace_compressed_sha256=step["source"]["sha256"],
                trace_sha256=step["source_trace_sha256"]))
            self.assertEqual(set(frozen["trajectory"]["names"]), set(self.cell.joints))
        original = fixture["_setup_trajectories"][-1]["trajectory"]["points"][0]["p"][0]
        fixture["_setup_trajectories"][-1]["trajectory"]["points"][0]["p"][0] += 0.1
        self.assertEqual(self.load()["_setup_trajectories"][-1]["trajectory"]["points"][0]["p"][0], original)

    def test_frozen_setup_endpoint_is_checked_even_with_updated_source_hashes(self):
        step = self.fixture["setup"]["sequence"][0]
        path = self.root / step["source"]["path"]
        trace = json.loads(gzip.decompress(path.read_bytes()))
        trace["trajectories"][0]["points"][-1]["p"][0] += 0.1
        raw = json.dumps(trace).encode()
        packed = gzip.compress(raw, mtime=0)
        path.write_bytes(packed)
        step["source"]["sha256"] = sha(packed)
        step["source_trace_sha256"] = sha(raw)
        self.save()
        with self.assertRaisesRegex(ValueError, "endpoint exceeds"):
            self.load()

    def test_changed_build_binary_is_recorded_without_rejecting_same_geometry(self):
        fixture = self.load()
        result = replay_witness(fixture, self.evaluator(fixture, library="a" * 64), 0.03)
        self.assertTrue(result["reproduced"])
        self.assertFalse(result["library_sha256"]["matches"])
        self.assertEqual(result["library_sha256"]["current"], "a" * 64)

    def test_source_file_tampering_is_rejected_before_geometric_replay(self):
        source = self.fixture["source"]["traces"]["coordinated"]
        path = self.root / source["path"]
        path.write_bytes(path.read_bytes() + b"\n")
        with self.assertRaisesRegex(ValueError, "Provenance SHA-256 mismatch"):
            self.load()

    def test_request_identity_and_command_provenance_cannot_be_reassigned(self):
        self.fixture["source"]["request_ids"]["coordinated"] = "a" * 32
        self.save()
        with self.assertRaisesRegex(ValueError, "request ID"):
            self.load()

    def test_incomplete_nonfinite_boolean_and_out_of_limit_states_are_rejected(self):
        original = deepcopy(self.fixture)
        joint = self.cell.joints[0]
        for section in ("initial_rad", "target_rad", "witness"):
            for mutation in ("missing", True, float("nan"), 1000):
                with self.subTest(section=section, mutation=mutation):
                    self.fixture = deepcopy(original)
                    state = self.fixture[section] if section != "witness" else self.fixture[section]["state_rad"]
                    if mutation == "missing":
                        state.pop(joint)
                    else:
                        state[joint] = mutation
                    self.save()
                    with self.assertRaises(ValueError):
                        self.load()

    def test_fixture_and_live_threshold_must_match_historical_policy(self):
        original = deepcopy(self.fixture)
        for value in (0.02, 0, True, float("inf")):
            with self.subTest(value=value):
                self.fixture = deepcopy(original)
                self.fixture["expected"]["threshold_m"] = value
                self.save()
                with self.assertRaises(ValueError):
                    self.load()
        self.fixture = original
        self.save()
        fixture = self.load()
        with self.assertRaisesRegex(ValueError, "threshold mismatch"):
            replay_witness(fixture, self.evaluator(fixture), 0.02)

    def test_changed_model_asset_or_configuration_is_rejected(self):
        for name in ("test-assets/shape.stl", "config/workcells/fr5_ur5e.yaml"):
            with self.subTest(name=name):
                path = self.root / name
                raw = path.read_bytes()
                path.write_bytes(raw + b"\n")
                with self.assertRaisesRegex(ValueError, "SHA-256 mismatch"):
                    self.load()
                path.write_bytes(raw)
        self.manifest["workcell_sha256"] = "a" * 64
        with self.assertRaisesRegex(ValueError, "fingerprint"):
            self.load()

    def test_native_source_and_witness_distance_or_body_changes_are_rejected(self):
        fixture = self.load()
        for options in ({"source": "a" * 64}, {"distance_error": 1e-6}, {"wrong_bodies": True}):
            with self.subTest(options=options), self.assertRaises(ValueError):
                replay_witness(fixture, self.evaluator(fixture, **options), 0.03)
        native = self.root / "native/path_distance.cpp"
        native.write_bytes(native.read_bytes() + b"\n")
        with self.assertRaisesRegex(ValueError, "native engine or source"):
            self.load()

    def test_setup_sequence_compressed_and_raw_provenance_and_targets_are_checked(self):
        original = deepcopy(self.fixture)
        for mutation in ("compressed", "raw", "target", "request_id"):
            with self.subTest(mutation=mutation):
                self.fixture = deepcopy(original)
                step = self.fixture["setup"]["sequence"][0]
                path = self.root / step["source"]["path"]
                packed = path.read_bytes()
                if mutation == "compressed":
                    path.write_bytes(packed + b"\n")
                elif mutation == "raw":
                    step["source_trace_sha256"] = "a" * 64
                elif mutation == "target":
                    step["target_rad"][self.cell.joints[0]] += 0.01
                else:
                    step["source_request_id"] = "a" * 32
                self.save()
                with self.assertRaises(ValueError):
                    self.load()
                path.write_bytes(packed)

    def test_provenance_cannot_escape_repository(self):
        self.fixture["source"]["session"]["path"] = "../session.json"
        self.save()
        with self.assertRaisesRegex(ValueError, "repository-relative"):
            self.load()


if __name__ == "__main__":
    unittest.main()
