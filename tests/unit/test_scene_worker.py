"""CPU child protocol tests with an injected tiny scene; no model compilation."""
from copy import deepcopy
import hashlib
import json
import os
from pathlib import Path
from tempfile import TemporaryDirectory
import unittest
from unittest.mock import patch

from mujoco_ros2_manipulation.operator_console.scene_worker import (InputRejected, MAX_INPUT_BYTES, SceneWorker, decode_input,
                          parent_present, publish_manifest, validate_joint_names)


def encoded(value):
    return json.dumps(value, allow_nan=False).encode()


class FakeScene:
    def __init__(self):
        self.calls = []
        self.fail = False
        self.blob = b'\0'*48
        self.digest = hashlib.sha256(self.blob).hexdigest()

    def manifest(self):
        return dict(schema_version=1, scene_sha256='1'*64, assets=[dict(sha256=self.digest, byte_length=len(self.blob))])

    def asset(self, digest):
        assert digest == self.digest
        return self.blob

    def pose(self, positions):
        self.calls.append(deepcopy(positions))
        if self.fail:
            raise RuntimeError('test pose failed')
        return dict(schema_version=1,scene_sha256='1'*64,model_sha256='2'*64,
                    geoms=[dict(id=0,position=[positions['q'],0.,0.],rotation=[1.,0.,0.,0.,1.,0.,0.,0.,1.])])


class SceneWorkerTests(unittest.TestCase):
    def setUp(self):
        temporary = TemporaryDirectory()
        self.addCleanup(temporary.cleanup)
        self.directory = Path(temporary.name)
        self.scene = FakeScene()
        self.worker = SceneWorker(self.scene, self.directory, ['q'], clock=lambda: 12.5)

    def write(self, value):
        (self.directory/'input.json').write_bytes(value if type(value) is bytes else encoded(value))

    def read(self, name):
        return json.loads((self.directory/name).read_bytes())

    def test_atomic_manifest_assets_are_content_addressed_and_verified(self):
        publish_manifest(self.scene, self.directory)
        self.assertEqual(self.read('manifest.json'), self.scene.manifest())
        self.assertEqual((self.directory/'assets'/f'{self.scene.digest}.bin').read_bytes(), self.scene.blob)
        self.assertFalse(list(self.directory.rglob('*.tmp')))
        self.scene.blob = b'tampered'
        with self.assertRaisesRegex(ValueError, 'descriptor'):
            publish_manifest(self.scene, self.directory)

    def test_sequence_is_published_once_and_stale_or_changed_old_input_cannot_replace_pose(self):
        self.assertFalse(self.worker.step())
        self.write(dict(sequence=1, positions=dict(q=.2)))
        self.assertTrue(self.worker.step())
        saved = (self.directory/'pose.json').read_bytes()
        first = self.read('pose.json')
        self.assertEqual(first['sequence'], 1)
        self.assertEqual(first['processed_monotonic'], 12.5)
        self.assertNotIn('source_state_age_sec', first)
        self.assertFalse(self.worker.step())
        self.assertEqual(len(self.scene.calls), 1)
        self.write(dict(sequence=1, positions=dict(q=.3)))
        self.assertFalse(self.worker.step())
        self.assertEqual((self.directory/'pose.json').read_bytes(), saved)
        self.assertEqual(self.read('error.json')['kind'], 'old_sequence')
        self.write(dict(sequence=2, positions=dict(q=.4)))
        self.assertTrue(self.worker.step())
        self.assertEqual(self.read('pose.json')['pose']['geoms'][0]['position'][0], .4)
        self.assertFalse((self.directory/'error.json').exists())

    def test_partial_duplicate_nan_oversized_and_bad_state_preserve_last_good_pose(self):
        self.write(dict(sequence=1, positions=dict(q=0)))
        self.assertTrue(self.worker.step())
        before = (self.directory/'pose.json').read_bytes()
        invalid = [b'{"sequence":2,', b'{"sequence":2,"positions":{"q":NaN}}',
            b'{"sequence":2,"sequence":3,"positions":{"q":0}}',
            b'{"sequence":2,"positions":{"q":'+b'1'*5000+b'}}',
            b' '*(MAX_INPUT_BYTES+1), dict(sequence=True,positions=dict(q=0)),
            dict(sequence=2,positions={}), dict(sequence=2,positions=dict(q=True)),
            dict(sequence=2,positions=dict(q=1e7)), dict(sequence=2,positions=dict(q=0),extra='private')]
        for value in invalid:
            with self.subTest(value=str(value)[:80]):
                self.write(value)
                self.assertFalse(self.worker.step())
                self.assertEqual((self.directory/'pose.json').read_bytes(), before)
                self.assertTrue((self.directory/'error.json').is_file())
                self.assertEqual(self.worker.last_sequence, 1)
        self.write(dict(sequence=3,positions=dict(q=.1)))
        self.assertTrue(self.worker.step())

    def test_symlink_and_fifo_input_never_reads_or_blocks(self):
        target = self.directory/'private.json'
        target.write_bytes(encoded(dict(sequence=1,positions=dict(q=99))))
        (self.directory/'input.json').symlink_to(target)
        self.assertFalse(self.worker.step())
        self.assertEqual(self.scene.calls, [])
        (self.directory/'input.json').unlink()
        os.mkfifo(self.directory/'input.json')
        self.assertFalse(self.worker.step())
        self.assertEqual(self.read('error.json')['kind'], 'invalid_input_file')
        self.assertFalse((self.directory/'pose.json').exists())

    def test_compute_and_nonfinite_serialization_failure_never_publish_or_advance(self):
        self.write(dict(sequence=1,positions=dict(q=0)))
        self.scene.fail = True
        self.assertFalse(self.worker.step())
        self.assertEqual(self.worker.last_sequence, 0)
        self.assertFalse((self.directory/'pose.json').exists())
        self.assertEqual(self.read('error.json')['kind'], 'pose_failed')
        self.scene.fail = False
        self.write(dict(sequence=2,positions=dict(q=.1)))
        self.assertTrue(self.worker.step())
        before = (self.directory/'pose.json').read_bytes()
        self.write(dict(sequence=3,positions=dict(q=.2)))
        with patch.object(self.scene, 'pose', return_value=dict(bad=float('nan'))):
            self.assertFalse(self.worker.step())
        self.assertEqual((self.directory/'pose.json').read_bytes(), before)
        self.assertEqual(self.worker.last_sequence, 2)

    def test_joint_names_and_json_numbers_are_exact_and_bounded(self):
        for value in (['q','q'], 'q', [True], [''], ['q']*513):
            with self.subTest(value=str(value)[:50]), self.assertRaises(ValueError):
                validate_joint_names(value)
        for sequence in (0,-1,True,1.,2**53):
            with self.subTest(sequence=sequence), self.assertRaises(InputRejected):
                decode_input(encoded(dict(sequence=sequence,positions=dict(q=0))), ['q'])
        with self.assertRaises(InputRejected):
            decode_input(b'{"sequence":1,"positions":{"q":1e999}}', ['q'])
        with self.assertRaises(InputRejected):
            decode_input(encoded(dict(sequence=1,positions=dict(q=10**1000))), ['q'])

    def test_parent_identity_uses_direct_parent_not_only_pid_liveness(self):
        self.assertTrue(parent_present(1234, getppid=lambda: 1234))
        for expected, observed in ((1234,1),(1234,4321),(1,1),(True,True)):
            with self.subTest(expected=expected,observed=observed):
                self.assertFalse(parent_present(expected, getppid=lambda: observed))


if __name__ == '__main__':
    unittest.main()
