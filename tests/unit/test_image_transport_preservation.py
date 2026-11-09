"""Verifier-owned original bundles survive success and incomplete acquisition."""
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

from mujoco_ros2_manipulation.operator_console import verify_image_transport as verification
from mujoco_ros2_manipulation.operator_console.synchronized_recording import SyncRecording
from mujoco_ros2_manipulation.paths import ROOT
from tests.unit.test_synchronized_recording import recording


class CapturePreservationTests(unittest.TestCase):
    def setUp(self):
        validation = ROOT/'runtime/validation/comparison-startup-cancel-20261110/retention-unit-fixtures'
        validation.mkdir(parents=True, exist_ok=True)
        self.root = Path(tempfile.mkdtemp(prefix='case-', dir=validation))
        self.session = self.root/'runtime/operator/20261110T000000Z-abcdef12'
        self.capture_id = 'a'*32
        self.source = self.session/'synchronized'/self.capture_id
        self.output = self.root/'runtime/validation/image-check'
        self.output.mkdir(parents=True)
        self.original = recording(asset=True)
        self.original.save(self.source/'recording.json')
        # The copy must retain these bytes, including noncanonical JSON layout.
        raw = json.loads((self.source/'recording.json').read_text())
        (self.source/'recording.json').write_text(json.dumps(raw, indent=2)+'\n')
        (self.source/'capture.json').write_bytes(b'{"synthetic_capture": true}\n')
        self.marker = f'OPERATOR_READY http://127.0.0.1:8934/ domain=220 session={self.session}\n'
        (self.output/'console.log').write_text(self.marker)

    def preserve(self, **kwargs):
        return verification.preserve_capture_bundle(self.session, self.capture_id,
            self.output, root=self.root, **kwargs)

    @property
    def destination(self):
        return self.output/'capture-originals'/self.capture_id

    def test_original_json_assets_and_jpegs_have_exact_source_destination_inventories(self):
        result = self.preserve(expected_recording_sha256=self.original.sha256)
        self.assertEqual(result['status'], 'preserved', result)
        self.assertTrue(result['copied']); self.assertTrue(result['byte_agreement'])
        self.assertTrue(result['recording_valid'])
        self.assertEqual(SyncRecording.load(self.destination/'recording.json').sha256, self.original.sha256)
        source = json.loads((self.root/result['source_inventory']).read_text())
        destination = json.loads((self.root/result['destination_inventory']).read_text())
        self.assertEqual(source['before'], source['after'])
        self.assertEqual(source['before'], destination)
        for item in destination['files']:
            name = item['relative_path']
            self.assertEqual((self.source/name).read_bytes(), (self.destination/name).read_bytes())
            self.assertEqual(item['file_kind'], 'regular')
            self.assertEqual(item['byte_length'], (self.destination/name).stat().st_size)
            self.assertEqual(item['sha256'], hashlib.sha256((self.destination/name).read_bytes()).hexdigest())
        self.assertFalse(Path(result['source']).is_absolute())
        self.assertTrue((self.root/result['preservation']).is_file())

    def test_invalid_completed_recording_and_missing_asset_are_still_byte_retained(self):
        for case in ('invalid_json', 'missing_asset', 'wrong_prior_identity'):
            with self.subTest(case=case):
                output = self.root/'runtime/validation'/case
                output.mkdir()
                raw = (self.source/'recording.json').read_bytes()
                if case == 'invalid_json':
                    (self.source/'recording.json').write_bytes(b'{incomplete original')
                missing = None
                if case == 'missing_asset':
                    missing = next((self.source/'assets').iterdir())
                    missing_bytes = missing.read_bytes()
                    missing.unlink()
                result = verification.preserve_capture_bundle(self.session, self.capture_id, output,
                    root=self.root, expected_recording_sha256='f'*64 if case == 'wrong_prior_identity' else None)
                self.assertEqual(result['status'], 'failed')
                self.assertTrue(result['copied']); self.assertTrue(result['byte_agreement'])
                self.assertFalse(result['recording_valid'])
                destination = output/'capture-originals'/self.capture_id
                self.assertEqual((destination/'recording.json').read_bytes(), (self.source/'recording.json').read_bytes())
                self.assertTrue(result['errors'])
                if case == 'wrong_prior_identity':
                    self.assertTrue(any('verified recording identity' in item['message'] for item in result['errors']))
                if missing is not None:
                    missing.write_bytes(missing_bytes)
                (self.source/'recording.json').write_bytes(raw)

    def test_failed_partial_bundle_is_preserved_without_becoming_a_playable_recording(self):
        (self.source/'recording.json').unlink()
        (self.source/'partial.json').write_bytes(b'{"unfinished": true, "raw":')
        result = self.preserve()
        self.assertEqual(result['status'], 'preserved')
        self.assertTrue(result['byte_agreement']); self.assertFalse(result['recording_valid'])
        self.assertEqual((self.destination/'partial.json').read_bytes(), (self.source/'partial.json').read_bytes())
        self.assertFalse((self.destination/'recording.json').exists())

    def test_copy_failure_retains_the_copied_prefix_and_both_inventory_receipts(self):
        publish = verification._publish
        def fail_asset(directory, name, raw, **kwargs):
            if name.endswith('.bin'):
                raise OSError('injected copy failure')
            return publish(directory, name, raw, **kwargs)
        with patch.object(verification, '_publish', side_effect=fail_asset):
            result = self.preserve()
        self.assertEqual(result['status'], 'failed')
        self.assertFalse(result['byte_agreement'])
        self.assertEqual((self.destination/'capture.json').read_bytes(), (self.source/'capture.json').read_bytes())
        self.assertEqual((self.destination/'recording.json').read_bytes(), (self.source/'recording.json').read_bytes())
        retained = json.loads((self.root/result['destination_inventory']).read_text())
        self.assertEqual({item['relative_path'] for item in retained['files']}, {'capture.json', 'recording.json'})
        self.assertTrue((self.root/result['source_inventory']).is_file())
        self.assertTrue((self.root/result['preservation']).is_file())

    def test_source_mutation_after_copy_cannot_be_reported_as_agreeing_evidence(self):
        inventory = verification._capture_inventory
        calls = []
        def change_before_second_read(directory):
            calls.append(directory)
            if len(calls) == 2:
                (self.source/'capture.json').write_bytes(b'{"changed": true}\n')
            return inventory(directory)
        with patch.object(verification, '_capture_inventory', side_effect=change_before_second_read):
            result = self.preserve()
        self.assertEqual(result['status'], 'failed')
        self.assertTrue(result['copied']); self.assertFalse(result['byte_agreement'])
        source = json.loads((self.root/result['source_inventory']).read_text())
        self.assertNotEqual(source['before'], source['after'])
        self.assertEqual((self.destination/'capture.json').read_bytes(), b'{"synthetic_capture": true}\n')

    def test_existing_destination_and_receipts_are_never_replaced(self):
        first = self.preserve()
        contents = {str(path.relative_to(self.output)):path.read_bytes() for path in self.output.rglob('*') if path.is_file()}
        second = self.preserve()
        self.assertEqual(first['status'], 'preserved')
        self.assertEqual(second['status'], 'failed')
        self.assertEqual(contents, {str(path.relative_to(self.output)):path.read_bytes() for path in self.output.rglob('*') if path.is_file()})

    def test_source_and_destination_symlinks_and_parent_escapes_are_rejected(self):
        outside = self.root/'outside.json'; outside.write_text('private fixture')
        (self.source/'capture.json').unlink()
        (self.source/'capture.json').symlink_to(outside)
        self.assertEqual(self.preserve()['status'], 'failed')
        self.assertEqual(outside.read_text(), 'private fixture')
        output = self.root/'runtime/validation/linked-output'; output.mkdir()
        (output/'capture-originals').symlink_to(self.root)
        result = verification.preserve_capture_bundle(self.session, self.capture_id, output, root=self.root)
        self.assertEqual(result['status'], 'failed')
        for capture_id in ('../outside', 'a'*31, '/outside'):
            with self.subTest(capture_id=capture_id):
                result = verification.preserve_capture_bundle(self.session, capture_id, self.output, root=self.root)
                self.assertEqual(result['status'], 'failed')
        result = verification.preserve_capture_bundle(self.session, self.capture_id, self.root/'outside-output', root=self.root)
        self.assertEqual(result['status'], 'failed')
        self.assertFalse((self.root/'outside-output').exists())

    def test_fifo_and_oversized_capture_are_bounded(self):
        (self.source/'capture.json').unlink(); os.mkfifo(self.source/'capture.json')
        script = '''import sys
from pathlib import Path
from mujoco_ros2_manipulation.operator_console.verify_image_transport import preserve_capture_bundle
root=Path(sys.argv[1]);session=root/'runtime/operator/20261110T000000Z-abcdef12'
value=preserve_capture_bundle(session,'a'*32,root/'runtime/validation/image-check',root=root)
assert value['status']=='failed',value
'''
        completed = subprocess.run([sys.executable, '-c', script, str(self.root)], cwd=ROOT,
            capture_output=True, timeout=3)
        self.assertEqual(completed.returncode, 0, completed.stderr.decode())
        (self.source/'capture.json').unlink(); (self.source/'capture.json').write_bytes(b' '*(64*1024+1))
        output = self.root/'runtime/validation/oversized'; output.mkdir()
        result = verification.preserve_capture_bundle(self.session, self.capture_id, output, root=self.root)
        self.assertEqual(result['status'], 'failed')

    def test_console_marker_requires_current_port_domain_owned_path_and_one_session(self):
        log = self.output/'console.log'
        self.assertEqual(verification.owned_console_session(log, self.root, 8934, 220), self.session)
        variants = (self.marker.replace(':8934/', ':8935/'), self.marker.replace('domain=220', 'domain=221'),
            self.marker+self.marker, self.marker.replace(str(self.session), str(self.root/'other')),
            self.marker.replace(str(self.session), str(self.session/'..'/self.session.name)))
        for marker in variants:
            log.write_text(marker)
            with self.subTest(marker=marker), self.assertRaises(ValueError):
                verification.owned_console_session(log, self.root, 8934, 220)

    def test_unknown_start_partial_is_preserved_and_primary_failure_is_unchanged(self):
        (self.source/'recording.json').unlink(); (self.source/'partial.json').write_text('{incomplete')
        primary = dict(type='OriginalTransportError', message='original failure')
        result = dict(status='failed', error=primary, checks=[], synchronized_capture_readiness=dict(start_requests=1))
        evidence = verification.preserve_synchronized_originals(self.output, result, None, root=self.root, port=8934, domain=220)
        self.assertEqual(evidence['status'], 'preserved')
        self.assertEqual(len(evidence['bundles']), 1)
        self.assertEqual(result['error'], primary); self.assertEqual(result['status'], 'failed')
        self.assertFalse(evidence['bundles'][0]['recording_valid'])

    def test_started_check_without_sha_does_not_hide_the_later_verified_recording_identity(self):
        changed = self.original.document
        changed['label'] = 'Valid but different original'
        (self.source/'recording.json').write_text(json.dumps(changed))
        self.assertNotEqual(SyncRecording.load(self.source/'recording.json').sha256, self.original.sha256)
        result = dict(status='passed', checks=[
            dict(name='synchronized_capture_started', id=self.capture_id),
            dict(name='synchronized_capture_and_readonly_replay', id=self.capture_id, recording_sha256=self.original.sha256)])
        evidence = verification.preserve_synchronized_originals(self.output, result, self.capture_id,
            root=self.root, port=8934, domain=220)
        self.assertEqual(evidence['status'], 'failed')
        self.assertEqual(result['status'], 'failed')
        self.assertTrue(evidence['bundles'][0]['byte_agreement'])
        self.assertTrue(any('verified recording identity' in item['message'] for item in evidence['errors']))

    def test_previously_verified_completed_recording_cannot_be_replaced_by_partial_only(self):
        (self.source/'recording.json').unlink()
        (self.source/'partial.json').write_text('{retained partial')
        result = self.preserve(expected_recording_sha256=self.original.sha256)
        self.assertEqual(result['status'], 'failed')
        self.assertTrue(result['byte_agreement'])
        self.assertFalse(result['recording_valid'])
        self.assertEqual((self.destination/'partial.json').read_text(), '{retained partial')

    def test_conflicting_verified_recording_identities_fail_before_archiving(self):
        result = dict(status='passed', checks=[dict(id=self.capture_id, recording_sha256=self.original.sha256),
                                             dict(id=self.capture_id, recording_sha256='f'*64)])
        evidence = verification.preserve_synchronized_originals(self.output, result, self.capture_id,
            root=self.root, port=8934, domain=220)
        self.assertEqual(evidence['status'], 'failed')
        self.assertFalse(self.destination.exists())

    def test_preservation_failure_fails_verification_without_replacing_primary_error(self):
        primary = dict(type='OriginalTransportError', message='original failure')
        result = dict(status='passed', error=primary, checks=[])
        (self.output/'console.log').write_text(self.marker.replace('domain=220', 'domain=221'))
        evidence = verification.preserve_synchronized_originals(self.output, result, self.capture_id,
            root=self.root, port=8934, domain=220)
        self.assertEqual(evidence['status'], 'failed')
        self.assertEqual(result['status'], 'failed'); self.assertEqual(result['error'], primary)
        self.assertTrue(evidence['errors'])


if __name__ == '__main__':
    unittest.main()
