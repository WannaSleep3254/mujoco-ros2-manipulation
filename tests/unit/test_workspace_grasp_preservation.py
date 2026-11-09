"""Native/event originals survive export and remain tied to the checked bundle."""
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import tempfile
import unittest
from unittest.mock import patch

from mujoco_ros2_manipulation.paths import ROOT
from mujoco_ros2_manipulation.operator_console.synchronized_recording import SyncRecording
from tests.integration import verify_workspace_grasp as verification
from tests.unit.test_grasp_synchronized_recording import grasp_fixture


class WorkspaceGraspPreservationTests(unittest.TestCase):
    def prepare(self, *, media=True):
        base=ROOT/'runtime/validation/grasp-synchronized-recording-20261110/source-preservation-fixtures'
        base.mkdir(parents=True,exist_ok=True)
        root=Path(tempfile.mkdtemp(prefix='case-',dir=base))
        session=root/'runtime/operator/20261110T000000Z-abcdef12'
        session.mkdir(parents=True)
        output=root/'runtime/validation/test'
        output.mkdir(parents=True)
        (output/'console.log').write_text(
            f'OPERATOR_READY http://127.0.0.1:8934/ domain=220 session={session}\n')
        document,assets,images=grasp_fixture(media=media)
        recording=SyncRecording(document,assets,images)
        capture='a'*32
        recording.save(session/'synchronized'/capture/'recording.json')
        jid=document['identity']['job_id']
        source=session.with_name(session.name+'-grasp')/jid
        (source/'run/capture').mkdir(parents=True)
        events=[{key:value for key,value in event.items()
                 if key!='parent_received_monotonic_ns'} for event in document['events']]
        (source/'run/capture/latest.json').write_text(json.dumps({'events':events}))
        if document['poses']:
            native=[{'sequence':row['sequence'],'stamp_ns':row['stamp_ns'],
                     'time_sec':row['native']['time_sec'],
                     'capture_monotonic_ns':row['native']['capture_monotonic_ns'],
                     'arm_positions_rad':row['positions_rad']} for row in document['poses']]
            payload=''.join(json.dumps(row)+'\n' for row in native).encode()
            (source/'run/native-observations.jsonl').write_bytes(payload)
            result={'native_observations_sha256':hashlib.sha256(payload).hexdigest()}
        else:
            result={}
        (source/'run/result.json').write_text(json.dumps(result))
        result_sha=hashlib.sha256((source/'run/result.json').read_bytes()).hexdigest()
        receipt={'status':'passed','checks':[
            {'name':'active_owner_blocks_switch_and_rejects_wrong_cancel','job_id':jid},
            {'name':'native_result_and_owned_cleanup','job':{'worker_result_sha256':result_sha}}]}
        return root,output,source,capture,recording,receipt

    def preserve(self, data):
        root,output,source,capture,recording,receipt=data
        with patch.object(verification,'ROOT',root):
            verification.preserve_grasp_originals(output,receipt,capture,port=8934,domain=220)
        return receipt

    def test_complete_native_event_rows_match_original_bytes(self):
        data=self.prepare()
        receipt=self.preserve(data)
        proof=receipt['grasp_source_originals']
        self.assertEqual(receipt['status'],'passed')
        self.assertEqual(proof['status'],'preserved')
        self.assertEqual(proof['original_event_rows_matched'],len(data[4].events))
        self.assertEqual(proof['original_native_pose_rows_matched'],len(data[4].poses))

    def test_event_only_cancel_needs_no_native_pose_file(self):
        data=self.prepare(media=False)
        receipt=self.preserve(data)
        self.assertEqual(receipt['status'],'passed')
        self.assertEqual(receipt['grasp_source_originals']['original_native_pose_rows_matched'],0)

    def test_source_event_mismatch_is_retained_and_fails(self):
        data=self.prepare()
        path=data[2]/'run/capture/latest.json'
        value=json.loads(path.read_bytes())
        value['events'][0]['stage']='different_source_event'
        path.write_text(json.dumps(value))
        receipt=self.preserve(data)
        self.assertEqual(receipt['status'],'failed')
        copied=data[1]/'grasp-originals'/data[4].identity['job_id']/'run/capture/latest.json'
        self.assertEqual(copied.read_bytes(),path.read_bytes())
        self.assertIn('original worker journal',receipt['grasp_source_originals']['errors'][0]['message'])

    def test_linked_native_original_is_rejected(self):
        data=self.prepare()
        path=data[2]/'run/native-observations.jsonl'
        outside=data[0]/'kept-original.jsonl'
        path.rename(outside)
        path.symlink_to(outside)
        receipt=self.preserve(data)
        self.assertEqual(receipt['status'],'failed')
        self.assertTrue(outside.is_file())

    def test_corrupt_transport_jpeg_and_partial_write_survive_unchanged(self):
        data=self.prepare(media=False)
        raw=b"corrupt original JPEG bytes"
        folder=data[2]/'run/capture'
        (folder/('d'*64+'.jpg')).write_bytes(raw)
        (folder/'image-writing').write_bytes(b"partial encoded prefix")
        receipt=self.preserve(data)
        self.assertEqual(receipt['status'],'passed')
        kept=data[1]/'grasp-originals'/data[4].identity['job_id']/'run/capture'
        self.assertEqual((kept/('d'*64+'.jpg')).read_bytes(),raw)
        self.assertEqual((kept/'image-writing').read_bytes(),b"partial encoded prefix")

    def test_native_result_hash_mismatch_fails_without_losing_bytes(self):
        data=self.prepare()
        path=data[2]/'run/native-observations.jsonl'
        original=path.read_bytes()
        path.write_bytes(original+b" ")
        receipt=self.preserve(data)
        self.assertEqual(receipt['status'],'failed')
        kept=data[1]/'grasp-originals'/data[4].identity['job_id']/'run/native-observations.jsonl'
        self.assertEqual(kept.read_bytes(),original+b" ")
        self.assertIn('Native result hash differs',receipt['grasp_source_originals']['errors'][0]['message'])

    def test_retained_result_must_match_confirmed_http_identity(self):
        data=self.prepare()
        path=data[2]/'run/result.json'
        value=json.loads(path.read_bytes())
        value['unexpected_changed_result']=True
        path.write_text(json.dumps(value))
        receipt=self.preserve(data)
        self.assertEqual(receipt['status'],'failed')
        self.assertIn('confirmed HTTP identity',receipt['grasp_source_originals']['errors'][0]['message'])

    def test_job_traversal_is_rejected_before_copy(self):
        data=self.prepare()
        data[5]['checks'][0]['job_id']='../../outside'
        receipt=self.preserve(data)
        self.assertEqual(receipt['status'],'failed')
        self.assertFalse((data[1]/'grasp-originals').exists())


if __name__=='__main__':
    unittest.main()
