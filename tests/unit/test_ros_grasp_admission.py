from copy import deepcopy
import io
import json
from pathlib import Path
import tempfile
import subprocess
import signal
import sys
import os
from unittest.mock import patch
import time
import unittest

from mujoco_ros2_manipulation.grasping.ros_admission import (
    AcceptedObservationRecorder, ReadOnlyAdmissionWorker, file_stats,
)
from mujoco_ros2_manipulation.grasping.ros_observation import ObservationUnavailable
from mujoco_ros2_manipulation.grasping.ros_task import (
    RosGraspTaskError, admitted_action_submission, verify_identity_proof,
)


FIXTURE = r"""
import json,sys,time,subprocess,signal
from pathlib import Path
from mujoco_ros2_manipulation.grasping.ros_admission import file_stats
from mujoco_ros2_manipulation.verification.reports import atomic_json
configuration=Path(sys.argv[1]);config=json.loads(configuration.read_text());child=None
try:
 for line in sys.stdin:
  request=json.loads(line);generation=request['generation'];mode=config.get('fixture_mode','slow')
  if mode=='nested':
   child=subprocess.Popen([sys.executable,'-c','import time;time.sleep(30)'])
  atomic_json(configuration.parent/'entered.json',dict(generation=generation,child_pid=child.pid if child else None))
  while mode in ('slow','nested') and not (configuration.parent/'release').exists():time.sleep(.001)
  stats=file_stats(config['inventory'])
  proof=dict(token=request['token'],generation=generation,manifest_sha256=config['manifest_sha256'],
    observed_sha256=config['input_hashes'],before=stats,after=stats)
  if mode=='changed':proof['observed_sha256']={'s':'late mutation'}
  if mode=='wrong_generation':proof['generation']+=1
  atomic_json(configuration.parent/f'proof-{generation:03}.json',proof)
except KeyboardInterrupt:pass
finally:
 if child is not None:
  if child.poll() is None:child.send_signal(signal.SIGINT)
  try:child.wait(timeout=.5)
  except subprocess.TimeoutExpired:child.kill();child.wait()
"""


class PumpedAdmissionTests(unittest.TestCase):
    def setUp(self):
        self.folder=tempfile.TemporaryDirectory();self.addCleanup(self.folder.cleanup)
        self.base=Path(self.folder.name)
        self.source=self.base/'source';self.source.write_bytes(b'original')
        self.fake=self.base/'fixed-fixture.py';self.fake.write_text(FIXTURE)
        self.original_popen=subprocess.Popen
        def launch(command,**kwargs):
            self.assertEqual(command[:4],[sys.executable,'-m','mujoco_ros2_manipulation.grasping.ros_admission','--identity-worker'])
            return self.original_popen([sys.executable,str(self.fake),command[-1]],**kwargs)
        self.launch=patch('mujoco_ros2_manipulation.grasping.ros_admission.subprocess.Popen',side_effect=launch)
        self.launch.start();self.addCleanup(self.launch.stop)

    def worker(self,mode='slow',clock=time.monotonic):
        worker=ReadOnlyAdmissionWorker(dict(target_directory='unused by fixture',manifest_sha256='m',
            input_hashes={'s':'h'},inventory=[str(self.source)],fixture_mode=mode),self.base/'ipc',clock=clock)
        self.addCleanup(worker.close)
        return worker

    def entered(self,worker):
        until=time.monotonic()+1.
        while not (worker.directory/'entered.json').exists() and time.monotonic()<until:time.sleep(.001)
        self.assertTrue((worker.directory/'entered.json').exists())
        return json.loads((worker.directory/'entered.json').read_text())

    def test_simulated_six_hundred_ms_read_starves_sync_path_but_pumped_new_frames_allow_one_goal(self):
        # A deterministic starvation path, not a claim about CI143's unmeasured layer.
        wall,capture,sequence,sent=[0.],[0.],[1],[]
        def abort():
            if wall[0]-capture[0]>.5:raise ObservationUnavailable('source capture stale')
        def synchronous():wall[0]+=.6
        with self.assertRaises(ObservationUnavailable):
            admitted_action_submission(abort,lambda:None,synchronous,lambda:sent.append('old'))
        wall[0]=capture[0]=0.
        worker=self.worker(clock=lambda:wall[0])
        def pump():
            self.entered(worker);wall[0]+=.1
            sequence[0]+=1;capture[0]=wall[0]  # genuinely new capture; original timestamp retained
            if wall[0]>=.6:(worker.directory/'release').touch()
            time.sleep(.001)
        admitted_action_submission(abort,lambda:None,
            lambda:worker.check(pump=pump,abort=abort),lambda:sent.append('new'))
        self.assertEqual(sent,['new']);self.assertGreaterEqual(sequence[0],7)
        closed=worker.close();self.assertTrue(closed['quiescent']);self.assertEqual(closed['leader_return_code'],0)

    def test_missing_native_capture_during_read_still_blocks_zero_goal(self):
        wall,sent=[0.],[];worker=self.worker(clock=lambda:wall[0])
        def abort():
            if wall[0]>.5:raise ObservationUnavailable('source capture stale')
        def pump():self.entered(worker);wall[0]+=.1;time.sleep(.001)
        with self.assertRaises(ObservationUnavailable):
            admitted_action_submission(abort,lambda:None,
                lambda:worker.check(pump=pump,abort=abort),lambda:sent.append(1))
        self.assertEqual(sent,[]);self.assertTrue(worker.snapshot()['abandoned'])
        self.assertTrue(worker.close()['quiescent'])

    def test_cancel_before_validation_starts_no_request_or_goal(self):
        worker=self.worker()
        def cancelled():raise RuntimeError('cancelled before validation')
        with self.assertRaisesRegex(RuntimeError,'cancelled before'):
            worker.check(pump=lambda:None,abort=cancelled)
        self.assertEqual(worker.snapshot()['generation'],0)
        self.assertFalse((worker.directory/'entered.json').exists())
        self.assertTrue(worker.close()['quiescent'])

    def test_cancel_during_nested_child_read_reaps_whole_group_and_ignores_late_changed_proof(self):
        cancelled,sent=[False],[];worker=self.worker('nested')
        def abort():
            if cancelled[0]:raise RuntimeError('operator cancelled')
        def pump():self.assertIsInstance(self.entered(worker)['child_pid'],int);cancelled[0]=True
        with self.assertRaisesRegex(RuntimeError,'operator cancelled'):
            admitted_action_submission(abort,lambda:None,
                lambda:worker.check(pump=pump,abort=abort),lambda:sent.append(1))
        self.assertEqual(sent,[])
        self.assertTrue(worker.close()['process_group_gone'])
        (worker.directory/'proof-001.json').write_text('{"generation":1,"observed_sha256":{"s":"late mutation"}}')
        with self.assertRaisesRegex(ValueError,'closed, abandoned'):
            worker.check(pump=lambda:None,abort=lambda:None)
        self.assertTrue(worker.snapshot()['late_completion_ignored'])

    def test_timeout_and_nested_request_cannot_authorize_or_leave_group(self):
        wall=[0.];worker=self.worker('nested',clock=lambda:wall[0])
        def pump():
            self.entered(worker)
            with self.assertRaisesRegex(ValueError,'busy'):worker.check(pump=lambda:None,abort=lambda:None)
            wall[0]=7.
        with self.assertRaises(TimeoutError):worker.check(pump=pump,abort=lambda:None)
        result=worker.close();self.assertTrue(result['quiescent']);self.assertTrue(result['process_group_gone'])

    def test_changed_or_reused_process_identity_is_never_signalled(self):
        worker=self.worker()
        changed=dict(worker._leader,starttime=worker._leader['starttime']+1)
        with patch('mujoco_ros2_manipulation.grasping.ros_admission._member_identity',return_value=changed), \
                patch('mujoco_ros2_manipulation.grasping.ros_admission.os.killpg') as signal_group:
            worker._interrupt()
            # killpg(...,0) only probes; no SIGINT/TERM/KILL may target the new identity.
            self.assertTrue(all(call.args[1]==0 for call in signal_group.call_args_list))
            self.assertIn('signal refused',worker.snapshot()['ownership_error'])
        self.assertTrue(worker.close()['quiescent'])

    def test_owner_rejects_wrong_generation_before_submit(self):
        worker=self.worker('wrong_generation');sent=[]
        with self.assertRaisesRegex(ValueError,'generation'):
            admitted_action_submission(lambda:None,lambda:None,
                lambda:worker.check(pump=lambda:time.sleep(.001),abort=lambda:None),lambda:sent.append(1))
        self.assertEqual(sent,[]);self.assertTrue(worker.close()['quiescent'])

    def test_owner_rejects_changed_digest_and_file_mutation_after_full_proof(self):
        worker=self.worker('changed');sent=[]
        proof=worker.check(pump=lambda:time.sleep(.001),abort=lambda:None)
        def verify(value):verify_identity_proof(value,manifest_sha256='m',expected_hashes={'s':'h'},inventory=[str(self.source)])
        with self.assertRaisesRegex(RosGraspTaskError,'changed before'):
            admitted_action_submission(lambda:None,lambda:None,lambda:verify(proof),lambda:sent.append(1))
        proof['observed_sha256']={'s':'h'};verify(proof)
        self.source.write_bytes(b'changed after proof')
        with self.assertRaisesRegex(RosGraspTaskError,'changed after'):
            admitted_action_submission(lambda:None,lambda:None,lambda:verify(proof),lambda:sent.append(1))
        self.assertEqual(sent,[]);self.assertTrue(worker.close()['quiescent'])

    def test_every_accepted_frame_is_logged_even_if_later_snapshot_is_ineligible(self):
        frames,receipts=io.BytesIO(),io.BytesIO()
        recorder=AcceptedObservationRecorder(frames,receipts,max_frame_bytes=1024,max_receipt_bytes=2048,clock_ns=lambda:700_000_001)
        value=dict(sequence=94,capture_monotonic_ns='200000000',stamp_ns='4958000000')
        def expired():raise ObservationUnavailable('source capture is stale')
        self.assertIsNone(recorder.record(value,received_ns=690_000_000,eligible_snapshot=expired))
        self.assertEqual(json.loads(frames.getvalue()),value)
        trace=json.loads(receipts.getvalue());self.assertFalse(trace['eligible'])
        self.assertFalse(trace['physical_authority_claimed'])
        self.assertEqual(trace['capture_monotonic_ns'],200_000_000)
        self.assertEqual(trace['received_monotonic_ns'],690_000_000)
        self.assertIn('stale',trace['eligibility_error'])
        eligible=object()
        next_value=dict(value,sequence=95,capture_monotonic_ns='700000000')
        self.assertIs(recorder.record(next_value,received_ns=700_000_001,eligible_snapshot=lambda:eligible),eligible)
        self.assertEqual([json.loads(line)['sequence'] for line in frames.getvalue().splitlines()],[94,95])
        self.assertEqual([json.loads(line)['eligible'] for line in receipts.getvalue().splitlines()],[False,True])

    def test_frame_and_receipt_limits_are_hard_errors(self):
        value=dict(sequence=1,capture_monotonic_ns='1')
        for frame_limit,receipt_limit in ((1,1024),(1024,1)):
            recorder=AcceptedObservationRecorder(io.BytesIO(),io.BytesIO(),max_frame_bytes=frame_limit,max_receipt_bytes=receipt_limit)
            with self.assertRaisesRegex(ValueError,'limit exceeded'):
                recorder.record(value,received_ns=2,eligible_snapshot=lambda:object())
