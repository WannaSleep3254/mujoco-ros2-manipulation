"""Exercise the real fixed validator process and target/Git/hash checks, no ROS."""
from pathlib import Path
import tempfile
import time
import unittest

from mujoco_ros2_manipulation.paths import ROOT
from mujoco_ros2_manipulation.model_build.integrity import file_sha256
from mujoco_ros2_manipulation.workcells.config import load_workcell
from mujoco_ros2_manipulation.grasping.ros_model import build_model
from mujoco_ros2_manipulation.grasping.ros_target import load_ros_grasp_target
from mujoco_ros2_manipulation.grasping.ros_task import identity_inventory, verify_identity_proof
from mujoco_ros2_manipulation.grasping.ros_admission import ReadOnlyAdmissionWorker


class NativeTargetAdmissionTests(unittest.TestCase):
    def test_fixed_process_checks_real_target_git_sources_and_reaps_cleanly(self):
        base=load_workcell('fr5_ur5e')
        if not base.output.exists() and not base.output.is_symlink():
            self.skipTest('Generated FR5 + UR5e workcell required')
        with tempfile.TemporaryDirectory(prefix='identity-model-',dir=ROOT/'runtime') as folder:
            folder=Path(folder)
            build_model(base,folder/'model')
            manifest_sha=file_sha256(folder/'model/grasp.json')
            target=load_ros_grasp_target(base,folder/'model',manifest_sha)
            paths=[ROOT/'src/mujoco_ros2_manipulation/grasping'/name for name in ('ros_task.py','ros_admission.py')]
            hashes={str(p):file_sha256(p) for p in paths}
            inventory=identity_inventory(target,hashes)
            worker=ReadOnlyAdmissionWorker(dict(target_directory=str(target.output),manifest_sha256=manifest_sha,
                input_hashes=hashes,inventory=list(inventory)),folder/'ipc')
            try:
                proof=worker.check(pump=lambda:time.sleep(.002),abort=lambda:None,timeout_sec=15.)
                verify_identity_proof(proof,manifest_sha256=manifest_sha,expected_hashes=hashes,inventory=inventory)
                self.assertEqual(proof['generation'],1)
            finally:
                closed=worker.close()
            self.assertTrue(closed['quiescent']);self.assertTrue(closed['process_group_gone'])
            self.assertEqual(closed['leader_return_code'],0)
