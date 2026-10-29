"""Startup-only SHA-paired selection of an immutable ROS grasp execution model."""
from dataclasses import dataclass, field
from copy import deepcopy
import json
from pathlib import Path
import re

from .ros_model import check_model, _canonical, FINGER_JOINTS, ARTIFACT


@dataclass(frozen=True)
class RosGraspTarget:
    base: object = field(repr=False,compare=False)
    output: Path
    manifest_sha256: str
    _report: bytes = field(repr=False)

    @property
    def id(self):return json.loads(self._report)["id"]
    @property
    def fingerprint(self):return json.loads(self._report)["fingerprint"]
    @property
    def root(self):return self.base.root
    @property
    def instances(self):return self.base.instances
    @property
    def joints(self):return self.base.joints
    @property
    def home(self):return json.loads(self._report)["home_rad"]
    @property
    def model_path(self):return self.output/"cell.xml"
    @property
    def finger_joints(self):return list(FINGER_JOINTS)
    @property
    def allowed_scalar_joints(self):return list(self.joints)+list(FINGER_JOINTS)
    @property
    def manifest(self):return json.loads(self._report)

    def description(self):
        value=json.loads(self._report)
        return dict(schema_version=1,artifact_kind=ARTIFACT,id=self.id,fingerprint=self.fingerprint,
            manifest_sha256=self.manifest_sha256,base_workcell_id=self.base.id,
            arm_joint_names=list(self.joints),finger_joint_names=list(FINGER_JOINTS),
            scalar_joint_units=deepcopy(value["scalar_joint_units"]),qpos_state_contract=deepcopy(value["qpos_state_contract"]),
            tool=deepcopy(value["tool"]),block=deepcopy(value["block"]),table=deepcopy(value["table"]),
            contact_policy=deepcopy(value["contact_policy"]),outputs_sha256=deepcopy(value["outputs"]),
            physics_solver=deepcopy(value["physics_solver"]),
            selection="explicit_local_startup_only",native_execution_verified=False)

    def validate(self):
        value=check_model(self.output,self.base,expected_sha256=self.manifest_sha256)
        if _canonical(value)!=self._report:
            raise ValueError("ROS grasp artifact changed; restart with a checked target")
        return self.description()


def load_ros_grasp_target(base,directory=None,manifest_sha256=None):
    if directory is None and manifest_sha256 is None:return None
    if directory is None or type(manifest_sha256) is not str or re.fullmatch(r"[0-9a-f]{64}",manifest_sha256) is None:
        raise ValueError("ROS grasp requires an explicit directory and manifest SHA256 pair")
    directory=Path(directory).absolute()
    value=check_model(directory,base,expected_sha256=manifest_sha256)
    target=RosGraspTarget(base,directory,manifest_sha256,_canonical(value))
    target.validate()
    return target
