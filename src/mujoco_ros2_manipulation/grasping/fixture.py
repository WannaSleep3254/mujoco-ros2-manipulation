"""Small Cartesian test fixture and free block, independent of manufacturer CAD.

The x/z carriage is a purpose-built position-actuated test rig, not FR5 forward
kinematics, a FR5 controller, or proof of robot payload/contact performance.
The block is always free: no weld, attachment constraint, pose overwrite, or
parenting to the TCP is used to make a grasp appear successful.
"""
from dataclasses import asdict, dataclass
import hashlib
import json
import math


@dataclass(frozen=True)
class FixtureSpec:
    schema_version: int = 1
    timestep_sec: float = .002
    friction: float = .8
    block_mass_kg: float = .08
    block_xyz_m: tuple = (0., 0., .025)
    block_present: bool = True
    right_finger_offset_y_m: float = 0.

    def __post_init__(self):
        if type(self.schema_version) is not int or self.schema_version != 1:
            raise ValueError("Expected synthetic grasp fixture schema 1")
        for name, bounds in (("timestep_sec", (.0005, .005)),
                             ("friction", (0., 2.)), ("block_mass_kg", (.001, 2.)),
                             ("right_finger_offset_y_m", (0., .1))):
            value = getattr(self, name)
            if type(value) not in (int, float) or not math.isfinite(value) or not bounds[0] <= value <= bounds[1]:
                raise ValueError("Invalid fixture parameter: " + name)
        if type(self.block_present) is not bool:
            raise ValueError("block_present must be boolean")
        if (type(self.block_xyz_m) not in (tuple, list) or len(self.block_xyz_m) != 3
                or any(type(v) not in (int, float) or not math.isfinite(v) or abs(v) > 1
                       for v in self.block_xyz_m)):
            raise ValueError("Expected a bounded metre block position")
        object.__setattr__(self, "block_xyz_m", tuple(self.block_xyz_m))

    def manifest(self):
        return dict(schema_version=1, fixture="synthetic_parallel_gripper_v1",
            simulation_only=True, carriage="position-actuated Cartesian x/z test rig; not FR5",
            length_unit="metre", mass_unit="kilogram", force_unit="newton",
            quaternion_order="wxyz", world_frame="fixture_world",
            tcp=dict(site="grasp_tcp", parent_body="carriage", xyz_m=[0., 0., -.09],
                     quaternion_wxyz=[1., 0., 0., 0.]),
            gripper=dict(finger_geom_names=["left_finger_geom", "right_finger_geom"],
                gap_axis="fixture x", finger_half_size_m=[.005, .02, .04],
                open_inner_gap_m=.07, joint_range_m=[0., .034],
                finger_actuator_force_limit_n=20.),
            object=dict(body="block" if self.block_present else None,
                geom="block_geom" if self.block_present else None,
                half_size_m=[.02, .015, .025], mass_kg=self.block_mass_kg,
                joint_type="free" if self.block_present else None),
            configuration=asdict(self),
            physical_attachment="none; free object and Coulomb contact friction only",
            planning_attachment="logical state only; does not apply a physical constraint")

    def xml(self):
        block = ""
        if self.block_present:
            pos = " ".join(format(v, ".17g") for v in self.block_xyz_m)
            block = f'''<body name="block" pos="{pos}"><freejoint name="block_free"/>
              <geom name="block_geom" type="box" size=".02 .015 .025" mass="{self.block_mass_kg}"
                rgba=".85 .5 .1 1" friction="{self.friction} .01 .001"/></body>'''
        return f'''<mujoco model="synthetic_parallel_gripper_v1">
          <compiler angle="radian"/>
          <option timestep="{self.timestep_sec}" integrator="implicitfast" iterations="100" impratio="10"/>
          <default><joint damping="1" armature=".001"/>
            <geom condim="4" friction="{self.friction} .01 .001" solref=".008 1" solimp=".95 .99 .001"/>
          </default>
          <worldbody>
            <geom name="table_geom" type="box" pos="0 0 -.025" size=".45 .25 .025" rgba=".3 .35 .4 1"/>
            <body name="carriage" pos="0 0 .21">
              <joint name="carriage_x" type="slide" axis="1 0 0" range="-.1 .3"/>
              <joint name="carriage_z" type="slide" axis="0 0 1" range="-.11 .1"/>
              <geom name="palm_geom" type="box" size=".055 .025 .01" mass=".5" rgba=".3 .5 .7 1"/>
              <site name="grasp_tcp" pos="0 0 -.09" size=".003" rgba="1 0 0 1"/>
              <body name="left_finger" pos="-.04 0 -.07">
                <joint name="left_finger_slide" type="slide" axis="1 0 0" range="0 .034"/>
                <geom name="left_finger_geom" type="box" size=".005 .02 .04" mass=".07" rgba=".7 .7 .75 1"/>
              </body>
              <body name="right_finger" pos=".04 {self.right_finger_offset_y_m} -.07">
                <joint name="right_finger_slide" type="slide" axis="-1 0 0" range="0 .034"/>
                <geom name="right_finger_geom" type="box" size=".005 .02 .04" mass=".07" rgba=".7 .7 .75 1"/>
              </body>
            </body>
            {block}
          </worldbody>
          <actuator>
            <position name="carriage_x_servo" joint="carriage_x" kp="4000" kv="100" forcelimited="true" forcerange="-100 100"/>
            <position name="carriage_z_servo" joint="carriage_z" kp="5000" kv="120" forcelimited="true" forcerange="-100 100"/>
            <position name="left_finger_servo" joint="left_finger_slide" kp="2000" kv="20" forcelimited="true" forcerange="-20 20"/>
            <position name="right_finger_servo" joint="right_finger_slide" kp="2000" kv="20" forcelimited="true" forcerange="-20 20"/>
          </actuator>
        </mujoco>'''

    def identity(self):
        manifest = self.manifest()
        manifest["mjcf_sha256"] = hashlib.sha256(self.xml().encode()).hexdigest()
        raw = json.dumps(manifest, sort_keys=True, separators=(",", ":"), allow_nan=False).encode()
        return dict(manifest, manifest_sha256=hashlib.sha256(raw).hexdigest())
