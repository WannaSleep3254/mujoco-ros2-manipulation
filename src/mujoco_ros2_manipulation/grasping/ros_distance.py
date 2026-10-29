"""Native3.12 offline clearance over one complete ROS grasp observation.

The optional held pose predicts swept block geometry; physical confirmation and
MoveIt attachment remain separate authorities. No live qpos is written here.
"""
import ctypes as C
import math
import threading

from ..model_build.integrity import file_sha256
from .ros_model import ARTIFACT


class GraspDistanceEvaluator:
    def __init__(self, target):
        if target.description().get("artifact_kind") != ARTIFACT:
            raise ValueError("Grasp distance requires an explicit checked grasp target")
        target.validate()
        self.target = target
        self.model_sha256 = file_sha256(target.model_path)
        self._lock = threading.RLock()
        self.handle = None
        library = target.root / "runtime/native/grasp_distance.so"
        self.api = C.CDLL(str(library))
        for name in ("grasp_distance_source", "grasp_distance_policy_source", "grasp_distance_version"):
            getattr(self.api, name).restype = C.c_char_p
        self.source_sha256 = file_sha256(target.root / "native/grasp_distance.cpp")
        self.policy_sha256 = file_sha256(target.root / "native/grasp_policy.hpp")
        if (self.api.grasp_distance_source().decode() != self.source_sha256
                or self.api.grasp_distance_policy_source().decode() != self.policy_sha256
                or self.api.grasp_distance_version().decode() != "3.12.0"):
            raise ValueError("Stale or incompatible native grasp distance evaluator")
        self.library_sha256 = file_sha256(library)
        self.api.grasp_distance_open.argtypes = [C.c_char_p, C.c_char_p, C.c_int]
        self.api.grasp_distance_open.restype = C.c_void_p
        self.api.grasp_distance_close.argtypes = [C.c_void_p]
        self.api.grasp_distance_pair_count.argtypes = [C.c_void_p]
        self.api.grasp_distance_pair_count.restype = C.c_int
        self.api.grasp_distance_geom.argtypes = [C.c_void_p, C.c_int]
        self.api.grasp_distance_geom.restype = C.c_char_p
        self.api.grasp_distance_eval.argtypes = [C.c_void_p, C.POINTER(C.c_double), C.c_int,
            C.c_int, C.POINTER(C.c_double), C.POINTER(C.c_double), C.POINTER(C.c_int), C.POINTER(C.c_double)]
        self.api.grasp_distance_eval.restype = C.c_int
        error = C.create_string_buffer(2048)
        self.handle = self.api.grasp_distance_open(str(target.model_path).encode(), error, len(error))
        if not self.handle:
            raise ValueError(error.value.decode())
        self.pair_count = self.api.grasp_distance_pair_count(self.handle)
        if not 1 <= self.pair_count <= 4096:
            self.close()
            raise ValueError("Invalid grasp collision pair coverage")
        self.layout = {entry["name"]: entry for entry in target.manifest["qpos_state_contract"]["layout"]}

    def close(self):
        with self._lock:
            if self.handle:
                self.api.grasp_distance_close(self.handle)
                self.handle = None

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()

    @staticmethod
    def _vector(values, size):
        if (not isinstance(values, (list, tuple)) or len(values) != size
                or not all(type(v) in (int, float) and math.isfinite(v) for v in values)):
            raise ValueError("Expected complete finite full-state geometry")
        return list(values)

    def at(self, qpos, *, phase, arm_state=None, held_relative_pose=None):
        with self._lock:
            return self._at(qpos, phase=phase, arm_state=arm_state, held_relative_pose=held_relative_pose)

    def _at(self, qpos, *, phase, arm_state=None, held_relative_pose=None):
        values = self._vector(qpos, 21)
        if type(phase) is not int or not 0 <= phase <= 7 or not self.handle:
            raise ValueError("Expected open evaluator and supported grasp phase")
        if arm_state is not None:
            if (not isinstance(arm_state, dict) or set(arm_state) != set(self.target.joints)
                    or not all(type(v) in (int, float) and math.isfinite(v) for v in arm_state.values())):
                raise ValueError("Expected complete twelve-arm candidate in radians")
            for name, value in arm_state.items():
                values[self.layout[name]["qpos_address"]] = value
        relative = None if held_relative_pose is None else (C.c_double*7)(*self._vector(held_relative_pose, 7))
        output, pair, tcp = (C.c_double*7)(), (C.c_int*2)(), (C.c_double*7)()
        if not self.api.grasp_distance_eval(self.handle, (C.c_double*21)(*values), 21,
                phase, relative, output, pair, tcp):
            raise ValueError("Native full-state grasp candidate rejected")
        names = [self.api.grasp_distance_geom(self.handle, g) for g in pair]
        return dict(distance_m=output[0], geom_ids=list(pair),
            geoms=[name.decode() if name else "geom:"+str(g) for name, g in zip(names, pair)],
            nearest_points_m=[list(output[1:4]), list(output[4:7])],
            tcp_pose=list(tcp), phase=phase, pair_count=self.pair_count,
            prediction="held_block_swept_geometry" if relative is not None else "observed_free_block")

    def metadata(self):
        return dict(engine="MuJoCo 3.12.0", source_sha256=self.source_sha256,
            policy_sha256=self.policy_sha256, library_sha256=self.library_sha256,
            model_sha256=self.model_sha256, pair_count=self.pair_count,
            scope="Both arms, fingers, free block and fixed world; same-weld/parent, masks and explicit body exclusions applied",
            allowed_contacts="Exact block/finger and block/support pairs for the current checked task phase",
            sampling="Discrete candidate FK and compiled convex mesh geometry; not continuous-time collision certification",
            physical_attachment=False)
