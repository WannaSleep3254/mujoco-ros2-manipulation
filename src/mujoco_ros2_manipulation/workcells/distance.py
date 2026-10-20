"""Opaque C ABI: offline FK and signed cross-arm mesh distances in native MuJoCo."""
import ctypes as C
import math
from mujoco_ros2_manipulation.model_build.integrity import file_sha256


class DistanceEvaluator:
    def __init__(self, cell):
        self.names = list(cell.joints)
        source = cell.root / "native/path_distance.cpp"
        library = cell.root / "runtime/native/path_distance.so"
        self.api = C.CDLL(str(library))
        for name in ("path_distance_source", "path_distance_version"):
            getattr(self.api, name).restype = C.c_char_p
        self.source_sha256 = file_sha256(source)
        if self.api.path_distance_source().decode() != self.source_sha256:
            raise ValueError("Stale distance evaluator; rebuild native runtime")
        self.version = self.api.path_distance_version().decode()
        if self.version != "3.12.0":
            raise ValueError("Distance evaluator must use pinned native MuJoCo 3.12.0")
        self.library_sha256 = file_sha256(library)
        self.api.path_distance_open.argtypes = [
            C.c_char_p,
            C.POINTER(C.c_char_p),
            C.c_int,
            C.POINTER(C.c_char_p),
            C.POINTER(C.c_char_p),
            C.c_char_p,
            C.c_int,
        ]
        self.api.path_distance_open.restype = C.c_void_p
        self.api.path_distance_close.argtypes = [C.c_void_p]
        self.api.path_distance_eval.argtypes = [
            C.c_void_p,
            C.POINTER(C.c_double),
            C.c_int,
            C.POINTER(C.c_double),
            C.POINTER(C.c_int),
        ]
        self.api.path_distance_eval.restype = C.c_int
        self.api.path_distance_body.argtypes = [C.c_void_p, C.c_int]
        self.api.path_distance_body.restype = C.c_char_p
        self.frames = [i.prefix + i.profile.ros["tip_frame"] for i in cell.instances]
        roots = [i.prefix + i.profile.ros["base_frame"] for i in cell.instances]
        encoded = lambda values: (C.c_char_p * len(values))(
            *(v.encode() for v in values)
        )
        error = C.create_string_buffer(2048)
        self.handle = self.api.path_distance_open(
            str(cell.output / "cell.xml").encode(),
            encoded(self.names),
            len(self.names),
            encoded(roots),
            encoded(self.frames),
            error,
            len(error),
        )
        if not self.handle:
            raise ValueError(error.value.decode())

    def close(self):
        if self.handle:
            self.api.path_distance_close(self.handle)
            self.handle = None

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()

    def at(self, state):
        if (
            not self.handle
            or set(state) != set(self.names)
            or not all(
                type(v) in (int, float) and math.isfinite(v) for v in state.values()
            )
        ):
            raise ValueError(
                "Expected a finite complete joint state and open evaluator"
            )
        values = (C.c_double * len(self.names))(*(state[n] for n in self.names))
        output = (C.c_double * 13)()
        pair = (C.c_int * 2)()
        if not self.api.path_distance_eval(
            self.handle, values, len(self.names), output, pair
        ):
            raise ValueError("Native distance evaluation failed")
        bodies = [self.api.path_distance_body(self.handle, g).decode() for g in pair]
        return dict(
            distance_m=output[0],
            bodies=bodies,
            nearest_points_m=[list(output[1:4]), list(output[4:7])],
            tips_m=[list(output[7:10]), list(output[10:13])],
        )

    def metadata(self):
        return dict(
            engine="MuJoCo " + self.version,
            source_sha256=self.source_sha256,
            library_sha256=self.library_sha256,
            distance_scope="All collision-enabled geom pairs belonging to the two robot base subtrees",
            tip_frames=self.frames,
            tip_definition="URDF tip frame origin; no physical tool/TCP offset assumed",
            geometry="MuJoCo compiled collision shapes; meshes use convex collision approximation",
        )
