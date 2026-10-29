"""Opaque C ABI: offline FK and signed cross-arm mesh distances in native MuJoCo."""
import ctypes as C
import math
from collections.abc import Mapping
from threading import RLock
from mujoco_ros2_manipulation.model_build.integrity import file_sha256

MAX_ENVIRONMENT_PAIRS = 4096
MAX_SELF_PAIRS = 4096


def _finite_scalar(value):
    try:
        return type(value) in (int, float) and math.isfinite(value)
    except OverflowError:
        return False


class DistanceEvaluator:
    def __init__(self, cell, *, environment=False, self_collision=False):
        self._lock = RLock()
        if type(environment) is not bool or type(self_collision) is not bool:
            raise ValueError("environment and self_collision must be booleans")
        self.environment_enabled = environment
        self.self_collision_enabled = self_collision
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
        if environment or self_collision:
            try:
                self.api.path_distance_extensions_invalidate.argtypes = [C.c_void_p]
                self.api.path_distance_extensions_invalidate.restype = None
            except BaseException:
                self.close()
                raise
        if environment:
            try:
                self.api.path_distance_environment_source.restype = C.c_char_p
                self.environment_source_sha256 = file_sha256(cell.root / "native/path_distance_environment.cpp")
                if self.api.path_distance_environment_source().decode() != self.environment_source_sha256:
                    raise ValueError("Stale environment distance evaluator; rebuild native runtime")
                self.api.path_distance_environment_enable.argtypes = [
                    C.c_void_p, C.POINTER(C.c_char_p), C.POINTER(C.c_int), C.c_char_p, C.c_int]
                self.api.path_distance_environment_enable.restype = C.c_int
                self.api.path_distance_environment_eval.argtypes = [
                    C.c_void_p, C.POINTER(C.c_double), C.POINTER(C.c_int)]
                self.api.path_distance_environment_eval.restype = C.c_int
                counts = (C.c_int * 4)()
                if not self.api.path_distance_environment_enable(self.handle, encoded(roots), counts, error, len(error)):
                    raise ValueError(error.value.decode())
                if not (all(value > 0 for value in counts[:3])
                        and 1 <= counts[3] <= MAX_ENVIRONMENT_PAIRS):
                    raise ValueError("Invalid bounded environment collision pair coverage")
                self.environment_counts = tuple(counts)
            except BaseException:
                self.close()
                raise
        if self_collision:
            try:
                self.api.path_distance_self_source.restype = C.c_char_p
                self.self_source_sha256 = file_sha256(cell.root / "native/path_distance_self.cpp")
                if self.api.path_distance_self_source().decode() != self.self_source_sha256:
                    raise ValueError("Stale self-collision distance evaluator; rebuild native runtime")
                # The aggregator owns the shared-FK readiness and close hooks.
                self.api.path_distance_environment_source.restype = C.c_char_p
                self.self_aggregator_source_sha256 = file_sha256(cell.root / "native/path_distance_environment.cpp")
                if self.api.path_distance_environment_source().decode() != self.self_aggregator_source_sha256:
                    raise ValueError("Stale self-collision snapshot aggregator; rebuild native runtime")
                self.api.path_distance_self_enable.argtypes = [
                    C.c_void_p, C.POINTER(C.c_char_p), C.POINTER(C.c_int), C.c_char_p, C.c_int]
                self.api.path_distance_self_enable.restype = C.c_int
                self.api.path_distance_self_eval.argtypes = [C.c_void_p, C.POINTER(C.c_double), C.POINTER(C.c_int)]
                self.api.path_distance_self_eval.restype = C.c_int
                counts = (C.c_int * 4)()
                if not self.api.path_distance_self_enable(self.handle, encoded(roots), counts, error, len(error)):
                    raise ValueError(error.value.decode())
                if not (all(value > 0 for value in counts) and 1 <= counts[2]+counts[3] <= MAX_SELF_PAIRS):
                    raise ValueError("Invalid bounded self-collision pair coverage")
                self.self_counts = tuple(counts)
            except BaseException:
                self.close()
                raise

    def close(self):
        with self._lock:
            if getattr(self, "handle", None):
                self.api.path_distance_close(self.handle)
                self.handle = None

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()

    def at(self, state):
        # ctypes releases the GIL. Base FK, both extension reads and close must
        # therefore be one transaction over the same native mjData snapshot.
        with self._lock:
            return self._at(state)

    def _at(self, state):
        if (
            not self.handle
            or not isinstance(state, Mapping)
            or set(state) != set(self.names)
            or not all(_finite_scalar(v) for v in state.values())
        ):
            if self.handle and (self.environment_enabled or self.self_collision_enabled):
                self.api.path_distance_extensions_invalidate(self.handle)
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
        result = dict(
            distance_m=output[0],
            bodies=bodies,
            nearest_points_m=[list(output[1:4]), list(output[4:7])],
            tips_m=[list(output[7:10]), list(output[10:13])],
        )
        if self.environment_enabled:
            environment_output = (C.c_double * 7)()
            environment_pair = (C.c_int * 2)()
            if not self.api.path_distance_environment_eval(self.handle, environment_output, environment_pair):
                self.api.path_distance_extensions_invalidate(self.handle)
                raise ValueError("Native environment distance evaluation failed")
            result["environment"] = dict(
                distance_m=environment_output[0],
                bodies=[self.api.path_distance_body(self.handle, geom).decode() for geom in environment_pair],
                nearest_points_m=[list(environment_output[1:4]), list(environment_output[4:7])],
                pair_count=self.environment_counts[3],
                scope="moving_robot_vs_fixed_environment",
            )
        if self.self_collision_enabled:
            self_output, self_pair = (C.c_double * 7)(), (C.c_int * 2)()
            if not self.api.path_distance_self_eval(self.handle, self_output, self_pair):
                self.api.path_distance_extensions_invalidate(self.handle)
                raise ValueError("Native self-collision distance evaluation failed")
            result["self_collision"] = dict(
                distance_m=self_output[0], geom_ids=list(self_pair),
                bodies=[(self.api.path_distance_body(self.handle, geom) or f"geom:{geom}".encode()).decode()
                        for geom in self_pair],
                nearest_points_m=[list(self_output[1:4]), list(self_output[4:7])],
                pair_count=sum(self.self_counts[2:]), scope="within_each_arm",
            )
        return result

    def metadata(self):
        result = dict(
            engine="MuJoCo " + self.version,
            source_sha256=self.source_sha256,
            library_sha256=self.library_sha256,
            distance_scope="All collision-enabled geom pairs belonging to the two robot base subtrees",
            tip_frames=self.frames,
            tip_definition="URDF tip frame origin; no physical tool/TCP offset assumed",
            geometry="MuJoCo compiled collision shapes; meshes use convex collision approximation",
        )
        if self.environment_enabled:
            result["environment"] = dict(
                scope="moving_robot_vs_fixed_environment",
                pair_count=self.environment_counts[3],
                max_pair_count=MAX_ENVIRONMENT_PAIRS,
                source_sha256=self.environment_source_sha256,
                moving_geom_count_by_arm=list(self.environment_counts[:2]),
                fixed_geom_count=self.environment_counts[2],
                distance_semantics="Signed closest moving-arm/fixed-world collision geometry distance; sampled, not continuous-time certified",
                filtering="Native contype/conaffinity and body exclusions; static world-welded robot mounts excluded",
                unsupported="Movable bodies outside robot roots, flex, hfield/SDF collision geometry and explicit contact geom pairs",
                fk="Reuses the same native FK snapshot as the cross-arm result; no second mj_forward",
            )
        if self.self_collision_enabled:
            result["self_collision"] = dict(
                scope="within_each_arm", pair_count=sum(self.self_counts[2:]),
                geom_count_by_arm=list(self.self_counts[:2]), pair_count_by_arm=list(self.self_counts[2:]),
                max_pair_count=MAX_SELF_PAIRS, source_sha256=self.self_source_sha256,
                aggregator_source_sha256=self.self_aggregator_source_sha256,
                library_sha256=self.library_sha256, engine="MuJoCo " + self.version,
                distance_semantics="Signed closest eligible same-arm geometry distance; sampled, not continuous-time certified",
                filtering="Native contype/conaffinity, same weld, body exclusions and parent-weld filtering with world exception",
                unsupported="Flex, mocap, executable plugins, hfield/SDF collision geometry and explicit contact geom pairs",
                fk="Reuses the same successful base-evaluator FK snapshot; no second mj_forward",
            )
        return result
