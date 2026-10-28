"""Owned FR5 six-axis dynamics plus free-object grasp prototype, without ROS.

Run with ``python -m mujoco_ros2_manipulation.grasping.fr5_run --source SOURCE
--output NEW_DIRECTORY``. Damped least squares only proposes local Cartesian
waypoints. It is not a collision-free planner, MoveIt IK, a ROS controller or a
replacement for the existing native contact-stop policy.
"""
import argparse
from copy import deepcopy
import json
import math
from pathlib import Path
import time

from .contract import GraspLifecycle
from .fr5_model import ARM_JOINTS, BLOCK_GEOM, FINGER_GEOMS, TABLE_GEOM, TCP_SITE, write_model, sha256


class PrototypeFailure(RuntimeError):
    pass


class PrototypeCancelled(RuntimeError):
    """The owned local simulation observed a cancellation request."""


def compile_pinned_model(directory, manifest):
    """Reject changed compiler inputs before any physical data or step exists."""
    from .display_export import _manifest_digest, _validate_asset_inputs
    from ..operator_console.scene_host import _regular
    from ..operator_console.scene_view import SceneLimits
    import mujoco
    limits = SceneLimits()
    path = Path(directory)/"fr5_grasp.xml"
    raw = _regular(path, limits.max_xml_bytes)
    if (manifest.get("manifest_sha256") != _manifest_digest(manifest)
            or manifest.get("mjcf_sha256") != sha256(raw)
            or manifest.get("physical_engine_used_for_static_fk") != "MuJoCo "+mujoco.__version__):
        raise ValueError("Physical prototype model/manifest/engine identity differs")
    meshes = _validate_asset_inputs(raw, manifest, limits)
    # Compile the exact checked XML bytes; every permitted external mesh read is
    # pinned on both sides of compilation. No physics step occurs on mismatch.
    model = mujoco.MjModel.from_xml_string(raw.decode("utf-8"))
    if (_regular(path, limits.max_xml_bytes) != raw
            or _validate_asset_inputs(raw, manifest, limits) != meshes):
        raise ValueError("Physical prototype compiler inputs changed during compilation")
    return model


def solve_ik(model, observed_qpos, position_m, rotation, *, maximum_iterations=120):
    """Bounded local site-Jacobian DLS using separate nonphysical scratch data."""
    import mujoco
    import numpy as np
    if (type(maximum_iterations) is not int or maximum_iterations < 1 or maximum_iterations > 200
            or np.asarray(position_m).shape != (3,) or np.asarray(rotation).shape != (3, 3)
            or not np.isfinite(position_m).all() or not np.isfinite(rotation).all()
            or np.asarray(observed_qpos).shape != (model.nq,) or not np.isfinite(observed_qpos).all()):
        raise ValueError("Invalid bounded FR5 IK request")
    scratch = mujoco.MjData(model)
    scratch.qpos[:] = observed_qpos
    ids = [model.joint(name).id for name in ARM_JOINTS]
    qadr = [int(model.jnt_qposadr[index]) for index in ids]
    vadr = [int(model.jnt_dofadr[index]) for index in ids]
    lower = model.jnt_range[ids, 0] + 1e-5
    upper = model.jnt_range[ids, 1] - 1e-5
    site = model.site(TCP_SITE).id
    jacp, jacr = np.zeros((3, model.nv)), np.zeros((3, model.nv))
    desired = np.asarray(position_m, dtype=float)
    target_rotation = np.asarray(rotation, dtype=float)
    if (not np.allclose(target_rotation.T @ target_rotation, np.eye(3), atol=1e-6, rtol=0)
            or abs(float(np.linalg.det(target_rotation))-1.) > 1e-6):
        raise ValueError("Expected a proper orthonormal FR5 target rotation")
    if not ((scratch.qpos[qadr] >= lower-1e-5).all() and (scratch.qpos[qadr] <= upper+1e-5).all()):
        raise ValueError("Observed FR5 IK seed is outside its joint limits")
    position_error = angle_error = float("inf")
    for iteration in range(maximum_iterations):
        mujoco.mj_forward(model, scratch)
        actual_rotation = scratch.site_xmat[site].reshape(3, 3)
        delta_p = desired - scratch.site_xpos[site]
        delta_r = .5 * sum((np.cross(actual_rotation[:, i], target_rotation[:, i]) for i in range(3)), np.zeros(3))
        position_error = float(np.linalg.norm(delta_p))
        angle_error = math.acos(float(np.clip((np.trace(target_rotation @ actual_rotation.T)-1)/2, -1., 1.)))
        if position_error <= .0002 and angle_error <= .002:
            return dict(success=True, iterations=iteration+1, target_rad=scratch.qpos[qadr].tolist(),
                position_error_m=position_error, orientation_error_rad=angle_error,
                method="local site-Jacobian damped least squares; not collision-free planning")
        if iteration == maximum_iterations-1:
            break  # Failure diagnostics must describe the returned/evaluated q.
        mujoco.mj_jacSite(model, scratch, jacp, jacr, site)
        jacobian = np.vstack((jacp[:, vadr], .2*jacr[:, vadr]))
        error = np.concatenate((delta_p, .2*delta_r))
        delta = jacobian.T @ np.linalg.solve(jacobian @ jacobian.T + .0001*np.eye(6), error)
        delta *= min(1., .12 / max(float(np.linalg.norm(delta)), 1e-12))
        scratch.qpos[qadr] = np.clip(scratch.qpos[qadr] + delta, lower, upper)
    return dict(success=False, iterations=maximum_iterations, target_rad=scratch.qpos[qadr].tolist(),
        position_error_m=position_error, orientation_error_rad=angle_error,
        method="local site-Jacobian damped least squares; not collision-free planning")


class FR5Prototype:
    def __init__(self, directory, manifest, *, observer=None, cancel=None, realtime=False):
        if ((observer is not None and not callable(observer))
                or (cancel is not None and not callable(cancel)) or type(realtime) is not bool):
            raise ValueError("Expected optional observation/cancellation callbacks and explicit realtime mode")
        import mujoco
        import numpy as np
        self.mj, self.np = mujoco, np
        self.manifest = deepcopy(manifest)
        self.model = compile_pinned_model(directory, self.manifest)
        self.data = mujoco.MjData(self.model)
        mujoco.mj_resetDataKeyframe(self.model, self.data, 0)
        mujoco.mj_forward(self.model, self.data)
        self.qadr = [int(self.model.joint(name).qposadr[0]) for name in ARM_JOINTS]
        self.vadr = [int(self.model.joint(name).dofadr[0]) for name in ARM_JOINTS]
        self.arm_act = [int(self.model.actuator(name).id) for name in ARM_JOINTS]
        self.finger_act = [int(self.model.actuator("fr5_"+side+"_finger_servo").id) for side in ("left", "right")]
        self.scalar_names = manifest["qpos_state_contract"]["scalar_joint_names"]
        self.scalar_qadr = [int(self.model.joint(name).qposadr[0]) for name in self.scalar_names]
        if self.model.nq != manifest["qpos_state_contract"]["nq"] or len(set(self.scalar_names)) != 8:
            raise ValueError("Prototype compiled qpos contract differs")
        self.tcp = int(self.model.site(TCP_SITE).id)
        self.block = int(self.model.body("fr5_grasp_block").id)
        self.names = [mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, i) for i in range(self.model.ngeom)]
        self.life = GraspLifecycle(manifest["manifest_sha256"], object_id="fr5_grasp_block",
            owner="fr5_synthetic_parallel_gripper", link="fr5_synthetic_gripper")
        self.samples, self.segments, self.ik_paths = [], [], []
        self.steps, self.max_force = 0, [0., 0.]
        self.reference = None
        self.error = None
        self.cancellation = None
        self.observer, self.cancel_source, self.realtime = observer, cancel, realtime
        self.last_phase, self.last_frame = "initializing", None
        self.last_published = float("-inf")
        self.realtime_origin = None
        self.started = time.monotonic()

    def checkpoint(self):
        if self.cancel_source is not None:
            requested = self.cancel_source()
            if type(requested) is not bool:
                raise PrototypeFailure("Invalid local cancellation observation")
            if requested:
                if self.cancellation is None:
                    self.cancellation = dict(request_observed=True, phase=self.last_phase,
                        observed_at_simulation_sec=float(self.data.time), integrated_steps=self.steps,
                        followup_task_stages=0, automatic_release=False, automatic_resume=False,
                        physical_stop_confirmed=False, physical_stop_reason="Integration stops immediately; no settled hold interval was observed",
                        action_state="not_applicable", scope="Owned local simulation only; no ROS Action exists")
                raise PrototypeCancelled("Owned FR5 simulation cancellation was observed")

    def pace(self):
        if not self.realtime:
            return
        if self.realtime_origin is None:
            self.realtime_origin = time.monotonic()-float(self.data.time)
        deadline = self.realtime_origin+float(self.data.time)+self.model.opt.timestep
        while deadline > time.monotonic():
            self.checkpoint()
            time.sleep(max(0., min(.005, deadline-time.monotonic())))

    def publish_observation(self, frame, *, force=False):
        if self.observer is None:
            return
        now = time.monotonic()
        if force or now-self.last_published >= .1:
            # A reporting callback must not be able to alter the frames used by
            # the physical gates or the lifecycle's retained evidence.
            try:
                self.observer(deepcopy(dict(frame=frame, task=self.life.snapshot(), integrated_steps=self.steps)))
            except Exception as error:
                raise PrototypeFailure("Observation publication failed: "+type(error).__name__) from error
            self.last_published = now

    def observe(self, phase):
        mj, np, m, d = self.mj, self.np, self.model, self.data
        if not (np.isfinite(d.qpos).all() and np.isfinite(d.qvel).all() and np.isfinite(d.qacc).all()
                and np.isfinite(d.ctrl).all() and np.isfinite(d.actuator_force).all()):
            raise PrototypeFailure("Nonfinite FR5 prototype state")
        if any(int(n) for n in d.warning.number):
            raise PrototypeFailure("MuJoCo warning in FR5 prototype")
        for index, name in enumerate(ARM_JOINTS):
            limit = self.manifest["arm_joint_limits"][name]
            if (not limit["lower"]-1e-4 <= d.qpos[self.qadr[index]] <= limit["upper"]+1e-4
                    or abs(d.qvel[self.vadr[index]]) > limit["velocity"]):
                raise PrototypeFailure("FR5 position/velocity limit exceeded: "+name)
        forces = [0., 0.]
        table_force = 0.
        contacts = []
        finger_pair = {frozenset((BLOCK_GEOM, name)) for name in FINGER_GEOMS}
        policy_phase = self.last_phase if phase == "final" else phase
        allowed_finger = policy_phase in ("close", "extract", "carry", "lower", "open", "retract")
        allowed_table = policy_phase in ("initializing", "initial_settle", "approach", "close", "extract", "lower", "open", "retract")
        unexpected = []
        wrench = np.zeros(6)
        for index in range(d.ncon):
            contact = d.contact[index]
            pair = (self.names[contact.geom1], self.names[contact.geom2])
            mj.mj_contactForce(m, d, index, wrench)
            normal = max(0., float(wrench[0]))
            if normal <= 1e-5:
                continue
            contacts.append(dict(geoms=list(pair), normal_force_n=normal, distance_m=float(contact.dist)))
            key = frozenset(pair)
            if not ((allowed_finger and key in finger_pair)
                    or (allowed_table and key == frozenset((BLOCK_GEOM, TABLE_GEOM)))):
                unexpected.append(list(pair))
            for side, name in enumerate(FINGER_GEOMS):
                if key == frozenset((BLOCK_GEOM, name)):
                    forces[side] += normal
            if key == frozenset((BLOCK_GEOM, TABLE_GEOM)):
                table_force += normal
        self.max_force = [max(a, b) for a, b in zip(self.max_force, forces)]
        tcp = d.site_xpos[self.tcp].copy()
        rotation = d.site_xmat[self.tcp].reshape(3, 3)
        block = d.xpos[self.block].copy()
        relative_position = rotation.T @ (block-tcp)
        relative_rotation = rotation.T @ d.xmat[self.block].reshape(3, 3)
        velocity = np.zeros(6)
        mj.mj_objectVelocity(m, d, mj.mjtObj.mjOBJ_BODY, self.block, velocity, 0)
        result = dict(phase=phase, simulation_sec=float(d.time), observed_monotonic_ns=time.monotonic_ns(),
            arm_rad=d.qpos[self.qadr].tolist(),
            qpos=d.qpos.tolist(),
            scalar_joint_positions={name: float(d.qpos[address]) for name, address in zip(self.scalar_names, self.scalar_qadr)},
            arm_velocity_rad_sec=d.qvel[self.vadr].tolist(), actuator_force=d.actuator_force.tolist(),
            tcp_position_m=tcp.tolist(), tcp_rotation_matrix=rotation.tolist(), block_position_m=block.tolist(),
            block_speed_m_s=float(np.linalg.norm(velocity[3:])), finger_force_n=forces,
            bilateral_contact=bool(min(forces) >= .2), table_force_n=table_force,
            relative_position_m=relative_position.tolist(), relative_rotation_matrix=relative_rotation.tolist(),
            relative_position_error_m=None, relative_angle_error_rad=None,
            contacts=contacts, unexpected_contacts=unexpected)
        if self.reference is not None:
            old_position, old_rotation = self.reference
            result["relative_position_error_m"] = float(np.linalg.norm(relative_position-old_position))
            result["relative_angle_error_rad"] = math.acos(float(np.clip((np.trace(old_rotation.T @ relative_rotation)-1)/2, -1., 1.)))
        return result

    def integrate(self, phase, targets, finger_target, duration):
        np, m, d = self.np, self.model, self.data
        self.checkpoint()
        self.last_phase = phase
        path = np.asarray(targets)
        count = max(1, int(round(duration/m.opt.timestep)))
        if path.ndim != 2 or path.shape[1] != 6 or not np.isfinite(path).all() or len(path) < 1:
            raise ValueError("Expected a finite six-axis prototype path")
        start_fingers = d.ctrl[self.finger_act].copy()
        frames = []
        for index in range(count):
            self.checkpoint()
            self.pace()
            self.checkpoint()
            t = min(1., (index+1)/(.7*count))
            blend = t*t*t*(10-15*t+6*t*t)
            offset = blend*(len(path)-1)
            low = min(int(offset), len(path)-1)
            high = min(low+1, len(path)-1)
            weight = offset-low
            d.ctrl[self.arm_act] = (1-weight)*path[low] + weight*path[high]
            d.ctrl[self.finger_act] = start_fingers+(finger_target-start_fingers)*blend
            self.mj.mj_step(m, d)
            self.mj.mj_forward(m, d)
            self.steps += 1
            frame = self.observe(phase)
            self.last_frame = deepcopy(frame)
            frames.append(frame)
            if self.steps % 10 == 0:
                if len(self.samples) >= 2500:
                    raise PrototypeFailure("Prototype sample bound exceeded")
                self.samples.append(frame)
            if frame["unexpected_contacts"]:
                self.samples.append(frame)
                raise PrototypeFailure("Unexpected contact in "+phase+": "+str(frame["unexpected_contacts"]))
            self.publish_observation(frame)
        self.segments.append(dict(phase=phase, integrated_steps=len(frames),
            end_simulation_sec=float(d.time), final=frames[-1]))
        self.publish_observation(frames[-1], force=True)
        return frames

    def hold(self, phase, finger_target, duration):
        return self.integrate(phase, [self.data.ctrl[self.arm_act].tolist()], finger_target, duration)

    def move_tcp(self, phase, position, finger_target, duration=2.):
        self.checkpoint()
        np = self.np
        target = np.asarray(position, dtype=float)
        observed_start = self.data.site_xpos[self.tcp].copy()
        observed_joints = self.data.qpos[self.qadr].copy()
        previous_command = self.data.ctrl[self.arm_act].copy()
        start_difference = float(np.max(np.abs(observed_joints-previous_command)))
        if start_difference > .02:
            raise PrototypeFailure("Observed FR5 state differs from the previous servo command")
        # Preserve the position-servo command across segment boundaries. Using
        # measured gravity-deflected qpos as a new position command would apply
        # that same static deflection twice before the upward motion begins.
        # The actual start remains separately observed and is never overwritten.
        scratch = self.mj.MjData(self.model)
        scratch.qpos[:] = self.data.qpos
        scratch.qpos[self.qadr] = previous_command
        self.mj.mj_forward(self.model, scratch)
        start = scratch.site_xpos[self.tcp].copy()
        rotation = np.asarray(self.manifest["tcp_rotation_matrix"])
        count = max(2, int(math.ceil(float(np.linalg.norm(target-start))/.003)))
        seed = self.data.qpos.copy()
        seed[self.qadr] = previous_command
        path = [previous_command.tolist()]
        solutions = []
        for fraction in np.linspace(0., 1., count+1)[1:]:
            self.checkpoint()
            pose = start+(target-start)*fraction
            solution = solve_ik(self.model, seed, pose, rotation)
            solutions.append(dict(position_m=pose.tolist(), **solution))
            if not solution["success"]:
                self.ik_paths.append(dict(phase=phase, solutions=solutions, complete=False))
                raise PrototypeFailure("Local IK did not reach "+phase+" waypoint")
            seed[self.qadr] = solution["target_rad"]
            path.append(solution["target_rad"])
        self.ik_paths.append(dict(phase=phase, solutions=solutions, complete=True,
            collision_free_planner=False, physical_data_pose_assignments=0,
            start_source="Previous position-servo command; actual state is separately observed",
            observed_start_position_m=observed_start.tolist(), nominal_start_position_m=start.tolist(),
            observed_start_rad=observed_joints.tolist(), previous_servo_command_rad=previous_command.tolist(),
            maximum_start_tracking_error_rad=start_difference))
        frames = self.integrate(phase, path, finger_target, duration)
        self.segments[-1]["nominal_tcp_goal_m"] = target.tolist()
        self.segments[-1]["observed_tcp_error_m"] = float(np.linalg.norm(np.asarray(frames[-1]["tcp_position_m"])-target))
        return frames

    def tail(self, frames):
        return frames[-int(round(.2/self.model.opt.timestep)):]

    def run(self):
        np = self.np
        pickup = np.asarray(self.manifest["pickup_tcp_position_m"])
        lift = np.asarray(self.manifest["lift_offset_m"])
        placement = pickup+np.asarray(self.manifest["placement_offset_m"])
        try:
            initial = self.hold("initial_settle", 0., .6)[-1]
            self.move_tcp("approach", pickup, 0.)
            self.checkpoint()
            self.life.approach_complete()
            closed = self.hold("close", .025, 1.)
            self.checkpoint()
            tail = self.tail(closed)
            self.life.contacts_confirmed(left_force_n=min(f["finger_force_n"][0] for f in tail),
                right_force_n=min(f["finger_force_n"][1] for f in tail), observed_sec=.2)
            self.reference = (np.asarray(closed[-1]["relative_position_m"]),
                              np.asarray(closed[-1]["relative_rotation_matrix"]))
            extracted = self.move_tcp("extract", pickup+lift, .025)
            self.checkpoint()
            tail = self.tail(extracted)
            self.lift_evidence = dict(fixture_sha256=self.manifest["manifest_sha256"],
                lift_m=max(0., tail[-1]["block_position_m"][2]-initial["block_position_m"][2]), stable_sec=.2,
                relative_position_error_m=max(f["relative_position_error_m"] for f in extracted),
                relative_angle_error_rad=max(f["relative_angle_error_rad"] for f in extracted),
                bilateral_fraction=sum(f["bilateral_contact"] for f in tail)/len(tail))
            self.life.lift_confirmed(**self.lift_evidence)
            self.checkpoint()
            self.life.attach(object_id="fr5_grasp_block", owner="fr5_synthetic_parallel_gripper",
                link="fr5_synthetic_gripper", fixture_sha256=self.manifest["manifest_sha256"])
            self.life.carry()
            carried = self.move_tcp("carry", placement+lift, .025)
            self.checkpoint()
            if any(f["relative_position_error_m"] > .005 or f["relative_angle_error_rad"] > .15
                   or not f["bilateral_contact"] for f in carried):
                raise PrototypeFailure("Object slip/lost bilateral contact while carried by FR5")
            self.move_tcp("lower", placement, .025)
            self.checkpoint()
            self.life.release()
            opened = self.hold("open", 0., 1.)
            self.checkpoint()
            tail = self.tail(opened)
            if any(max(f["finger_force_n"]) >= .1 for f in tail):
                raise PrototypeFailure("Physical finger/object release not observed")
            gap = .07-sum(self.data.qpos[self.model.joint("fr5_"+side+"_finger_slide").qposadr[0]] for side in ("left", "right"))
            self.life.release_confirmed(fixture_sha256=self.manifest["manifest_sha256"],
                bilateral_contact_absent_sec=.2, open_gap_m=float(gap))
            released = self.move_tcp("retract", placement+lift, 0.)
            self.checkpoint()
            tail = self.tail(released)
            self.life.settled(table_contact_sec=sum(f["table_force_n"] > .1 for f in tail)*self.model.opt.timestep,
                position_error_m=float(np.linalg.norm(np.asarray(tail[-1]["block_position_m"])-placement)),
                speed_m_s=max(f["block_speed_m_s"] for f in tail))
        except PrototypeCancelled as error:
            self.error = dict(type=type(error).__name__, message=str(error))
            if self.life.stage not in ("failed", "cancelled", "succeeded"):
                self.life.cancel()
        except (ValueError, RuntimeError, ArithmeticError) as error:
            self.error = dict(type=type(error).__name__, message=str(error)[:2048])
            if self.life.stage not in ("failed", "cancelled", "succeeded"):
                self.life.fail(str(error)[:256], slipped="slip" in str(error).lower())
        return self.report()

    def report(self):
        final_error = None
        try:
            final = self.observe("final")
        except (ValueError, RuntimeError, ArithmeticError) as error:
            final, final_error = None, dict(type=type(error).__name__, message=str(error)[:1024])
        return dict(schema_version=1, prototype="fr5_parallel_gripper_v1", engine="MuJoCo "+self.mj.__version__,
            scope="Detached FR5 six-axis dynamics/contact prototype; not ROS/MoveIt/native or production viewer integration",
            outcome=self.life.stage, error=self.error, final_observation_error=final_error,
            cancellation=self.cancellation,
            manifest_sha256=self.manifest["manifest_sha256"], task=self.life.snapshot(),
            qpos_state_contract=self.manifest["qpos_state_contract"],
            physics=dict(integrated_steps=self.steps, simulation_sec=float(self.data.time),
                elapsed_wall_sec=time.monotonic()-self.started, free_object=True,
                equality_constraints=int(self.model.neq), physical_joint_pose_overwrites_after_initialization=0,
                object_pose_overwrites_after_initialization=0, max_finger_force_n=self.max_force,
                warning_counts=[int(count) for count in self.data.warning.number],
                source_arm_position_servos=True, extra_applied_forces=False),
            contact_policy=dict(finger_object_allowed_phases=["close", "extract", "carry", "lower", "open", "retract"],
                table_object_allowed_phases=["initial_settle", "approach", "close", "extract", "lower", "open", "retract"],
                all_other_observed_contacts="fail before a subsequent integration step",
                inherited_and_implicit_filters="See manifest; no added broad contact exclusions"),
            lift_evidence=getattr(self, "lift_evidence", None), ik_paths=self.ik_paths,
            segments=self.segments, samples=self.samples, final=final,
            sampling=dict(every_physics_steps=10, all_step_gates=True, maximum_samples=2500),
            observation_mode=dict(realtime_factor=1 if self.realtime else None,
                observer_enabled=self.observer is not None, maximum_periodic_publication_hz=10,
                phase_boundaries_may_publish_immediately=True),
            w07_complete=False,
            limitations=["Source FR5 model is simulated, not a verified physical robot or manufactured gripper.",
                "Local DLS waypoints are not MoveIt plans or collision-free global paths.",
                "New tool/free-object state is not connected to the existing ROS/native/viewer contracts.",
                "Logical attachment does not constrain the free block or mutate a MoveIt planning scene.",
                "MuJoCo Python 3.14.0 differs from the existing ROS native 3.12.0 runtime.",
                "One representative deterministic trial is not a grasp reliability evaluation."])


def run_prototype(source, output):
    manifest = write_model(source, output)
    prototype = FR5Prototype(output, manifest)
    result = prototype.run()
    result["implementation_sha256"] = {name: sha256(Path(__file__).with_name(name).read_bytes())
        for name in ("fr5_model.py", "fr5_run.py", "contract.py")}
    raw = (json.dumps(result, indent=2, allow_nan=False)+"\n").encode()
    if len(raw) > 8*1024*1024:
        raise RuntimeError("FR5 prototype result exceeds its bound")
    (Path(output)/"result.json").write_bytes(raw)
    return dict(outcome=result["outcome"], error=result["error"], result_sha256=sha256(raw),
        manifest_sha256=manifest["manifest_sha256"], w07_complete=False)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args(argv)
    summary = run_prototype(args.source, args.output)
    print(json.dumps(summary, indent=2, allow_nan=False))
    return 0 if summary["outcome"] == "succeeded" else 1


if __name__ == "__main__":
    raise SystemExit(main())
