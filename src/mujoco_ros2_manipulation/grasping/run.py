"""Run bounded, owned synthetic grasp experiments without ROS or a renderer.

Run with ``python -m mujoco_ros2_manipulation.grasping --output NEW_DIRECTORY``.
Every case is a fresh in-memory model. Failure cases keep their actual outcome,
observations and terminal state; the suite's expectation check is a separate
field. No existing server, runtime model, ROS domain, or planning scene is used.
"""
import argparse
from dataclasses import replace
import hashlib
import json
import math
from pathlib import Path
import platform
import time

from .contract import GraspLifecycle, GraspTransitionError
from .fixture import FixtureSpec


CASES = ("normal", "missing_object", "one_sided_contact", "slip", "cancel", "unexpected_contact")
MAX_SAMPLES = 1200
MAX_ARTIFACT_BYTES = 2 * 1024 * 1024


def _json(value):
    return json.dumps(value, sort_keys=True, indent=2, allow_nan=False) + "\n"


class Experiment:
    """A private Cartesian fixture; no FR5 kinematic or execution claims."""

    def __init__(self, spec, *, case):
        if not isinstance(spec, FixtureSpec) or case not in CASES:
            raise ValueError("Expected explicit fixture specification and known case")
        import mujoco
        import numpy as np
        self.mj, self.np = mujoco, np
        self.spec, self.case = spec, case
        self.identity = spec.identity()
        self.model = mujoco.MjModel.from_xml_string(spec.xml())
        self.data = mujoco.MjData(self.model)
        self.life = GraspLifecycle(self.identity["manifest_sha256"])
        self.samples, self.segment_summaries = [], []
        self.steps = 0
        self.reference = None
        self.max_force = [0., 0.]
        self.unexpected_contacts = set()
        self.cancellation = None
        self.execution_error = None
        self.tcp_id = int(self.model.site("grasp_tcp").id)
        self.block_id = int(self.model.body("block").id) if spec.block_present else None
        self.geom_names = [mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_GEOM, i)
                           for i in range(self.model.ngeom)]
        self.mj.mj_forward(self.model, self.data)
        self.started = time.monotonic()

    def observe(self, phase):
        mj, np, m, d = self.mj, self.np, self.model, self.data
        if not (np.isfinite(d.qpos).all() and np.isfinite(d.qvel).all()
                and np.isfinite(d.ctrl).all() and np.isfinite(d.qacc).all()):
            raise RuntimeError("Nonfinite synthetic fixture state")
        if any(int(count) > 0 for count in d.warning.number):
            raise RuntimeError("MuJoCo warning in synthetic fixture")
        forces = [0., 0.]
        table_force = 0.
        allowed = {frozenset(("block_geom", name)) for name in
                   ("left_finger_geom", "right_finger_geom", "table_geom")}
        contact_pairs = []
        force = np.zeros(6)
        for index in range(d.ncon):
            contact = d.contact[index]
            pair = (self.geom_names[contact.geom1], self.geom_names[contact.geom2])
            mj.mj_contactForce(m, d, index, force)
            if not np.isfinite(force).all():
                raise RuntimeError("Nonfinite synthetic contact force")
            normal = max(0., float(force[0]))
            if normal <= 1e-6:
                continue
            contact_pairs.append(dict(geoms=list(pair), normal_force_n=normal,
                                      signed_distance_m=float(contact.dist)))
            pair_set = frozenset(pair)
            if pair_set not in allowed:
                self.unexpected_contacts.add(tuple(sorted(pair)))
            for side, name in enumerate(("left_finger_geom", "right_finger_geom")):
                if pair_set == frozenset(("block_geom", name)):
                    forces[side] += normal
            if pair_set == frozenset(("block_geom", "table_geom")):
                table_force += normal
        self.max_force = [max(a, b) for a, b in zip(self.max_force, forces)]
        tcp = d.site_xpos[self.tcp_id].copy()
        rotation = d.site_xmat[self.tcp_id].reshape(3, 3)
        result = dict(simulation_sec=float(d.time), phase=phase,
            carriage_joint_position_m=[float(v) for v in d.qpos[:2]],
            carriage_speed_m_s=float(np.linalg.norm(d.qvel[:2])),
            fixture_joint_speed_m_s=float(np.max(np.abs(d.qvel[:4]))),
            finger_joint_position_m=[float(v) for v in d.qpos[2:4]],
            inner_gap_m=float(.07 - sum(d.qpos[2:4])),
            tcp_position_m=tcp.tolist(), finger_object_normal_force_n=forces,
            bilateral_contact=bool(min(forces) >= .2), table_object_normal_force_n=table_force,
            contacts=contact_pairs, object_position_m=None,
            object_speed_m_s=None, relative_object_position_m=None,
            relative_object_rotation=None, relative_position_error_m=None,
            relative_angle_error_rad=None)
        if self.block_id is not None:
            position = d.xpos[self.block_id].copy()
            relative = rotation.T @ (position - tcp)
            relative_rotation = rotation.T @ d.xmat[self.block_id].reshape(3, 3)
            velocity = np.zeros(6)
            mj.mj_objectVelocity(m, d, mj.mjtObj.mjOBJ_BODY, self.block_id, velocity, 0)
            result.update(object_position_m=position.tolist(), object_speed_m_s=float(np.linalg.norm(velocity[3:])),
                relative_object_position_m=relative.tolist(), relative_object_rotation=relative_rotation.tolist())
            if self.reference is not None:
                old_position, old_rotation = self.reference
                cosine = float(np.clip((np.trace(old_rotation.T @ relative_rotation) - 1.) / 2., -1., 1.))
                result.update(relative_position_error_m=float(np.linalg.norm(relative - old_position)),
                              relative_angle_error_rad=math.acos(cosine))
        return result

    def segment(self, phase, target, duration, *, cancel_at_fraction=None):
        np, m, d = self.np, self.model, self.data
        if (len(target) != m.nu or type(duration) not in (int, float) or not math.isfinite(duration)
                or duration <= 0 or duration > 2
                or any(type(v) not in (int, float) or not math.isfinite(v) for v in target)
                or any(not low <= v <= high for v, (low, high) in zip(target, m.jnt_range[:4]))
                or (cancel_at_fraction is not None and (type(cancel_at_fraction) not in (int, float)
                    or not math.isfinite(cancel_at_fraction) or not 0 < cancel_at_fraction <= 1))):
            raise ValueError("Invalid bounded fixture movement")
        start = d.ctrl.copy()
        end = np.array(target, dtype=float)
        total = int(round(duration / m.opt.timestep))
        if total < 1:
            raise ValueError("Fixture movement must include at least one physics step")
        frames = []
        for index in range(total):
            fraction = min(1., (index + 1) / (.7 * total))
            blend = fraction * fraction * (3. - 2. * fraction)
            d.ctrl[:] = start + (end - start) * blend
            self.mj.mj_step(m, d)
            # Forward computes contacts/constraint forces at the recorded qpos,
            # rather than pairing pre-integration contacts with the next pose.
            self.mj.mj_forward(m, d)
            self.steps += 1
            frame = self.observe(phase)
            frames.append(frame)
            if self.steps % 10 == 0:
                if len(self.samples) >= MAX_SAMPLES:
                    raise RuntimeError("Synthetic grasp sample bound exceeded")
                self.samples.append(frame)
            if self.unexpected_contacts:
                if self.life.stage not in ("failed", "cancelled", "succeeded"):
                    self.life.fail("Unexpected fixture contact")
                break
            if cancel_at_fraction is not None and (index + 1) / total >= cancel_at_fraction:
                self.life.cancel()
                self.cancellation = dict(requested_at_simulation_sec=float(d.time),
                    carriage_hold_target_m=d.ctrl[:2].tolist(), followup_task_stages=0,
                    automatic_release=False, stop_confirmed=False)
                break
        self.segment_summaries.append(dict(phase=phase, integrated_steps=len(frames),
            first_simulation_sec=frames[0]["simulation_sec"], last_simulation_sec=frames[-1]["simulation_sec"],
            bilateral_fraction=sum(f["bilateral_contact"] for f in frames) / len(frames),
            max_left_force_n=max(f["finger_object_normal_force_n"][0] for f in frames),
            max_right_force_n=max(f["finger_object_normal_force_n"][1] for f in frames)))
        return frames

    def _tail(self, frames, seconds=.2):
        return frames[-int(round(seconds / self.model.opt.timestep)):]

    def run(self):
        initial = self.segment("initial_settle", [0., 0., 0., 0.], .4)[-1]
        if self.life.stage == "failed":
            return self.report()
        self.segment("approach", [0., -.09, 0., 0.], 1.)
        if self.life.stage == "failed":
            return self.report()
        self.life.approach_complete()
        closed = self.segment("close", [0., -.09, .025, .025], 1.)
        if self.life.stage == "failed":
            return self.report()
        tail = self._tail(closed)
        left = min(f["finger_object_normal_force_n"][0] for f in tail)
        right = min(f["finger_object_normal_force_n"][1] for f in tail)
        if min(left, right) < .2:
            self.life.fail("Object absent or bilateral sustained finger contact missing")
            return self.report()
        self.life.contacts_confirmed(left_force_n=left, right_force_n=right,
                                    observed_sec=len(tail) * self.model.opt.timestep)
        last = closed[-1]
        self.reference = (self.np.array(last["relative_object_position_m"]),
                          self.np.array(last["relative_object_rotation"]))
        lifted = self.segment("extract", [0., .01, .025, .025], 1.)
        if self.life.stage == "failed":
            return self.report()
        tail = self._tail(lifted)
        stable = dict(fixture_sha256=self.identity["manifest_sha256"],
            lift_m=max(0., tail[-1]["object_position_m"][2] - initial["object_position_m"][2]),
            stable_sec=len(tail) * self.model.opt.timestep,
            relative_position_error_m=max(f["relative_position_error_m"] for f in lifted),
            relative_angle_error_rad=max(f["relative_angle_error_rad"] for f in lifted),
            bilateral_fraction=sum(f["bilateral_contact"] for f in tail) / len(tail))
        self.lift_evidence = stable
        try:
            self.life.lift_confirmed(**stable)
        except GraspTransitionError as error:
            self.life.fail(str(error), slipped=True)
            return self.report()
        self.life.attach(object_id="block", owner="synthetic_gripper", link="carriage",
                         fixture_sha256=self.identity["manifest_sha256"])
        self.life.carry()
        carried = self.segment("carry", [.15, .01, .025, .025], 1.,
                               cancel_at_fraction=.5 if self.case == "cancel" else None)
        if self.life.stage == "cancelled":
            # Hold the last actuator targets. Cancellation does not imply an
            # automatic release, teleport, detach, or next task stage.
            held = self.segment("cancel_hold", self.data.ctrl.copy().tolist(), .4)
            tail = self._tail(held)
            grasp_still_observed = (all(f["bilateral_contact"] for f in tail)
                and all(f["relative_position_error_m"] <= .005
                        and f["relative_angle_error_rad"] <= .15 for f in tail))
            stop_observed = all(f["fixture_joint_speed_m_s"] <= .02 for f in tail)
            self.cancellation.update(hold_until_simulation_sec=float(self.data.time),
                final_carriage_speed_m_s=held[-1]["carriage_speed_m_s"],
                final_object_speed_m_s=held[-1]["object_speed_m_s"],
                stop_confirmed=stop_observed, stop_observed_sec=len(tail)*self.model.opt.timestep,
                grasp_still_observed=grasp_still_observed)
            self.life.cancelled_observation(stop_observed=stop_observed,
                                           grasp_still_observed=grasp_still_observed)
            return self.report()
        if self.life.stage == "failed":
            return self.report()
        if any(f["relative_position_error_m"] > .005 or f["relative_angle_error_rad"] > .15
               for f in carried):
            self.life.fail("Object slipped while carrying", slipped=True)
            return self.report()
        self.segment("lower", [.15, -.09, .025, .025], 1.)
        if self.life.stage == "failed":
            return self.report()
        self.life.release()
        opened = self.segment("open", [.15, -.09, 0., 0.], 1.)
        if self.life.stage == "failed":
            return self.report()
        tail = self._tail(opened)
        absent = sum(not f["bilateral_contact"] for f in tail) * self.model.opt.timestep
        self.life.release_confirmed(fixture_sha256=self.identity["manifest_sha256"],
            bilateral_contact_absent_sec=absent, open_gap_m=opened[-1]["inner_gap_m"])
        released = self.segment("retract_and_settle", [.15, 0., 0., 0.], 1.)
        if self.life.stage == "failed":
            return self.report()
        tail = self._tail(released)
        target = self.np.array([.15, 0., .025])
        self.life.settled(table_contact_sec=sum(f["table_object_normal_force_n"] > .1 for f in tail)
            * self.model.opt.timestep,
            position_error_m=float(self.np.linalg.norm(self.np.array(tail[-1]["object_position_m"]) - target)),
            speed_m_s=max(f["object_speed_m_s"] for f in tail))
        return self.report()

    def report(self):
        final_error = None
        try:
            final = self.observe("final")
        except (ValueError, RuntimeError, ArithmeticError) as error:
            final = None
            final_error = dict(type=type(error).__name__, message=str(error)[:1024])
        return dict(schema_version=1, case=self.case, engine="MuJoCo " + self.mj.__version__,
            scope="Independent Cartesian fixture/contact experiment; not FR5, ROS, MoveIt or production viewer execution",
            fixture_identity=self.identity, outcome=self.life.stage,
            execution_error=self.execution_error,
            task=self.life.snapshot(), physics=dict(integrated_steps=self.steps,
                simulation_sec=float(self.data.time), elapsed_wall_sec=time.monotonic() - self.started,
                free_object=True if self.spec.block_present else None,
                weld_or_attachment_constraints=int(self.model.neq),
                warning_counts=[int(count) for count in self.data.warning.number],
                object_pose_overwrites=0, max_finger_object_normal_force_n=self.max_force,
                unexpected_contact_pairs=[list(pair) for pair in sorted(self.unexpected_contacts)]),
            lift_evidence=getattr(self, "lift_evidence", None), cancellation=self.cancellation,
            segments=self.segment_summaries, final=final, final_observation_error=final_error,
            samples=self.samples, sampling=dict(every_physics_steps=10, maximum=MAX_SAMPLES,
                contact_evaluation="mj_forward at the recorded post-integration state",
                full_step_gate="All integrated steps checked; samples are a decimated record"),
            limitations=["Synthetic gripper dimensions, mass, friction and gains are experimental settings.",
                "Position-actuated x/z carriage is not a FR5 arm or its kinematics/dynamics.",
                "Logical planning_attachment does not call MoveIt or constrain the free object.",
                "No current native contact-stop policy or robot/viewer model is changed.",
                "One deterministic case per condition; not grasp reliability or real-hardware safety evidence."])


def case_spec(case):
    if case not in CASES:
        raise ValueError("Unknown synthetic grasp case")
    spec = FixtureSpec()
    if case == "missing_object":
        spec = replace(spec, block_present=False)
    elif case == "one_sided_contact":
        spec = replace(spec, right_finger_offset_y_m=.08)
    elif case == "slip":
        spec = replace(spec, friction=.01)
    elif case == "unexpected_contact":
        # Preserve the distinct development failure: this condition causes a
        # palm collision during approach, not a valid bilateral slip experiment.
        spec = replace(spec, friction=.001)
    return spec


def run_case(case):
    spec = case_spec(case)
    experiment = Experiment(spec, case=case)
    try:
        result = experiment.run()
    except (ValueError, RuntimeError, ArithmeticError) as error:
        # Keep the actual failure record and continue the other independent
        # cases. An implementation/observation exception never counts as a
        # successful expected physical rejection.
        experiment.execution_error = dict(type=type(error).__name__, message=str(error)[:1024])
        if experiment.life.stage not in ("failed", "cancelled", "succeeded"):
            experiment.life.fail("Experiment raised " + type(error).__name__)
        result = experiment.report()
    expected = "succeeded" if case == "normal" else "cancelled" if case == "cancel" else "failed"
    checks = dict(expected_terminal=result["outcome"] == expected,
        no_execution_error=result["execution_error"] is None,
        final_observation_valid=result["final_observation_error"] is None,
        warning_free=not any(result["physics"]["warning_counts"]),
        no_fake_physical_attachment=result["physics"]["weld_or_attachment_constraints"] == 0
            and result["physics"]["object_pose_overwrites"] == 0,
        no_unexpected_contacts=not result["physics"]["unexpected_contact_pairs"])
    if result["execution_error"] is not None or result["final_observation_error"] is not None:
        result["expectation"] = dict(expected_outcome=expected, checks=checks, passed=False)
        return spec, result
    if case == "unexpected_contact":
        checks.pop("no_unexpected_contacts")
        checks["unexpected_contact_stopped_before_close"] = (
            ["block_geom", "palm_geom"] in result["physics"]["unexpected_contact_pairs"]
            and not any(s["phase"] in ("close", "extract", "carry", "lower", "open")
                        for s in result["segments"]))
    if case == "missing_object":
        checks["no_contact_or_carry"] = (max(result["physics"]["max_finger_object_normal_force_n"]) == 0
            and not any(s["phase"] in ("extract", "carry") for s in result["segments"]))
    elif case == "one_sided_contact":
        left, right = result["physics"]["max_finger_object_normal_force_n"]
        checks["one_side_observed_without_carry"] = (left > .2 and right == 0
            and not any(s["phase"] in ("extract", "carry") for s in result["segments"]))
    elif case == "slip":
        checks["physical_failure_separate_from_attachment"] = (result["task"]["physical_grasp"] == "slipped"
            and result["task"]["planning_attachment"] == "world"
            and result["lift_evidence"]["relative_position_error_m"] > .005)
    elif case == "cancel":
        checks["cancel_holds_without_followup_release"] = (result["cancellation"]["stop_confirmed"]
            and result["task"]["planning_attachment"] == "attached"
            and not any(s["phase"] in ("lower", "open", "retract_and_settle") for s in result["segments"]))
    result["expectation"] = dict(expected_outcome=expected, checks=checks, passed=all(checks.values()))
    return spec, result


def invalid_attachment_receipt():
    """Negative state checks are not represented as a physics execution."""
    digest = FixtureSpec().identity()["manifest_sha256"]
    rows = []

    def holding():
        life = GraspLifecycle(digest)
        life.approach_complete()
        life.contacts_confirmed(left_force_n=1., right_force_n=1., observed_sec=.2)
        life.lift_confirmed(fixture_sha256=digest, lift_m=.1, stable_sec=.2,
            relative_position_error_m=.001, relative_angle_error_rad=.01, bilateral_fraction=1.)
        return life

    valid = dict(object_id="block", owner="synthetic_gripper", link="carriage", fixture_sha256=digest)
    scenarios = [("before_physical_confirmation", GraspLifecycle(digest), dict(valid))]
    for field, value in (("object_id", "other"), ("owner", "other"), ("link", "other"),
                         ("fixture_sha256", "0"*64)):
        scenarios.append(("wrong_"+field, holding(), dict(valid, **{field: value})))
    duplicate = holding()
    duplicate.attach(**valid)
    scenarios.append(("duplicate_attach", duplicate, dict(valid)))
    for name, life, values in scenarios:
        before = life.snapshot()
        error = None
        try:
            life.attach(**values)
        except GraspTransitionError as rejected:
            error = str(rejected)
        rows.append(dict(condition=name, rejected=error is not None,
            state_unchanged=life.snapshot() == before, error=error))
    return dict(case="invalid_attachment", scope="Logical state validation only; no physics executed",
        evidence="Hand-supplied values exercise state validation; not physical grasp evidence",
        conditions=rows, rejected=sum(row["rejected"] for row in rows),
        state_unchanged=all(row["state_unchanged"] for row in rows),
        passed=all(row["rejected"] and row["state_unchanged"] for row in rows))


def run_suite(output):
    directory = Path(output)
    # No existing artifacts are overwritten, even on a repeated invocation.
    directory.mkdir(parents=True, exist_ok=False)
    receipts = []
    engines = set()
    for case in CASES:
        spec = case_spec(case)
        try:
            spec, result = run_case(case)
        except (ImportError, OSError, ValueError, RuntimeError, ArithmeticError) as error:
            result = dict(schema_version=1, case=case, outcome="error",
                scope="Independent fixture initialization/observation failed; no success inferred",
                fixture_identity=spec.identity(),
                execution_error=dict(type=type(error).__name__, message=str(error)[:1024]),
                expectation=dict(expected_outcome="succeeded" if case == "normal" else
                    "cancelled" if case == "cancel" else "failed", checks={}, passed=False))
        if result.get("engine"):
            engines.add(result["engine"])
        model_name, result_name = case + ".xml", case + ".json"
        (directory / model_name).write_text(spec.xml())
        raw = _json(result).encode()
        if len(raw) > MAX_ARTIFACT_BYTES:
            raise RuntimeError("Synthetic grasp artifact exceeds its byte bound")
        (directory / result_name).write_bytes(raw)
        receipts.append(dict(case=case, outcome=result["outcome"],
            expectation_passed=result["expectation"]["passed"],
            result=result_name, result_sha256=hashlib.sha256(raw).hexdigest(),
            mjcf=model_name, mjcf_sha256=result["fixture_identity"]["mjcf_sha256"]))
    invalid = invalid_attachment_receipt()
    invalid_raw = _json(invalid)
    (directory / "invalid_attachment.json").write_text(invalid_raw)
    summary = dict(schema_version=1, suite="synthetic_grasp_v1", python=platform.python_version(),
        engines=sorted(engines), independent_of_ros_native_runtime=True,
        implementation_sha256={name: hashlib.sha256(Path(__file__).with_name(name).read_bytes()).hexdigest()
            for name in ("fixture.py", "contract.py", "run.py")},
        cases=receipts, invalid_attachment=invalid, passed=all(v["expectation_passed"] for v in receipts) and invalid["passed"],
        invalid_attachment_sha256=hashlib.sha256(invalid_raw.encode()).hexdigest(),
        w07_complete=False, scope="Independent contact fixture and lifecycle only; FR5/Pose/IK/ROS integration remains separate")
    (directory / "summary.json").write_text(_json(summary))
    return summary


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", required=True, help="New owned directory for fixtures and failure receipts")
    args = parser.parse_args(argv)
    try:
        result = run_suite(args.output)
    except (OSError, ValueError, RuntimeError) as error:
        parser.exit(1, "Synthetic grasp verification failed: " + str(error) + "\n")
    print(_json(result), end="")
    return 0 if result["passed"] else 1
