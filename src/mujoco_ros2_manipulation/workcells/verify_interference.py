"""Reject interfering independent plans, then execute a checked both-arm plan."""

import argparse
from copy import deepcopy
from contextlib import contextmanager
import json
import time
from pathlib import Path

import rclpy
from builtin_interfaces.msg import Time
from moveit_msgs.msg import MoveItErrorCodes
from rcl_interfaces.srv import GetParameters
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory

from mujoco_ros2_manipulation.paths import ROOT
from mujoco_ros2_manipulation.model_build.integrity import file_sha256
from mujoco_ros2_manipulation.verification.diagnostics import (
    diagnostic_environment,
    finalize_physics,
)
from mujoco_ros2_manipulation.verification.runner import run_verification
from mujoco_ros2_manipulation.verification.reports import atomic_json
from mujoco_ros2_manipulation.workcells.config import load_workcell, identifier
from mujoco_ros2_manipulation.workcells.integrity import check_workcell
from mujoco_ros2_manipulation.workcells.interference import (
    load_case,
    Spline,
    stretch,
    sampled_parallel,
)
from mujoco_ros2_manipulation.workcells.distance import DistanceEvaluator
from .path_quality import evaluate_path
from .self_clearance import (self_policy_snapshot, require_self_clearance, self_evaluator_identity, SelfClearanceRejected)
from mujoco_ros2_manipulation.workcells.plan_selection import (
    MAX_PLAN_CANDIDATES,
    select_clearance_plan,
)
from mujoco_ros2_manipulation.workcells.timing import load_policy
from mujoco_ros2_manipulation.workcells.trajectory import partitions
from mujoco_ros2_manipulation.workcells.verify import WorkcellVerification, require


def preflight(cell, case):
    metadata = check_workcell(cell)
    _, data = load_case(cell, case)
    for key in ("start_rad", "goal_rad"):
        for name, value in data[key].items():
            limits = metadata["joint_limits"][name]
            require(
                limits["lower"] <= value <= limits["upper"],
                "Case exceeds joint limit: " + name,
            )


class ParallelPlanRejected(RuntimeError):
    def __init__(self, assessment):
        super().__init__("Refusing interfering parallel plans before dispatch")
        self.assessment = assessment


class InterferenceVerification(WorkcellVerification):
    def __init__(self, cell, progress=None, case="dual_arm_crossing"):
        self.self_policy_path, self.self_policy, self.self_policy_sha256 = self_policy_snapshot(cell)
        self._self_clock_received_ns = self._self_state_received_ns = None
        super().__init__(cell, progress)
        self.case_path, self.case = load_case(cell, case)
        self.case_sha256 = file_sha256(self.case_path)
        self.evidence = {}
        self.initial = None
        self.assessments = {}
        self.executions = []

    def clock(self, message):
        received = time.monotonic_ns()
        super().clock(message)
        self._self_clock_received_ns = received

    def joints(self, message):
        received = time.monotonic_ns()
        previous = self.joint_count
        super().joints(message)
        if self.joint_count != previous:
            self._self_state_received_ns = received

    def _self_dispatch_fresh(self, scheduled_start):
        now = time.monotonic_ns()
        require(self._self_state_received_ns is not None
                and 0 <= (now-self._self_state_received_ns)/1e9 <= .3, "Joint state is stale at native self admission")
        require(self._self_clock_received_ns is not None
                and 0 <= (now-self._self_clock_received_ns)/1e9 <= .3, "Simulation clock is stale at native self admission")
        require(not self.clock_backwards, "Simulation clock moved backwards")
        clock_age_sec = (now-self._self_clock_received_ns)/1e9
        require(scheduled_start is None or scheduled_start-self.clock_last-clock_age_sec > .1,
                "Native self admission exhausted the original future start reservation")

    def save_evidence(self, stage):
        path = self.launch_log_path.parent / "trajectories.json"
        atomic_json(
            path,
            dict(
                schema_version=1,
                stage=stage,
                case=self.case,
                case_sha256=self.case_sha256,
                initial_rad=self.initial,
                assessments=self.assessments,
                executions=self.executions,
                self_policy=self.self_policy, self_policy_sha256=self.self_policy_sha256,
                **self.evidence,
            ),
        )
        return path

    def planned(self, group, target, start):
        # The comparison policy applies to the crossing motion, not its setup path.
        if group != "both_arms" or target != self.case["goal_rad"]:
            answer = self.plan(group, target, start)
            require(
                answer.error_code.val == MoveItErrorCodes.SUCCESS,
                "MoveIt failed: " + group,
            )
            return self.anchored(answer.trajectory.joint_trajectory)
        policy_path, policy = load_policy(self.cell)
        require(policy["case"] == self.case["id"], "Planning policy/case mismatch")
        selection = dict(
            group=group,
            start_rad=dict(start),
            target_rad=target,
            max_candidates=MAX_PLAN_CANDIDATES,
            policy=policy,
            policy_sha256=file_sha256(policy_path),
            candidates=[],
        )
        self.evidence.setdefault("planning_selection", []).append(selection)

        def propose():
            answer = self.plan(group, target, start)
            code = answer.error_code.val
            trajectory = (
                self.anchored(answer.trajectory.joint_trajectory)
                if code == MoveItErrorCodes.SUCCESS
                else None
            )
            return code, trajectory

        def save(candidates):
            selection["candidates"] = candidates
            self.save_evidence("clearance_candidate_selection")

        with DistanceEvaluator(self.cell, self_collision=True) as evaluator:
            selection["evaluator"] = evaluator.metadata()
            return select_clearance_plan(
                propose,
                start,
                self.cell,
                evaluator,
                policy,
                lambda curve: self.assess([curve], start),
                save, self_policy=self.self_policy,
            )

    def assess(self, curves, initial, *, complete=True):
        samples = sampled_parallel(
            curves, initial, complete=complete, dt=self.case["sample_dt_sec"]
        )
        invalid, cross, witness = 0, 0, None
        for stamp, state in samples:
            answer = self.validity_of(state)
            if answer.valid:
                continue
            invalid += 1
            pairs = [(c.contact_body_1, c.contact_body_2) for c in answer.contacts]
            prefixes = [i.prefix for i in self.cell.instances]
            inter_arm = any(
                (a.startswith(prefixes[0]) and b.startswith(prefixes[1]))
                or (a.startswith(prefixes[1]) and b.startswith(prefixes[0]))
                for a, b in pairs
            )
            cross += int(inter_arm)
            if witness is None or (inter_arm and not witness["inter_arm"]):
                witness = dict(
                    time_from_start_sec=stamp,
                    state_rad=state,
                    contacts=pairs,
                    inter_arm=inter_arm,
                )
        return dict(
            samples=len(samples),
            invalid_samples=invalid,
            inter_arm_collision_samples=cross,
            first_collision=witness,
        )

    def split(self, trajectory, start):
        indices = partitions(trajectory.joint_names, self.cell.instances)
        commands = []
        for inst in self.cell.instances:
            arm = JointTrajectory()
            arm.header.stamp = Time(sec=int(start), nanosec=int((start % 1) * 1e9))
            arm.joint_names = inst.joints
            for point in trajectory.points:
                cloned = deepcopy(point)
                for key in ("positions", "velocities", "accelerations", "effort"):
                    values = getattr(point, key)
                    if values:
                        setattr(cloned, key, [values[j] for j in indices[inst.id]])
                arm.points.append(cloned)
            commands.append((inst, arm))
        return commands

    def checked_dispatch(
        self, commands, initial, *, scheduled_start, expect_rejection=False
    ):
        curves = [Spline.from_ros(t) for _, t in commands]
        assessment = self.assess(curves, initial)
        if assessment["invalid_samples"]:
            raise ParallelPlanRejected(assessment)
        require(
            not expect_rejection,
            "Reference plans did not demonstrate interference; no goals dispatched",
        )
        self.dispatch(commands, scheduled_start=scheduled_start)
        return assessment

    def _self_command_key(self, commands, *, headers=True):
        values=[]
        for inst,trajectory in commands:
            header=getattr(trajectory,"header",None)
            stamp=getattr(header,"stamp",None)
            value=dict(arm=getattr(inst,"id",inst),controller=getattr(inst,"controller",None),
                frame=getattr(header,"frame_id",""),names=list(trajectory.joint_names),points=[
                    dict(p=list(p.positions),v=list(p.velocities),a=list(p.accelerations),
                         effort=list(getattr(p,"effort",[])),sec=p.time_from_start.sec,ns=p.time_from_start.nanosec)
                    for p in trajectory.points])
            if headers:value["stamp"]=None if stamp is None else [stamp.sec,stamp.nanosec]
            values.append(value)
        return json.dumps(values,sort_keys=True,separators=(",",":"),allow_nan=False)

    def _self_file_stats(self, paths):
        require(0<len(paths)<=4096,"Native model file inventory exceeds its bound")
        def key(stat):return [stat.st_dev,stat.st_ino,stat.st_mode,stat.st_size,stat.st_mtime_ns,stat.st_ctime_ns]
        return {str(p):[key(Path(p).lstat()),key(Path(p).stat())] for p in paths}

    def _self_model_binding(self, expected=None):
        if expected is not None:
            require(self._self_file_stats(expected["stats"])==expected["stats"],"Native model files changed after preparation")
            return expected
        paths={(self.cell.output/"cell.xml"),self.cell.output,self.cell.output/"source.json"}
        report=json.loads((self.cell.output/"source.json").read_text())
        paths.update(self.cell.output/name for name in report["outputs"])
        paths.update(self.cell.root/name for name in report["inputs"])
        for inst in self.cell.instances:
            profile=inst.profile
            metadata=json.loads((profile.output/"source.json").read_text())
            paths.update((profile.output,profile.output/"source.json"))
            paths.update(profile.root/name for name in metadata["integrity"]["inputs"]["files"])
            paths.update(profile.output/name for name in metadata["integrity"]["outputs"])
            visual=profile.output/"visual_meshes"
            if visual.exists():paths.add(visual)
        stats=self._self_file_stats(sorted(map(str,paths)))
        check_workcell(self.cell)  # full content checks happen before any fresh reservation
        require(self._self_file_stats(stats)==stats,"Native model changed during preparation")
        return dict(xml_sha256=file_sha256((self.cell.output/"cell.xml")),
            manifest_sha256=file_sha256(self.cell.output/"source.json"),stats=stats)

    def _self_boundary(self, prepared, label):
        now=time.monotonic_ns();record=prepared["record"]
        clock_age=None if self._self_clock_received_ns is None else (now-self._self_clock_received_ns)/1e9
        state_age=None if self._self_state_received_ns is None else (now-self._self_state_received_ns)/1e9
        seal=getattr(self,"_native_reservation",(None,))
        start=seal[1] if seal[0] is prepared else prepared.get("fixed_start")
        row=dict(stage=label,monotonic_ns=now,clock_sec=self.clock_last,clock_age_sec=clock_age,state_age_sec=state_age,
            remaining_start_sec=None if start is None or clock_age is None else start-self.clock_last-clock_age,
            dispatched_goals=self.dispatched)
        rows=record.setdefault("boundaries",[])
        if len(rows)<32:rows.append(row)
        else:
            rows[-1]=row;record["boundary_records_omitted"]=record.get("boundary_records_omitted",0)+1

    def _self_proof_key(self, prepared):
        return json.dumps({key:prepared[key] for key in ("command_key","initial","policy","policy_sha256","identity","model","fresh","observed_anchor","fixed_start")},
            sort_keys=True,separators=(",",":"),allow_nan=False)

    def _confirm_prepared(self, prepared, commands, *, scheduled_start=None):
        require(getattr(self,"_native_prepared",None) is prepared and prepared["active"],"Native preparation owner/lifetime differs")
        if prepared["prior_abort"] is not None:prepared["prior_abort"]()
        require(self._self_proof_key(prepared)==prepared["binding"],"Native preparation proof was changed")
        require(self._self_command_key(commands,headers=not prepared["fresh"])==prepared["command_key"],
                "Prepared relative trajectory, arm order or derivatives changed")
        require(dict(self.commanded)==prepared["initial"],"Commanded start changed after native preparation")
        require(self.self_policy==prepared["policy"] and self.self_policy_sha256==prepared["policy_sha256"]
                and file_sha256(self.self_policy_path)==prepared["policy_sha256"],
                "Self-clearance policy changed before submission")
        require(self_evaluator_identity(prepared["evaluator"],self.cell.root)==prepared["identity"],
                "Native self-clearance implementation changed before submission")
        self._self_model_binding(prepared["model"])
        prepared["record"]["observed_start"]=require_self_clearance(prepared["evaluator"],self.state(),
            self.self_policy["observed_start_min_self_distance_m"])
        if prepared["fresh"]:
            tolerance=self.settings["tracking_tolerance_rad"]
            require(max(abs(self.state()[n]-prepared["observed_anchor"][n]) for n in prepared["observed_anchor"])<tolerance,
                    "Observed start changed after native preparation")
        self._self_boundary(prepared,"final_current_identity_geometry")
        self._self_dispatch_fresh(scheduled_start)
        if prepared["prior_abort"] is not None:prepared["prior_abort"]()
        self._self_dispatch_fresh(scheduled_start)

    @contextmanager
    def prepare_dispatch(self, trajectories, *, scheduled_start=None, fresh_relative=False):
        """Check full relative geometry before a caller may reserve a fresh start.

        Already stamped callers retain their original absolute reservation;
        this context never chooses, extends or rebases a start time.
        """
        require(getattr(self,"_native_prepared",None) is None,"A native dispatch preparation is already active")
        commands=deepcopy(list(trajectories));initial=deepcopy(dict(self.commanded));policy=deepcopy(self.self_policy);policy_sha=self.self_policy_sha256
        command_key=self._self_command_key(commands,headers=not fresh_relative)
        if fresh_relative:
            require(scheduled_start is None,"Fresh relative preparation cannot replace an existing reservation")
            require(all(getattr(t,"header",None) is not None and t.header.stamp.sec==0 and t.header.stamp.nanosec==0
                        for _,t in commands),"Fresh relative commands are already stamped")
        record=dict(scheduled_start_sec=scheduled_start,reservation_mint_count=0,
            reservation_origin="fresh_relative_caller" if fresh_relative else "existing_absolute_preserved",
            initial_commanded_rad=deepcopy(initial),status="preparing",dispatched_goals_before=self.dispatched,stage_elapsed_sec={})
        self.evidence.setdefault("dispatch_admission",[]).append(record)
        prepared=dict(record=record,fresh=fresh_relative,active=False,used=False,fixed_start=scheduled_start,
            prior_abort=getattr(self,"dispatch_abort",None))
        primary=None;started=time.monotonic_ns()
        self._self_boundary(prepared,"prepare_begin")
        try:
            model=self._self_model_binding()
            curves=[Spline.from_ros(t) for _,t in commands]
            names={name for curve in curves for name in curve.names};held=[name for name in initial if name not in names]
            if held:
                duration=max(c.duration for c in curves)
                curves.append(Spline(held,[dict(t=t,p=[initial[n] for n in held],v=[],a=[]) for t in (0.,duration)]))
            require(file_sha256(self.self_policy_path)==policy_sha,"Self-clearance policy changed; restart and replan")
            with DistanceEvaluator(self.cell,self_collision=True) as evaluator:
                identity=self_evaluator_identity(evaluator,self.cell.root)
                scan_started=time.monotonic_ns()
                result=evaluate_path(curves,initial,self.cell,evaluator,dt=policy["sample_dt_sec"],
                    self_reject_below=policy["planning_min_self_distance_m"])
                record["stage_elapsed_sec"]["native_full_scan"]=(time.monotonic_ns()-scan_started)/1e9
                complete=result["complete"]
                record.update(native_self_clearance=deepcopy(result),evaluator=deepcopy(evaluator.metadata()),model_identity={
                    "xml_sha256":model["xml_sha256"],"manifest_sha256":model["manifest_sha256"],"bound_files":len(model["stats"])},status="pre_dispatch")
                checkpoint=time.monotonic_ns();self.save_evidence("native_self_dispatch_gate")
                record["stage_elapsed_sec"]["prepare_checkpoint"]=(time.monotonic_ns()-checkpoint)/1e9
                if not complete:
                    record["status"]="rejected"
                    raise SelfClearanceRejected("Native self-clearance rejected the legacy schedule before dispatch")
                self.ready_guard()
                prepared.update(commands=deepcopy(commands),command_key=command_key,
                    initial=deepcopy(initial),policy=policy,policy_sha256=policy_sha,identity=identity,
                    model=model,evaluator=evaluator,observed_anchor=self.state(),active=True)
                prepared["binding"]=self._self_proof_key(prepared)
                self._native_prepared=prepared
                self._confirm_prepared(prepared,commands,scheduled_start=None if fresh_relative else scheduled_start)
                record["stage_elapsed_sec"]["prepare_total"]=(time.monotonic_ns()-started)/1e9
                self._self_boundary(prepared,"prepare_complete_before_reservation")
                yield prepared
        except BaseException as error:
            primary=error;record["error"]=dict(type=type(error).__name__,message=str(error)[:2000])
            if record["status"]!="rejected":record["status"]="failed"
            raise
        finally:
            prepared["active"]=False
            if getattr(self,"_native_prepared",None) is prepared:self._native_prepared=None
            record.update(dispatched_goals_after=self.dispatched,execution_transport=deepcopy(getattr(self,"execution_transport",None)),
                contact_guard=deepcopy(self.contact_guard))
            self._self_boundary(prepared,"terminal_or_prefinal_failure")
            if getattr(self,"_native_reservation",(None,))[0] is prepared:self._native_reservation=(None,)
            try:self.save_evidence("dispatch_terminal")
            except BaseException as error:
                record["persistence_error"]=dict(type=type(error).__name__,message=str(error)[:2000])
                if primary is None:raise

    def reserve_prepared(self, prepared):
        """The fresh caller mints its original one-second lead exactly once."""
        require(prepared["fresh"] and not prepared["used"] and getattr(self,"_native_reservation",(None,))[0] is not prepared,
                "Native preparation already reserved or consumed")
        self._confirm_prepared(prepared,prepared["commands"])
        start=self.clock_last+1.0
        sec,ns=divmod(round(start*1e9),10**9)
        minted=time.monotonic_ns()
        # Authority is private/immutable, separate from mutable diagnostics.
        self._native_reservation=(prepared,start,(sec,ns),self.clock_last,minted)
        prepared["record"].update(scheduled_start_sec=start,reservation_mint_count=1,
            reserved_from_clock_sec=self.clock_last,reservation_lead_sec=1.0,reservation_minted_monotonic_ns=minted)
        self._self_boundary(prepared,"caller_reserved_once")
        return start

    def stamp_prepared(self, trajectories, prepared):
        require(getattr(self,"_native_reservation",(None,))[0] is prepared,"Fresh native reservation is missing")
        commands=deepcopy(list(trajectories))
        require(self._self_command_key(commands,headers=False)==prepared["command_key"],"Prepared relative trajectory changed before stamping")
        sec,ns=self._native_reservation[2]
        for _,trajectory in commands:trajectory.header.stamp=Time(sec=sec,nanosec=ns)
        return commands

    def dispatch_prepared(self, trajectories, prepared, *, inject_cancel_at=None, scheduled_start=None):
        commands=list(trajectories)
        require(not prepared["used"],"Native dispatch preparation was already consumed")
        prepared["used"]=True
        if prepared["fresh"]:
            seal=getattr(self,"_native_reservation",(None,))
            require(seal[0] is prepared and scheduled_start==seal[1],"Original native start reservation differs")
            require(prepared["record"]["reservation_mint_count"]==1 and prepared["record"]["scheduled_start_sec"]==seal[1],
                    "Native reservation diagnostics changed")
            require(all((t.header.stamp.sec,t.header.stamp.nanosec)==seal[2] for _,t in commands),"Prepared absolute headers differ")
        else:
            require(scheduled_start==prepared["fixed_start"],"Original native start reservation differs")
        previous_abort=getattr(self,"dispatch_abort",None)
        def admission_abort():
            if previous_abort is not None:previous_abort()
            require(not self.contact_guard or not self.contact_guard["latched"],"Native contact guard latched")
            ledger=getattr(self,"execution_ledger",None)
            new_submission=ledger is not None and any(not item.submission_started for item in ledger.attempts)
            if new_submission:self._confirm_prepared(prepared,commands,scheduled_start=scheduled_start)
            if previous_abort is not None:previous_abort()
            if new_submission:self._self_dispatch_fresh(scheduled_start)
        if previous_abort is not None:previous_abort()
        require(not self.contact_guard or not self.contact_guard["latched"],"Native contact guard latched")
        self._confirm_prepared(prepared,commands,scheduled_start=scheduled_start)
        self.dispatch_abort=admission_abort
        try:
            result=super().dispatch(commands,inject_cancel_at=inject_cancel_at,scheduled_start=scheduled_start)
            prepared["record"]["status"]="terminal"
            return result
        finally:self.dispatch_abort=previous_abort

    def dispatch(self, trajectories, *, inject_cancel_at=None, scheduled_start=None):
        """Strict admission for existing absolute/saved schedules; never retime."""
        commands=list(trajectories)
        with self.prepare_dispatch(commands,scheduled_start=scheduled_start) as prepared:
            return self.dispatch_prepared(commands,prepared,inject_cancel_at=inject_cancel_at,scheduled_start=scheduled_start)

    def before_avoidance_execution(self):
        pass

    def execute_joint(self, trajectory, initial, label):
        curve = Spline.from_ros(trajectory)
        check = self.assess([curve], initial)
        require(check["invalid_samples"] == 0, "Coordinated spline is colliding")
        if label == "avoidance":
            self.before_avoidance_execution()
        relative_commands=self.split(trajectory,0.)
        with self.prepare_dispatch(relative_commands,fresh_relative=True) as prepared:
            before=self.state()
            start=self.reserve_prepared(prepared)
            self.dispatch_prepared(self.stamp_prepared(relative_commands,prepared),prepared,scheduled_start=start)
        endpoint = dict(zip(curve.names, curve.points[-1]["p"]))
        error = self.settle([endpoint[n] for n in self.joint_names])
        require(
            all(f.result().status == 4 for _, f in self.handles),
            "Both goals must succeed",
        )
        require(
            not self.contact_guard["latched"], "A stop interlock interrupted execution"
        )
        record = dict(
            label=label,
            duration_sec=curve.duration,
            collision_check=check,
            common_start_sec=start,
            goal_terminal_statuses=[f.result().status for _, f in self.handles],
            tracking_error_rad=error,
            movement_rad={
                i.id: max(abs(self.state()[n] - before[n]) for n in i.joints)
                for i in self.cell.instances
            },
        )
        self.executions.append(record)
        return record

    def discover(self):
        self.progress("interference_discovery")
        self.until(
            lambda: self.positions is not None and self.clock_count > 10,
            30,
            "shared states",
        )
        self.ready_guard()
        for client in self.arm_clients.values():
            self.until(client.server_is_ready, 10, "arm server")
        for client in (self.validity, self.planner):
            self.until(client.service_is_ready, 15, "MoveIt service")
        for inst in self.cell.instances:
            service = self.create_client(
                GetParameters, f"/{inst.controller}/get_parameters"
            )
            self.until(service.service_is_ready, 5, "controller parameters")
            request = GetParameters.Request(names=["interpolation_method"])
            response = self.result(
                service.call_async(request), 5, "controller interpolation"
            )
            require(
                response.values[0].string_value == "splines",
                "Controller spline interpolation required",
            )
        service = self.create_client(GetParameters, "/move_group/get_parameters")
        self.until(service.service_is_ready, 5, "MoveIt parameters")
        keys = [
            "ompl.path_tolerance",
            "ompl.resample_dt",
            "ompl.both_arms.longest_valid_segment_fraction",
        ]
        response = self.result(
            service.call_async(GetParameters.Request(names=keys)),
            5,
            "strict planning parameters",
        )
        observed = [p.double_value for p in response.values]
        expected = [
            self.case["planning"][k]
            for k in ("path_tolerance", "resample_dt", "longest_valid_segment_fraction")
        ]
        require(observed == expected, "Strict scenario planning parameters not applied")
        self.settle(self.cell.home)
        for key in ("start_rad", "goal_rad"):
            require(
                self.validity_of(self.case[key]).valid,
                "Case endpoint is colliding: " + key,
            )

    def run(self):
        self.discover()
        self.progress("move_to_common_start")
        setup = self.planned("both_arms", self.case["start_rad"], self.commanded)
        self.evidence["setup"] = Spline.from_ros(setup).record()
        self.save_evidence("setup_plan")
        self.execute_joint(setup, dict(self.commanded), "setup")
        initial = self.initial = dict(self.commanded)
        measured_initial = self.state()
        goal = self.case["goal_rad"]
        independent = []
        self.progress("independent_arm_plans")
        for inst in self.cell.instances:
            trajectory = self.planned(
                inst.group, {n: goal[n] for n in inst.joints}, initial
            )
            independent.append((inst, trajectory))
        common_duration = max(Spline.from_ros(t).duration for _, t in independent)
        independent = [(inst, stretch(t, common_duration)) for inst, t in independent]
        single_checks = {}
        for inst, trajectory in independent:
            curve = Spline.from_ros(trajectory)
            check = self.assess([curve], initial, complete=False)
            require(
                check["invalid_samples"] == 0,
                "Independent arm plan is not safe with peer at start",
            )
            single_checks[inst.id] = check
            self.evidence["independent_" + inst.id] = curve.record()
            self.assessments[inst.id] = check
            self.save_evidence("independent_" + inst.id)
        self.progress("parallel_interference_rejected_before_dispatch")
        before_dispatch = self.dispatched
        try:
            self.checked_dispatch(
                independent,
                initial,
                scheduled_start=self.clock_last + 1,
                expect_rejection=True,
            )
        except ParallelPlanRejected as error:
            parallel = error.assessment
        else:
            raise RuntimeError("Interfering plans passed the dispatch gate")
        self.assessments["parallel"] = parallel
        self.save_evidence("parallel_rejected")
        require(
            parallel["inter_arm_collision_samples"] > 0,
            "Reference plans did not demonstrate cross-arm interference",
        )
        require(self.dispatched == before_dispatch, "Unsafe arm goals were dispatched")
        self.progress("coordinated_avoidance_plan")
        coordinated = self.planned("both_arms", goal, initial)
        self.evidence["coordinated"] = Spline.from_ros(coordinated).record()
        self.save_evidence("coordinated_plan")
        execution = self.execute_joint(coordinated, initial, "avoidance")
        require(
            all(v > 0.8 for v in execution["movement_rad"].values()),
            "Both arms did not traverse the crossing case",
        )
        require(
            max(abs(self.state()[n] - goal[n]) for n in self.joint_names) < 0.01,
            "Case goal was not reached",
        )
        require(
            file_sha256(self.case_path) == self.case_sha256,
            "Case changed during verification",
        )
        path = self.save_evidence("complete")
        return dict(
            workcell_id=self.cell.id,
            interference_case=self.case["id"],
            case_file_sha256=self.case_sha256,
            initial_commanded_rad=initial,
            initial_measured_rad=measured_initial,
            goal_constraint_rad=goal,
            independent_arm_checks=single_checks,
            parallel_reference=parallel,
            reference_duration_sec=common_duration,
            unsafe_goals_dispatched=0,
            parallel_dispatch_gate_passed=True,
            coordinated_execution=execution,
            setup_execution=self.executions[0],
            planning_wall_timings=self.planning_timings,
            interpolation="controller splines: linear/cubic/quintic according to available derivatives",
            planning_parameters=self.case["planning"],
            interpolation_max_dt_sec=self.case["sample_dt_sec"],
            interpolation_max_step_rad=0.025,
            trajectories_path=str(path),
            trajectories_sha256=file_sha256(path),
            contact_guard=deepcopy(self.contact_guard),
        )


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--case", default="dual_arm_crossing")
    parser.add_argument("--domain", type=int, default=126)
    parser.add_argument("--gui", action="store_true")
    parser.add_argument("--screenshot-dir", type=Path)
    parser.add_argument("--record-dir", type=Path)
    args = parser.parse_args()
    try:
        identifier(args.case)
    except ValueError as error:
        parser.error(str(error))
    if args.record_dir and not args.gui:
        parser.error("--record-dir requires --gui")
    factory = lambda cell, progress: InterferenceVerification(cell, progress, args.case)
    if args.record_dir:
        from mujoco_ros2_manipulation.workcells.record_interference import (
            RecordedInterference,
        )

        factory = lambda cell, progress: RecordedInterference(
            cell, progress, args.case, args.record_dir
        )
    args.robot, args.scene, args.stop_mode = "fr5_ur5e", "default", "interrupt"
    args.require_gpu = args.gui
    return run_verification(
        args,
        ROOT,
        rclpy,
        factory,
        Trigger,
        profile_loader=lambda *a, **kw: load_workcell(root=kw["root"]),
        validator=lambda cell: preflight(cell, args.case),
        command_builder=lambda cell, args: [
            "bash",
            str(ROOT / "run_scene.sh"),
            f"workcell:={cell.id}",
            f"planning_case:={args.case}",
            f"gui:={str(args.gui).lower()}",
            f"rviz:={str(args.gui).lower()}",
        ],
        report_prefix="interference_" + args.case + "_",
        launch_environment=diagnostic_environment,
        finalize=finalize_physics,
    )


if __name__ == "__main__":
    raise SystemExit(main())
