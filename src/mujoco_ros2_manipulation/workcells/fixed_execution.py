"""One fresh fixed-suite row; preparation and scored computation stay separate.

This module intentionally has no CLI or launch orchestration. The caller validates
its complete suite and owns run_verification, native physics checks and cleanup.
A planning failure is an evaluation outcome, never a reason to rerun a scored row.
"""
from contextlib import contextmanager
from copy import deepcopy
import json
import math
from pathlib import Path
import re
import uuid

from std_msgs.msg import String

from mujoco_ros2_manipulation.model_build.integrity import file_sha256
from mujoco_ros2_manipulation.verification.reports import atomic_json
from mujoco_ros2_manipulation.workcells.motion_request import COMMAND_TOPIC, command
from mujoco_ros2_manipulation.workcells.motion_server import MotionCommandServer
from mujoco_ros2_manipulation.workcells.fixed_suite import POLICY
from mujoco_ros2_manipulation.workcells.reproduction import load_reproduction
from mujoco_ros2_manipulation.workcells.verify import require
from mujoco_ros2_manipulation.workcells.verify_motion_reproduction import MotionReproduction


SCORED_BUDGET_SEC = 5.0
INITIAL_TOLERANCE_RAD = .0015
MAINTENANCE_BUDGET_SEC = 30.0
MAINTENANCE_GOAL_TOLERANCE_RAD = .00001
MAX_CORRECTIONS = 3
MAX_CORRECTION_RAD = .03
PLANNING_FAILURE_KINDS = frozenset(("candidate_exhausted", "budget_exhausted", "planner_failed"))


class FixedCaseExecution(MotionReproduction):
    """Run one checked case/variant/repeat in a newly owned simulator session.

    Fresh plans are used for every scored request and ordinary preparation.
    Only the public home fixture's fourth preparation path may be reused, with
    MotionReproduction.plan's normal anchoring and all current execution gates.
    The feedback corrections select ordinary command targets; they never replace
    the planner, modify observed joint state, or relax collision checks.
    """

    def __init__(self, cell, progress, case, row, *, output=None, suite_sha256=None):
        require(isinstance(case, dict) and isinstance(row, dict), "Expected fixed case and row objects")
        require(case.get("workcell") == cell.id and row.get("case_id") == case.get("case_id"),
                "Fixed row/case/workcell identity mismatch")
        require(case.get("expectation") in ("valid_goal", "invalid_goal"), "Unknown fixed case expectation")
        require(row.get("variant") in ("single", "bounded")
                and type(row.get("max_plan_candidates")) is int
                and row["max_plan_candidates"] == {"single": 1, "bounded": 12}[row["variant"]]
                and type(row.get("repeat")) is int and row["repeat"] >= 1,
                "Fixed row variant, candidate cap or repeat mismatch")
        require(suite_sha256 is None or (isinstance(suite_sha256, str)
                and re.fullmatch(r"[0-9a-f]{64}", suite_sha256)), "Invalid suite SHA256")
        self.case, self.row = deepcopy(case), deepcopy(row)
        directory = output or cell.root / "runtime/validation/fixed-case-sessions" / uuid.uuid4().hex
        # Deliberately skip MotionReproduction.__init__: each row has its own
        # session, scored settings and optional fixture, rather than repeat state.
        MotionCommandServer.__init__(self, cell, directory, progress,
            compute_budget_sec=SCORED_BUDGET_SEC, max_plan_candidates=row["max_plan_candidates"],
            policy_name=case["policy_name"])
        require({key: self.policy.get(key) for key in POLICY} == POLICY,
                "Fixed evaluation policy changed; expected 30mm planned/20mm observed clearance")
        self.command_publisher = self.create_publisher(String, COMMAND_TOPIC, 10)
        self.preparation = None
        self._scored_started = False
        self.fixture = None
        self.result_record = dict(
            schema_version=1, status="running", case_id=case["case_id"], variant=row["variant"],
            repeat=row["repeat"], row_index=row.get("index"), suite_sha256=suite_sha256,
            session_id=self.session_id, workcell=cell.id, policy_name=case["policy_name"], policy=deepcopy(self.policy),
            source_sha256=file_sha256(Path(__file__)), case=deepcopy(case),
            initial_tolerance_rad=INITIAL_TOLERANCE_RAD, nominal_initial_rad=deepcopy(case["initial_rad"]),
            actual_initial_rad=None, endpoint_preflight=None,
            scored_settings=dict(compute_budget_wall_sec=SCORED_BUDGET_SEC,
                                 max_plan_candidates=row["max_plan_candidates"], goal_tolerance_rad=self.goal_tolerance_rad),
            maintenance_settings=dict(compute_budget_wall_sec=MAINTENANCE_BUDGET_SEC,
                max_plan_candidates=12, goal_tolerance_rad=MAINTENANCE_GOAL_TOLERANCE_RAD,
                max_corrections=MAX_CORRECTIONS, max_correction_rad=MAX_CORRECTION_RAD),
            preparation_stages=[], maintenance=[], scored_attempted=False, scored_request_id=None, scored=None, error=None,
            note="One scored request per fresh row. Preparation, warmup, endpoint preflight and bounded feedback corrections are unscored. Partial path minima remain partial evidence.")
        self.save_result()
        for state, role in ((case["initial_rad"], "initial"), (case["target_rad"], "target")):
            self._check_state(state, role)

    def result_path(self):
        return self.directory / "fixed-case.json"

    def save_result(self):
        atomic_json(self.result_path(), self.result_record)

    def _check_state(self, state, role):
        require(isinstance(state, dict) and set(state) == set(self.joint_names), role + " must cover all workcell joints")
        require(all(type(q) in (int, float) and math.isfinite(q)
                    and self.metadata["joint_limits"][name]["lower"] <= q <= self.metadata["joint_limits"][name]["upper"]
                    for name, q in state.items()), role + " contains a nonfinite or out-of-limit position")

    def _error(self, desired, actual=None):
        actual = self.state() if actual is None else actual
        return max(abs(desired[name] - actual[name]) for name in self.joint_names)

    def _healthy(self):
        require(not self.blocked and self.contact_guard is not None and not self.contact_guard["latched"],
                "Execution/interlock failure; no later fixed-suite request is allowed")
        self.check_abort()

    @contextmanager
    def _maintenance_settings(self):
        previous = self.compute_budget_sec, self.max_plan_candidates, self.goal_tolerance_rad
        self.compute_budget_sec, self.max_plan_candidates = MAINTENANCE_BUDGET_SEC, 12
        self.goal_tolerance_rad = MAINTENANCE_GOAL_TOLERANCE_RAD
        try:
            yield
        finally:
            self.compute_budget_sec, self.max_plan_candidates, self.goal_tolerance_rad = previous

    def submit(self, target, *, role, scored=False, desired_actual=None, correction_rad=None):
        """Preserve the terminal raw trace and SHA before evaluating its outcome."""
        self._check_state(target, role + " target")
        if scored:
            require(not self._scored_started, "A fixed row may issue only one scored request")
        payload = command(target, "coordinated")
        if scored:
            self._scored_started = True
            self.result_record.update(scored_attempted=True, scored_request_id=payload["request_id"])
            self.save_result()  # Attempt is durable; this does not prove receipt or dispatch.
        path = self.directory / (payload["request_id"] + ".json")
        commanded_before = dict(self.commanded)
        problem = None
        try:
            self.command_publisher.publish(String(data=json.dumps(payload)))
            self.until(lambda: self.pending is not None or payload["request_id"] in self.replies,
                       5, "fixed case command topic receipt")
            if self.pending is not None:
                require(self.pending.data["request_id"] == payload["request_id"],
                        "Another request owns the fixed case server")
                self.process_next()
        except BaseException as error:
            problem = error
        record = None
        if path.is_file():
            record = json.loads(path.read_text())
            require(record["request_id"] == payload["request_id"], "Fixed case trace request identity mismatch")
            entry = dict(role=role, request_id=payload["request_id"], status=record["status"],
                trace_file=path.name, trace_sha256=file_sha256(path),
                command=deepcopy(record.get("command", payload)), initial_state_rad=deepcopy(record.get("initial_state_rad")),
                commanded_before_rad=commanded_before, commanded_after_rad=dict(self.commanded),
                desired_actual_state_rad=deepcopy(desired_actual), correction_rad=deepcopy(correction_rad),
                dispatched_goals=record.get("dispatched_goals", 0), action_results=deepcopy(record.get("action_results", [])),
                no_motion_required=record.get("no_motion_required", False),
                failure_kind=record.get("failure_kind"), failure_stage=record.get("failure_stage"),
                error=deepcopy(record.get("error")), metrics=deepcopy(record.get("metrics", {})),
                compute_budget=deepcopy(record.get("compute_budget")),
                preparation_source=deepcopy(record.get("preparation_source")))
            if scored:
                require(self.result_record["scored"] is None, "A fixed row may have only one scored request")
                self.result_record["scored"] = entry
            else:
                self.result_record["maintenance"].append(entry)
            self.save_result()
        if problem is not None:
            raise problem
        require(record is not None, "Fixed request has no raw trace; preserve the aborted row")
        return record, entry

    def _successful_execution(self, record, *, physical_motion):
        self._healthy()
        require(record["status"] == "succeeded", "Request did not succeed: " + str(record.get("error")))
        if record.get("no_motion_required"):
            require(not physical_motion and record.get("dispatched_goals") == 0
                    and record["metrics"].get("received_to_first_motion_wall_sec") is None,
                    "Scored movement or no-op evidence is inconsistent")
            return
        threshold = self.policy["min_inter_arm_distance_m"] + self.policy["tracking_reserve_m"]
        require(record.get("dispatched_goals") == 2 and len(record.get("action_results", [])) == 2
                and all(item["status"] == 4 and item["error_code"] == 0 for item in record["action_results"]),
                "Successful fixed request did not complete both actual arm actions")
        require(record["metrics"].get("received_to_first_motion_wall_sec") is not None,
                "Successful moving request has no physical motion observation")
        planned, observed = record.get("planned_clearance", {}), record.get("observed_clearance", {})
        require(planned.get("complete") is True and planned["minimum"]["distance_m"] >= threshold
                and observed.get("complete") is True
                and observed["minimum"]["distance_m"] >= self.policy["min_inter_arm_distance_m"],
                "Successful fixed request lacks complete passing planned/observed clearance")
        require(record.get("compute_budget", {}).get("exceeded") is False,
                "Successful request exceeded its declared computation budget")

    def _maintenance_request(self, target, *, role, desired_actual=None, correction_rad=None, preparation=None):
        self._healthy()
        self.quiet()
        previous_preparation = self.preparation
        try:
            with self._maintenance_settings():
                self.preparation = preparation
                record, entry = self.submit(target, role=role, desired_actual=desired_actual,
                                             correction_rad=correction_rad)
        finally:
            self.preparation = previous_preparation
        self._successful_execution(record, physical_motion=False)
        self.quiet()
        entry["actual_after_rad"] = self.state()
        if desired_actual is not None:
            entry["desired_minus_actual_rad"] = {name: desired_actual[name] - self.state()[name] for name in self.joint_names}
            entry["max_actual_error_rad"] = self._error(desired_actual)
        self.save_result()
        return entry

    def drive_to_state(self, desired, role, *, initial_request=True):
        """Reach the actual fixed state using at most three bounded unscored corrections.

        A home fixture may already have supplied the initial request, so its
        remaining actual-state check can start directly with feedback correction.
        All targets still enter through the normal production command topic.
        """
        self._check_state(desired, role + " desired state")
        self._healthy()
        self.quiet()
        stage = dict(role=role, desired_actual_state_rad=dict(desired), initial_actual_rad=self.state(),
                     initial_error_rad=self._error(desired), initial_request=initial_request,
                     request_ids=[], correction_requests=0, status="running")
        self.result_record["preparation_stages"].append(stage)
        self.save_result()
        if initial_request:
            entry = self._maintenance_request(desired, role=role + ":nominal", desired_actual=desired)
            stage["request_ids"].append(entry["request_id"])
            self.save_result()
        for index in range(1, MAX_CORRECTIONS + 1):
            self._healthy()
            self.quiet()
            actual = self.state()
            if self._error(desired, actual) <= INITIAL_TOLERANCE_RAD:
                break
            correction = {name: desired[name] - actual[name] for name in self.joint_names}
            require(max(abs(value) for value in correction.values()) <= MAX_CORRECTION_RAD,
                    "Unscored state correction exceeds 0.03rad; preparation aborted")
            target = {name: self.commanded[name] + correction[name] for name in self.joint_names}
            entry = self._maintenance_request(target, role=role + ":correction:" + str(index),
                desired_actual=desired, correction_rad=correction)
            stage["request_ids"].append(entry["request_id"])
            stage["correction_requests"] = index
            self.save_result()
        self.quiet()
        stage.update(final_actual_rad=self.state(), final_error_rad=self._error(desired))
        stage["status"] = "converged" if stage["final_error_rad"] <= INITIAL_TOLERANCE_RAD else "not_converged"
        self.save_result()
        require(stage["status"] == "converged", "Fixed-state preparation did not converge within three corrections")
        return stage

    def _prepare_case_initial(self):
        preparation = self.case["preparation"]
        if preparation["kind"] == "home_fixture":
            relative = Path(preparation["fixture"])
            require(not relative.is_absolute() and ".." not in relative.parts, "Home fixture must be repository-relative")
            path = (self.cell.root / relative).resolve()
            require(path.is_relative_to(self.cell.root.resolve()), "Home fixture escapes the repository")
            self.fixture = load_reproduction(path, self.cell, self.metadata)
            require(self.fixture["initial_rad"] == self.case["initial_rad"]
                    and self.fixture["target_rad"] == self.case["target_rad"], "Home fixture differs from the fixed case")
            sequence = self.fixture["setup"]["sequence"]
            require(len(sequence) == 4 and len(self.fixture["_setup_trajectories"]) == 4,
                    "Expected the four validated public preparation steps")
            self.result_record["home_fixture"] = dict(path=relative.as_posix(), sha256=file_sha256(path),
                id=self.fixture["id"], policy="Fresh first three plans; validated saved fourth path; all production gates apply")
            self.save_result()
            for index, step in enumerate(sequence):
                saved = self.fixture["_setup_trajectories"][index] if index == len(sequence) - 1 else None
                self._maintenance_request(step["target_rad"], role="case_initial:home_fixture:" + str(index + 1),
                    desired_actual=step["target_rad"], preparation=saved)
            self.drive_to_state(self.case["initial_rad"], "case_initial:home_fixture_actual", initial_request=False)
        else:
            require(preparation["kind"] == "joint_goal", "Unknown case preparation")
            for index, target in enumerate(preparation["targets"]):
                self.drive_to_state(target, "case_initial:joint_goal:" + str(index + 1))
            require(self._error(self.case["initial_rad"]) <= INITIAL_TOLERANCE_RAD,
                    "Joint preparation did not reach the case's fixed initial state")

    def _endpoint_preflight(self, actual_initial):
        threshold = self.policy["min_inter_arm_distance_m"] + self.policy["tracking_reserve_m"]
        preflight = dict(scored=False, scope="Unscored endpoint check; production recomputes endpoints inside its scored budget",
                         threshold_m=threshold, endpoints={})
        self.result_record["endpoint_preflight"] = preflight
        self.save_result()
        for role, state in (("start", actual_initial), ("goal", self.case["target_rad"])):
            self._healthy()
            native = deepcopy(self.evaluator.at(state))
            require(type(native.get("distance_m")) in (int, float) and math.isfinite(native["distance_m"]),
                    "Endpoint preflight returned nonfinite native distance")
            row = dict(state_rad=dict(state), native=native,
                       native_policy_valid=native["distance_m"] >= threshold, moveit_valid=None)
            preflight["endpoints"][role] = row
            self.save_result()
            row["moveit_valid"] = bool(self.validity_of(state).valid)
            self._healthy()
            row["valid"] = row["native_policy_valid"] and row["moveit_valid"]
            self.save_result()
        endpoints = preflight["endpoints"]
        require(endpoints["start"]["valid"], "Prepared fixed start is invalid in the current scene")
        if self.case["expectation"] == "valid_goal":
            require(endpoints["goal"]["valid"], "Declared valid goal is invalid in the current scene")
        else:
            require(not endpoints["goal"]["valid"], "Declared invalid goal is valid in the current scene")
        if self.case["case_id"] == "environment_post":
            # This is an unscored diagnostic, never a fallback command path.
            # Require the actual MoveIt scene to see the declared fixed box.
            obstacle_link = "workcell_" + self.case["provenance"]["parameters"]["fixed_obstacle"]["name"]
            probe = dict(scored=False, method="21 joint-linear state samples, not a motion plan",
                         obstacle_link=obstacle_link, samples=[], post_contact_samples=0)
            preflight["environment_direct_probe"] = probe
            self.save_result()
            for index in range(21):
                alpha = index / 20
                state = {name: actual_initial[name] + alpha *
                         (self.case["target_rad"][name] - actual_initial[name]) for name in self.joint_names}
                self._healthy()
                response = self.validity_of(state)
                contacts = [dict(body1=c.contact_body_1, body2=c.contact_body_2) for c in response.contacts]
                hit = any(obstacle_link in (c["body1"], c["body2"]) for c in contacts)
                probe["samples"].append(dict(alpha=alpha, state_rad=state, valid=bool(response.valid), contacts=contacts))
                probe["post_contact_samples"] += int(hit)
                self.save_result()
            require(probe["post_contact_samples"] > 0,
                    "MoveIt scene did not detect the fixed post on direct interpolation")
        return preflight

    def _judge_scored(self, record, entry):
        self._healthy()
        require(record.get("initial_state_rad") is not None, "Scored request never captured an actual start")
        entry["initial_error_rad"] = self._error(self.case["initial_rad"], record["initial_state_rad"])
        self.save_result()
        require(entry["initial_error_rad"] <= INITIAL_TOLERANCE_RAD,
                "Scored request did not start from the declared fixed state")
        if self.case["expectation"] == "valid_goal" and record["status"] == "succeeded":
            self._successful_execution(record, physical_motion=True)
            entry["outcome"] = "motion_succeeded"
        else:
            require(record["status"] == "failed" and record.get("dispatched_goals") == 0
                    and record["metrics"].get("received_to_first_motion_wall_sec") is None
                    and not record.get("action_results"), "Scored failure dispatched, moved or has an unexpected terminal status")
            if self.case["expectation"] == "invalid_goal":
                require(record.get("failure_kind") == "invalid_goal", "Invalid goal was not rejected by the production endpoint gate")
                entry["outcome"] = "expected_invalid_goal"
            else:
                require(record.get("failure_kind") in PLANNING_FAILURE_KINDS,
                        "Unexpected planning/transport/state error; the fixed evaluation row is aborted")
                entry["outcome"] = "planning_failed"
        self.save_result()

    def run(self):
        try:
            self.prepare()
            self.until(lambda: self.command_publisher.get_subscription_count() >= 1,
                       5, "fixed case command subscription")
            nominal = dict(zip(self.joint_names, self.cell.home))
            self.drive_to_state(nominal, "nominal_home")
            warmup = dict(nominal)
            for instance in self.cell.instances:
                warmup[instance.joints[0]] += .05
            self.drive_to_state(warmup, "warmup_first_joints_plus_005")
            self.drive_to_state(nominal, "nominal_home_after_warmup")
            self._prepare_case_initial()
            self._healthy()
            self.quiet()
            actual_initial = self.state()
            self.result_record["actual_initial_rad"] = dict(actual_initial)
            self.result_record["initial_error_rad"] = self._error(self.case["initial_rad"], actual_initial)
            self.save_result()
            require(self.result_record["initial_error_rad"] <= INITIAL_TOLERANCE_RAD,
                    "Actual fixed initial state exceeds the suite tolerance before scoring")
            self._endpoint_preflight(actual_initial)
            self.progress("fixed_case_scored_request")
            record, entry = self.submit(self.case["target_rad"], role="scored", scored=True)
            self._judge_scored(record, entry)
            self.result_record["status"] = "completed"
        except BaseException as error:
            self.result_record.update(status="aborted", error=dict(type=type(error).__name__, message=str(error)))
            raise
        finally:
            self.save_result()
        return dict(fixed_case_result_path=str(self.result_path()), fixed_case_result_sha256=file_sha256(self.result_path()),
            command_trace_directory=str(self.directory), scored_request_id=self.result_record["scored"]["request_id"],
            contact_guard=deepcopy(self.contact_guard))
