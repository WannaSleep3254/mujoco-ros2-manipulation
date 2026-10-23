"""Public evidence projection is limited to validated motion-record shapes."""
from copy import deepcopy
import hashlib
import json
import unittest

from mujoco_ros2_manipulation.workcells.evidence_export import audit_public_evidence, export_evidence


class EvidenceExportTests(unittest.TestCase):
    def raw(self, value):
        return json.dumps(value).encode()

    def command(self):
        return dict(schema_version=1, request_id="a" * 32, strategy="coordinated", target_rad={f"joint_{i}": i / 10 for i in range(12)})

    def trace(self):
        return dict(schema_version=1, request_id="a" * 32, status="failed", command=self.command(),
                    initial_state_rad=self.command()["target_rad"], clock=dict(origin_ns=123456789),
                    received_utc="private timestamp", contact_guard=dict(host="private host"),
                    plans=[dict(group="both_arms", wall_sec=.123, server_planning_sec=.1, operator_email="private@example.invalid")],
                    events=[dict(name="terminal", wall_sec=.25, simulation_sec=None, user="private user")],
                    spans=[dict(name="safety_checks", start_wall_sec=.2, end_wall_sec=.25, duration_wall_sec=.05, status="failed")],
                    metrics=dict(received_to_terminal_wall_sec=.25),
                    planned_trajectories=[dict(names=["joint_0"], points=[dict(t=0, p=[.123456789], v=[0], a=[0])])],
                    error=dict(type="RuntimeError", message="Planned clearance below 30mm policy"))

    def test_host_environment_and_monotonic_origin_are_excluded_but_measurements_unchanged(self):
        trace = self.trace()
        raw = self.raw(trace)
        result = export_evidence(raw, "trace")
        for field in ("clock", "received_utc", "contact_guard"):
            self.assertNotIn(field, result)
        for field in ("command", "initial_state_rad", "metrics", "spans", "planned_trajectories"):
            self.assertEqual(result[field], trace[field])
        self.assertNotIn("operator_email", result["plans"][0])
        self.assertNotIn("user", result["events"][0])
        self.assertEqual(result["evidence_export"]["raw_source_sha256"], hashlib.sha256(raw).hexdigest())
        self.assertTrue(audit_public_evidence(result)["passed"])
        session = dict(schema_version=1, workcell="fr5_ur5e", code_commit="b" * 40,
                       host=dict(cpu="private hardware"), runtime=dict(middleware="private configuration"), code_worktree_dirty=True,
                       model_manifest=dict(schema_version=1, workcell_sha256="c" * 64, joints=["joint_0"], home_rad=[0], joint_limits={},
                                           inputs={}, outputs={}, home_error_rad=[.1]),
                       policy=dict(sample_dt_sec=.01), evaluator=dict(engine="MuJoCo 3.12.0"), implementation_sha256={},
                       workcell_contract=dict(environment=dict(frame="world", objects=[])))
        exported = export_evidence(self.raw(session), "session")
        self.assertNotIn("host", exported)
        self.assertNotIn("runtime", exported)
        self.assertNotIn("home_error_rad", exported["model_manifest"])
        self.assertEqual(exported["workcell_contract"]["environment"], session["workcell_contract"]["environment"])

    def test_command_extra_identity_and_nonfinite_or_boolean_joint_values_are_rejected(self):
        trace = self.trace()
        trace["command"]["operator_email"] = "private@example.invalid"
        with self.assertRaisesRegex(ValueError, "strict four-field"):
            export_evidence(self.raw(trace), "trace")
        for value in (True, float("nan"), float("inf")):
            trace = self.trace()
            trace["command"]["target_rad"]["joint_0"] = value
            with self.subTest(value=value), self.assertRaises(ValueError):
                export_evidence(self.raw(trace), "trace")

    def test_free_text_email_private_path_secret_and_host_token_are_rejected(self):
        for message in ("private@example.invalid", "Failed reading /home/private/file.json", "log /tmp/session/output.json", "api_key=private", "local-host-identity"):
            trace = self.trace()
            trace["error"]["message"] = message
            with self.subTest(message=message), self.assertRaises(ValueError):
                export_evidence(self.raw(trace), "trace", forbidden_tokens=("local-host-identity",))
        with self.assertRaisesRegex(ValueError, "Private evidence field"):
            audit_public_evidence({"nested": {"origin_ns": 42}})

    def test_setup_projection_keeps_only_replay_inputs_and_verification_drops_nested_clock_origin(self):
        trace = self.trace()
        trace["trajectories"] = deepcopy(trace["planned_trajectories"])
        result = export_evidence(self.raw(trace), "setup")
        self.assertEqual(set(result), {"schema_version", "request_id", "status", "command", "initial_state_rad", "trajectories", "evidence_export"})
        verification = dict(schema_version=1, status="completed", requests=[dict(role="home_return", request_id="a" * 32,
            strategy="coordinated", status="failed", timing=dict(wall_clock=dict(wall="time.monotonic_ns", simulation="ROS time", origin_ns=123),
            request_metrics=dict(received_to_terminal_wall_sec=.25)))], maintenance=dict(requests=1, statuses=["succeeded"], note="private detail"))
        result = export_evidence(self.raw(verification), "verification")
        self.assertEqual(result["requests"][0]["timing"]["wall_clock"], dict(wall="time.monotonic_ns", simulation="ROS time"))
        self.assertNotIn("note", result["maintenance"])

    def test_result_and_physics_export_flags_counters_and_source_hashes(self):
        result = export_evidence(self.raw(dict(schema_version=2, status="passed", shutdown_clean=True,
            log_path="/home/private/launch.log", command_trace_directory="/tmp/private", pid=123,
            fixture_sha256="a" * 64, physics_report_sha256="b" * 64, reproduction_sha256="c" * 64)), "result")
        self.assertEqual(result["measured_fixture_sha256"], "a" * 64)
        self.assertNotIn("log_path", result)
        self.assertNotIn("pid", result)
        physics = export_evidence(self.raw(dict(schema_version=1, complete=True, steps=42, contact_steps=0, pid=123)), "physics")
        self.assertEqual(physics["steps"], 42)
        self.assertNotIn("pid", physics)


    def selection_trace(self):
        trace = self.trace()
        state = deepcopy(trace["initial_state_rad"])
        curve = dict(names=list(state), points=[
            dict(t=0.0, p=list(state.values()), v=[], a=[]),
            dict(t=1.0, p=list(state.values()), v=[], a=[]),
        ])
        minimum = dict(time_sec=0.5, state_rad=state, distance_m=0.037123456789,
                       bodies=["fr5_link3", "ur5e_link4"], nearest_points_m=[[0, 0, 0], [0, 0.037123456789, 0]],
                       tips_m=[[0.2, 0.3, 0.4], [0.1, 0.2, 0.3]])
        native = dict(complete=True, samples=101, sample_dt_max_sec=0.01, duration_sec=1.0,
                      joint_total_variation_rad=0.0, joint_travel_rad={name: 0.0 for name in state},
                      arm_joint_path_rad=dict(fr5=0.0, ur5e=0.0), tip_frame_path_m=dict(fr5=0.0, ur5e=0.0),
                      minimum=minimum, distance_semantics="Signed closest sampled collision geometry distance")
        progress = dict(samples=101, minimum={key: deepcopy(value) for key, value in minimum.items() if key != "time_sec"})
        rejected = dict(attempt=1, accepted=False, status="rejected", stage="moveit", elapsed_wall_sec=0.41,
                        moveit_error_code=1, curve=deepcopy(curve), native=deepcopy(native), native_progress=deepcopy(progress),
                        moveit=dict(complete=False, samples=51, invalid_samples=1,
                                    witness=dict(time_from_start_sec=0.5, state_rad=deepcopy(state),
                                                 contacts=[dict(body1="fr5_link3", body2="fixture")], inter_arm=False)),
                        rejection_kind="moveit_invalid", rejection="MoveIt rejected interpolated schedule")
        selected = dict(attempt=2, accepted=True, status="selected", stage="selection", elapsed_wall_sec=0.51,
                        moveit_error_code=1, curve=curve, native=native, native_progress=progress,
                        moveit=dict(complete=True, samples=101, invalid_samples=0, witness=None))
        rejected["proposal_kind"] = "coordinated_rrt"
        selected.update(proposal_kind="arm_order_waypoint", arm_order=["fr5", "ur5e"],
                        waypoint_state_rad=deepcopy(state), segment_trajectories=[deepcopy(curve), deepcopy(curve)])
        trace.update(plan_selection=dict(group="both_arms", start_rad=deepcopy(state), target_rad=deepcopy(state),
                     strategy="bounded_first_valid", goal_tolerance_rad=0.003, threshold_m=0.03,
                     proposal_goal_tolerances_rad=[0.003, 0.003], status="selected", stage="selection", max_candidates=12,
                     attempted=2, selected_attempt=2, elapsed_wall_sec=0.93, candidates=[rejected, selected]),
                     failure_kind="execution_failed", endpoint_moveit_validity=dict(start=True, goal=True),
                     planned_candidate_attempt=2, compute_budget=dict(limit_wall_sec=30.0, elapsed_wall_sec=0.95,
                     exceeded=False, enforcement="Cooperative checks; calls are not preempted"))
        trace["plans"].append(dict(group="both_arms", source="saved_preparation_anchored", wall_sec=None,
             server_planning_sec=None, requested_planner_id=None, goal_tolerance_rad=0.003,
             note="No MoveIt planning RPC; current full final checks still apply"))
        return trace

    def test_replanning_session_policy_and_full_candidate_diagnostics_are_retained(self):
        session = dict(schema_version=1, workcell="fr5_ur5e", compute_budget_sec=30.0, max_plan_candidates=12,
                       selection_policy="Bounded first valid spatial candidate; not shortest-path optimization",
                       host=dict(hostname="private-machine"))
        public_session = export_evidence(self.raw(session), "session")
        for key in ("compute_budget_sec", "max_plan_candidates", "selection_policy"):
            self.assertEqual(public_session[key], session[key])
        self.assertNotIn("host", public_session)
        trace = self.selection_trace()
        public = export_evidence(self.raw(trace), "trace")
        for key in ("plan_selection", "failure_kind", "endpoint_moveit_validity", "planned_candidate_attempt", "compute_budget"):
            self.assertEqual(public[key], trace[key])
        replay = public["plans"][-1]
        for key in ("source", "note", "goal_tolerance_rad"):
            self.assertEqual(replay[key], trace["plans"][-1][key])
        self.assertIsNone(replay["wall_sec"])
        self.assertTrue(audit_public_evidence(public)["passed"])

    def test_candidate_projection_removes_unknown_nested_local_metadata(self):
        trace = self.selection_trace()
        selection = trace["plan_selection"]
        candidate = selection["candidates"][0]
        private = dict(raw_host="private-machine", log_path="/home/private/worker.log")
        for item in (selection, candidate, candidate["curve"], candidate["curve"]["points"][0],
                     candidate["native"], candidate["native"]["minimum"], candidate["native_progress"],
                     candidate["native_progress"]["minimum"], candidate["moveit"], candidate["moveit"]["witness"],
                     candidate["moveit"]["witness"]["contacts"][0], trace["compute_budget"], trace["endpoint_moveit_validity"]):
            item.update(private)
        public = export_evidence(self.raw(trace), "trace")
        encoded = json.dumps(public)
        self.assertNotIn("private-machine", encoded)
        self.assertNotIn("worker.log", encoded)
        self.assertEqual(public["plan_selection"]["candidates"][0]["native"]["minimum"]["distance_m"], 0.037123456789)
        self.assertEqual(public["plan_selection"]["candidates"][0]["moveit"]["witness"]["contacts"],
                         [dict(body1="fr5_link3", body2="fixture")])

    def test_interrupted_candidate_preserves_partial_native_queries_and_original_error(self):
        trace = self.selection_trace()
        selection = trace["plan_selection"]
        candidate = selection["candidates"][-1]
        error = dict(type="PlanningBudgetExceeded", message="Planning wall budget exhausted; no dispatch")
        for target in (selection, candidate):
            target.update(status="interrupted", stage="native", error=deepcopy(error),
                          save_error=dict(type="OSError", message="Evidence persistence failed"))
        selection["selected_attempt"] = None
        candidate.update(accepted=False, evaluation_error=deepcopy(error))
        candidate.pop("native")
        candidate.pop("moveit")
        candidate["native_progress"]["samples"] = 20
        trace["compute_budget"]["exceeded"] = True
        trace["failure_kind"] = "budget_exhausted"
        public = export_evidence(self.raw(trace), "trace")
        self.assertEqual(public["plan_selection"], selection)
        self.assertNotIn("native", public["plan_selection"]["candidates"][-1])
        self.assertEqual(public["plan_selection"]["candidates"][-1]["native_progress"]["samples"], 20)
        self.assertTrue(public["compute_budget"]["exceeded"])

    def test_private_paths_in_retained_candidate_error_and_other_text_are_rejected(self):
        def add_error(target, key, message):
            target[key] = dict(type="RuntimeError", message=message)
        mutations = [
            lambda trace, message: add_error(trace["plan_selection"], "error", message),
            lambda trace, message: add_error(trace["plan_selection"], "save_error", message),
            lambda trace, message: add_error(trace["plan_selection"]["candidates"][0], "error", message),
            lambda trace, message: add_error(trace["plan_selection"]["candidates"][0], "evaluation_error", message),
            lambda trace, message: add_error(trace["plan_selection"]["candidates"][0], "save_error", message),
            lambda trace, message: trace["plan_selection"]["candidates"][0].update(rejection=message),
            lambda trace, message: trace["plan_selection"]["candidates"][0]["native"].update(distance_semantics=message),
            lambda trace, message: trace["plans"][-1].update(note=message),
            lambda trace, message: trace["plans"][-1].update(source=message),
        ]
        for index, mutate in enumerate(mutations):
            trace = self.selection_trace()
            mutate(trace, "Failed reading /home/private/native/model.xml")
            with self.subTest(field=index), self.assertRaisesRegex(ValueError, "Private path"):
                export_evidence(self.raw(trace), "trace")
        trace = self.selection_trace()
        trace["plan_selection"]["candidates"][0]["rejection"] = "Failure on private-machine"
        with self.assertRaisesRegex(ValueError, "Local identity token"):
            export_evidence(self.raw(trace), "trace", forbidden_tokens=("private-machine",))

    def test_non_numeric_identity_in_candidate_joint_maps_is_rejected(self):
        for key in ("start_rad", "target_rad"):
            trace = self.selection_trace()
            trace["plan_selection"][key]["raw_host"] = "private-machine"
            with self.subTest(key=key), self.assertRaisesRegex(ValueError, "named finite numeric"):
                export_evidence(self.raw(trace), "trace")
        trace = self.selection_trace()
        trace["plan_selection"]["candidates"][0]["native"]["minimum"]["state_rad"]["raw_host"] = "private-machine"
        with self.assertRaisesRegex(ValueError, "named finite numeric"):
            export_evidence(self.raw(trace), "trace")

    def test_verification_injections_are_explicitly_rejected_in_trace_and_setup(self):
        for mode in ("withhold_one_success", "cancel_after_failed_proposal"):
            for kind in ("trace", "setup"):
                trace = self.selection_trace()
                trace["verification_injection"] = dict(mode=mode, scope="Verification-only proposal injection",
                    source_request_id=trace["request_id"], source_curve_sha256="c" * 64,
                    initial_state_rad=trace["initial_state_rad"], target_rad=trace["command"]["target_rad"],
                    source_curve=trace["plan_selection"]["candidates"][-1]["curve"])
                with self.subTest(mode=mode, kind=kind), self.assertRaisesRegex(ValueError, "Verification-only injected trace"):
                    export_evidence(self.raw(trace), kind)
        trace = self.selection_trace()
        trace["plans"][-1]["source"] = "verification_same_request_saved_proposal"
        with self.assertRaisesRegex(ValueError, "Verification-only planning source"):
            export_evidence(self.raw(trace), "trace")
        with self.assertRaisesRegex(ValueError, "Verification-only planning source"):
            export_evidence(self.raw(trace), "setup")

    def test_legacy_moveit_contact_pair_witness_is_retained(self):
        trace = self.selection_trace()
        moveit = trace["plan_selection"]["candidates"][0]["moveit"]
        moveit["first_collision"] = dict(time_from_start_sec=0.5, state_rad=trace["initial_state_rad"],
                                         contacts=[["fr5_link3", "ur5e_link4"]], inter_arm=True)
        moveit["inter_arm_collision_samples"] = 1
        public = export_evidence(self.raw(trace), "trace")
        self.assertEqual(public["plan_selection"]["candidates"][0]["moveit"], moveit)


    def test_partial_waypoint_segments_are_projected_without_local_metadata(self):
        trace = self.selection_trace()
        candidate = trace["plan_selection"]["candidates"][-1]
        candidate.update(status="interrupted", stage="proposal", accepted=False)
        candidate.pop("curve")
        candidate["segment_trajectories"].pop()
        segment = candidate["segment_trajectories"][0]
        segment.update(raw_host="private-machine", log_path="/tmp/segment.log")
        segment["points"][0].update(raw_host="private-machine", source_path="/home/private/segment.json")
        public = export_evidence(self.raw(trace), "trace")
        projected = public["plan_selection"]["candidates"][-1]
        self.assertEqual(projected["proposal_kind"], "arm_order_waypoint")
        self.assertEqual(projected["arm_order"], ["fr5", "ur5e"])
        self.assertEqual(projected["waypoint_state_rad"], candidate["waypoint_state_rad"])
        self.assertEqual(len(projected["segment_trajectories"]), 1)
        self.assertEqual(set(projected["segment_trajectories"][0]), {"names", "points"})
        self.assertEqual(set(projected["segment_trajectories"][0]["points"][0]), {"t", "p", "v", "a"})
        self.assertNotIn("private-machine", json.dumps(public))
        self.assertNotIn("segment.log", json.dumps(public))
        self.assertNotIn("curve", projected)

    def test_waypoint_segments_reject_private_paths_and_malformed_state_or_segment_count(self):
        trace = self.selection_trace()
        candidate = trace["plan_selection"]["candidates"][-1]
        candidate["segment_trajectories"][0]["names"][0] = "/home/private/robot.xml"
        with self.assertRaisesRegex(ValueError, "Private path"):
            export_evidence(self.raw(trace), "trace")
        trace = self.selection_trace()
        trace["plan_selection"]["candidates"][-1]["waypoint_state_rad"]["raw_host"] = "private-machine"
        with self.assertRaisesRegex(ValueError, "named finite numeric"):
            export_evidence(self.raw(trace), "trace")
        trace = self.selection_trace()
        candidate = trace["plan_selection"]["candidates"][-1]
        candidate["segment_trajectories"].append(deepcopy(candidate["curve"]))
        with self.assertRaisesRegex(ValueError, "at most two"):
            export_evidence(self.raw(trace), "trace")


if __name__ == "__main__":
    unittest.main()
