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


if __name__ == "__main__":
    unittest.main()
