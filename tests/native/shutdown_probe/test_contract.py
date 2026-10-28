"""Pure gate/ordering checks; these do not import or start ROS."""
import importlib.util
import json
from pathlib import Path
import tempfile
import unittest
from copy import deepcopy
from unittest.mock import patch

path = Path(__file__).resolve().parents[2] / "integration/verify_native_shutdown_race.py"
spec = importlib.util.spec_from_file_location("shutdown_race_contract", path)
probe = importlib.util.module_from_spec(spec)
spec.loader.exec_module(probe)


class ProbeContracts(unittest.TestCase):
    def events(self, returned=True):
        names = ["entered", "released"] + (["returned"] if returned else [])
        return [dict(event=name, run_id="a" * 32, pid=123, tid=124, monotonic_ns=index + 1)
                for index, name in enumerate(names)]

    def test_fixed_requires_actual_return_then_join_before_teardown(self):
        final = "\n".join(("SHUTDOWN_PROBE event=entered", "SHUTDOWN_PROBE event=released",
            "SHUTDOWN_PROBE event=returned", probe.JOINED, probe.CONTEXT, probe.TEARDOWN))
        result = probe.ordering("SHUTDOWN_PROBE event=entered", final, self.events(), "fixed")
        self.assertTrue(result["safe_ordering_observed"])
        self.assertFalse(result["native_root_cause_closed"])
        for broken in (final.replace(probe.JOINED, "missing-join"),
                       final.replace("SHUTDOWN_PROBE event=returned", "missing-return"),
                       final.replace(probe.JOINED, probe.TEARDOWN).replace(probe.CONTEXT, probe.JOINED)):
            with self.subTest(final=broken), self.assertRaises(AssertionError):
                probe.ordering("SHUTDOWN_PROBE event=entered", broken, self.events(), "fixed")
        with self.assertRaises(AssertionError):
            probe.ordering(probe.TEARDOWN, final, self.events(), "fixed")

    def test_legacy_overlap_is_unsafe_evidence_and_does_not_require_a_crash(self):
        result = probe.ordering(probe.TEARDOWN, "SHUTDOWN_PROBE event=released", self.events(False), "legacy")
        self.assertTrue(result["expected_unsafe_overlap_observed"])
        self.assertFalse(result["safe_ordering_observed"])
        self.assertFalse(result["native_root_cause_closed"])
        with self.assertRaises(AssertionError):
            probe.ordering("still-held-without-teardown", "", self.events(False), "legacy")

    def test_timeout_foreign_thread_or_pre_released_snapshot_is_not_proof(self):
        changes = [self.events() + [dict(event="hold_timeout", run_id="a"*32, pid=123, tid=124, monotonic_ns=4)],
                   self.events(), self.events()]
        changes[1][-1]["tid"] = 999
        changes[2][-1]["monotonic_ns"] = 1
        for events in changes:
            with self.subTest(events=events), self.assertRaises(AssertionError):
                probe.ordering(probe.TEARDOWN, "", events, "legacy")
        with self.assertRaises(AssertionError):
            probe.ordering(probe.TEARDOWN + "\nSHUTDOWN_PROBE event=released", "", self.events(False), "legacy")

    def test_gate_is_private_exclusive_and_cannot_overwrite_an_existing_gate(self):
        with tempfile.TemporaryDirectory() as directory:
            target = Path(directory) / "arm"
            probe.gate(target, "a" * 32)
            self.assertEqual(target.read_text(), "a" * 32 + "\n")
            self.assertEqual(target.stat().st_mode & 0o777, 0o600)
            with self.assertRaises(FileExistsError):
                probe.gate(target, "b" * 32)
            self.assertEqual(target.read_text(), "a" * 32 + "\n")

    def bound_record(self):
        return dict(run_id="a"*32, native_pid=123,
            entered=dict(event="entered", pid=123, tid=124, run_id="a"*32, monotonic_ns=11),
            signal=dict(pid=123, signal="SIGINT", before_monotonic_ns=20, after_monotonic_ns=21),
            release_created_monotonic_ns=30,
            probe_events=[dict(event=name, pid=123, tid=124, run_id="a"*32, monotonic_ns=stamp)
                for name,stamp in (("entered",11),("released",31),("returned",32))])

    def test_verify_acceptance_binds_each_event_to_owned_entry_and_signal_watermarks(self):
        self.assertTrue(probe.validate_bound_events(self.bound_record())["bound_to_validated_entry"])
        # Both durable files must publish exactly the same entered event before
        # the owner can observe the gate and send SIGINT.
        row=self.bound_record();row["entered"]["monotonic_ns"]=10
        with self.assertRaises(AssertionError):
            probe.validate_bound_events(row)

    def test_verify_rejects_missing_identity_and_consistently_foreign_event_stream(self):
        for field in ("pid","tid","run_id"):
            missing=self.bound_record()
            for event in missing["probe_events"]: event.pop(field)
            foreign=self.bound_record()
            for event in foreign["probe_events"]: event[field]="b"*32 if field=="run_id" else 999
            for row in (missing,foreign):
                with self.subTest(field=field,row=row),self.assertRaises(AssertionError):
                    probe.validate_bound_events(row)

    def test_verify_rejects_duplicate_prefix_negative_or_boolean_fields_and_wrong_times(self):
        original=self.bound_record();invalid=[]
        for field,value in (("pid",True),("tid",0),("monotonic_ns",True),("monotonic_ns",20)):
            row=deepcopy(original);row["probe_events"][0][field]=value;invalid.append(row)
        row=deepcopy(original);row["probe_events"][1]["monotonic_ns"]=11;invalid.append(row)
        row=deepcopy(original);row["probe_events"][0]["event"]="released";invalid.append(row)
        row=deepcopy(original);row["signal"]["pid"]=999;invalid.append(row)
        row=deepcopy(original);row["release_created_monotonic_ns"]=40;invalid.append(row)
        row=deepcopy(original);row["entered"]["tid"]=999;invalid.append(row)
        for row in invalid:
            with self.subTest(row=row),self.assertRaises(AssertionError):
                probe.validate_bound_events(row)

    def test_gate_publishes_only_complete_private_file_atomically(self):
        with tempfile.TemporaryDirectory() as directory:
            target=Path(directory)/"release"
            actual=probe.os.link
            def inspect(source,destination,**kwargs):
                self.assertFalse(target.exists())
                self.assertEqual(Path(source).read_text(),"a"*32+"\n")
                self.assertEqual(Path(source).stat().st_mode & 0o777,0o600)
                return actual(source,destination,**kwargs)
            with patch.object(probe.os,"link",side_effect=inspect):
                probe.gate(target,"a"*32)
            self.assertEqual([item.name for item in Path(directory).iterdir()],["release"])

    def test_entered_record_requires_complete_json_and_owner_checks_run_first(self):
        with tempfile.TemporaryDirectory() as directory:
            target = Path(directory) / "entered.json"
            target.write_text('{"pid":')
            self.assertIsNone(probe.entered_record(target))
            target.write_text(json.dumps(dict(event="entered", run_id="foreign", pid=1)))
            with self.assertRaisesRegex(AssertionError, "another run"):
                probe.validate_owner(probe.entered_record(target), run_id="a"*32, launch_pid=1,
                    native=target, expected_sha="0"*64, shim=target, jtc=target)


if __name__ == "__main__":
    unittest.main()
