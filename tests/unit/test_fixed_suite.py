"""Byte-pinned cases, model identity, complete states and unfiltered row catalogs."""
from copy import deepcopy
from dataclasses import replace
import gzip
import json
from pathlib import Path
import tempfile
import unittest
import yaml

from mujoco_ros2_manipulation.paths import ROOT
from mujoco_ros2_manipulation.workcells.config import Workcell, load_workcell
from mujoco_ros2_manipulation.workcells.fixed_suite import (
    CASE_IDS, CROSSING_SOURCE, HOME_SOURCE, INVALID_SOURCE, POLICY, POLICY_NAMES, POST_GEOMETRY, RANDOM_SOURCE,
    SOURCE_SHA256, check_suite, make_suite, planned_rows, suite_hash,
)


class FixedSuiteTests(unittest.TestCase):
    def setUp(self):
        self.base = load_workcell()
        post_data = deepcopy(self.base.data)
        post_data.update(id="fr5_ur5e_eval_post", label="Evaluation fixed post")
        post_data["objects"].append(dict(name="evaluation_post", **deepcopy(POST_GEOMETRY)))
        self.post = Workcell(self.base.root, post_data, self.base.instances)
        source = json.loads(gzip.decompress((ROOT / RANDOM_SOURCE).read_bytes()))
        limits = source["session"]["model_manifest"]["joint_limits"]
        self.metadata = {cell.id:dict(schema_version=1, workcell_sha256=cell.fingerprint,
            joints=cell.joints, home_rad=cell.home, joint_limits=deepcopy(limits)) for cell in (self.base,self.post)}
        self.suite = make_suite(self.base, self.post)

    def check(self, data=None, metadata=None, root=ROOT):
        return check_suite(self.suite if data is None else data, self.base, self.post,
            self.metadata if metadata is None else metadata, root=root)

    def source_tree(self, directory):
        root = Path(directory)
        for relative in SOURCE_SHA256:
            target = root / relative
            target.parent.mkdir(parents=True, exist_ok=True)
            target.write_bytes((ROOT / relative).read_bytes())
        return root

    def test_ten_cases_include_all_original_random_targets_and_public_invalid_goal(self):
        self.assertEqual([case["case_id"] for case in self.suite["cases"]], list(CASE_IDS))
        workload = json.loads(gzip.decompress((ROOT / RANDOM_SOURCE).read_bytes()))["workload"]
        for index, original in enumerate(workload["cases"]):
            case = self.suite["cases"][index+1]
            self.assertEqual(case["initial_rad"], workload["initial_rad"])
            self.assertEqual(case["target_rad"], original["target_rad"])
        invalid = json.loads((ROOT / INVALID_SOURCE).read_text())["requests"][6]["command"]["target_rad"]
        self.assertEqual(self.suite["cases"][-1]["target_rad"], invalid)
        self.assertEqual([case["case_id"] for case in self.suite["cases"] if case["expectation"] == "invalid_goal"], ["invalid_goal"])
        self.assertEqual(self.check(), self.suite)

    def test_defined_small_and_post_targets_match_explicit_offsets_and_geometry(self):
        home = dict(zip(self.base.joints, self.base.home))
        for position, offsets in ((0,(.12,.12)),(8,(2.0,-2.0))):
            case = self.suite["cases"][position]
            expected = dict(home)
            for instance,offset in zip(self.base.instances,offsets):
                expected[instance.joints[0]] += offset
            self.assertEqual(case["initial_rad"], home)
            self.assertEqual(case["target_rad"], expected)
            self.assertEqual(case["provenance"]["kind"], "defined")
        self.assertEqual(self.suite["cases"][8]["policy_name"], "eval_post_timing")
        self.assertEqual(self.suite["cases"][8]["workcell"], self.post.id)
        self.assertEqual(self.suite["cases"][8]["provenance"]["parameters"]["fixed_obstacle"], self.post.data["objects"][-1])

    def test_crossing_and_home_use_original_inputs_and_declared_preparation(self):
        crossing = json.loads(gzip.decompress((ROOT / CROSSING_SOURCE).read_bytes()))
        self.assertEqual(self.suite["cases"][6]["initial_rad"], crossing["initial_rad"])
        self.assertEqual(self.suite["cases"][6]["target_rad"], crossing["goal_rad"])
        home = json.loads((ROOT / HOME_SOURCE).read_text())
        case = self.suite["cases"][7]
        self.assertEqual(case["initial_rad"], home["initial_rad"])
        self.assertEqual(case["target_rad"], home["target_rad"])
        self.assertEqual(case["preparation"], dict(kind="home_fixture", fixture=HOME_SOURCE))
        for case in self.suite["cases"]:
            if case["case_id"] != "home_return":
                self.assertEqual(case["preparation"], dict(kind="joint_goal", targets=[case["initial_rad"]]))

    def test_changed_target_order_drop_duplicate_and_extra_fields_are_rejected(self):
        changes = []
        value = deepcopy(self.suite); value["cases"][0]["target_rad"][self.base.joints[0]] += .001; changes.append(value)
        value = deepcopy(self.suite); value["cases"][1:3] = reversed(value["cases"][1:3]); changes.append(value)
        value = deepcopy(self.suite); value["cases"].pop(); changes.append(value)
        value = deepcopy(self.suite); value["cases"][1] = deepcopy(value["cases"][0]); changes.append(value)
        value = deepcopy(self.suite); value["filtered_successes_only"] = True; changes.append(value)
        value = deepcopy(self.suite); value["cases"][1]["provenance"]["refs"][0]["sha256"] = "a"*64; changes.append(value)
        value = deepcopy(self.suite); value["cases"][0]["preparation"]["targets"] = []; changes.append(value)
        for value in changes:
            with self.subTest(data=value), self.assertRaises(ValueError): self.check(value)

    def test_all_four_sources_are_pinned_to_complete_file_bytes(self):
        for relative in SOURCE_SHA256:
            with self.subTest(source=relative), tempfile.TemporaryDirectory() as tmp:
                root = self.source_tree(tmp)
                self.assertEqual(self.check(root=root), self.suite)
                path = root / relative
                path.write_bytes(path.read_bytes() + b"\n")
                with self.assertRaisesRegex(ValueError, "SHA256 mismatch"):
                    self.check(root=root)
                with self.assertRaisesRegex(ValueError, "SHA256 mismatch"):
                    make_suite(self.base,self.post,root=root)

    def test_missing_source_cannot_silently_fall_back_to_current_runtime(self):
        with tempfile.TemporaryDirectory() as tmp:
            root = self.source_tree(tmp)
            (root / INVALID_SOURCE).unlink()
            with self.assertRaisesRegex(ValueError, "Missing fixed-suite source"):
                self.check(root=root)

    def test_named_positions_must_be_complete_finite_numeric_and_in_limits(self):
        for field in ("initial_rad", "target_rad"):
            for kind in ("missing", "unknown", "nan", "infinity", "boolean", "limit", "huge"):
                value = deepcopy(self.suite)
                state = value["cases"][0][field]
                name = self.base.joints[0]
                if kind == "missing": state.pop(name)
                elif kind == "unknown": state["other"] = 0.0
                else: state[name] = {"nan":float("nan"), "infinity":float("inf"), "boolean":True, "limit":100.0, "huge":10**1000}[kind]
                with self.subTest(field=field, kind=kind), self.assertRaises(ValueError): self.check(value)
        value = deepcopy(self.suite)
        value["cases"][0]["preparation"]["targets"][0][self.base.joints[0]] = float("nan")
        with self.assertRaisesRegex(ValueError, "finite"): self.check(value)

    def test_metadata_binds_both_current_cells_joint_order_home_and_limits(self):
        for cell in (self.base,self.post):
            for kind in ("missing_cell", "fingerprint", "joints", "home", "bool_home", "missing_limit", "reversed_limit", "nan_limit", "bool_schema"):
                value = deepcopy(self.metadata)
                meta = value[cell.id]
                if kind == "missing_cell": value.pop(cell.id)
                elif kind == "fingerprint": meta["workcell_sha256"] = "f"*64
                elif kind == "joints": meta["joints"].reverse()
                elif kind == "home": meta["home_rad"][0] += .001
                elif kind == "bool_home": meta["home_rad"][0] = False
                elif kind == "missing_limit": meta["joint_limits"].pop(cell.joints[0])
                elif kind == "reversed_limit": meta["joint_limits"][cell.joints[0]].update(lower=1, upper=-1)
                elif kind == "nan_limit": meta["joint_limits"][cell.joints[0]]["lower"] = float("nan")
                else: meta["schema_version"] = True
                with self.subTest(cell=cell.id, kind=kind), self.assertRaises(ValueError): self.check(metadata=value)
        metadata = deepcopy(self.metadata)
        metadata[self.base.id]["joint_limits"][self.base.joints[0]].update(lower=-.01, upper=.01)
        with self.assertRaisesRegex(ValueError, "exceeds metadata joint limits"):
            self.check(metadata=metadata)

    def test_altered_post_geometry_or_base_physics_is_not_the_same_defined_case(self):
        for kind in ("size", "placement", "physics", "missing", "joint_order"):
            data = deepcopy(self.post.data)
            if kind == "size": data["objects"][-1]["size"][0] += .01
            elif kind == "placement": data["robots"][0]["xyz"][0] += .01
            elif kind == "physics": data["physics"]["timestep"] *= 2
            elif kind == "missing": data["objects"].pop()
            else: data["id"] = "wrong_post"
            with self.subTest(kind=kind), self.assertRaises(ValueError):
                make_suite(self.base,Workcell(ROOT,data,self.base.instances))

    def test_variant_budget_tolerance_and_policy_changes_are_rejected(self):
        changes = []
        for key,value in (("schema_version",True),("compute_budget_wall_sec",6.0),("compute_budget_wall_sec",5),
                          ("initial_tolerance_rad",.002),("id","filtered-motion")):
            suite = deepcopy(self.suite); suite[key] = value; changes.append(suite)
        value = deepcopy(self.suite); value["variants"].reverse(); changes.append(value)
        value = deepcopy(self.suite); value["variants"][0]["max_plan_candidates"] = True; changes.append(value)
        value = deepcopy(self.suite); value["variants"][1]["max_plan_candidates"] = 6; changes.append(value)
        value = deepcopy(self.suite); value["cases"][0]["policy_name"] = "other"; changes.append(value)
        value = deepcopy(self.suite); value["workcell_fingerprints"][self.base.id] = "a"*64; changes.append(value)
        for key in POLICY:
            value = deepcopy(self.suite); value["policy"][key] *= 1.1; changes.append(value)
        value = deepcopy(self.suite); value["policy"].pop("sample_dt_sec"); changes.append(value)
        value = deepcopy(self.suite); value["policy"]["other"] = .1; changes.append(value)
        for bad in (4, True, "4.0", float("nan"), float("inf")):
            value = deepcopy(self.suite); value["policy"]["max_delay_sec"] = bad; changes.append(value)
        for value in changes:
            with self.subTest(data=value), self.assertRaises(ValueError): self.check(value)

    def test_runtime_policies_are_checked_per_cell_root_and_not_the_public_source_override(self):
        for cell_id in (self.base.id,self.post.id):
            for kind in ("distance", "case", "workcell", "schema", "numeric_type", "missing"):
                with self.subTest(cell=cell_id,kind=kind), tempfile.TemporaryDirectory() as tmp:
                    root = Path(tmp)
                    directory = root / "config/planning"
                    directory.mkdir(parents=True)
                    for name in POLICY_NAMES.values():
                        (directory/(name+".yaml")).write_bytes((ROOT/"config/planning"/(name+".yaml")).read_bytes())
                    base,post = replace(self.base,root=root),replace(self.post,root=root)
                    self.assertEqual(check_suite(self.suite,base,post,self.metadata),self.suite)
                    path = directory/(POLICY_NAMES[cell_id]+".yaml")
                    data = yaml.safe_load(path.read_text())
                    if kind == "distance": data["min_inter_arm_distance_m"] = .025
                    elif kind == "case": data["case"] = "different_case"
                    elif kind == "workcell": data["workcell"] = "different_workcell"
                    elif kind == "schema": data["schema_version"] = True
                    elif kind == "numeric_type": data["max_delay_sec"] = 4
                    if kind == "missing": path.unlink()
                    else: path.write_text(yaml.safe_dump(data))
                    with self.assertRaisesRegex(ValueError,"runtime policy"):
                        check_suite(self.suite,base,post,self.metadata)

    def test_public_provenance_contains_only_four_pinned_relative_sources(self):
        paths = set()
        for case in self.suite["cases"]:
            provenance = case["provenance"]
            for ref in provenance.get("refs", []):
                paths.add(ref["path"])
                self.assertFalse(Path(ref["path"]).is_absolute())
                self.assertEqual(ref["sha256"], SOURCE_SHA256[ref["path"]])
                self.assertTrue(ref["selector"])
        self.assertEqual(paths,set(SOURCE_SHA256))
        self.assertNotIn("runtime/",json.dumps(self.suite))
        self.assertNotIn("/home/",json.dumps(self.suite))
        changed = deepcopy(self.suite)
        changed["cases"][1]["provenance"]["refs"][0]["path"] = "/tmp/private-original.json"
        with self.assertRaises(ValueError): self.check(changed)

    def test_hash_is_stable_across_object_key_order_but_tracks_input_changes(self):
        first = suite_hash(self.suite)
        self.assertEqual(len(first),64)
        self.assertEqual(first,suite_hash(make_suite(self.base,self.post)))
        self.assertEqual(first,suite_hash(dict(reversed(list(self.suite.items())))))
        changed = deepcopy(self.suite); changed["cases"].reverse()
        self.assertNotEqual(first,suite_hash(changed))
        changed = deepcopy(self.suite); changed["cases"][0]["target_rad"][self.base.joints[0]] = float("nan")
        with self.assertRaises(ValueError): suite_hash(changed)

    def test_three_repeats_alternate_order_without_losing_catalog_rows(self):
        rows = planned_rows(self.suite,repeats=3)
        self.assertEqual(len(rows),60)
        self.assertEqual([row["index"] for row in rows],list(range(60)))
        for repeat_index in range(3):
            for case_index,case in enumerate(self.suite["cases"]):
                pair = rows[repeat_index*20+case_index*2:repeat_index*20+case_index*2+2]
                ids = ["single","bounded"] if (repeat_index+case_index)%2 == 0 else ["bounded","single"]
                self.assertEqual([row["variant"] for row in pair],ids)
                self.assertEqual([row["max_plan_candidates"] for row in pair],[1 if name == "single" else 12 for name in ids])
                self.assertTrue(all(row["case_id"] == case["case_id"] and row["repeat"] == repeat_index+1 for row in pair))
        self.assertTrue(all(row["status"] == "not_run" and row["request_id"] is None and row["selected"] for row in rows))

    def test_partial_selection_preserves_every_not_run_row_and_original_indices(self):
        rows = planned_rows(self.suite,repeats=2,case_ids=["home_return","invalid_goal"])
        self.assertEqual(len(rows),40)
        self.assertEqual(sum(row["selected"] for row in rows),8)
        self.assertTrue(all(row["selected"] == (row["case_id"] in {"home_return","invalid_goal"}) for row in rows))
        self.assertEqual([{k:v for k,v in row.items() if k != "selected"} for row in rows],
            [{k:v for k,v in row.items() if k != "selected"} for row in planned_rows(self.suite,repeats=2)])
        self.assertTrue(all(not row["selected"] and row["status"] == "not_run" for row in planned_rows(self.suite,case_ids=[])))

    def test_bad_repeat_counts_selections_and_catalog_ids_are_rejected(self):
        for repeats in (0,11,True,False,1.0,None,"1"):
            with self.subTest(repeats=repeats), self.assertRaises(ValueError): planned_rows(self.suite,repeats=repeats)
        for selected in (["unknown"],["local_small","local_small"],"local_small",[None],1,[["local_small"]]):
            with self.subTest(selected=selected), self.assertRaises(ValueError): planned_rows(self.suite,case_ids=selected)
        for identity in ("../case","local_small"):
            value = deepcopy(self.suite); value["cases"][1]["case_id"] = identity
            with self.subTest(identity=identity), self.assertRaises(ValueError): planned_rows(value)

    def test_factory_validator_and_rows_return_detached_data(self):
        before = deepcopy(self.suite)
        validated = self.check()
        validated["cases"][0]["target_rad"][self.base.joints[0]] = 999
        rows = planned_rows(self.suite)
        rows[0]["status"] = "failed"
        again = make_suite(self.base,self.post)
        again["cases"][0]["preparation"]["targets"][0][self.base.joints[0]] = 999
        self.assertEqual(self.suite,before)


if __name__ == "__main__":
    unittest.main()
