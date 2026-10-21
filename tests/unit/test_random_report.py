"""Paired evaluation must retain every goal, failure, reset and measured start."""
from copy import deepcopy
import hashlib
import json
import unittest
from types import SimpleNamespace as NS

from mujoco_ros2_manipulation.reporting.random_motion import checked_requests, random_report
from mujoco_ros2_manipulation.reporting.data import validate
from mujoco_ros2_manipulation.workcells.random_workload import make_workload, workload_hash


def fixture():
    cell=NS(id="pair",fingerprint="model",joints=["l","r"],home=[0.0,0.0])
    limits={n:dict(lower=-1.0,upper=1.0) for n in cell.joints}
    workload=make_workload(cell,limits,count=1)
    session=dict(session_id="session",workcell="pair",code_commit="a"*40,
        model_manifest=dict(workcell_sha256="model",joints=cell.joints,home_rad=cell.home,joint_limits=limits),
        policy=dict(min_inter_arm_distance_m=0.02,tracking_reserve_m=0.01),
        runtime=dict(os="Linux",python="3.10",middleware="cyclone"),host=dict(cpu="test"),evaluator=dict(engine="MuJoCo 3.12.0"))
    rows=[]; requests=[]
    for index,strategy in enumerate(workload["cases"][0]["strategy_order"]):
        identifier=f"{index+1:032x}"
        status="succeeded" if index==0 else "failed"
        rows.append(dict(case_id="case-0001",strategy=strategy,status=status,request_id=identifier,initial_drift_rad=0.0))
        requests.append(dict(request_id=identifier,status=status,command=dict(strategy=strategy,target_rad=workload["cases"][0]["target_rad"]),
            initial_state_rad=dict(l=0.0,r=0.0),compute_budget=dict(limit_wall_sec=5.0,elapsed_wall_sec=0.2,exceeded=False),
            dispatched_goals=2 if index==0 else 0,action_results=[dict(status=4),dict(status=4)] if index==0 else [],
            spans=[],plans=[],planned_clearance=dict(complete=True,minimum=dict(distance_m=0.1),duration_sec=2.0,joint_total_variation_rad=0.2),
            observed_clearance=dict(minimum=dict(distance_m=0.1)),metrics=dict(received_to_first_motion_wall_sec=1.2 if index==0 else None,received_to_terminal_wall_sec=3.0)))
    maintenance=[]
    for index,role in enumerate(["reset","warmup","reset","warmup","reset","reset","reset","reset"],10):
        identifier=f"{index:032x}"
        maintenance.append(dict(role=role,request_id=identifier,status="succeeded"))
        requests.append(dict(request_id=identifier,status="succeeded"))
    benchmark=dict(session_id="session",status="completed",error=None,expected_budget_exhaustion=False,
        workload_sha256=workload_hash(workload),compute_budget_wall_sec=5.0,initial_tolerance_rad=0.0015,reference_state_rad=dict(l=0.0,r=0.0),rows=rows,maintenance=maintenance)
    run=dict(status="passed",shutdown_clean=True,physics_checks_passed=True,physics=dict(contact_steps=0),
        run_id="fixture",workload_sha256=workload_hash(workload),scored_requests=2,one_warm_simulation=True)
    evidence=dict(verification=run,session=session,benchmark=benchmark,workload=workload,requests=requests)
    resign(evidence)
    return evidence


def resign(evidence):
    evidence["verification"]["random_benchmark_sha256"]=hashlib.sha256((json.dumps(evidence["benchmark"],indent=2,allow_nan=False)+"\n").encode()).hexdigest()


class RandomReportTests(unittest.TestCase):
    def test_failed_goal_stays_in_denominator_and_has_no_fabricated_movement(self):
        evidence=fixture()
        report=random_report(evidence,"random-test","2026-10-21","b"*64)
        validate(report,".",check_assets=False)
        self.assertEqual([g["observations"] for g in report["sample_groups"]],[1,1])
        self.assertEqual([g["failures"] for g in report["sample_groups"]],[0,1])
        first=next(m for m in report["metrics"] if m["id"]=="first-motion-median")
        self.assertIsNone(first["values"][1]["value"])
        self.assertEqual(first["values"][1]["sample_count"],0)

    def test_filtered_goals_missing_reset_and_changed_budget_cannot_be_published(self):
        for field in ("row","trace","reset","budget","target","start","physics","injected"):
            changed=fixture()
            if field=="row": changed["benchmark"]["rows"].pop()
            if field=="trace": changed["requests"].pop(1)
            if field=="reset": changed["benchmark"]["maintenance"].pop()
            if field=="budget": changed["requests"][1]["compute_budget"]["limit_wall_sec"]=10
            if field=="target": changed["requests"][1]["command"]["target_rad"]["l"]+=0.1
            if field=="start": changed["requests"][1]["initial_state_rad"]["l"]=0.02
            if field=="physics": changed["verification"]["physics"]["contact_steps"]=1
            if field=="injected": changed["benchmark"]["expected_budget_exhaustion"]=True
            resign(changed)
            with self.subTest(field=field),self.assertRaises(ValueError): checked_requests(changed)

    def test_execution_failure_cannot_be_counted_as_an_ordinary_planning_failure(self):
        changed=fixture();changed["requests"][1]["dispatched_goals"]=2
        with self.assertRaisesRegex(ValueError,"physical execution"):checked_requests(changed)


class PublishedRandomReportTests(unittest.TestCase):
    def evidence(self):
        import gzip
        from pathlib import Path
        folder=Path(__file__).resolve().parents[2]/"docs/reports/2026-10-21-random-motion"
        raw=(folder/"evidence.json.gz").read_bytes()
        return folder,raw,json.loads(gzip.decompress(raw))

    def test_published_report_matches_every_request_and_fixed_profile(self):
        folder,raw,evidence=self.evidence()
        report=json.loads((folder/"report.json").read_text())
        self.assertEqual(report,random_report(evidence,report["report_id"],report["published_date"],hashlib.sha256(raw).hexdigest()))
        self.assertEqual(len(evidence["benchmark"]["rows"]),10)
        self.assertEqual(len(evidence["requests"]),26)
        self.assertEqual(len(report["sample_groups"]),3)

    def test_fixed_profile_cannot_change_call_counts_or_source_commit(self):
        _,_,original=self.evidence()
        for field in ("count","commit"):
            evidence=deepcopy(original)
            if field=="count": evidence["fixed_search"]["plan"]["profiling"]["native_distance_calls"]+=1
            if field=="commit": evidence["fixed_search"]["evaluation_commit"]="0"*40
            with self.subTest(field=field),self.assertRaises(ValueError):
                random_report(evidence,"random-test","2026-10-21","b"*64)
