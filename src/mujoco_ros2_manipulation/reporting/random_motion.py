"""Publish paired random-goal measurements without discarding unsuccessful goals."""
import argparse
from datetime import date
import gzip
import hashlib
import json
from pathlib import Path
import re
from statistics import median
from types import SimpleNamespace

from .data import json_text, read_json, validate
from ..workcells.random_workload import check_workload, workload_hash


def checked_requests(evidence):
    run, session, benchmark, workload = (evidence[k] for k in ("verification", "session", "benchmark", "workload"))
    if run["status"] != "passed" or not run["shutdown_clean"] or not run["physics_checks_passed"] or run["physics"]["contact_steps"] != 0:
        raise ValueError("Publication requires completed, clean, zero-contact physical verification")
    if benchmark["status"] != "completed" or benchmark["error"] is not None:
        raise ValueError("Aborted or incomplete benchmark cannot be published as completed")
    if benchmark["expected_budget_exhaustion"]:
        raise ValueError("Injected budget-exhaustion regression is not a random-performance sample")
    if hashlib.sha256((json.dumps(benchmark, indent=2, allow_nan=False) + "\n").encode()).hexdigest() != run["random_benchmark_sha256"]:
        raise ValueError("Benchmark differs from the verified result hash")
    metadata = session["model_manifest"]
    cell = SimpleNamespace(id=session["workcell"], fingerprint=metadata["workcell_sha256"],
                           joints=metadata["joints"], home=metadata["home_rad"])
    check_workload(workload, cell, metadata["joint_limits"])
    if workload_hash(workload) != benchmark["workload_sha256"] or run["workload_sha256"] != benchmark["workload_sha256"]:
        raise ValueError("Mismatched workload hashes")
    expected = [(c["case_id"], s) for c in workload["cases"] for s in c["strategy_order"]]
    roles = ["reset", "warmup", "reset", "warmup", "reset"] + ["reset"] * (len(expected) + 1)
    if ([r["role"] for r in benchmark["maintenance"]] != roles
            or run["scored_requests"] != len(expected) or not run["one_warm_simulation"]
            or benchmark["session_id"] != session["session_id"]):
        raise ValueError("Missing warmups/resets or mismatched warm-session provenance")
    if [(r["case_id"], r["strategy"]) for r in benchmark["rows"]] != expected:
        raise ValueError("Missing, duplicated, reordered or filtered scored cases")
    ids = [r["request_id"] for r in benchmark["rows"] + benchmark["maintenance"]]
    requests = {r["request_id"]: r for r in evidence["requests"]}
    if len(ids) != len(set(ids)) or len(requests) != len(evidence["requests"]) or set(ids) != set(requests):
        raise ValueError("Incomplete or duplicated request evidence")
    targets = {c["case_id"]: c["target_rad"] for c in workload["cases"]}
    grouped = {"coordinated": [], "departure_timing": []}
    for row in benchmark["rows"]:
        record = requests[row["request_id"]]
        if (record["status"] != row["status"] or record["status"] not in ("succeeded", "failed")
                or record["command"]["target_rad"] != targets[row["case_id"]]
                or record["command"]["strategy"] != row["strategy"]
                or record["compute_budget"]["limit_wall_sec"] != benchmark["compute_budget_wall_sec"]):
            raise ValueError("Mismatched goal, strategy, budget or terminal status")
        drift = max(abs(v - benchmark["reference_state_rad"][n]) for n, v in record["initial_state_rad"].items())
        if drift != row["initial_drift_rad"] or drift > benchmark["initial_tolerance_rad"]:
            raise ValueError("Paired initial-state tolerance exceeded")
        if record["status"] == "failed" and record["dispatched_goals"] != 0:
            raise ValueError("A physical execution failure must not be hidden in planner statistics")
        if record["status"] == "succeeded" and not record.get("no_motion_required"):
            if (record["compute_budget"]["exceeded"] or record["dispatched_goals"] != 2
                    or [r["status"] for r in record["action_results"]] != [4, 4]
                    or record["metrics"]["received_to_first_motion_wall_sec"] is None
                    or not record["planned_clearance"]["complete"]
                    or record["planned_clearance"]["minimum"]["distance_m"] < session["policy"]["min_inter_arm_distance_m"] + session["policy"]["tracking_reserve_m"]
                    or record["observed_clearance"]["minimum"]["distance_m"] < session["policy"]["min_inter_arm_distance_m"]):
                raise ValueError("Successful motion lacks completed budget, actions, movement or clearance evidence")
        if record.get("contact_guard", {}).get("latched"):
            raise ValueError("Physical interlock activation cannot be published as successful benchmarking")
        grouped[row["strategy"]].append(record)
    if any(requests[r["request_id"]]["status"] != "succeeded" for r in benchmark["maintenance"]):
        raise ValueError("Failed warmup/reset invalidates the paired evaluation")
    return grouped


def checked_fixed_search(evidence):
    fixed = evidence.get("fixed_search")
    if fixed is None:
        return None
    plan, bundle, run = (fixed[k] for k in ("plan", "bundle", "source_verification"))
    session = evidence["session"]
    encoded = lambda value: (json.dumps(value, indent=2, allow_nan=False)+"\n").encode()
    if (hashlib.sha256(encoded(bundle)).hexdigest() != plan["bundle_sha256"]
            or hashlib.sha256(encoded(run)).hexdigest() != bundle["source_report_sha256"]
            or plan["model_manifest_sha256"] != evidence["verification"]["model_manifest_sha256"]
            or fixed["evaluation_commit"] != session["code_commit"]
            or plan["evaluator"] != session["evaluator"] or plan["policy"] != session["policy"]
            or run["status"] != "passed" or not run["shutdown_clean"] or not run["physics_checks_passed"]
            or run["physics"]["contact_steps"] != 0):
        raise ValueError("Fixed-search input/model/engine/source provenance does not match")
    profile = plan["profiling"]
    candidates = plan["candidates"]
    if (profile["candidate_count"] != len(candidates) or not candidates
            or profile["native_distance_calls"] != sum(c["native_distance_calls"] for c in candidates)
            or any(c["status"] != "completed" or c["native_distance_calls"] != c["samples"] for c in candidates)
            or abs(profile["candidate_wall_sec"] - sum(c["elapsed_wall_sec"] for c in candidates)) > 1e-8
            or abs(profile["native_distance_wall_sec"] - sum(c["native_distance_wall_sec"] for c in candidates)) > 1e-8
            or abs(profile["elapsed_wall_sec"] - profile["candidate_wall_sec"] - profile["other_wall_sec"]) > 1e-8
            or profile["native_distance_wall_sec"] > profile["candidate_wall_sec"]
            or not plan["evaluation"]["complete"]
            or plan["evaluation"]["minimum"]["distance_m"] < plan["policy"]["min_inter_arm_distance_m"] + plan["policy"]["tracking_reserve_m"]):
        raise ValueError("Fixed-search profiling counts, time boundaries or clearance are inconsistent")
    return plan


def random_report(evidence, identifier, published, sha):
    grouped = checked_requests(evidence)
    run, session, benchmark, workload = (evidence[k] for k in ("verification", "session", "benchmark", "workload"))
    labels = dict(coordinated="양팔 공동 계획", departure_timing="독립 계획 + 출발 조정")
    groups = [dict(id=s.replace("_", "-"), label=labels[s], kind="headless", unique_cases=workload["count"],
        observations=len(records), successes=sum(r["status"] == "succeeded" for r in records),
        failures=sum(r["status"] != "succeeded" for r in records),
        seed_policy=f"Random({workload['seed']}), 같은 목표 파일, 사례별 순서 교대",
        description=f"명목 홈 ±{workload['radius_rad']}rad와 관절 한계의 교집합 균등 표본. 충돌 목표를 미리 걸러내지 않음.")
        for s, records in grouped.items()]
    def span(record, name):
        values = [v["duration_wall_sec"] for v in record["spans"] if v["name"] == name and v.get("duration_wall_sec") is not None]
        return sum(values) if values else None
    def plan_sum(record):
        values = [v["wall_sec"] for v in record.get("plans", [])]
        return sum(values) if values else None
    definitions = [
        ("first-motion", "수신 → 관측 최초 이동", "s", "wall", lambda r:r["metrics"]["received_to_first_motion_wall_sec"], True),
        ("compute", "계획·검증 예산 구간", "s", "wall", lambda r:r["compute_budget"]["elapsed_wall_sec"], False),
        ("planning", "MoveIt 서비스 왕복 합", "s", "wall", plan_sum, False),
        ("departure", "출발 탐색 전체", "s", "wall", lambda r:span(r,"departure_search"), False),
        ("native-distance", "탐색 중 네이티브 거리 계산", "s", "wall", lambda r:r.get("departure_search",{}).get("profiling",{}).get("native_distance_wall_sec"), False),
        ("distance-calls", "탐색 거리 계산 호출", "count", "not_applicable", lambda r:r.get("departure_search",{}).get("profiling",{}).get("native_distance_calls"), False),
        ("candidates", "탐색 후보 수", "count", "not_applicable", lambda r:r.get("departure_search",{}).get("profiling",{}).get("candidate_count"), False),
        ("candidate-time", "탐색 후보 평가 합", "s", "wall", lambda r:r.get("departure_search",{}).get("profiling",{}).get("candidate_wall_sec"), False),
        ("search-other", "탐색의 후보 평가 외 구간", "s", "wall", lambda r:r.get("departure_search",{}).get("profiling",{}).get("other_wall_sec"), False),
        ("terminal", "수신 → 최종 결과", "s", "wall", lambda r:r["metrics"]["received_to_terminal_wall_sec"], False),
        ("joint-travel", "계획 관절 누적 이동량", "rad", "not_applicable", lambda r:r.get("planned_clearance",{}).get("joint_total_variation_rad"), True),
        ("motion-duration", "계획 궤적 길이", "s", "simulation", lambda r:r.get("planned_clearance",{}).get("duration_sec"), True),
    ]
    metrics = []
    for key, label, unit, clock, getter, success_only in definitions:
        for statistic, reducer in (("median",median),("max",max)):
            values = []
            for strategy, records in grouped.items():
                samples = [getter(r) for r in records if (not success_only or r["status"] == "succeeded")]
                samples = [v for v in samples if v is not None]
                values.append(dict(group_id=strategy.replace("_","-"), value=reducer(samples) if samples else None,
                    sample_count=len(samples), statistic=statistic if samples else "not_measured"))
            metrics.append(dict(id=key+"-"+statistic,label=label+" ("+statistic+")",unit=unit,clock=clock,
                scope="성공 요청의 유효 표본" if success_only else "실패를 포함한 유효 표본",values=values,
                note="이동 지연은 1초 sim 예약 포함. 네이티브 시간은 탐색 전체의 일부로 중복 합산하지 않음. 미사용 구간은 null."))
    result = dict(schema_version=1, report_id=identifier, published_date=published, status="preliminary",
        title="고정 seed 랜덤 관절 목표·계산 예산 비교", overview=f"FR5·UR5e의 근거리 랜덤 목표 {workload['count']}개를 두 전략으로 실행한 재현 평가. 전체 작업 공간 성능평가가 아닙니다.",
        findings=[f"목표 {workload['count']}개 × 2전략. seed {workload['seed']}, 반경 {workload['radius_rad']}rad, 같은 계산 예산 {benchmark['compute_budget_wall_sec']}wall초.",
            *(f"{g['label']}: 성공 {g['successes']}/{g['observations']}, 실패 {g['failures']}. 실패 요청도 원본과 분모에 포함." for g in groups),
            f"기준 실측 자세 대비 시작 허용오차 {benchmark['initial_tolerance_rad']}rad. 명목 홈·실측 기준·각 요청 시작 상태를 보존.",
            f"복귀·워밍업 {len(benchmark['maintenance'])}개 요청을 별도 보존하고 점수에서 분리. 한 시뮬레이터 정상 종료, 접촉 {run['physics']['contact_steps']}스텝."],
        limitations=["홈 주변 관절 목표의 소규모 표본. Cartesian/TCP 작업, 교차 전용 과제, 전역 작업 공간의 성공률로 일반화할 수 없음.",
            "목표 생성 seed는 고정하지만 OMPL 내부 난수열과 OS 스케줄링을 고정하지 않음. 같은 입력도 경로와 시간은 달라질 수 있음.",
            "최초 이동은 /joint_states 수신의 0.001rad 초과 변화이며 1초 sim 예약을 포함. 시뮬레이터 시작 비용은 제외.",
            "예산은 상태 스냅샷 후 계획·탐색·최종 검사·상태 재확인까지의 협력식 wall 제한. 이미 진행 중인 RPC/native 호출을 선점하지 않아 초과 시간이 기록될 수 있음.",
            "성공 요청의 지연·경로 표본 수와 전체 요청 분모는 다를 수 있음. p95/p99 또는 통계적 신뢰 구간을 제시하지 않음.",
            "MuJoCo convex 근사·이산 거리 검사 결과. 실제 하드웨어·공구 TCP·연속 시간 안전 인증을 검증하지 않음."],
        provenance=dict(repository="https://github.com/WannaSleep3254/mujoco-ros2-manipulation",commit=session["code_commit"],
            workcell=session["workcell"],models=["FR5","UR5e"],scenario="paired_seeded_joint_goals",input_kind="random_joint_targets"),
        environment=dict(os=session["runtime"]["os"],cpu=session["host"]["cpu"],python=session["runtime"]["python"],
            ros="ROS 2 Humble / "+session["runtime"]["middleware"],native_mujoco=session["evaluator"]["engine"],mode="headless",
            verification_run=run["run_id"],workload_sha256=benchmark["workload_sha256"]),sample_groups=groups,
        stages=[dict(id="receive",label="명령 수신·상태 확인",clock="wall",coverage="measured",note="요청 ID·단일 monotonic 기록 시계"),
            dict(id="compute",label="계획·출발 탐색·안전검사",clock="wall",coverage="measured",note="같은 전체 계산 예산; 후보별 시간과 네이티브 호출 횟수"),
            dict(id="motion",label="실제 움직임·완료",clock="wall",coverage="measured",note="수신 관절 상태·양팔 Action과 정상 종료 검증")],
        metrics=metrics, assets=[dict(path="evidence.json.gz",label="전체 요청·복귀·원본 workload·세션·물리 검증 (gzip JSON)",role="data",sha256=sha)],
        references=[dict(label="평가 구현 커밋",url="https://github.com/WannaSleep3254/mujoco-ros2-manipulation/commit/"+session["code_commit"])])
    delays=[r.get("departure_search",{}).get("delay_sec") for r in grouped["departure_timing"]]
    delays=[v for v in delays if v is not None]
    if delays and all(v == 0 for v in delays):
        result["findings"].append(f"랜덤 출발 조정 {len(delays)}회에서 선택 지연은 모두 0sim초. 교차 전용 탐색은 별도 표본으로 구분.")
        result["limitations"].append("이번 랜덤 목표의 교차 대기·회피 난이도는 낮음. 다양한 교차·장애물 조건의 회피 성공률은 추가 평가가 필요함.")
    plan = checked_fixed_search(evidence)
    if plan:
        profile = plan["profiling"]
        groups.append(dict(id="fixed-search", label="교차 사례: 별도 고정 궤적 탐색 1회", kind="offline", unique_cases=1, observations=1,
            successes=1, failures=0, seed_policy="현재 모델에서 검증 후 고정한 입력 spline 1개; OMPL 난수열 고정 아님",
            description="랜덤 요청과 별개인 교차 참조의 출발 탐색. 수신→동작 지연이나 랜덤 비교 분모에 합산하지 않음."))
        for key,label,unit in (("elapsed_wall_sec","교차 탐색 전체","s"),("candidate_wall_sec","교차 후보 평가 합","s"),
                ("native_distance_wall_sec","교차 네이티브 거리 계산","s"),("other_wall_sec","교차 후보 평가 외 구간","s"),
                ("native_distance_calls","교차 거리 계산 호출","count"),("candidate_count","교차 탐색 후보 수","count")):
            metrics.append(dict(id="fixed-"+key.replace("_","-"),label=label,unit=unit,clock="wall" if unit=="s" else "not_applicable",
                scope="별도 고정 교차 궤적의 오프라인 탐색 1회",values=[dict(group_id="fixed-search",value=profile[key],sample_count=1,statistic="single")],
                note="전체 = 후보 평가 + 그 외. 네이티브 시간은 후보 평가의 일부. 모델 로딩·RPC·동작은 포함하지 않음."))
        result["findings"].append(f"별도 교차 입력 탐색: 후보 {profile['candidate_count']}개, 거리 {profile['native_distance_calls']}회, 전체 {profile['elapsed_wall_sec']:.3f}wall초. 선택 지연 {plan['delay_sec']}sim초는 계산 시간과 구분.")
        result["limitations"].append("별도 고정 교차 탐색은 현재 모델에서 새로 검증·고정한 입력 1회. 과거 1차의 다른 궤적·환경과 직접 속도 향상을 비교하지 않음.")
    return result


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run",type=Path)
    parser.add_argument("--traces",type=Path)
    parser.add_argument("--report-id")
    parser.add_argument("--date",default=date.today().isoformat())
    parser.add_argument("--check",type=Path)
    parser.add_argument("--fixed-search",type=Path,help="Optional separate fixed crossing input and profiling evidence")
    args=parser.parse_args()
    if args.check:
        folder=args.check
        report=read_json(folder/"report.json")
        raw=(folder/"evidence.json.gz").read_bytes()
        expected=random_report(json.loads(gzip.decompress(raw)),report["report_id"],report["published_date"],hashlib.sha256(raw).hexdigest())
        if expected != report: parser.error("Report differs from preserved random-goal evidence")
        validate(report,folder)
        print("Random-goal report matches all scored and maintenance requests")
        return
    if not args.run or not args.report_id or not re.fullmatch(r"[a-z0-9]+(?:-[a-z0-9]+)*",args.report_id) or len(args.report_id)>100:
        parser.error("Supply --run and a plain new --report-id")
    date.fromisoformat(args.date)
    run=read_json(args.run)
    source=args.traces or Path(run["command_trace_directory"])
    evidence=dict(verification=run,session=read_json(source/"session.json"),benchmark=read_json(source/"benchmark.json"),
        workload=read_json(source/"workload.json"),requests=[read_json(p) for p in sorted(source.glob("*.json")) if re.fullmatch(r"[0-9a-f]{32}\.json",p.name)])
    if args.fixed_search:
        evidence["fixed_search"]=read_json(args.fixed_search)
    raw=gzip.compress(json_text(evidence).encode(),mtime=0)
    report=random_report(evidence,args.report_id,args.date,hashlib.sha256(raw).hexdigest())
    from ..paths import workspace_root
    output=workspace_root()/"docs/reports"/args.report_id
    if output.exists(): parser.error("Report ID already exists; preserve earlier publications")
    validate(report,output,check_assets=False)
    output.mkdir()
    (output/"evidence.json.gz").write_bytes(raw)
    (output/"report.json").write_text(json_text(report))
    validate(report,output)
    print(output)


if __name__ == "__main__":
    main()
