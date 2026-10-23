"""Turn audited fixed-suite public evidence into publication schema v1.

Original runtime files are accepted only by the separate public exporter. This
adapter consumes its audited projection and never reads current Git provenance.
"""
import argparse
from copy import deepcopy
from datetime import date
import gzip
import hashlib
import json
from pathlib import Path
import re
from statistics import median

from mujoco_ros2_manipulation.workcells.config import load_workcell
from mujoco_ros2_manipulation.workcells.evidence_export import audit_public_evidence
from mujoco_ros2_manipulation.workcells.fixed_results import aggregate_results
from mujoco_ros2_manipulation.workcells.fixed_suite import BASE_ID, POST_ID, make_suite, suite_hash
from .data import json_text, read_json, validate


REPOSITORY = "https://github.com/WannaSleep3254/mujoco-ros2-manipulation"
PUBLIC_FIELDS = {"schema_version", "artifact_kind", "report_id", "status", "suite", "suite_sha256",
    "planned_catalog", "rows", "summary", "measurement", "source_sha256", "scope", "limitations",
    "export_note", "privacy_audit"}


def _require(value, message):
    if not value:
        raise ValueError(message)


def _digest(value, size=64):
    _require(isinstance(value, str) and re.fullmatch(r"[0-9a-f]{" + str(size) + r"}", value),
             "Invalid public evidence digest")
    return value


def _public(evidence, expected_suite):
    """Validate public scope and recompute every aggregate before formatting."""
    _require(isinstance(evidence, dict) and set(evidence) == PUBLIC_FIELDS
             and evidence.get("schema_version") == 1
             and evidence.get("artifact_kind") == "fixed-motion-public-evidence-v1"
             and evidence.get("status") in ("completed", "aborted"), "Expected terminal public fixed evidence")
    identifier = evidence.get("report_id")
    _require(isinstance(identifier, str) and len(identifier) <= 100
             and re.fullmatch(r"[a-z0-9]+(?:-[a-z0-9]+)*", identifier), "Invalid fixed report ID")
    suite = evidence["suite"]
    _require(suite == expected_suite and evidence["suite_sha256"] == suite_hash(expected_suite),
             "Public suite differs from the trusted fixed factory")
    summary = aggregate_results(suite, evidence["planned_catalog"], evidence["rows"])
    _require(summary == evidence["summary"], "Public rows differ from the preserved aggregate")
    scope = evidence["scope"]
    valid = sum(case["expectation"] == "valid_goal" for case in suite["cases"])
    _require(scope == dict(fixed_cases=len(suite["cases"]), valid_goal_cases=valid,
        expected_invalid_goal_cases=len(suite["cases"])-valid,
        repeats=max(row["repeat"] for row in evidence["planned_catalog"]), candidate_caps=[1,12],
        compute_budget_wall_sec=5., initial_tolerance_rad=.0015,
        sample_role="full_evaluation" if max(row["repeat"] for row in evidence["planned_catalog"]) == 3
            and all(row["selected"] for row in evidence["planned_catalog"])
            and evidence["status"] == "completed" else "partial_or_diagnostic"), "Public scope changed")
    if evidence["status"] == "completed":
        _require(all(not row["selected"] or row["status"] == "completed" for row in evidence["rows"]),
                 "Completed public run contains an incomplete selected row")
    measurement = evidence["measurement"]
    _require(isinstance(measurement, dict) and measurement, "Report needs a measured session configuration")
    _digest(measurement.get("code_commit"), 40)
    _require(type(measurement.get("measurement_worktree_dirty")) is bool, "Missing measured worktree status")
    _require(evidence["privacy_audit"].get("passed") is True, "Public privacy audit did not pass")
    audit_public_evidence(evidence)
    return suite, summary, measurement


def _group_id(case, variant):
    return (case["case_id"] + "-" + variant["id"]).replace("_", "-")


def _median_value(group_id, values):
    values = [value for value in values if value is not None]
    return dict(group_id=group_id, value=None if not values else median(values), sample_count=len(values),
                statistic="not_measured" if not values else ("single" if len(values) == 1 else "median"))


def fixed_report(evidence, published, evidence_sha256, *, expected_suite, evidence_name="evidence.json.gz"):
    """Build a deterministic preliminary report from the preserved public bytes.

    Cases remain separate sample groups. Negative goals have explicit expected
    rejection counts in their descriptions and do not enter motion-success rates.
    """
    date.fromisoformat(published)
    _digest(evidence_sha256)
    _require(evidence_name in ("evidence.json", "evidence.json.gz"), "Unexpected public evidence asset name")
    suite, summary, measurement = _public(evidence, expected_suite)
    groups, grouped = [], []
    for case in suite["cases"]:
        for variant in suite["variants"]:
            rows = [row for row in evidence["rows"] if row["case_id"] == case["case_id"] and row["variant"] == variant["id"]]
            scored = [row for row in rows if row["scored"]]
            selected = sum(row["selected"] for row in rows)
            attempted = sum(row["attempted"] for row in rows)
            succeeded = sum(row["status"] == "completed" and row["outcome"] == "motion_succeeded" for row in scored)
            invalid = case["expectation"] == "invalid_goal"
            rejected = sum(row["status"] == "completed" and row["outcome"] == "expected_invalid_goal" for row in scored)
            unknown = sum(row["attempted"] and not row["scored"] for row in rows)
            description = (f"예정 {len(rows)}, 선택 {selected}, 발행 시도 {attempted}, 추적 확보 {len(scored)}, "
                f"미실행 {sum(row['status'] == 'not_run' for row in rows)}, 결과 불명 {unknown}. ")
            if invalid:
                moved = sum(row["metrics"] is not None and row["metrics"].get("first_motion_wall_sec") is not None for row in scored)
                dispatched = sum(row["metrics"]["dispatched_goals"] for row in scored if row["metrics"] is not None)
                description += f"부정 대조: invalid_goal 거부 {rejected}/{len(scored)}, 관측 이동 {moved}회, Action 전송 {dispatched}개. 정상 목표의 성공률 분모에서 제외합니다."
            else:
                description += f"물리·종료 검증까지 완료한 동작 성공 {succeeded}/{len(scored)}. 초기화·보정·warmup은 채점에서 제외합니다."
            group = dict(id=_group_id(case, variant), label=f"{case['label']} · 후보 상한 {variant['max_plan_candidates']}",
                kind="headless" if scored else "unmeasured", unique_cases=int(bool(scored)), observations=len(scored),
                successes=None if invalid else succeeded, failures=None if invalid else len(scored)-succeeded,
                seed_policy="초기·목표 관절은 고정, 반복별 조건 순서는 교대. MoveIt RRT 내부 seed와 샘플 경로는 조건 간 공유하지 않습니다.",
                description=description)
            groups.append(group)
            grouped.append((group, case, scored))

    def metric(key, label, unit, clock, getter, note):
        return dict(id=key, label=label, unit=unit, clock=clock,
            scope="각 고정 사례·후보 상한별 관측 반복. 표의 n은 해당 값이 존재하는 요청 수입니다.",
            values=[_median_value(group["id"], [getter(row) for row in rows]) for group, _, rows in grouped], note=note)

    def scalar(row, key):
        return None if row["metrics"] is None else row["metrics"].get(key)

    def complete(row, source, key, scale=1.):
        metrics = row["metrics"]
        path = None if metrics is None else metrics[source]
        value = None if path is None or path["status"] != "complete" else path.get(key)
        return None if value is None else value*scale

    metrics = [
        metric("compute", "계산 구간 wall 시간", "s", "wall", lambda row:scalar(row,"compute_wall_sec"),
            "계획 실패를 포함해 모든 확보된 요청을 집계합니다. 양 조건의 계산 예산은 5wall초이며 checkpoint 방식이므로 반환 직후의 초과를 숨기지 않습니다."),
        metric("first-motion", "수신 → 관측 최초 이동", "s", "wall", lambda row:scalar(row,"first_motion_wall_sec"),
            "이동이 관측된 요청만 포함합니다. /joint_states의 0.001rad 초과 변화 기준이며 예약 출발 대기를 포함합니다. 사용자 클릭 지연은 포함하지 않습니다."),
        metric("terminal", "수신 → 최종 결과", "s", "wall", lambda row:scalar(row,"terminal_wall_sec"),
            "정상·계획 실패·부정 대조의 최종 기록까지이며 초기 구동·보정·warmup 시간은 제외합니다."),
        metric("candidates", "검사한 공간 후보 수", "count", "not_applicable", lambda row:scalar(row,"candidate_count"),
            "상한 1 또는 12에 도달하기 전에 유효한 첫 후보를 고릅니다. 상한 12가 매번 12개를 계산한다는 뜻은 아닙니다."),
        metric("planned-duration", "완전 계획 경로의 시간 길이", "s", "simulation", lambda row:complete(row,"planned","duration_sec"),
            "전체 native 보간 검사가 완료된 계획 경로만 포함합니다. 계획 실패의 부분 경로와 실제 wall 실행 시간은 합치지 않습니다."),
        metric("planned-joint-travel", "완전 계획 경로의 관절 이동 합", "rad", "not_applicable", lambda row:complete(row,"planned","joint_travel_rad"),
            "양팔 모든 관절의 샘플 간 절대 변화 합입니다. 같은 최초 후보를 공유하지 않으므로 두 조건의 경로 차이를 상한의 인과 효과로 해석하지 않습니다."),
        metric("planned-clearance", "완전 계획 경로의 샘플 최소 간격", "mm", "not_applicable", lambda row:complete(row,"planned","minimum_distance_m",1000.),
            "완전 검사된 경로만 집계하며 계획 기준은 30mm입니다. prefix 최소값은 공개 근거의 별도 필드로 남기고 이 표에 넣지 않습니다. 연속 시간 안전성 증명은 아닙니다."),
        metric("observed-clearance", "완전 관측 경로의 샘플 최소 간격", "mm", "not_applicable", lambda row:complete(row,"observed","minimum_distance_m",1000.),
            "관측 검사가 완료된 실제 관절 상태 경로만 집계하며 실행 후 기준은 20mm입니다. 관측 누락 구간과 실물 안전성을 보장하지 않습니다."),
    ]
    findings = [f"고정 사례 {len(suite['cases'])}개 중 정상 목표 {evidence['scope']['valid_goal_cases']}개와 부정 대조 {evidence['scope']['expected_invalid_goal_cases']}개를 분리했습니다. 사례·조건당 예정 반복은 {evidence['scope']['repeats']}회입니다."]
    for variant in suite["variants"]:
        item = summary["by_variant"][variant["id"]]
        main, negative = item["valid_goal"], item["expected_invalid_goal"]
        successes = [row for row in evidence["rows"] if row["variant"] == variant["id"]
            and row["status"] == "completed" and row["outcome"] == "motion_succeeded"]
        later = sum(row["metrics"]["selected_attempt"] >= 2 for row in successes)
        findings.append(f"후보 상한 {variant['max_plan_candidates']}: 정상 목표 선택 {main['selected']}회, 동작 성공 {main['motion_succeeded']}회, 계획 실패 {main['planning_failed']}회, 추적 확보 {main['scored']}회. 성공 중 후속 후보 선택 성공 {later}/{len(successes)}회입니다. 부정 대조는 기대한 invalid_goal 거부 {negative['expected_invalid_goal']}/{negative['selected']}회로 별도 집계합니다.")
    successful = [row for row in evidence["rows"] if row["status"] == "completed" and row["outcome"] == "motion_succeeded"]
    if successful and all(row["metrics"]["selected_attempt"] == 1 for row in successful):
        findings.append(f"완료된 동작 성공 {len(successful)}회는 모두 첫 후보를 선택했습니다. 이 측정에서 후속 후보에 의한 회복 성공은 관측되지 않았습니다.")
    for case in suite["cases"]:
        rows = [row for row in evidence["rows"] if row["case_id"] == case["case_id"] and row["selected"]]
        if case["expectation"] == "valid_goal" and rows and all(row["scored"] for row in rows) and not any(
                row["status"] == "completed" and row["outcome"] == "motion_succeeded" for row in rows):
            failed = sum(row["outcome"] == "planning_failed" for row in rows)
            dispatched = sum(row["metrics"]["dispatched_goals"] for row in rows)
            findings.append(f"{case['label']}: 선택한 {len(rows)}회 모두 추적을 확보했으나 동작 성공은 없었습니다. 계획 실패 {failed}/{len(rows)}회, Action 전송 {dispatched}개입니다. 이 고정 입력·예산·후보 정책에서의 결과이며 전역적으로 해가 없음을 뜻하지 않습니다.")
    findings.extend([f"전체 예정 {summary['all_rows']['planned']}행을 보존했습니다. 발행 시도 {summary['all_rows']['attempted']}, 추적 확보 {summary['all_rows']['scored']}, 미실행 {summary['all_rows']['not_run']}, 검증 실패 {summary['all_rows']['failed']}행입니다.",
        "같은 고정 입력을 각 조건에서 별도 계획했으며, 계산·경로 표의 차이만으로 후보 상한의 성능 우열을 확정하지 않습니다."])
    limitations = [
        "소수의 고정 FR5·UR5e 사례를 같은 기구·준비 절차에서 반복한 예비 평가입니다. 사례 간 상관이 있으며 일반 로봇·장애물·모션플래너의 성능으로 확대하지 않습니다.",
        f"사례·조건당 예정 n={evidence['scope']['repeats']}이며 존재하는 값만 중앙값으로 표시합니다. p95/p99·신뢰구간·통계적 유의성을 주장하지 않습니다.",
        "조건 간 OMPL/RRT의 내부 난수·최초 제안 경로를 고정하거나 공유하지 않았습니다. 둘 다 첫 후보를 고른 경우에도 시간·경로 차이가 날 수 있어 상한 변경의 인과 효과와 분리해야 합니다.",
        "초기 상태는 명목값에서 최대 0.0015rad 이내로 맞췄습니다. 초기화·보정·warmup·시뮬레이터 시작은 요청 지표에서 제외하며 서로 다른 시작 상태·시계의 과거 리포트와 직접 합산하지 않습니다.",
        "계획 최소 간격은 완전 경로와 부분 경로를 구분합니다. 부분 경로의 prefix 최소값을 전체 경로 최소값으로 사용하지 않으며 native 샘플 검사는 연속 시간 또는 실제 장치 안전 인증이 아닙니다.",
        "계산 예산은 blocking 호출을 강제로 중단하는 실시간 deadline이 아닙니다. 실제 초과와 실패를 포함해 기록하며 미실행·발행 후 결과 불명 요청은 근거에서 숨기지 않습니다.",
        "공개 근거는 원본 SHA와 선별한 수치·구성만 보존합니다. 전체 곡선은 포함하지 않아 이 공개 자료만으로 기하 재평가를 수행할 수 없습니다. 원본 runtime 경로·프로세스 명령·개인 식별값·원문 오류 메시지는 공개하지 않습니다."]
    if evidence["scope"]["sample_role"] != "full_evaluation":
        limitations.insert(0, "선택 일부·반복 부족 또는 중단된 진단 실행입니다. 예정된 전체 60행 평가의 완료 결과로 취급하지 않습니다.")
    if measurement["measurement_worktree_dirty"]:
        limitations.append("측정 시 작업 트리에 변경이 있었습니다. 표시 커밋만으로 실행 소스를 재구성할 수 없으며 공개 근거의 측정 당시 implementation SHA를 함께 확인해야 합니다.")
    environment = measurement["environment"]
    result = dict(schema_version=1, report_id=evidence["report_id"], published_date=published,
        title="고정 목표·동일 계산 예산의 공간 재계획 예비 평가", status="preliminary",
        overview="FR5·UR5e의 고정 초기·목표 상태에서 공간 후보 상한 1과 12를 공통 5wall초 예산으로 비교했습니다. 정상 목표의 동작·계획 실패와 의도한 무효 목표 거부를 분리합니다.",
        findings=findings, limitations=limitations,
        provenance=dict(repository=REPOSITORY, commit=measurement["code_commit"],
            workcell=", ".join(sorted(suite["workcell_fingerprints"])), models=["Fairino FR5", "Universal Robots UR5e"],
            scenario=suite["id"], input_kind="fixed_joint_targets"),
        environment=dict(os=environment["os"], cpu=environment["cpu_model"], python=environment["python"],
            ros="ROS 2; middleware: "+environment["middleware"]+"; 배포판은 세션에 미기록",
            native_mujoco=environment["native_engine"], mode="headless; each selected row uses a fresh owned session",
            measurement_worktree="dirty" if measurement["measurement_worktree_dirty"] else "clean",
            recorder="request-local wall clock; simulation trajectory stamps are separate"),
        sample_groups=groups,
        stages=[dict(id=key,label=label,clock=clock,coverage=coverage,note=note) for key,label,clock,coverage,note in [
            ("preparation","초기화·상태 맞춤·warmup","wall","unmeasured","채점 전 절차이며 이 리포트의 지연·성공 분모에서 제외합니다."),
            ("computation","계획·후보 선택·안전검사","wall","measured","요청 수신 뒤 5wall초 공통 예산. 실제 반환 초과와 계획 실패를 포함합니다."),
            ("planned-path","계획 경로 품질","simulation","partial","전체 native 보간이 완료된 경로만 본표에 표시하고 부분 최소값은 별도 근거로 남깁니다."),
            ("motion","관측 최초 이동","wall","partial","이동이 관측된 요청의 /joint_states 기준이며 예약 출발 대기를 포함합니다."),
            ("observed-path","관측 경로 품질","simulation","partial","완전한 실행 후 native 관측 검사가 있는 요청만 집계합니다."),
            ("result","최종 결과·물리·종료 검증","wall","measured","요청 실패를 남기고 실행·물리·정상 종료까지 검증한 경우만 완료로 분류합니다.")]],
        metrics=metrics, assets=[dict(path=evidence_name,label="전체 예정 행·측정 구성·원본 SHA를 포함한 공개 추출 근거",role="data",sha256=evidence_sha256)],
        references=[dict(label="측정 당시 구현 커밋",url=REPOSITORY+"/commit/"+measurement["code_commit"])])
    # The complete data-derived input was audited in _public. Report schema v1
    # deliberately requires a cpu field and known HTTPS references, which are
    # forbidden in raw evidence audits but are trusted static report fields here.
    return result


def _read_evidence(path):
    raw = path.read_bytes()
    payload = gzip.decompress(raw) if path.suffix == ".gz" else raw
    def pairs(items):
        value = {}
        for key, item in items:
            _require(key not in value, "Duplicate public JSON key")
            value[key] = item
        return value
    return raw, json.loads(payload, object_pairs_hook=pairs)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--evidence", type=Path, help="Audited public evidence.json[.gz], never raw runtime records")
    parser.add_argument("--output", type=Path, help="New report directory, whose basename must equal report_id")
    parser.add_argument("--date", default=date.today().isoformat())
    parser.add_argument("--check", type=Path, help="Verify preserved report.json against its public evidence asset")
    args = parser.parse_args()
    expected = make_suite(load_workcell(BASE_ID), load_workcell(POST_ID))
    if args.check is not None:
        report = read_json(args.check / "report.json")
        asset = next(item for item in report["assets"] if item["path"] in ("evidence.json", "evidence.json.gz"))
        raw, evidence = _read_evidence(args.check / asset["path"])
        checked = fixed_report(evidence, report["published_date"], hashlib.sha256(raw).hexdigest(),
                               expected_suite=expected, evidence_name=asset["path"])
        _require(report == checked, "Published fixed report differs from preserved public evidence")
        validate(report, args.check)
        print("Fixed report matches preserved audited evidence")
        return 0
    if args.evidence is None or args.output is None:
        parser.error("Supply --evidence and a new --output directory, or --check")
    if args.output.exists():
        parser.error("Output already exists; report assets are never overwritten")
    raw, evidence = _read_evidence(args.evidence)
    name = "evidence.json.gz" if args.evidence.suffix == ".gz" else "evidence.json"
    report = fixed_report(evidence, args.date, hashlib.sha256(raw).hexdigest(), expected_suite=expected, evidence_name=name)
    _require(args.output.name == report["report_id"], "Report directory must match report_id")
    validate(report, args.output, check_assets=False)
    args.output.mkdir(parents=True)
    with (args.output / name).open("xb") as stream:
        stream.write(raw)
    with (args.output / "report.json").open("x", encoding="utf-8") as stream:
        stream.write(json_text(report))
    validate(report, args.output)
    print("Created one preliminary fixed report from audited public evidence")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
