"""Import complete command regression evidence into publication report schema v1."""
import argparse
from collections import Counter
from datetime import date
import hashlib
import json
from pathlib import Path
import platform
import re
import sys

from .data import json_text, read_json, validate


def command_report(evidence, identifier, published, evidence_sha256):
    run, session, requests = evidence["verification"], evidence["session"], evidence["requests"]
    if run["status"] != "passed" or not run["shutdown_clean"] or not run["physics_checks_passed"]:
        raise ValueError("This importer requires a completed regression; failed motion requests remain included")
    expected_ids = {entry["request_id"] for entry in run["command_checks"]} | {run["separate_client_result"]["request_id"], run["standalone_shutdown_trace"]["request_id"]}
    if len(requests) != len(run["command_checks"]) + 3 or not expected_ids.issubset({r["request_id"] for r in requests}):
        raise ValueError("Incomplete request evidence; do not silently drop failed or rejected requests")
    if session["code_commit"] != evidence["shutdown_session"]["code_commit"] or session["implementation_sha256"] != evidence["shutdown_session"]["implementation_sha256"]:
        raise ValueError("Mixed implementation commits or source hashes")
    labels = ["공동 계획", "출발 조정", "관절 제한 위반 거부", "충돌 목표 계획 실패", "실행 전 취소", "이동 중 취소", "취소 후 복구", "이미 도달한 목표"]
    names = {entry["request_id"]: label for entry, label in zip(run["command_checks"], labels)}
    names[run["separate_client_result"]["request_id"]] = "별도 CLI 명령"
    names[run["standalone_shutdown_trace"]["request_id"]] = "명령 노드 SIGINT"
    requests = sorted(requests, key=lambda item: item["received_utc"])
    if len({r["request_id"] for r in requests}) != len(requests):
        raise ValueError("Duplicate request evidence")
    groups = []
    for index, record in enumerate(requests, 1):
        label = names.get(record["request_id"], "동시 요청 거부" if "Busy" in str(record["error"]) else "추가 요청")
        groups.append(dict(id=f"request-{index}", label=label, kind="headless", unique_cases=1, observations=1,
            successes=int(record["status"] == "succeeded"), failures=int(record["status"] != "succeeded"),
            seed_policy="고정 회귀 절차; 충돌 목표 대조 탐색만 Random(42). 랜덤 동작 표본 아님.",
            description=f"ID {record['request_id']}; 반환 {record['status']}. 기대된 실패·취소·거부도 동작 성공으로 집계하지 않음."))

    def spans(record, name):
        matches = [s for s in record["spans"] if s["name"] == name and s.get("duration_wall_sec") is not None]
        return sum(s["duration_wall_sec"] for s in matches) if matches else None

    def plans(record, key):
        values = [p[key] for p in record.get("plans", [])]
        return sum(values) if values else None

    def acceptance(record):
        start = next((e["wall_sec"] for e in record["events"] if e["name"] == "dispatch"), None)
        ends = [e["wall_sec"] for e in record["events"] if e["name"] == "goal_acceptance"]
        return max(ends) - start if ends and start is not None else None

    definitions = [
        ("first-motion", "수신 → 관측 최초 이동", lambda r:r["metrics"]["received_to_first_motion_wall_sec"], "1초 sim 예약 대기를 포함. 실제 관절 상태의 첫 0.001rad 초과 변화 수신."),
        ("plan-round-trip", "계획 서비스 왕복 합", lambda r:plans(r,"wall_sec"), "한 요청 안의 순차 호출 합. pure DDS 지연이 아님."),
        ("server-planning", "서버 planning_time 합", lambda r:plans(r,"server_planning_sec"), "MoveIt 응답 그대로. 응답 실패의 0 값도 보존하며 전체 응답 지연에 더하지 않음."),
        ("departure-search", "출발 시점 탐색", lambda r:spans(r,"departure_search"), "사용하지 않았으면 null. 선택된 팔 대기(sim 시간)와 다름."),
        ("safety", "최종 안전검사", lambda r:spans(r,"safety_checks"), "MuJoCo 거리와 MoveIt 보간 유효성 검사. 상태 재확인 대기와 구분."),
        ("action-acceptance", "전송 시작 → 마지막 팔 수락", acceptance, "두 Action의 수락 수신. 실제 이동 시작과 구분."),
        ("actions-done", "수신 → 양팔 Action 완료", lambda r:r["metrics"]["received_to_actions_done_wall_sec"], "정상 Action 결과 수신. 취소·거부는 이 성공 완료 지표가 null."),
        ("settled", "수신 → 관절 수렴", lambda r:r["metrics"]["received_to_settled_wall_sec"], "정상 목표 수렴. 이미 도달한 목표도 별도 no_motion_required 표시."),
        ("terminal", "수신 → 최종 결과 표시", lambda r:r["metrics"]["received_to_terminal_wall_sec"], "같은 기록 프로세스의 terminal 표시까지. 클라이언트 왕복 지연과 구분."),
    ]
    metrics = []
    for key, label, getter, note in definitions:
        values = []
        for group, record in zip(groups, requests):
            value = getter(record)
            values.append(dict(group_id=group["id"], value=value, sample_count=0 if value is None else 1,
                               statistic="not_measured" if value is None else "single"))
        metrics.append(dict(id=key, label=label, unit="s", clock="wall", scope="각 요청 1회의 계측 구간", values=values, note=note))
    counts = Counter(r["status"] for r in requests)
    first = requests[0]["metrics"]["received_to_first_motion_wall_sec"]
    second = requests[1]["metrics"]["received_to_first_motion_wall_sec"]
    return dict(schema_version=1, report_id=identifier, published_date=published,
        title="상시 모션 명령·지령 → 동작 구간 계측 검증", status="preliminary",
        overview="FR5·UR5e 한 시뮬레이션의 짧은 회귀 절차. 상시 요청, 실패·취소, 별도 CLI와 명령 노드 SIGINT를 검증하고 wall 구간을 연결했습니다.",
        findings=[f"원본 요청 {len(requests)}개: 성공 {counts['succeeded']}, 실패 {counts['failed']}, 취소 {counts['cancelled']}, 거부 {counts['rejected']}. 의도된 부정 대조를 포함하므로 성공률 평가가 아님.",
            f"첫 공동 요청의 수신→최초 이동 {first:.3f}wall초, 출발 조정 요청 {second:.3f}wall초. 각각 1회이며 서로 다른 연속 목표라 성능 우열 비교가 아님.",
            "최초 이동 지연에는 1초 sim 예약 출발이 포함됨. 계획 서비스·안전검사·수락 지연은 별도 표로 제공.",
            "중복 ID 재실행 방지, 동시 요청 거부, 실제 충돌 목표 계획 실패, 양팔 취소·복구·SIGINT 정상 종료 검증 통과.",
            f"네이티브 접촉 {run['physics']['contact_steps']}스텝, 정상 시뮬레이터 종료 확인. 이미 도달한 목표는 새 Action 0개·최초 이동 null."],
        limitations=["명령 노드 수신부터의 지연이며 사용자 클릭·키보드 입력·클라이언트→서버 전송 이전 시간은 포함하지 않음.",
            "하나의 짧은 회귀 절차이며 랜덤 목표 성공률·p95/p99·cold/warm 분포를 평가하지 않음. 절차의 일부 목표는 당시 측정 상태에서 계산함.",
            "시뮬레이터·MoveIt·거리 모델 최초 구동은 수신 전에 끝남. 각 요청의 원본 목표와 시각·코드 해시를 함께 보존.",
            "1차 교차 회피 리포트와 움직임·시작 상태·입력 경계가 달라 2.133초 오프라인 계산과 직접 성능 비교하지 않음.",
            "관측 최초 이동은 /joint_states 수신 기준이며 첫 물리 적분·실물 엔코더 응답과 다름. SIGKILL 종료 감시는 포함하지 않음."],
        provenance=dict(repository="https://github.com/WannaSleep3254/mujoco-ros2-manipulation", commit=session["code_commit"],
            workcell="fr5_ur5e", models=["FR5","UR5e"], scenario="persistent_command_regression", input_kind="fixed_joint_targets"),
        environment=dict(os=session["runtime"]["os"], cpu=session["host"]["cpu"], python=session["runtime"]["python"],
            ros="ROS 2 Humble / rmw_cyclonedds_cpp", native_mujoco=session["evaluator"]["engine"], mode="headless",
            verification_run=run["run_id"], recorder="time.monotonic_ns; per-process origin; simulation stamps separate"),
        sample_groups=groups,
        stages=[dict(id=key,label=label,clock="wall",coverage="measured",note=note) for key,label,note in [
            ("receive","명령 수신·상태 스냅샷","단일 요청 ID와 관측 상태 나이; simulator startup 제외"),
            ("planning","MoveIt 계획·출발 조정","명시적 planner ID·왕복·서버 시간·선택 후보를 분리"),
            ("safety","최종 안전검사","계획 30mm 기준과 MoveIt 전체 보간 상태 검사"),
            ("accept","Action 전송·수락","팔별 이벤트. 1초 sim 예약은 CPU 계산과 구분"),
            ("motion","관측 최초 이동","관절 상태 수신 시 최초 0.001rad 초과 변화"),
            ("result","Action·수렴·종료","실패·취소·거부도 보존하고 이동 없는 경우 null")]],
        metrics=metrics, assets=[dict(path="evidence.json",label="요청·세션·회귀 실행 원본 스냅샷",role="data",sha256=evidence_sha256)],
        references=[dict(label="고정 구현 커밋",url="https://github.com/WannaSleep3254/mujoco-ros2-manipulation/commit/"+session["code_commit"])])


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--run",type=Path)
    parser.add_argument("--traces",type=Path)
    parser.add_argument("--report-id")
    parser.add_argument("--date",default=date.today().isoformat())
    parser.add_argument("--check",type=Path,help="Check an existing published report against its preserved evidence")
    args=parser.parse_args()
    if args.check:
        path=args.check
        original=read_json(path/"report.json")
        raw=(path/"evidence.json").read_bytes()
        expected=command_report(json.loads(raw),original["report_id"],original["published_date"],hashlib.sha256(raw).hexdigest())
        if original != expected:
            parser.error("Published command report differs from preserved measurements")
        validate(original,path)
        print("Command report matches preserved request evidence")
        return
    if not args.run or not args.report_id or not re.fullmatch(r"[a-z0-9]+(?:-[a-z0-9]+)*",args.report_id) or len(args.report_id)>100:
        parser.error("Supply --run and a plain --report-id")
    date.fromisoformat(args.date)
    from ..paths import workspace_root
    run=read_json(args.run)
    source=args.traces or Path(run["command_trace_directory"])
    session=read_json(source/"session.json")
    records=[read_json(p) for p in source.glob("*.json") if re.fullmatch(r"[0-9a-f]{32}\.json",p.name)]
    records.append(run["standalone_shutdown_trace"])
    shutdown_session=read_json(source/"standalone-shutdown/session.json")
    evidence=dict(verification=run,session=session,shutdown_session=shutdown_session,requests=records)
    raw=json_text(evidence).encode()
    report=command_report(evidence,args.report_id,args.date,hashlib.sha256(raw).hexdigest())
    output=workspace_root()/"docs/reports"/args.report_id
    if output.exists():
        parser.error("Output report ID already exists")
    validate(report,output,check_assets=False)
    output.mkdir()
    (output/"evidence.json").write_bytes(raw)
    (output/"report.json").write_text(json_text(report))
    validate(report,output)
    print(output)


if __name__ == "__main__":
    main()
