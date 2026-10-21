"""One-way adapter for the preserved 2026-10-21 evidence; no new benchmark."""
from pathlib import Path
import statistics

from .data import ReportError, read_json, sha256, validate


def phase1_report(directory):
    directory = Path(directory)
    evidence = read_json(directory / "evidence.json")
    trials = evidence["offline_benchmark"]["trials"]
    source = evidence["sources"]["source-verification.json.gz"]["data"]
    if len(trials) != 5 or evidence["offline_benchmark"]["unique_inputs"] != 1 or len(source["planning_wall_timings"]) != 4:
        raise ReportError("This adapter only imports the preserved phase-one fixed-input evidence")
    original = read_json(directory / "manifest.json")
    for name, metadata in original["files"].items():
        if sha256(directory / name) != metadata["sha256"]:
            raise ReportError(f"Archived phase-one asset changed: {name}")
    groups = [dict(id="offline", label="고정 입력 오프라인 5회", kind="offline",
        unique_cases=1, observations=len(trials), successes=None, failures=None,
        seed_policy="새 랜덤 목표·planner seed 없음. 같은 입력과 프로세스를 반복.",
        description="매 회 모델·평가기 재생성. OS 캐시 초기화·CPU 격리 없음. 로봇 동작 성공률 표본 아님.")]
    for mode in ("gui", "headless"):
        for method in ("baseline", "timing"):
            run = evidence["report_comparison"][mode][method]
            success = int(run["statuses"] == [4, 4] and run["shutdown"]
                          and run["contacts"] == 0 and run["error"] < 0.01)
            groups.append(dict(id=f"{mode}-{method}", label=f'{mode} / {"기존 공동 회피" if method == "baseline" else "출발 조정"}',
                kind=mode, unique_cases=1, observations=1, successes=success, failures=1-success,
                seed_policy="같은 동결 궤적 묶음 재사용. 새 랜덤 목표 아님.",
                description=f'실행 {run["run_id"]}. 성공은 양팔 Action·수렴·접촉 0·정상 종료 판정.'))
    for name in ("setup", "left", "right", "both"):
        groups.append(dict(id=f"plan-{name}", label=f"MoveIt 요청 / {name}", kind="headless",
            unique_cases=1, observations=1, successes=None, failures=None,
            seed_policy="고정 사례의 서로 다른 요청 4개. 4개 랜덤 사례 아님.",
            description=f'원본 생성 실행 {source["run_id"]}의 서비스 왕복 1회.'))
    groups.append(dict(id="e2e", label="지령 → 최초 이동", kind="unmeasured", unique_cases=0,
        observations=0, successes=None, failures=None, seed_policy="미실행",
        description="사용자 지령과 실제 움직임을 동일 wall 요청 ID로 연결하지 못함."))

    def value(group, number, n=1, statistic="single"):
        return dict(group_id=group, value=number, sample_count=n, statistic=statistic)

    def metric(identifier, label, unit, clock, values, scope, note):
        return dict(id=identifier, label=label, unit=unit, clock=clock, values=values, scope=scope, note=note)

    metrics = [metric("command-to-motion", "지령 접수 → 최초 이동", "s", "wall",
        [value("e2e", None, 0, "not_measured")], "전체 요청 반응 지연", "0초가 아닌 미측정. 아래 오프라인·sim 시간을 합산하지 않음.")]
    for key, label in (("total_with_cleanup_sec", "오프라인 계산 총 소요"),
                       ("departure_search_sec", "출발 시점 탐색"), ("native_load_sec", "모델·평가기 생성")):
        metrics.append(metric(key.replace("_", "-"), label, "s", "wall",
            [value("offline", statistics.median(t[key] for t in trials), len(trials), "median")],
            "고정 입력 1개 × 같은 프로세스 5회", "새 MoveIt 계획·ROS 검사·전송·실행·전체 launch 시간 제외."))
    metrics.append(metric("planning-service", "MoveIt 계획 서비스 응답", "s", "wall",
        [value("plan-" + name, timing["wall_sec"]) for name, timing in zip(
            ("setup", "left", "right", "both"), source["planning_wall_timings"])],
        "고정 사례 원본 생성 실행의 요청별 기록", "서버 계산·ROS 왕복·클라이언트 대기 포함. 내부 플래너 시간이나 순수 DDS 비용이 아님."))
    specifications = [
        ("motion", "명령된 궤적 시간", "s", "simulation", "시간 파라미터화된 명령 길이", "전체 wall 완료 지연이 아님."),
        ("travel", "계획 관절 누적 이동량", "rad", "not_applicable", "모든 관절의 |Δq| 합", "미터 단위 이동거리·소비 에너지가 아님."),
        ("planned_gap", "계획 최소 팔 간 간격", "mm", "not_applicable", "native 거리와 JTC spline의 이산 검사", "계획 기준 30mm, 최대 10ms 간격. 연속 시간 전체의 보증 아님."),
        ("observed_gap", "관측 최소 팔 간 간격", "mm", "not_applicable", "한 실행의 수신 관절 자세에서 얻은 최소값", "수신 자세 2763개, 최대 간격 약 12ms. 값 하나는 실행 1회의 결과."),
        ("error", "최종 목표 최대 오차", "rad", "not_applicable", "양팔 관절 목표 수렴", "성공 기준 0.01rad 미만의 연속 상태 메시지와 Action 결과를 함께 검사."),
        ("wall", "검증 전체 소요", "s", "wall", "각 새 launch의 준비·명령·관측·정리", "동일 비교 관측 구간 27.628sim초 포함. 계획 지연으로 해석하지 않음."),
        ("contacts", "접촉 수", "count", "not_applicable", "해당 실행의 전체 물리 스텝 감시", "동작 후 수렴·정상 종료와 함께 판정. 실물 제동 시험 아님."),
        ("rtf", "Paced real-time factor", "ratio", "not_applicable", "해당 native 계측 구간", "ROS·GUI 전체 처리 성능이나 최대 처리량이 아님."),
    ]
    for key, label, unit, clock, scope, note in specifications:
        metrics.append(metric(key.replace("_", "-"), label, unit, clock,
            [value(f"{mode}-{method}", evidence["report_comparison"][mode][method][key])
             for mode in ("gui", "headless") for method in ("baseline", "timing")], scope, note))
    for arm, label in (("left", "FR5"), ("right", "UR5e")):
        metrics.append(metric(arm + "-tip", label + " 끝단 프레임 경로", "m", "not_applicable",
            [value(f"{mode}-{method}", evidence["report_comparison"][mode][method]["tip"][arm])
             for mode in ("gui", "headless") for method in ("baseline", "timing")],
            "계획 경로의 프레임 원점 이동량", "left_wrist3_link / right_tool0 기준. 실제 공구 TCP가 아님."))
    result = dict(schema_version=1, report_id="2026-10-21-phase1", title="1차 성능평가: FR5·UR5e 경로와 출발 조정",
        published_date="2026-10-21", status="preliminary",
        overview="동일한 양팔 교차 사례 1개에서 큰 공동 우회를 출발 지연으로 줄인 예비 평가. GUI/headless 각 1회 비교와 고정 입력 오프라인 5회 측정.",
        findings=["GUI 명령 궤적 26.628→8.622sim초, 관절 누적 이동량 24.692→2.024rad.",
            "오프라인 총 중앙값 2.133초, 출발 탐색 1.891초. 사용자 지령→동작의 전체 wall 지연은 미측정.",
            "58개는 두 팔의 출발 지연 후보. FR5 대기 2.3sim초는 계산 지연이 아님.",
            "이전 소스 사본과 달리 최신 공동 장면에는 RRTConnect 설정·서비스 wall 기록·공통 바닥 충돌 모델이 있음."],
        limitations=["고유 시작·목표 1개이며 랜덤 목표의 성공률·P95/P99를 평가하지 않음.",
            "물리 시뮬레이션 결과. 실물 로봇·실제 TCP·그리퍼·접촉 조립 성능을 검증하지 않음.",
            "이산 자세·convex 충돌 근사에 따른 거리. 모든 연속 시각의 간격이나 전역 최적성을 보장하지 않음.",
            "다음 순서: 상시 명령과 구간 계측 → 탐색 비용 분석 → 동일 랜덤 목표 비교 → 작업별 확장."],
        provenance=dict(repository="https://github.com/WannaSleep3254/mujoco-ros2-manipulation", commit=evidence["git_head"],
            workcell="fr5_ur5e", models=["FR5", "UR5e"], scenario="dual_arm_crossing", input_kind="fixed_joint_targets"),
        environment={**evidence["current_environment"], "ros": "ROS 2 Humble", "measurement_dates": "GUI/headless: 2026-10-20; offline: 2026-10-21; Asia/Seoul"},
        sample_groups=groups,
        stages=[dict(id="input", label="지령·장면 준비", clock="wall", coverage="unmeasured", note="동일 요청 ID의 접수·상태 스냅샷 시각 미기록."),
            dict(id="compute", label="경로·출발 조정", clock="wall", coverage="partial", note="서비스 왕복과 별도 오프라인 탐색 측정. 같은 실행의 전체 구간 분해는 아님."),
            dict(id="validation", label="최종 안전검사", clock="wall", coverage="unmeasured", note="검사 성공은 확인했으나 구간 전체 소요시간 미분리."),
            dict(id="dispatch", label="Action 수락·예약", clock="simulation", coverage="partial", note="현재 sim 시각 +1초 예약과 시작 0.1초 전 양팔 수락 확인. wall 전송 지연 미계측."),
            dict(id="motion", label="실제 움직임·완료", clock="simulation", coverage="estimated", note="수신 상태·Action·수렴 확인. 최초 이동은 0.005rad·0.05초 기준의 후처리 추정. 실물 아님.")],
        metrics=metrics, assets=[], references=[
            dict(label="기준 구현 / PR #13", url="https://github.com/WannaSleep3254/mujoco-ros2-manipulation/pull/13"),
            dict(label="기존 상세 리포트·세션 대조 / PR #14", url="https://github.com/WannaSleep3254/mujoco-ros2-manipulation/pull/14")])
    for name, label, role in [
        ("comparison-traces.png", "보존된 GUI 관측: 관절 이동량·팔 간 간격", "figure"),
        ("phase1-performance.html", "기존 상세 HTML: 영상·대안·세션 대조 (원본 보존)", "legacy_report"),
        ("evidence.json", "원본 보고서 스냅샷·반복 시간·시계열 JSON", "data"),
        ("offline-trials.csv", "오프라인 5회 원본 CSV", "data"),
        ("browser-validation.json", "기존 상세 HTML의 브라우저 검증 기록", "data")]:
        result["assets"].append(dict(path=name, label=label, role=role, sha256=sha256(directory / name)))
    return validate(result, directory)
