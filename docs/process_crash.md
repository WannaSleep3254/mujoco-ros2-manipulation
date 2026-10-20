# 핵심 프로세스 크래시 시 전체 실행 종료 검증

[README](../README.md) · [공동 장면](workcells.md) · [접촉 자동 정지](contact_stop.md) · [CI](../.github/workflows/ci.yml)

## 범위와 성공 조건

FR5·UR5e가 실제로 움직이는 동안 `move_group` 또는 `ros2_control_node` 하나를
SIGKILL로 종료하고, 기존 launch의 핵심 프로세스 종료 처리로 전체 실행이 끝나는지 검사합니다.
각 대상은 새 headless launch에서 독립적으로 실행합니다. 운영용 종료 명령이 아니라 실패 주입 시험입니다.

1. 생성물 무결성·네이티브 접촉 인터록·MoveIt 충돌 검사·두 Action 준비를 확인합니다.
2. 양팔 첫 관절에 10초짜리 0.5rad 이동을 예약합니다. 양팔 모두 0.01rad 초과 이동하고
   0.01rad/s 초과 속도가 관측되며, 두 목표가 미완료이고 5초 넘는 명령이 남아 있어야 합니다.
3. 현재 실행 로그에 정확히 하나의 시작 기록이 있는 지원 대상만 선택합니다.
   `/proc` 실행 파일과 프로세스 그룹을 확인하고 PID 재사용에 대비한 pidfd로 신호를 보냅니다.
   다른 터미널·ROS 실행의 프로세스에는 신호를 보내지 않습니다.
4. 검증기는 Action 취소·launch 종료 명령을 보내지 않습니다. 벽시계 기준 8초 안에
   launch가 종료 코드 0으로 끝나고 프로세스 그룹이 없어져야 합니다.
5. 대기 중인 DDS 메시지를 비운 뒤 0.5초 동안 관절 상태·시계 발행이 더 없음을 확인합니다.
   로그의 유일한 비정상 종료가 주입한 노드 이름·PID·종료 코드 -9와 일치해야 합니다.
   다른 크래시, 시간 초과, 강제 정리, 남은 프로세스는 실패입니다.

일반 검증기는 이 예외를 사용하지 않으며 크래시를 계속 실패로 처리합니다.
`expected_process_exits`는 크래시 전용 실행에만 기록하고, 나머지는 `shutdown_failures`로 구분합니다.
늦게 추가된 크래시도 최종 로그 재검사에서 실패합니다.

## 실행과 결과

```bash
source ./manipulation_env.sh
python3 verify_process_crash.py --target move_group --domain 115
python3 verify_process_crash.py --target ros2_control_node --domain 116
```

설치된 패키지는 `manipulation-verify-process-crash`도 제공합니다.
ROS Humble 공동 장면 CI 작업에서 위 두 명령을 실행하고 성공·실패 원본 보고서와 로그를 artifact로 보관합니다.
필수 검사 이름 6개와 main 보호 규칙은 유지합니다.

2026-10-20 로컬 ROS Humble headless 실행 결과:

| 주입 대상 | 전체 그룹 종료까지 벽시계 | 주입 시 남은 궤적 시간 | 잔류 프로세스 | 증거 |
|---|---:|---:|---|---|
| `move_group` | 4.324초 | 9.75초 | 없음 | [원본 결과](process_crash/move_group.json) |
| `ros2_control_node` | 2.306초 | 9.75초 | 없음 | [원본 결과](process_crash/ros2_control_node.json) |

[MoveIt 실행 로그](process_crash/move_group.txt) · [MuJoCo 실행 로그](process_crash/ros2_control_node.txt)

로컬 단위 검사 73개·모델 검사 15개(총 88개), 구문·문서 링크·패키지 sdist/wheel 빌드,
기존 실제 공동 장면 정상 실행과 접촉 자동 정지 회귀 검증도 통과했습니다.

최신 결과는 `runtime/validation/crash_<대상>_fr5_ur5e_headless.json`,
실행별 원본은 해당 보고서의 `log_path` 옆 `result.json`에 남습니다.
위 원본 JSON의 절대 경로는 실행 PC의 기록이며, 이 저장소의 재현 명령과 함께 해석합니다.

## 해석의 한계와 남은 작업

전체 그룹 종료 시간은 모든 자식 프로세스가 정리될 때까지의 시간입니다.
접촉 검출 지연이나 물리 정지 반응 시간으로 해석하지 않습니다.
이번 두 실행에서는 주입 후 새 관절 샘플이 0개였으므로 관측 이동량 0은 위치 유지의 증거가 아닙니다.
특히 물리 엔진 프로세스를 강제 종료하면 이후 상태·최종 네이티브 보고서를 얻지 못할 수 있습니다.

따라서 `physical_hold_verified=false`, `physics_checks_passed=false`와 미실행 이유를 명시합니다.
`motion_checks_passed=true`는 이 검증의 양팔 이동 전제가 충족됐다는 뜻이며,
정상 MoveIt·TF·전체 물리 스텝 검사나 M5 전체 완료를 뜻하지 않습니다.
접촉 시 물리 고정·양팔 취소 검증은 기존 별도 시험을 유지합니다.

[시계 누락·반복 주입과 벽시계 감시](clock_stop.md)는 후속 PR에서 추가했습니다.
[양팔 전용 간섭 회피 사례](interference_avoidance.md)는 후속 작업에서 추가했습니다. 프로세스 크래시 후 위치 유지 검증은 남아 있습니다. GUI·실제 하드웨어 검사는 이번에 실행하지 않았습니다.
