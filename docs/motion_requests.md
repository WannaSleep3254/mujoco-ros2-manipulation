# 상시 모션 명령과 요청별 구간 계측

MuJoCo 공동 장면을 한 번 실행한 상태에서 새 관절 목표를 받아 계획·검사·실행합니다.
현재 대상은 FR5·UR5e 공동 장면이며, 실제 로봇 연결은 지원하지 않습니다.

## 실행

터미널 1에서 기존 시뮬레이션을 실행합니다.

```bash
bash run_scene.sh workcell:=fr5_ur5e gui:=true rviz:=true planning_case:=dual_arm_crossing
```

터미널 2에서 같은 ROS 도메인을 사용해 명령 노드를 실행하고 `READY`를 기다립니다.

```bash
source ./manipulation_env.sh
python3 motion_server.py
```

터미널 3에서 아래 예제로 양팔 첫 관절을 홈보다 0.2rad 이동시키는 새 목표 파일을 만들 수 있습니다.

```bash
source ./manipulation_env.sh
python3 - <<'PY'
import json
from pathlib import Path
from mujoco_ros2_manipulation.workcells.config import load_workcell
cell = load_workcell()
target = dict(zip(cell.joints, cell.home))
for arm in cell.instances:
    target[arm.joints[0]] += 0.2
with Path("runtime/demo-target.json").open("x") as stream:
    json.dump(target, stream)
PY
python3 motion_command.py --target runtime/demo-target.json
python3 motion_command.py --home
```

기존 목표 파일을 덮어쓰지 않도록 예제는 새 파일에만 기록합니다.
일반 명령은 완전한 관절 목표 JSON 파일을 보냅니다. 파일은 12개 관절명과 rad 값을 포함하는 객체입니다.
기존 모델·관절명은 `config/workcells/fr5_ur5e.yaml`과 생성 URDF를 기준으로 합니다.

```bash
source ./manipulation_env.sh
python3 motion_command.py --target /절대경로/target.json
python3 motion_command.py --home --strategy departure_timing
python3 motion_command.py --cancel 출력된_32자리_요청_ID
```

`--home`은 현재 위치에서 홈까지 새 경로를 계획합니다. 취소 명령은 별도 터미널에서도 보낼 수 있습니다.
한 도메인에는 명령 노드를 하나만 실행합니다. 같은 컨트롤러에 다른 노드가 명령을 보내지 않는 독립 시뮬레이션을 사용합니다.

## 명령 계약

`/workcell/motion/command`의 `std_msgs/msg/String` JSON은 다음 네 필드를 갖습니다.

| 필드 | 의미 |
|---|---|
| schema_version | 정수 1 |
| request_id | 중복 없는 소문자 16진수 32자리. 클라이언트가 기본 생성 |
| strategy | `coordinated` 또는 `departure_timing` |
| target_rad | 모든 공동 장면 관절을 한 번씩 포함하는 유한한 제한 범위 내 목표 |

`coordinated`는 양팔 공동 MoveIt 계획을 사용합니다.
`departure_timing`은 팔별 계획을 같은 시간으로 늦춰 맞춘 뒤 기존 출발 시점 탐색을 수행합니다.
출발 탐색이 실패하면 큰 우회로 자동 변경하지 않고 요청을 실패로 남깁니다.
실행 직전에는 양팔 전체 상태로 MuJoCo 거리와 MoveIt 보간 상태 유효성을 다시 검사합니다.
기존 계획 30mm·관측 20mm 기준과 네이티브 2ms 접촉·시계 정지 감시는 유지합니다.
이 거리 검사는 이산 표본과 convex 충돌 형상에 기반하며 연속 시간 안전 인증이 아닙니다.

동시에 들어온 다른 요청은 대기열에 쌓지 않고 거부합니다.
같은 ID는 기존 상태를 다시 반환하며, 목표를 바꾸어 재사용해도 재실행하지 않습니다. 새 동작에는 새 ID를 씁니다.
명령 수신 토픽은 volatile이며 서버 재시작 때 과거 명령을 재생하지 않습니다.
상태는 `/workcell/motion/status`에 reliable/transient-local로 발행하며 원본은 파일에 보관합니다.
취소 토픽 `/workcell/motion/cancel`의 JSON은 `request_id` 한 필드입니다.

## 계측 경계

`runtime/commands/<세션>/<요청_ID>.json`에 요청별 원본, 구간, 이벤트, 계획 정보와 관측을 보존합니다.
검증 실행은 같은 형식을 `runtime/validation/command-sessions/`에 기록합니다.

| 계측 | 시계·해석 |
|---|---|
| 수신 → 최초 이동 | 명령 노드의 `time.monotonic_ns` 간격. `/joint_states`에서 0.001rad 초과 변화가 처음 관측된 수신 시각 |
| 계획 서비스 왕복 | 요청 생성부터 응답 수신까지의 wall 시간 |
| 서버 planning_time | MoveIt 응답 필드 그대로. 왕복 시간과 별도 기록하며 차이를 순수 DDS 지연으로 단정하지 않음 |
| 출발 탐색·안전검사 | 같은 기록 프로세스의 wall 구간. 사용하지 않으면 null과 이유 |
| Action 전송·수락·결과 | 팔별 wall 이벤트와 별도 sim 시각 |
| 수신 → Action 완료 / 수렴 / 최종 결과 | 각각 별도 종료점. 검증·기록 처리 시간이 포함되는 최종 결과와 구분 |
| 예약 출발·의도된 팔 대기 | sim 시간. CPU 계산 시간에 더하지 않음 |
| 클라이언트 전송 → 결과 수신 | 클라이언트의 별도 monotonic wall 간격. 서버 시계와 직접 빼지 않음 |

최초 이동은 물리 엔진의 첫 적분 시각 또는 실물 엔코더 시각과 다릅니다. 발행·DDS·수신 처리 해상도가 포함됩니다.
시뮬레이터·MoveIt·거리 모델의 최초 구동 비용은 요청 수신 전에 발생하며 세션 정보에 구분합니다.
요청 planner ID는 `RRTConnectkConfigDefault`로 명시합니다. `GetMotionPlan` 응답은 실제 선택 planner ID를 제공하지 않아 해당 값은 null입니다.
실패·취소 요청의 최초 이동이 없으면 null이며 0초로 채우지 않습니다.

취소는 대기 중 요청의 실행을 막고 실행 중이면 양팔 Action을 취소합니다.
MoveIt 서비스와 Python/native 계산 자체를 선점 중단하는 기능은 아니므로 해당 호출이 반환된 뒤 취소를 확인할 수 있습니다.
명령 노드의 SIGINT/SIGTERM 종료는 ROS 컨텍스트를 유지한 채 진행 중 양팔 Action을 취소한 뒤 노드를 닫습니다.
클라이언트의 대기 시간 초과는 자동 취소가 아닙니다. 제출 ID로 취소하고 상태·원본을 확인합니다.
실행 실패 후 제어기 정리·정지가 불확실하면 노드는 다음 명령을 거부하며 시뮬레이션과 함께 점검·재시작합니다.
SIGKILL·전원 차단처럼 취소를 전송할 기회가 없는 명령 노드 강제 종료 감시는 이번 구현에 포함하지 않습니다.

## 검증

```bash
source ./manipulation_env.sh
python3 verify_motion_commands.py --domain 138
```

하나의 새 시뮬레이션에서 연속 공동/출발 조정 요청, 중복 ID, 잘못된 목표, 실제 충돌 목표 계획 실패,
대기 취소, 실제 이동 후 양팔 취소, 동시 요청 거부와 취소 후 새 명령, 별도 CLI와 동작 중 명령 노드 종료를 검증합니다.
이미 도달한 0.001rad 이내 목표는 새 관절 명령을 보내지 않고 최초 이동을 null로 유지합니다.
이는 작은 고정 회귀 사례이며 랜덤 목표 성공률이나 p95/p99 성능평가가 아닙니다.


## 요청 계측 결과 발행

검증이 완료된 실행을 새 평가 ID로 가져옵니다. 요청 실패·취소·거부를 포함한 전체 스냅샷을 보존합니다.

```bash
runtime/report-tools/bin/python scripts/import_motion_request_report.py \
  --run runtime/validation/runs/실행_ID/result.json \
  --report-id 새날짜-request-timing --date YYYY-MM-DD
runtime/report-tools/bin/python scripts/reports.py build
```

[2026-10-21 요청 계측 리포트](reports/2026-10-21-request-timing/index.html)는 고정 구현의 발행 예입니다.
CI 자료를 다른 경로로 다운로드했으면 `--traces`로 command session 폴더를 지정합니다.
기존 발행본은 `--check docs/reports/평가_ID`로 원본과 수치를 대조합니다.

공식 제어 인터페이스: [JTC Action과 완료·선점 정책](https://control.ros.org/humble/doc/ros2_controllers/joint_trajectory_controller/doc/userdoc.html).

## 계산 예산과 반복 평가

명령 노드의 `--compute-budget` 기본값은 30wall초입니다. 상태 스냅샷 후 계획·탐색·최종 검사·상태 재확인 구간에 협력식으로 적용하고, 초과 시 이동 지령 전에 실패합니다. 각 RPC와 네이티브 샘플을 선점하지 않으므로 실제 초과 시간이 기록될 수 있습니다. 실행·예약 시간과 구분합니다. `compute_budget`와 `computation_started/done` 이벤트, 출발 탐색의 후보별 비용을 원본에 추가했습니다.

[고정 seed 랜덤 목표 비교](random_motion_benchmark.md)는 두 전략에 같은 목표·예산을 적용하고, 복귀·워밍업을 별도로 보존합니다.
