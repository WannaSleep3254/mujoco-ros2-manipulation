# 공통 모듈 경계와 데이터 계약

[구조 안내](architecture.md) · [통합 작업리스트](development_worklist.md) · [명령·시간 기록](motion_requests.md) · [공동 작업 환경](workcells.md)

이 프로젝트는 로봇·시뮬레이터·계획기·제어 인터페이스·뷰어를 교체하고 확장할 수 있는 개발 환경을 목표로 합니다.
현재 실행 경로는 MuJoCo·ROS 2·MoveIt에 연결되어 있습니다. 이번 단계는 이미 분리된 데이터·계산 경계를 명시하고,
같은 모델·작업 환경·관절 정의를 후속 어댑터와 재현 도구가 확인할 수 있게 합니다. 범용 런타임 플러그인이나
ROS 2 없는 실행 경로를 구현한 상태는 아닙니다.

## 현재 경계와 결합

| 경계 | 현재 공유하는 내용 | 현재 구현에 남아 있는 결합 |
|---|---|---|
| 모델·설치 환경 | RobotProfile·Workcell, 제조사 소스 및 생성 자산 해시, 관절 소유권·설치 자세 | 프로필에 ROS 이름·MuJoCo 물리·서보 설정이 함께 있으며 생성기가 URDF/MJCF와 제어기 설정을 함께 만듭니다. |
| 목표 전달 | `motion_request.py`의 완전한 이름 기반 관절 목표와 요청 ID | 실제 전달은 ROS String 토픽과 로컬 HTTP가 담당합니다. v1 명령에는 모델 식별자가 없으며 한 ROS 도메인의 단일 서버가 시뮬레이션을 소유합니다. |
| 계획·후처리 | Spline의 이름·상대 시간·위치·속도·가속도, 출발 조정 및 경로 평가 함수 | 상시 서버가 WorkcellVerification을 상속하며 MoveIt 계획·유효성 검사와 ROS 액션을 직접 호출합니다. |
| 거리 검사 | `evaluator.at(완전한 상태)`의 거리·최근접 점·tip 위치 | 기본 DistanceEvaluator는 고정한 네이티브 MuJoCo 라이브러리와 충돌 형상을 사용합니다. |
| 실행·취소 | 요청 ID, 양팔 소유권, 상태·종료 기록 | FollowJointTrajectory와 ros2_control의 spline 보간, MuJoCo 접촉·시계 인터록을 사용합니다. |
| 표시·조작 | ConsoleState, HTTP API, 명령 규약, 요청별 기록 | ROS bridge의 작업 환경 확인과 상태 수신, 별도 CPU 표시 worker와 선택형 MuJoCo 영상 생성, 시작·종료 명령이 현재 스택을 전제로 합니다. |

주요 구현은 [설정](../src/mujoco_ros2_manipulation/workcells/config.py),
[명령](../src/mujoco_ros2_manipulation/workcells/motion_request.py),
[계획·실행 어댑터](../src/mujoco_ros2_manipulation/workcells/verify.py),
[상시 서버](../src/mujoco_ros2_manipulation/workcells/motion_server.py),
[경로 표현](../src/mujoco_ros2_manipulation/workcells/interference.py),
[거리 어댑터](../src/mujoco_ros2_manipulation/workcells/distance.py),
[콘솔 ROS bridge](../src/mujoco_ros2_manipulation/operator_console/ros_adapter.py),
[영상 생성](../src/mujoco_ros2_manipulation/operator_console/render_worker.py)에 있습니다.

## 정적 작업 환경 설명 v1

[`describe_workcell(cell, validated_manifest)`](../src/mujoco_ros2_manipulation/workcells/contracts.py)는
검증한 Workcell과 `check_workcell(cell)`의 결과에서 JSON으로 저장 가능한 설명을 만듭니다.
함수는 표준 Python 라이브러리만 사용하며 ROS·MuJoCo를 import하거나 파일을 읽지 않습니다.
따라서 자산 파일의 존재·변조 검사는 호출자가 먼저 수행해야 합니다. 설명을 만든 뒤 파일이 바뀌지 않는다는 보증도 제공하지 않습니다.

```python
from mujoco_ros2_manipulation.workcells.config import load_workcell
from mujoco_ros2_manipulation.workcells.integrity import check_workcell
from mujoco_ros2_manipulation.workcells.contracts import describe_workcell

cell = load_workcell()
validated_manifest = check_workcell(cell)
description = describe_workcell(cell, validated_manifest)
```

| 필드 | 의미 |
|---|---|
| `schema_version`, `kind` | 이 설명의 버전 1, `workcell_description`. 기존 명령 v1과는 별도 규약입니다. |
| `workcell_id`, `label` | 설정상의 작업 환경 식별자와 표시 이름입니다. |
| `identity` | Workcell fingerprint, 인스턴스별 프로필 fingerprint, 입력 경로별 SHA-256, 생성 파일별 SHA-256입니다. |
| `assets` | 저장소 기준 출력 디렉터리, 파일별 상대 경로·역할·SHA-256입니다. 절대 체크아웃 경로를 내보내지 않습니다. |
| `joint_names`, `joint_limits`, `home_rad` | 순서를 명시한 전체 관절명, 관절명별 한계, 관절명별 홈 자세입니다. |
| `robots` | 인스턴스·모델·접두사, 제조사 관절명→작업 환경 관절명 대응, base/tip frame, world 기준 설치 자세, 현재 ROS 그룹·제어기입니다. |
| `environment` | world 기준 고정 물체 목록입니다. 기존 작업 환경의 이름·형상·크기·xyz·rpy·rgba 표현을 유지합니다. |
| `units`, `world_frame` | 길이 m, 현재 회전 관절 rad, 속도 rad/s, 가속도 rad/s², 토크 N·m, 시간 s입니다. world와 URDF 고정축 roll/pitch/yaw 규칙을 명시합니다. |
| `current_adapters`, `scope` | 현재 연결된 스택과 고정 도형·두 로봇·회전 관절·완전한 관절 목표라는 적용 범위입니다. |

Workcell fingerprint는 작업 환경 설정과 프로필의 식별값이며, 전체 메시 파일의 해시를 뜻하지 않습니다.
실제 생성물과 그 입력의 일치는 `identity.inputs_sha256`·`identity.outputs_sha256`와 기존 무결성 검사로 확인합니다.
설명에 포함한 해시도 호출 시 전달받은 값입니다. 다른 어댑터가 다른 파일을 읽으면 해당 파일의 해시·버전과 변환 근거를 별도로 남겨야 합니다.

설치 자세의 `xyz_m`·`rpy_rad`는 제조사 로봇을 작업 환경의 world 좌표계에 배치하는 변환입니다.
로봇 내부의 link 변환이나 공구 TCP를 대신하지 않습니다. tip은 현재 URDF tip frame 원점이며 물리 공구 오프셋은 가정하지 않습니다.
환경 box의 size는 전체 변 길이, cylinder의 size는 반지름·전체 높이입니다. MuJoCo 내부 half-size와 섞지 않습니다.
현재 회전 관절 단위를 직선축·레일에 그대로 적용할 수 없으며, 그런 확장에서는 관절 종류와 단위 규약을 먼저 늘려야 합니다.

반환값은 입력과 참조를 공유하지 않습니다. 상시 명령 서버의 세션과 콘솔 `/api/bootstrap`의
`config.workcell_contract`에 같은 설명을 포함하여 세션·재현 기록의 모델 정체성을 연결합니다.
이 설명은 기존 `source.json` 생성 스키마와 명령 네 필드를 변경하거나 새 어댑터를 자동 로드하지 않습니다.

## 상태·시간 계약

계획·거리·실행 비교에는 동일한 관절명을 갖는 **완전한 전체 상태**를 사용합니다.
두 팔 중 한 팔만 계획하더라도 다른 팔의 실제 시작 상태를 함께 보존합니다. 배열 인덱스만으로
제조사 관절·작업 환경 관절·제어기 관절을 대응시키지 않으며 `robots[].joint_map`과 이름으로 변환합니다.

시간은 다음처럼 구분합니다.

- 원천 상태의 simulation stamp는 시뮬레이션 진행 시각입니다. JointState stamp와 `/clock`을 확인하고 정지·역행 시 실행을 막습니다.
- 수신 monotonic 시각과 age는 해당 프로세스가 상태를 얼마나 최근 받았는지 나타냅니다. 다른 프로세스의 monotonic 원점을 직접 빼지 않습니다.
- RequestTrace의 wall 시각은 서버 수신을 원점으로 한 단일 프로세스의 경과 시간입니다. simulation 시간과 더하지 않습니다.
- 세션 ID는 서버 재시작을 구분합니다. 재시작·모델 변경 후 이전 세션의 상태·요청을 새 실행에 자동 재사용하지 않습니다.

상시 서버는 초기 전체 상태, 원천 JointState stamp, 상태·시계 수신 age를 trace에 남깁니다.
콘솔도 원천 JointState stamp를 전달하고 `/api/state`의 `joint_state_stamp_simulation_sec`로 노출합니다.
이 값은 최근 `/clock`의 `simulation_sec` 및 수신 기준 `state_age_sec`와 별개입니다.
잘못된 원천 stamp는 null로 표시하며, 기존 관절 수신 age·시계 정지 감시·입장 규칙은 유지합니다.
원천 stamp를 노출한 것만으로 정밀한 상태 동기화·네트워크 지연 보정·보간이 구현된 것은 아닙니다.
다른 상태 전송·보간을 추가할 때에는 원천 stamp, 수신 시각, 세션/모델 identity를 함께 확인해야 합니다.

## 요청 수명주기와 실패 구분

기존 명령 v1은 `schema_version`, `request_id`, `strategy`, `target_rad`만 받습니다.
목표는 작업 환경의 전체 관절을 한 번씩 포함해야 하며, 취소는 해당 `request_id`를 사용합니다.
서버에서 한 번 관측한 요청 ID는 원래 응답을 재생하고 두 번째 궤적을 실행하지 않습니다.
새 목표는 새 ID를 사용하며, 같은 내용으로 재전송해도 서버가 바뀌었다면 이전 응답을 보장하지 않습니다.

처리 순서는 수신·입장 검사 → 대기 → 정지 상태 snapshot → 계획·출발 탐색 → 최종 거리·MoveIt 검사 →
상태 변화 확인 → 양팔 전송·수락 → 실행 결과 → 정착 → 관측 거리 검사 → 종료입니다.
목표가 이미 현재 자세에 있으면 계획·실행을 생략해도 현재 충돌·거리 조건은 확인합니다.

| 판정 | 기록과 해석 |
|---|---|
| 입장 거부 | 잘못된 규약·미준비·busy 등은 `rejected`입니다. 궤적을 보내지 않습니다. |
| 계획·검사 실패 | 경로 없음, 계산 예산 초과, 30mm 계획 간격 미달 등은 현재 `failed`이며 `dispatched_goals=0`이면 실행 전 실패입니다. |
| 실행 실패 | 궤적 전송 이후 목표 거부·추종 실패·인터록 등은 전송 수·action 결과·정지 기록을 함께 확인합니다. 접촉·시계 인터록은 재시작이 필요합니다. |
| 취소 | 취소 요청 수신은 완료가 아닙니다. 목표 수락·peer 취소·정지·최종 `cancelled` 또는 `failed`를 구분합니다. |
| 클라이언트 timeout | 서버의 종료나 취소를 뜻하지 않습니다. 요청 ID로 결과 또는 취소 완료를 확인해야 합니다. |

30mm는 두 팔 사이의 계획 기준 20mm에 추종 여유 10mm를 더한 현재 정책입니다.
실행 후 관측 간격은 20mm이며 두 값은 환경 전체의 공통 안전 여유나 실물 안전 인증이 아닙니다.
환경·자체 충돌은 MoveIt 검사, 런타임 접촉·시계 정지는 별도 인터록이 담당합니다.

지연 분석은 서버 수신→snapshot, 계획 서비스 왕복·서버 planning_time, 출발 탐색, 최종 검사, 실행 예약 대기,
첫 이동, action 완료, 정착을 구분합니다. 현재 계산 예산은 정지 상태 snapshot 뒤에 시작하고,
실행 시작은 계산 완료 뒤 시뮬레이션 시계에 1초를 더해 예약합니다. 시작 준비 시간은 상시 요청 지연과 별도입니다.
서비스 왕복에서 planning_time을 뺀 값에는 대기·처리·후처리가 섞일 수 있으므로 순수 ROS/DDS 지연으로 단정하지 않습니다.

## 교체 어댑터의 확인 항목

교체는 아래 의무를 만족하는 구현과 같은 입력의 비교 검증을 갖춘 뒤 진행합니다.

| 교체 경계 | 유지하거나 명시적으로 변환할 내용 |
|---|---|
| 모델·CAD 입력 | 자산 identity·단위·원점·설치 변환·관절명, visual/collision의 차이와 충돌 근사 방식입니다. |
| 물리 엔진 | 전체 상태·시계·세션, 관절 제어의 의미, 접촉·정지·재시작 동작입니다. 엔진별 접촉 수치가 같다고 가정하지 않습니다. |
| 계획기 | 완전한 초기 상태와 목표, 동일한 작업 환경·관절 한계·계산 예산, 경로 표현과 오류입니다. 후처리·최종 검사는 우회하지 않습니다. |
| 실행기·전송 | 관절 소유권·보간·예약 시간·목표 수락·취소 완료·실패 후 정지, 변경된 상태의 오래된 경로 실행 거부입니다. |
| 거리 평가기 | 거리 단위·부호·범위·형상 근사·frame·최근접 점·tip 정의입니다. 표본 검사를 연속 시간 보증으로 바꾸어 부르지 않습니다. |
| 뷰어 | 같은 자산·frame·이름 기반 상태, 수신 정지·재연결 표시입니다. 표시용 로봇을 두 번째 제어용 물리 인스턴스로 취급하지 않습니다. |

현재 `evaluator.at(state)`와 Spline record를 비교 경계로 사용할 수 있습니다.
표시용 상태 공급은 별도 CPU scene worker와 세션·모델·관절 이름 계약으로 분리했습니다.
다음 분리 후보는 MoveIt 요청/응답 변환, FJT 실행·취소, 상태 수신·identity 확인입니다.
이 후보는 후속 설계이며 범용 `Planner`·`Executor`·`Simulator` 플러그인을 이미 제공한다는 의미가 아닙니다.
비공개 QML 로컬뷰어와도 필요할 경우 자산·좌표·관절 대응 규약을 맞출 수 있지만, ROS 2 없는 실행이나 CAD 변환의
정확성을 이번 설명 함수만으로 검증하지는 않습니다.

## 검증 범위

[단위 검사](../tests/unit/test_workcell_contracts.py)는 실제 설정을 바탕으로 설명의 identity·단위·frame·관절 대응과
입력 불변성을 확인합니다. 불일치 manifest·잘못된 schema/hash/limits를 거부하고, 별도 Python 프로세스에서
ROS·MuJoCo·NumPy·YAML import를 차단한 상태로 설명을 만들 수 있는지도 확인합니다.
[콘솔 상태 검사](../tests/unit/test_operator_console.py)는 원천 stamp·수신 age·최근 clock을 구분하고,
원천 stamp가 수신 지연 판정을 바꾸지 않는지 확인합니다. 실제 콘솔 검증기는 bootstrap의 identity·관절 규약과
실제 수신 상태의 원천 stamp 존재도 검사합니다.
실제 엔진의 동작·충돌·계획 성공률과 어댑터 교체 후 호환성은 이 단위 검사 범위가 아닙니다.


## 로컬 환경 자산과 표시 경계

[설계파일 반영 절차](environment_design_import.md)는 단위·좌표계가 명시된 로컬 자산과 설치 목록을
별도 `environment-overlay-v1`로 합성합니다. 기존 Workcell v1 생성기를 바꾸거나 `source.json` 호환성을 가장하지 않습니다.
순수 파일·해시 검사와 별도 CPU 기하 검사를 나누고, 검증한 로컬 목록만 읽기 전용 뷰어에 등록합니다.
이 선택은 ROS 제어 환경을 바꾸지 않습니다. 환경이 포함된 실제 실행에는 시뮬레이터·MoveIt·명령 서버의
모델 일치를 함께 확인하는 별도 어댑터와 실행 검증이 필요합니다.
