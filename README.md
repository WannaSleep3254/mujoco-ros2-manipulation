# MuJoCo·ROS 2 로봇 제어 시뮬레이션

로봇의 동작을 시뮬레이션하고 검증하는 환경을 만드는 개인 프로젝트입니다.
MuJoCo, ROS 2, MoveIt 2를 연결해 경로를 계획하고 실행하며, 충돌 여부와 로봇 상태를 확인하는 기능을 개발하고 있습니다.

다음 세 가지를 목표로 개발하고 있습니다.

- **구성 요소 분리:** 로봇 모델, 시뮬레이터, 모션 플래너, 제어 인터페이스, 뷰어를 각각 교체하거나 확장할 수 있도록 구성합니다.
- **현장 환경 재현:** 현장의 설계 파일과 설비 배치를 시뮬레이션에 반영합니다.
- **동작 검증:** 구성한 환경에서 로봇의 경로와 실행 결과, 충돌 여부를 확인하고 기록합니다.

현재는 FAIRINO FR5·FR10과 Universal Robots UR5e를 대상으로 동작 검증 기능을 개발하고 있습니다.
이후 지원 로봇을 늘리고, 구성 요소 교체와 설계 파일 기반 환경 구성을 지원할 계획입니다.

[**프로젝트 소개**](https://wannasleep3254.github.io/mujoco-ros2-manipulation/operator/project-overview-2026-10-23/index.html) ·
[**프로젝트 사이트**](https://wannasleep3254.github.io/mujoco-ros2-manipulation/) ·
[**검증·평가 이력**](https://wannasleep3254.github.io/mujoco-ros2-manipulation/#evaluations)

## 대표 시연

[![같은 FR5·UR5e 장면을 웹 조작 화면·MuJoCo·RViz에서 확인하는 실제 시연](docs/operator/project-overview-2026-10-23/operation-poster.png)](https://wannasleep3254.github.io/mujoco-ros2-manipulation/operator/project-overview-2026-10-23/index.html#validation)

웹 화면에서 목표를 입력하면 MoveIt이 경로를 계획하고, ROS 2 제어기를 통해 MuJoCo에서 실행합니다. 같은 장면을 MuJoCo와 RViz에서 확인하는 과정을 촬영했습니다.
2026년 10월 22일에 촬영했으며, FR5의 이동, 접근, 후퇴, 홈 복귀 요청을 실행한 영상입니다. 물체 파지는 포함하지 않습니다.
웹 조작 화면에서 실행과 취소를 요청하고, 상태와 오류, 실행 이력을 확인할 수 있습니다.
취소와 복구 기능을 시험한 결과는 [조작 화면 검증 기록](docs/operator_console.md#검증)에 정리했습니다.
[접촉 시 양팔 정지와 장애물 없는 동작 비교](docs/contact_comparison.md)도 볼 수 있습니다.

## 직접 구현한 부분

MuJoCo는 물리 시뮬레이션과 렌더링, ROS 2와 ros2_control은 통신과 궤적 제어,
MoveIt 2와 OMPL은 모션 계획을 담당합니다.
MuJoCo와 ros2_control의 연동에는 [mujoco_ros2_control](https://github.com/ros-controls/mujoco_ros2_control)을 사용합니다.
이 프로젝트에서는 제조사 로봇 모델을 이 도구들과 연동하기 위한 설정, 모델 생성과 검사, 실행 및 상태 확인 기능을 구현하고 있습니다.
필요한 런타임 패치도 함께 관리합니다.

| 개발 항목 | 해결한 문제와 구현 | 문서와 코드 |
|---|---|---|
| 모델과 설정 공통화 | 로봇별 차이를 YAML 프로필로 관리하고 관절 이름으로 연결합니다. 생성 파일의 해시를 확인하고, URDF와 MJCF가 같은 관절값에서 계산한 자세를 비교해 변환 오류를 찾습니다. | [모델별 실행 기록](docs/usage-and-validation.md#검증-환경과-결과) · [프로필](config/robots/) · [생성물 검사](src/mujoco_ros2_manipulation/model_build/integrity.py) · [변환 회귀 검사](tests/model/test_conversion.py) |
| 실행 결과 검증 | Action이 성공했는지와 관절 오차가 허용 범위 안에서 유지되는지를 함께 확인합니다. 실행 성공, 목표 도달, 취소, 정상 종료를 구분해 기록합니다. | [요청 회귀 결과](https://wannasleep3254.github.io/mujoco-ros2-manipulation/2026-10-21-request-timing/index.html) · [검증 절차](docs/verification.md) · [상태 기반 판정](src/mujoco_ros2_manipulation/verification/ros_checks.py) |
| 양팔 동작과 계산 개선 | 이동 경로를 바꾸지 않고 출발 시점을 조정하도록 구현했습니다. 중복되는 평가 결과는 재사용하고, 불필요한 후보는 탐색에서 제외합니다. 요청 ID로 계산 시간과 실제 이동 시간을 나누어 측정합니다. | [동일 입력 A/B](docs/search_optimization.md) · [출발 탐색](src/mujoco_ros2_manipulation/workcells/timing.py) · [요청 계측](src/mujoco_ros2_manipulation/workcells/request_trace.py) |
| 오류 분석과 실패 대응 | 종료할 때 UI 자원을 해제하는 순서를 수정했습니다. 접촉이나 시계 이상을 감지하면 물리 시뮬레이션을 고정하고 양팔 동작을 취소합니다. MoveIt 플러그인의 수명 주기 문제에는 특정 환경에서만 적용하는 우회 방법을 사용합니다. | [접촉 대조 실행](docs/contact_comparison.md) · [종료 원인과 검증](docs/troubleshooting.md) · [종료 패치](patches/mujoco_ros2_control-0.1.2-shutdown.patch) · [물리 스텝 감시](native/step_monitor.hpp) |

기존 순차 탐색과 중복 평가 및 불필요한 후보 검사를 줄인 탐색을 고정 입력 6개에서 각각 5회씩 비교했습니다.
두 방식에서 선택된 팔, 출발 지연, 전체 궤적, 거리 평가 결과가 같은지 확인했습니다.
이 중 **교차 입력**은 시작 자세와 목표 자세에는 충돌이 없지만, 팔별 경로를 같은 시간축에 합치면 이동 도중 팔 사이에 간섭이 생기는 시험 조건입니다.
이 입력에서 탐색에 걸린 실제 시간의 중앙값은 **1.975초에서 1.537초로 줄었습니다.**
이 수치는 해당 입력의 오프라인 탐색 시간입니다. 요청부터 실행까지 걸리는 전체 시간이나 다른 작업에서의 성능은 별도로 평가해야 합니다.
[측정 조건과 원본](https://wannasleep3254.github.io/mujoco-ros2-manipulation/2026-10-21-search-optimization/index.html)

## 계획부터 실행 결과 확인까지

```text
웹 조작 화면 / CLI / RViz
  → 명령 서버 또는 MoveIt에서 경로 계획과 실행 요청
  → JointTrajectoryController (ros2_control)
  → mujoco_ros2_control → MuJoCo 관절 위치 서보
  → 관절 상태·TF·시계 수신 → 웹 화면 / RViz / 실행 기록
```

FR5와 UR5e는 하나의 물리 시뮬레이션과 시계, 전체 관절 상태를 공유하며, 각 팔은 별도의 궤적 제어기로 제어합니다.
바닥과 받침대의 고정 충돌 형상은 MuJoCo와 MoveIt에 같은 설정으로 반영합니다.
웹 화면의 렌더러는 수신한 관절값으로 현재 자세를 그립니다. 물리 상태를 적분하거나 제어 명령을 보내지는 않습니다.
[모델과 장면, 빌드 구조](docs/architecture.md) · [공동 장면](docs/workcells.md)

## 실행 시작하기

검증 환경: **Ubuntu 22.04 · Python 3.10 · ROS 2 Humble · amd64**
**새 PC에서는 [설치와 빌드 안내](docs/setup.md)를 먼저 진행해야 합니다.**
아래 명령은 의존성과 네이티브 런타임 설치를 마친 저장소 루트에서 실행합니다.
실행 중인 시뮬레이션을 종료한 뒤 공동 장면을 준비합니다.

```bash
python3 build_workspace.py --component models --workcell fr5_ur5e
source ./manipulation_env.sh
python3 operator_console.py --domain 89 --port 8765
```

[로컬 조작 화면](http://127.0.0.1:8765/)에서
**시뮬레이션 시작 → 실행 가능 확인 → 양팔 목표 설정 → 계획 후 실행** 순서로 조작합니다.
실행 후에는 화면의 성공 표시, Action 완료 여부, 관절 오차가 허용 범위 안에 있는지를 확인합니다.
이미 목표 자세에 있다면 안전검사를 거쳐 이동 없이 성공으로 처리합니다.
종료할 때는 **실행 종료 → 화면 서버 터미널 Ctrl+C** 순서를 따릅니다. 브라우저 탭만 닫으면 시뮬레이션이 종료되지 않습니다.
현재 웹 화면에서는 FR5·UR5e 공동 장면을 조작할 수 있습니다. MuJoCo, RViz, CLI도 사용할 수 있습니다.

**동작 시연** 메뉴에서는 장애물 접촉 정지와 경로 계획 비교를 직접 실행할 수 있습니다.
각 비교는 두 조건을 새 시뮬레이션에서 차례로 실행합니다. 시작 방법과 결과 표시는 [시연 사용법](docs/operator_console.md#충돌-정지와-경로-비교를-직접-실행하기)에 정리했습니다.

단일 로봇은 공동 장면과 화면 서버를 종료한 뒤 같은 터미널 환경에서 실행합니다.

```bash
bash run_sim.sh robot:=fr5
```

`robot:=fr10` 또는 `robot:=ur5e`로 초기 테스트 모델을 선택할 수 있습니다.
현재 실행을 종료한 뒤 다른 모델을 시작합니다.
모델 설정, GUI 옵션, 검증 명령과 환경별 주의사항은 [상세 사용·검증 안내](docs/usage-and-validation.md)에 모았습니다.

## 현재 범위와 다음 목표

현재까지 제조사 모델 변환, 관절 위치 궤적 실행, 두 로봇의 공동 장면,
계획과 간격 검사, 접촉과 시계 감시, 동작 취소, 목표 도달 확인, 정상 종료와 결과 기록 기능을 확인했습니다.
모델별 실행, GUI 동시 실행, 고정 입력과 근거리 목표를 사용한 평가는 각 기록에 적힌 환경과 조건에서 수행했습니다.

- **로봇과 장면:** FR5·FR10·UR5e와 FR5·UR5e 공동 장면을 지원합니다. 주변 설비는 박스와 원통 같은 고정 형상으로 구성합니다.
- **목표 입력:** 관절값으로 목표를 지정하고 공동 계획이나 출발 시점 조정으로 실행합니다.
- **거리 조건을 만족하지 못한 경로:** 실행하지 않고 실패로 기록합니다. 일반 요청에서 다른 공간 경로를 자동으로 찾는 기능은 앞으로 개발할 계획입니다.
- **아직 검증하지 않은 항목:** 전체 작업 공간에서의 성공률, 장시간 안정성, 실제 파지와 실물 제어입니다.

앞으로는 다음 작업을 진행할 계획입니다.

- 거리 조건을 만족하지 못하는 경로에서 다른 경로를 찾도록 개선하고, 요청부터 실행까지 걸리는 시간을 줄입니다.
- 설계 파일과 설치 배치를 가져오는 절차를 마련하고, 모듈을 교체하기 쉽도록 연결 방식을 정리합니다.
- Pose/IK 목표 입력, TCP, 그리퍼, 물체 상태를 다루는 기능을 추가하고 테스트 로봇을 늘립니다.
- 실제 로봇과 연동할 때 실행 요청, 상태 피드백, 취소를 처리하는 방식과 접촉 작업의 제어 방법을 검토합니다.

접촉 정지는 **공동 시뮬레이션의 물리 상태를 고정하는 기능**입니다.
실물 안전 인증이나 비상 정지 기능으로 검증한 결과는 아닙니다.
[현재 개발 상태와 남은 작업](docs/simulator_status.md)

## 주요 문서

| 자료 | 내용 |
|---|---|
| [설치와 빌드](docs/setup.md) | 새 PC 환경 준비, 사용할 소스와 의존성 버전, 빌드 방법 |
| [상세 사용·검증 안내](docs/usage-and-validation.md) | 모델별 명령, 실행 옵션, 환경별 주의사항과 기존 비교 시연 |
| [패키지와 장면, 빌드 구조](docs/architecture.md) | 설정과 코드의 역할, 공유 충돌 형상, 생성 파일과 CI |
| [현재 개발·검증 상태](docs/simulator_status.md) | 구현한 기능, 확인한 동작과 남은 작업 |
| [뷰어 개발 방향성](https://wannasleep3254.github.io/mujoco-ros2-manipulation/operator/viewer-direction-2026-10-22/index.html) | 화면 구성, 기술 대안과 개선 계획 |
| [모션플래닝 개발 방향성](https://wannasleep3254.github.io/mujoco-ros2-manipulation/operator/motion-planning-direction-2026-10-22/index.html) | 지연과 우회 원인 분석, 빈피킹과 모션 계획 기술, 실물 로봇 연동 계획 |

제조사 모델과 연동 패키지의 버전 및 라이선스는 [출처 안내](docs/setup.md#소스-버전과-출처)에 기록했습니다.
