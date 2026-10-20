# FR5·UR5e 공동 작업장면

[README](../README.md) · [구조](architecture.md) · [확장 계획](plans/base_and_multi_robot.md)

단일 MuJoCo 세계에 FR5와 UR5e를 배치하고, 하나의 SystemInterface·controller_manager·관절 상태 발행기와
팔별 제어기 두 개를 사용합니다. 기존 단일 로봇 명령은 그대로 유지합니다.

## 실행

기존 설치 환경과 네이티브 런타임이 준비되어 있다면 다음 명령으로 두 모델과 공동 장면을 생성합니다.

```bash
python3 build_workspace.py --component models --workcell fr5_ur5e
bash run_scene.sh workcell:=fr5_ur5e
# 창 없이 실행
bash run_scene.sh workcell:=fr5_ur5e gui:=false rviz:=false
```

새 PC에서 런타임까지 빌드할 때는 `python3 build_workspace.py --download-debs --workcell fr5_ur5e`를 사용합니다.
이미 단일 모델이 생성되어 있다면 `.venv/bin/python prepare_workcell.py --workcell fr5_ur5e`만 실행할 수 있습니다.
원본 모델은 중립 설치 자세의 `default` 장면을 사용합니다. 개별 설치는 공동 장면의 인스턴스에 지정합니다.
기본 장면의 환경 물체는 제거한 뒤 공동 장면의 물체를 한 번만 추가합니다.

독립 자동 검증은 기존 실행을 종료한 뒤 실행합니다. 검증기는 자신의 launch를 별도 프로세스 그룹으로 시작하고 정리합니다.

```bash
source ./manipulation_env.sh
python3 verify_scene.py --workcell fr5_ur5e --domain 93
```

설치형 명령은 `manipulation-prepare-workcell`, `manipulation-verify-workcell`입니다.
`MUJOCO_ROS2_WORKSPACE`를 지정하면 저장소 밖에서도 사용합니다. GUI·GPU·성능 검사는 [접촉 감시와 성능 검증](workcell_diagnostics.md)을 참고합니다.

## 설정과 이름

[config/workcells/fr5_ur5e.yaml](../config/workcells/fr5_ur5e.yaml)은 공통 물리 조건·고정 물체·로봇 인스턴스를 정의합니다.
현재는 두 인스턴스를 지원하며 이름으로부터 접두사와 제어기·계획 그룹을 도출합니다.
이름 중복, 서로 구별하기 어려운 접두사, 알 수 없는 모델, 비정상 수치와 잘못된 치수를 거부합니다.

| 인스턴스 | 모델 | 관절 예시 | 제어기 | 계획 그룹 | 끝단 |
|---|---|---|---|---|---|
| left | FR5 | `left_j1` | `left_controller` | `left_arm` | `left_wrist3_link` |
| right | UR5e | `right_shoulder_pan_joint` | `right_controller` | `right_arm` | `right_tool0` |
| 전체 | 12관절 | 양팔 전체 | 두 제어기로 분배 | `both_arms` | 관절 공간 공동 목표 |

Action 경로는 `/left_controller/follow_joint_trajectory`, `/right_controller/follow_joint_trajectory`입니다.
상태는 `/joint_states`, 시계는 `/clock` 하나이며 모든 제어기에 `use_sim_time`을 적용합니다.
직접 Action 명령은 MoveIt 충돌 검사를 자동 수행하지 않습니다. 검증기는 계획·상태 검사를 통과한 궤적만 정상 실행으로 전달합니다.

개별 팔의 기존 목표는 단일 로봇 환경용입니다. 두 팔의 목표를 결합하면 손목끼리 충돌할 수 있어,
작업장면의 `verification.plan_target_rad`에서 정상 검증 목표를 따로 설정합니다.
실제 확인한 충돌 목표는 거부 검사의 자료로 사용합니다. 충돌 제외는 제조사 SRDF의 각 팔 내부 쌍만 복사하며,
팔 사이 충돌을 제외하지 않습니다. 양팔 그룹은 관절 공간 계획이며 양팔 동시 Pose IK는 구현하지 않았습니다.

## 생성물과 무결성

`runtime/generated/workcells/fr5_ur5e/` 아래에 다음 파일을 생성합니다. Git에는 포함하지 않습니다.

| 파일 | 내용 |
|---|---|
| `cell.xml` | 접두사가 적용된 메시·관절·구동기, 환경·배치·12관절 초기 자세 |
| `cell.urdf` | 합성 ROS 모델과 단일 하드웨어 인터페이스 |
| `cell.srdf` | 팔별 체인, 양팔 그룹, 기존 충돌 제외와 초기 자세 |
| `controllers.yaml` | 상태 발행기와 팔별 관절 소유권 |
| `moveit.yaml`, `moveit.rviz` | 계획·IK·관절 제한·제어기·RViz 설정 |
| `source.json` | 장면·프로필 지문, 입력/출력 해시, 원래 관절과 초기 자세·정착 검사 |

참조하는 단일 모델도 실행 전에 무결성을 확인합니다. 원본 메시·프로필·생성기 또는 공동 장면이 바뀌면
재생성해야 합니다. 임시 디렉터리에서 컴파일·물리 정착을 통과한 후 파일을 교체하고 명세를 마지막에 게시합니다.
실행 중인 모델은 재생성하지 않습니다. 게시 도중 실패하면 해시가 달라 다음 실행이 거부됩니다.
동일 FR5 두 대의 메시·구동기 이름 분리와 컴파일도 모델 검사에 포함하며, 이 조합의 실제 ROS 작업 검증과는 구분합니다.

## 검사와 실행 조정

`verify_scene.py`는 다음을 실제 ROS·MoveIt·MuJoCo에서 확인합니다.

1. 12관절, 활성 제어기 3개, 시계/상태 발행 노드 각각 하나와 양팔 TF.
2. 각 팔의 단독 궤적 완료와 전체 상태 수렴. 나머지 팔의 변화가 0.01rad 미만인지 확인.
3. 다른 팔을 전체 상태에 포함하는 팔별 계획과 12관절 공동 계획.
4. 최대 0.025rad·0.05초 간격으로 보간한 전체 상태의 MoveIt 충돌 검사. 실패하면 정상 궤적을 전송하지 않음.
5. 같은 시뮬레이션 시작 시각을 갖는 두 Action, 양쪽 결과 성공과 전체 12관절 오차 0.01rad 미만의 새 메시지 10회 연속 확인.
6. 팔 사이 충돌 목표가 계획 단계에서 거부되고 해당 목표의 Action이 전송되지 않는지 확인.
7. 한쪽 목표 거부 시 수락된 상대 목표를 취소. 두 팔이 움직이는 중 한쪽을 취소하는 실패 주입에서도 양쪽 취소 결과와 정지를 확인.
8. launch 종료·프로세스 그룹 잔류·강제 종료·크래시를 함께 기록.

관절 상태로 관측한 시작 시각 차이 허용값은 0.10초입니다. 발행 주기와 움직임 임계값의 영향을 받는 관측값이며
실제 모터 동기나 제어 스레드 지터 측정값은 아닙니다. CI는 공동 장면을 두 번 연속 실행하며 실패 재시도는 하지 않습니다.

위치 서보의 이전 명령을 유지하도록 공동 제어기에 `open_loop_control`을 적용하고, 첫 궤적점은 마지막 성공 명령으로 고정한 뒤
다시 충돌 검사를 수행합니다. 실제 위치 피드백·Action 결과·수렴 검사는 계속 사용합니다.
이는 미래 시작 시각 전 현재 측정값으로 보간하면서 생기는 중력 처짐을 구별하기 위한 구성입니다.
[Humble 궤적 표현](https://control.ros.org/humble/doc/ros2_controllers/joint_trajectory_controller/doc/trajectory.html),
[컨트롤러 보간 소스](https://github.com/ros-controls/ros2_controllers/blob/humble/joint_trajectory_controller/src/trajectory.cpp)를 참고합니다.

## 기록과 남은 범위

최근 보고서는 `runtime/validation/workcell_fr5_ur5e_headless.json`, 실행별 JSON·로그는 `runtime/validation/runs/`에 보관합니다.
코드·모델 해시와 실제 결과는 [검증 기록](workcell_validation.json)에 정리합니다. 실패·중단 보고서도 같은 수명주기를 사용합니다.

이산 상태 검사는 연속 시간의 무충돌 보장이 아닙니다. 네이티브 물리 스텝의 접촉 기록과 GUI·성능의 1차 측정은
[접촉 감시와 성능 검증](workcell_diagnostics.md)에 추가했습니다. 시계 정지·프로세스 비정상 종료의 양팔 정지 실패 주입,
개별 경로가 간섭하지만 공동 계획으로 우회하는 전용 사례, 실제 하드웨어와 전체 성능 비교(M6)는 남아 있습니다.
고급 실패 주입과 공동 회피 전용 사례를 완료하기 전에는 M5 전체 완료로 표시하지 않습니다.
CAD 메시 받침대·레일·그리퍼·도구 TCP는 기존 후속 범위입니다.
