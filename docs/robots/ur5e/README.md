# Universal Robots UR5e

[모델 자료 목록](../README.md) · [프로젝트 README](../../../README.md)

Universal Robots 공식 Humble Description의 Xacro를 사용합니다. 원본 DAE의 장면 변환·단위·색상·표면 법선을 보존해 OBJ로 변환하고, 공식 flange/tool0 좌표계와 시뮬레이션용 MoveIt 설정을 연결했습니다.

![Universal Robots UR5e 초기 자세 기준 렌더](images/home.png)

`home_rad`의 초기 자세를 MuJoCo Python 3.14.0으로 렌더한 이미지입니다.
물리 정착 전의 기구학적 기준 자세이며, 카메라는 로봇 형상에 맞춰 자동 배치합니다.
[촬영 조건과 관절값](reference_image.json)을 함께 보관합니다.

## 모델 구성

| 항목 | 설정 |
|---|---|
| 모델 선택 ID | `ur5e` |
| 프로필 | [config/robots/ur5e.yaml](../../../config/robots/ur5e.yaml) |
| MoveIt 패키지 | `ur5e_moveit_config` |
| 계획 그룹 | `ur5e_manipulator` |
| 궤적 제어기 | `ur5e_controller` |
| 계획 끝단 | `tool0` |
| 기본 프레임 | `base_link` |
| 제조사 URDF 질량 합 | 21.70000 kg |
| SRDF 충돌 제외 쌍 | 11개 |
| 시각 모델 | 원본 DAE를 색상별 OBJ로 변환 |
| 물리 충돌 모델 | 제조사 STL의 MuJoCo convex hull 근사 |

`wrist_3_link → flange → tool0` 고정 변환을 원본대로 보존합니다. 계획 체인은 `base_link → tool0`이며, 그리퍼 TCP 오프셋은 아직 없습니다. 기본 기구학은 특정 실물 로봇의 캘리브레이션 값이 아닙니다. 끝단 프레임의 위치·자세와 시각 메시의 배치를 독립적인 URDF 순기구학 계산과 대조했습니다.

질량·관절 제한은 URDF의 모델 값입니다. 실물 계측 또는 모터 식별 결과가 아니며,
서보 게인은 시뮬레이션 예제 설정입니다. 아래 속도 값은 URDF 제한으로, MuJoCo에
자동적인 관절 속도 제한을 추가하는 값은 아닙니다.

| 관절 순서 | 초기 각도 (rad) | Kp | Kv | URDF effort (Nm) | URDF velocity (rad/s) |
|---|---:|---:|---:|---:|---:|
| `shoulder_pan_joint` | 0 | 2500 | 100 | 150 | 3.1416 |
| `shoulder_lift_joint` | -1.5708 | 7000 | 180 | 150 | 3.1416 |
| `elbow_joint` | 1.5708 | 5000 | 140 | 150 | 3.1416 |
| `wrist_1_joint` | -1.5708 | 1000 | 30 | 28 | 3.1416 |
| `wrist_2_joint` | -1.5708 | 700 | 20 | 28 | 3.1416 |
| `wrist_3_joint` | 0 | 500 | 15 | 28 | 3.1416 |

물리 계산 간격은 0.002초, ros2_control 갱신은 100 Hz, 관절 상태 발행 설정은
50 Hz입니다. 위치 명령을 서보 토크로 변환하므로 중력·관성·토크 포화의 영향을 받습니다.

## 실행과 모델 변경

설치와 최초 빌드는 [공통 설치 안내](../../../README.md#git에-포함하는-파일과-다른-pc에서의-준비)를
따릅니다. 아래 명령은 저장소 루트에서 실행합니다.

```bash
bash run_sim.sh robot:=ur5e
```

MuJoCo와 RViz가 함께 열립니다. RViz MotionPlanning의 Planning Group 값을
`ur5e_manipulator`로 선택하고, 현재 상태에서 목표 마커를 이동한 뒤
`Plan`으로 경로를 확인하고 `Execute`로 실행합니다. 관절 각도 단위는 rad입니다.

GUI 없이 실행하거나 MuJoCo만 실행하려면:

```bash
bash run_sim.sh robot:=ur5e gui:=false rviz:=false
bash run_sim.sh robot:=ur5e moveit:=false rviz:=false
```

프로필의 초기 자세·게인·검증 목표를 수정한 뒤에는 모델을 재생성하고 재실행합니다.
초기 자세와 게인 배열은 위 관절 순서를 따릅니다.

```bash
.venv/bin/python prepare_robot.py --robot ur5e
bash run_sim.sh robot:=ur5e
```

모델 재생성 결과는 실행 중인 창에 자동 반영되지 않습니다. 다른 모델로 바꾸려면
현재 launch를 Ctrl+C로 종료하고 원하는 `robot:=` 값으로 다시 실행합니다.
공통 실행 도메인은 89입니다. `use_sim_time`은 시간 기준이며 모델 선택 옵션은 아닙니다.

## 실제 GUI 스크린샷

2026-10-07에 별도 ROS 도메인 91에서 직접 궤적 명령과 MoveIt ROS API
계획·실행을 완료한 뒤 촬영했습니다. 이미지 픽셀을 보정하거나 합성하지 않았으며,
촬영용 launch의 창만 크기·카메라를 조정했습니다.

### MuJoCo

![Universal Robots UR5e MuJoCo 실행 화면](images/mujoco_gui.png)

위 화면은 ROS 런타임의 MuJoCo 3.12.0 GUI입니다. 초기 자세 렌더와는 실행 시점과
카메라가 다릅니다. 실행 후 자세와 상태 패널을 함께 확인할 수 있습니다.

### RViz / MoveIt 2

![Universal Robots UR5e RViz MotionPlanning 화면](images/rviz.png)

주황색 로봇은 RViz의 계획 요청 표시이고 반투명 모델은 Planning Scene의 현재 상태입니다.
검증기는 ROS API로 궤적을 생성·실행합니다. 실제 관절 상태와 실행 결과는
[촬영 실행의 검증 기록](gui_capture_validation.json)에 보관했습니다.

## 검증 결과

기존 [전체 검증 요약](../../ur5e_validation.json)은 실행 모드별 검사와 종료 검사를 기록합니다.
아래 날짜는 해당 기록의 한국 시간이며, 이번 스크린샷 촬영 실행의 날짜는 별도 JSON에 있습니다.

| 실행 모드 | 기존 검증 시각 (KST) | 궤적·MoveIt·TF·시계 | 최대 관절 오차 (rad) | 종료 |
|---|---|---|---:|---|
| headless · SIGINT | 2026-10-07 15:52 | 통과 | 0.004052 | `0` · 잔류 없음 |
| GUI · SIGINT | 2026-10-07 15:52 | 통과 | 0.004052 | `0` · 잔류 없음 |
| GUI · 창 닫기 경로 | 2026-10-07 15:52 | 통과 | 0.004052 | `0` · 잔류 없음 |

궤적 Action의 성공 결과를 받은 뒤, 관절 오차가 0.01 rad 이내인 상태 메시지를
10개 연속 수신해 완료를 확인합니다. 창 닫기 검증은 Trigger 서비스로 GUI와 같은
종료 경로를 실행한 검사입니다. 실제 마우스 클릭 검증으로 기록하지 않습니다.

다시 검증하려면 ROS 환경을 적용하고 시스템 Python을 사용합니다.

```bash
source ./manipulation_env.sh
python3 verify_robot.py --robot ur5e --domain 91
python3 verify_robot.py --robot ur5e --domain 91 --gui
python3 verify_robot.py --robot ur5e --domain 91 --gui --stop-mode window
```

이 오차는 예제 목표와 게인에서 얻은 결과입니다. 다른 모델과 성능 순위를 비교하는
벤치마크 결과가 아닙니다. 그리퍼·TCP 도구·조립 부품·외부 장애물 장면과 실물 로봇
연결은 아직 구현하지 않았고, MuJoCo 바닥은 MoveIt 계획 장면에 등록하지 않았습니다.

## 출처와 자료

- [제조사 소스](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/tree/65fa221f6d9e1904b30b6a05ae39cb24a0c40fac) — 고정 커밋 `65fa221f6d9e1904b30b6a05ae39cb24a0c40fac`.
- [모델 생성·출처 기록](../../../models/ur5e/source.json).
- [프로필](../../../config/robots/ur5e.yaml) 및 [공통 제어 설정](../../../config/controller_defaults.yaml).
- [전체 검증 요약](../../ur5e_validation.json)과 [이번 GUI 촬영 검증](gui_capture_validation.json).
- [이미지 목록·해시·촬영 정보](assets.json) 및 [자료 재생성 방법](../capture.md).
