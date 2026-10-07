# 동작과 종료 검증

[프로젝트 README](../README.md) · [설치와 빌드](setup.md) · [모델별 자료](robots/README.md)

[verify_robot.py](../verify_robot.py)는 선택한 모델의 실제 ROS 노드를 별도 프로세스로 실행해
궤적·MoveIt·상태·TF·시계와 종료를 검사합니다. 아래 명령은 저장소 루트에서 실행합니다.

## 명령 종료 시점의 확인

연속 명령은 지정한 시간만 기다리는 방식이 아니라 다음 결과를 확인한 뒤 진행합니다.

1. `FollowJointTrajectory` 목표가 수락되고 Action 상태가 `SUCCEEDED`인지 확인합니다.
2. 제어기 결과의 `error_code`가 `SUCCESSFUL`인지 확인합니다.
3. 새 `/joint_states` 메시지에서 모든 관절의 목표 오차가 프로필의 허용 값보다 작은지 확인합니다.
   현재 세 프로필은 0.01 rad 미만인 메시지를 10개 연속 수신해야 완료됩니다.
4. MoveIt 계획·실행에서도 성공 코드와 마지막 궤적점에 대한 같은 상태 확인을 수행합니다.

상태 확인에는 10초 제한이 있습니다. 이 검사는 관절 위치 오차의 연속 충족을 확인하며,
별도의 접촉 완료나 공구 작업 완료 판정은 포함하지 않습니다.
`verification` 설정은 [모델별 프로필](../config/robots)에 있습니다.

## 모델별 실행

ROS 검증은 공통 환경을 적용한 시스템 Python으로 실행합니다.
각 명령은 검증 후 자신의 launch를 종료하므로 순서대로 실행할 수 있습니다.

```bash
source ./manipulation_env.sh
python3 verify_robot.py --robot fr5 --domain 92
python3 verify_robot.py --robot fr10 --domain 90
python3 verify_robot.py --robot ur5e --domain 91
```

기본 대화형 시뮬레이션은 도메인 89이고 검증기의 기본값은 90입니다.
위 명령은 모델마다 별도 도메인을 명시합니다. 같은 도메인에 다른 시뮬레이션을 실행하지 않습니다.

GUI 검증과 MuJoCo 창 닫기 경로 검증은 데스크톱 터미널에서 실행합니다.
`--robot`과 `--domain`을 바꾸면 다른 모델에도 같은 검사를 적용할 수 있습니다.

```bash
python3 verify_robot.py --robot fr5 --domain 92 --gui
python3 verify_robot.py --robot fr5 --domain 92 --gui --stop-mode window
```

기본 종료는 launch 부모에 SIGINT를 보내며, 창 닫기 검증은 Trigger 서비스로
MuJoCo의 `exitrequest`를 설정합니다. GUI 닫기 버튼과 같은 `RenderLoop` 경로를 실행한 검사이며,
실제 마우스 클릭 검사로 기록하지 않습니다. RViz 닫기 버튼은 별도로 검증하지 않았습니다.
검증에 사용한 창은 종료되므로 계속 조작하려면 `bash run_sim.sh robot:=fr5`로 실행합니다.

## 검사 항목과 결과 파일

| 검사 | 통과 조건 |
|---|---|
| 상태와 제어기 | 모든 설정 관절의 유한한 상태 값, 상태 제어기·궤적 제어기 `active` |
| 직접 궤적 | Action 성공 결과와 관절 목표 오차의 연속 충족 |
| MoveIt | OMPL 계획 성공, 비어 있지 않은 궤적, 실행 성공과 관절 상태 확인 |
| TF와 시계 | 프로필의 예상 프레임 수신, 시뮬레이션 시간 증가·역행 없음 |
| GUI | RViz 계획 패널 준비, MuJoCo·RViz GPU 프로세스 관측 기록 |
| 종료 | 비정상 종료·시간 초과 없음, launch 코드 0, 프로세스 그룹 소멸 |

GUI 모드의 `nvidia-smi` 결과는 관측 정보로 저장하며 GPU 프로세스 관측 여부 자체가
검증 통과를 결정하는 조건은 아닙니다. GPU는 렌더링 확인에 사용합니다.

로컬 결과는 `runtime/validation/<모델>_<headless|gui|gui_window>.json`과
같은 이름의 `.log`에 저장됩니다. GUI GPU 기록은 `<모델>_gui_nvidia_smi.txt`입니다.

| JSON 필드 | 의미 |
|---|---|
| `motion_checks_passed` | 궤적·MoveIt·상태·TF·시계 검사 통과 |
| `direct_tracking_error_rad` / `moveit_tracking_error_rad` | 상태 확인을 마친 시점의 관절별 오차 |
| `shutdown_clean` | 종료 검사 통과 |
| `launch_return_code` | launch 프로세스의 종료 코드, 정상값 0 |
| `process_group_gone` | 검증이 실행한 프로세스 그룹의 잔류 없음 |
| `shutdown_failures` / `shutdown_timed_out` | 종료 중 비정상 노드와 시간 초과 |
| `robot_profile_sha256` | 검증에 사용한 프로필의 해시 |

동작 검사 완료 후 종료 검사에서 오류가 나면 동작 결과를 보존하고 검증기 종료 코드 1을 반환합니다.
종료 오류의 원인과 로컬 수정은 [문제 해결](troubleshooting.md)에 정리했습니다.

## 보관한 실행 기록

| 모델 | 검증 날짜 (KST) | headless / GUI / 창 닫기 경로 | 최대 관절 오차 | 요약 |
|---|---|---|---:|---|
| FR5 | 2026-10-06 | 모두 통과·정상 종료 | 약 0.0036 rad | [FR5 기록](fr5_shutdown_validation.json) |
| FR10 | 2026-10-06 | 모두 통과·정상 종료 | 약 0.0014 rad | [FR10 기록](fr10_validation.json) |
| UR5e | 2026-10-07 | 모두 통과·정상 종료 | 약 0.0041 rad | [UR5e 기록](ur5e_validation.json) |

FR5의 보관 기록에서는 MoveIt 계획이 headless·GUI 각각 49개 궤적점으로 생성되었고,
최대 오차는 약 0.21도였습니다. GTX 1050 Ti의 MuJoCo·RViz 그래픽 프로세스와
RViz OpenGL 4.6, MotionPlanning 연결도 확인했습니다.

모델마다 목표·게인·형상이 다르므로 위 수치는 모델 간 성능 순위가 아닙니다.
2026-10-07 GUI 촬영 실행의 원본 결과는 각 모델의
`docs/robots/<모델>/gui_capture_validation.json`에 별도로 보관합니다.
[촬영 안내](robots/capture.md)에 이미지와 검증 기록을 함께 갱신하는 방법이 있습니다.

## 구조와 변환 검사

```bash
.venv/bin/python -m unittest discover -s tests -v
```

11개 검사는 다음을 포함하며, 2026-10-07 실행에서 11개 통과·건너뜀 0개를 확인했습니다.

- 프로필 선택, 관절 수·이름 변경, 잘못된 배열과 미준비·오래된 모델의 실행 차단.
- 관절 순서를 변경해도 유지되는 MuJoCo 이름 매핑과 세 모델의 끝단 순기구학 일치.
- DAE 장면 변환·단위·색상·법선, 세 자세의 UR5e 메시 배치와 물리 모델 보존.

변환 검사에는 제조사 소스와 생성 모델이 필요합니다. 해당 자료가 없으면 관련 검사를
건너뛰므로 결과의 건너뜀 개수도 확인합니다.
코드는 [프로필 검사](../tests/test_robot_profiles.py)와 [시각 모델 검사](../tests/test_collada_visuals.py)에 있습니다.

기존 `verify_fr5.py`는 FR5 호환 진입점입니다. 신규 모델은 공통 검증기를 사용합니다.
[초기 1관절 브리지 검증](single_joint_demo.md)은 별도 데모와 ROS 도메인을 사용합니다.
