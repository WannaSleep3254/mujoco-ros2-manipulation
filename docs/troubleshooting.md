# 문제 해결과 종료 오류 분석

[프로젝트 README](../README.md) · [설치와 빌드](setup.md) · [검증 안내](verification.md)

## 종료 시 SIGSEGV

초기 `exit code -11`은 종료 중 잘못된 메모리 접근으로 발생한 SIGSEGV입니다.
궤적 동작 검사는 통과한 뒤 종료 단계에서 발생했습니다.
현재 PC의 로그·스택·소스와 수정 전후 실행을 비교해 다음과 같이 처리했습니다.

| 구분 | 관측과 원인 판단 | 적용한 해결 |
|---|---|---|
| MuJoCo GUI | OpenGL/GLFW 자원을 만든 렌더링 스레드가 끝난 뒤 다른 스레드에서 UI를 해제했습니다. `libGLdispatch` 충돌과 소스의 해제 순서를 확인했습니다. | `RenderLoop` 직후 같은 렌더링 스레드에서 `platform_ui.reset()`으로 UI 자원을 해제합니다. |
| MoveIt / RViz | `rclcpp::CallbackGroup`와 궤적 실행 관리자의 종료 스택에서 충돌했습니다. 남은 ROS 객체보다 플러그인 코드가 먼저 해제되는 수명 문제로 판단했고, 해제를 지연한 비교 실행에서 충돌이 사라졌습니다. | `libmoveit_*`에 `RTLD_NODELETE`를 적용해 프로세스 종료까지 코드를 유지합니다. `move_group`과 RViz에 적용한 임시 우회 처리입니다. |
| 전체 프로세스 정리 | 핵심 노드나 창 하나가 종료된 뒤 다른 시뮬레이션 프로세스가 남을 수 있었습니다. | 핵심 노드 종료를 launch가 감지해 전체를 종료합니다. 검증기는 launch 부모에 SIGINT를 한 번 보내 중복 종료 신호를 피합니다. |

MoveIt 문제를 최초로 유발한 개별 플러그인은 특정하지 못했습니다.
적용 근거와 검증 결과를 기록한 우회 처리이며, 모든 ROS/MoveIt 버전의 원인 확정을 뜻하지 않습니다.

### 수정 파일과 적용 범위

- [mujoco_ros2_control 0.1.2 패치](../patches/mujoco_ros2_control-0.1.2-shutdown.patch): UI 자원 해제 순서, 창 닫기 서비스, 로컬 빌드 설정.
- [moveit_plugin_lifetime.c](../native/moveit_plugin_lifetime.c): MoveIt 플러그인 코드 수명 우회 처리.
- [manipulation.launch.py](../manipulation.launch.py): `move_group`·RViz에만 `LD_PRELOAD` 적용, 핵심 노드 종료 시 전체 정리.
- [verify_robot.py](../verify_robot.py): launch 부모 신호 전달, 종료 코드·오류 로그·잔류 프로세스 검사.

`bash build_runtime.sh`로 프로젝트 안에 재빌드합니다. `/opt/ros`와 NVIDIA 드라이버는 변경하지 않습니다.
검증한 조합은 Ubuntu 22.04.5, ROS 2 Humble, MoveIt 2.5.10,
mujoco_ros2_control 0.1.2, ROS 런타임 MuJoCo 3.12.0입니다.

### 종료 검증 결과

FR5·FR10·UR5e에서 headless SIGINT, GUI SIGINT, MuJoCo 창 닫기 경로를 통과했습니다.
각 실행은 launch 종료 코드 0, 자식 프로세스 정상 종료, 잔류 프로세스 없음으로 확인했습니다.
RViz 종료 때 `context is invalid` 진단 메시지는 남을 수 있으나 보관한 최종 검증에서
SIGSEGV는 재현되지 않았습니다. 모델별 기록은 [검증 안내](verification.md#보관한-실행-기록)에 있습니다.

실행 중인 시뮬레이션에 창 닫기 경로를 요청하려면 같은 ROS 환경의 터미널에서 실행합니다.

```bash
source ./manipulation_env.sh
ros2 service call /mujoco_ros2_control_node/close_window std_srvs/srv/Trigger '{}'
```

이 서비스는 `exitrequest`를 설정해 닫기 버튼과 같은 `RenderLoop` 종료 경로를 실행합니다.
자동 검증도 이 서비스를 사용했으며 실제 마우스 클릭 검사로 기록하지 않습니다.
RViz 닫기 버튼의 전체 정리는 연결되어 있지만 버튼 동작은 별도로 검증하지 않았습니다.

## Python에서 MuJoCo 모듈을 찾지 못할 때

Python MuJoCo는 프로젝트의 `.venv`에 설치되어 있습니다.
모델 생성·뷰어·초기 1관절 브리지는 `.venv/bin/python`으로 실행합니다.

```bash
.venv/bin/python -c 'import mujoco; print(mujoco.__version__)'
```

로봇팔의 `verify_robot.py`는 ROS 환경을 적용한 시스템 `python3`로 실행합니다.
Python 데모의 MuJoCo 3.14.0과 ROS 런타임 MuJoCo 3.12.0은 별도 구성입니다.

## 프로필 수정 후 오래된 모델 오류

실행기와 검증기는 프로필 해시를 검사합니다. 프로필을 바꿨다면 해당 모델을 재생성합니다.

```bash
.venv/bin/python prepare_robot.py --robot ur5e
bash run_sim.sh robot:=ur5e
```

기존 launch를 먼저 종료해야 재생성한 모델로 다시 실행할 수 있습니다.
UR5e DAE 변환 의존성이 없으면 [설치 안내](setup.md#python-환경)에 따라 `requirements.txt`를 적용합니다.

## ROS 명령이 다른 환경으로 전달될 때

로봇팔 관련 터미널마다 `source ./manipulation_env.sh`를 적용합니다.
공통 실행 도메인은 89, 초기 1관절 데모의 `ros2_env.sh`는 87입니다.
검증기는 `--domain` 값을 사용하므로 실행 노드와 명령 터미널의 도메인이 같아야 합니다.
기존 다른 워크스페이스 환경을 정리하고 프로젝트 패키지를 로드하는 역할도 공통 환경 스크립트가 맡습니다.
초기 데모의 rqt 체크박스·시계 QoS 설명은 [1관절 데모 안내](single_joint_demo.md#rqt에서-모니터링)에 있습니다.
