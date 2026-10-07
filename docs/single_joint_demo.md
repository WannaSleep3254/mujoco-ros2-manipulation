# 초기 1관절 ROS 2 데모

[프로젝트 README](../README.md) · [설치와 빌드](setup.md) · [로봇팔 검증](verification.md)

MuJoCo 설치와 ROS 명령·상태 통신을 처음 확인할 때 사용한 단일 회전 관절 데모입니다.
`ROS 2 목표 각도 → Python 브리지 → MuJoCo 위치 서보` 구조이며,
ros2_control·MoveIt을 사용하는 FR5·FR10·UR5e와는 별도의 실행 경로입니다.

Ubuntu 22.04, Python 3.10, ROS 2 Humble 환경에서 `.venv`의 MuJoCo 3.14.0과
NumPy 1.26.4로 실행했습니다. 가상환경은 시스템 ROS Python 패키지를 상속합니다.
아래 명령은 저장소 루트에서 실행합니다.

## MuJoCo 기본 뷰어

```bash
.venv/bin/python -m mujoco.viewer --mjcf=models/single_joint.xml
```

이미 열린 뷰어에 XML을 끌어 놓아도 로딩할 수 있습니다.
`joint1_servo` Control 슬라이더로 목표 각도를 rad 단위로 지정합니다.
이 뷰어의 관절 조작은 로컬 GUI 제어이며, ROS 명령 연결은 아래 Python 브리지로 실행합니다.
모델은 수직축 주위로 회전하므로 중력 보상 검증에는 사용하지 않습니다.

## ROS 2로 GUI 모델 제어

터미널 1에서 GUI와 브리지를 실행합니다.

```bash
source ./ros2_env.sh
.venv/bin/python mujoco_ros2_bridge.py
```

터미널 2에서도 같은 환경을 적용한 뒤 목표를 보냅니다.

```bash
source ./ros2_env.sh
ros2 topic pub --once /mujoco/target_position std_msgs/msg/Float64 '{data: 0.8}'
```

0.8 rad은 약 45.8도입니다. 반대 방향으로 이동하고 상태를 확인하려면:

```bash
ros2 topic pub --once /mujoco/target_position std_msgs/msg/Float64 '{data: -0.5}'
ros2 topic echo /joint_states sensor_msgs/msg/JointState --once
ros2 topic echo /clock rosgraph_msgs/msg/Clock --once
```

| 토픽 | 메시지 | 역할 |
|---|---|---|
| `/mujoco/target_position` | `std_msgs/msg/Float64` | 목표 각도 입력, rad |
| `/joint_states` | `sensor_msgs/msg/JointState` | 각도·각속도·적용 구동 토크 |
| `/clock` | `rosgraph_msgs/msg/Clock` | MuJoCo 시뮬레이션 시간 |

물리 계산 간격은 0.002초, 상태·시간 발행은 50 Hz, GUI 동기화는 최대 약 60 Hz입니다.
상태 타임스탬프는 MuJoCo 시간입니다. 이 시간을 사용하는 소비 노드에는 `use_sim_time:=true`를 설정합니다.

`ros2_env.sh`는 도메인 87과 localhost 통신을 설정하므로 관련 터미널마다 적용합니다.
ROS 브리지 실행 중에는 목표 토픽이 제어 입력을 결정하며 Control 슬라이더 값은 다음 물리 스텝에서 덮어씁니다.
창을 닫거나 터미널 1에서 Ctrl+C를 누르면 종료합니다.

화면 없이 브리지를 실행하려면:

```bash
source ./ros2_env.sh
.venv/bin/python mujoco_ros2_bridge.py --headless
```

## rqt에서 모니터링

새 터미널에서 프로젝트로 이동하고 환경을 적용합니다.

```bash
source ./ros2_env.sh
rqt
```

Topic Monitor에서 `/joint_states`와 `/clock`을 선택합니다.
Message Publisher에서 `/mujoco/target_position`의 `data` 값을 입력하고
**퍼블리셔 체크박스를 켜야** 지정 주기로 명령이 전송됩니다.

`/clock`은 rqt의 reliable 구독과 기본 best-effort 시간 구독 모두를 지원하도록
reliable QoS로 발행합니다. 이전 브리지에서 RELIABILITY 불일치 경고가 있었다면
브리지를 종료하고 다시 실행해야 수정이 적용됩니다.
`lo is not multicast-capable: disabling multicast` 메시지는 localhost 설정에서 나타날 수 있습니다.
명령·상태 수신이 정상이라면 해당 메시지 자체는 통신 실패를 뜻하지 않습니다.

## 자동 검증

```bash
source ./ros2_env.sh
.venv/bin/python verify_mujoco_ros2.py
```

검증기는 실제 `mujoco_ros2_bridge.py`와 명령 클라이언트를 별도 프로세스로 실행합니다.
+0.5 rad과 -0.3 rad 목표를 차례로 전송하고, 목표 오차 0.01 rad 이내의
관절 상태 메시지를 10개 연속 수신해야 통과합니다. 도메인 87을 사용합니다.

GUI를 포함한 검증은 데스크톱 터미널에서 실행합니다.

```bash
.venv/bin/python verify_mujoco_ros2.py --gui
```

검증은 잠시 창을 열고 목표 각도를 전송한 뒤 종료합니다.
FR5·FR10·UR5e는 [공통 로봇팔 검증기](verification.md)를 사용하며,
이 데모의 `/mujoco/target_position` 토픽으로 제어하지 않습니다.

## 초기 검증 기록

아래 내용은 기존 README에 보관했던 설치·연동 결과입니다.
이번 문서 정리에서 다시 측정한 결과는 아닙니다.

| 항목 | 보관 결과 |
|---|---|
| MuJoCo import·모델 컴파일 | 통과 |
| 물리 10,000 스텝, 시뮬레이션 20초 | 통과, 벽시계 약 0.017초 |
| headless·GUI의 프로세스 간 ROS 명령·상태 통신 | 통과 |
| +0.5 rad 목표 | headless 약 +0.4976 rad / GUI 약 +0.4977 rad |
| -0.3 rad 목표 | headless 약 -0.2977 rad / GUI 약 -0.2980 rad |
| 시뮬레이션 시계 | 두 모드에서 타임스탬프 증가 확인 |
| EGL 오프스크린 렌더 | RGB 배열 크기 `(120, 160, 3)`, 통과 |

스텝 실행 시간은 매우 작은 1관절 모델의 결과이며 로봇팔 성능 벤치마크가 아닙니다.
EGL 렌더 성공만으로 사용한 GPU를 특정할 수는 없습니다.

2026-10-02 데스크톱 스크린샷에서 `single_joint_validation` 로딩과
목표·실제 각도 0.59 rad을 확인했습니다. `glxinfo -B`는 GTX 1050 Ti를 표시했고,
`nvidia-smi`에는 `.venv/bin/python` 그래픽 프로세스가 나타났습니다.
초기 에이전트 환경에서 관측한 드라이버·디스플레이 실패는 이 데스크톱 확인 결과와 구분합니다.

이후 rqt 화면에서는 50 Hz 관절 상태와 0.8 rad 명령에 대한 약 0.8 rad 위치를 확인했습니다.
동시에 발견된 시계 QoS 불일치를 reliable 발행으로 수정했습니다.
도메인 88의 호환성 검사에서 best-effort·reliable 구독자가 각각 증가하는 시계 메시지 73개를
수신했고 관절 추종도 통과했습니다. 이 도메인은 실행 중인 도메인 87 데모와 분리했습니다.

가상환경의 초기 생성 방식은 [설치 안내](setup.md#python-환경)에 보존했습니다.
