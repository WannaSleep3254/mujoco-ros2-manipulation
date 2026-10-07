# 설치와 빌드

[프로젝트 README](../README.md) · [모델별 자료](robots/README.md) · [검증 안내](verification.md)

프로젝트의 빌드·실행 스크립트는 Ubuntu 22.04 / ROS 2 Humble 기준입니다.
모든 명령은 저장소 루트에서 실행합니다. 현재 PC 경로는
`~/projects/isaac_sim_setup`이며, 다른 경로에 복제해도 실행기에서 저장소 위치를 찾습니다.

## 새 PC 준비

ROS 2 Humble, MoveIt 2, ros2_control, ros2_controllers, Cyclone DDS, Xacro,
colcon, C++ 빌드 도구와 `python3-venv`, `python3-pip`를 먼저 준비합니다.
GUI 실행에는 사용할 수 있는 데스크톱 OpenGL 환경이 필요합니다.
현재 검증 조합은 MoveIt 2.5.10, Python 3.10입니다.

```bash
git clone https://github.com/WannaSleep3254/mujoco-ros2-manipulation.git
cd mujoco-ros2-manipulation
```

### Python 환경

ROS 2 Python 바인딩을 사용할 수 있도록 시스템 패키지를 상속하는 가상환경을 만듭니다.

```bash
python3 -m venv --system-site-packages .venv
.venv/bin/python -m pip install -r requirements.txt
```

[requirements.txt](../requirements.txt)는 Python MuJoCo 3.14.0, NumPy 1.26.4,
UR5e DAE 변환용 pycollada 0.9.3과 YAML·PNG 처리를 위한 의존성을 지정합니다.
PyYAML은 ROS 환경의 5.4.1에서 검증했습니다.

현재 PC의 초기 설치에서는 `ensurepip` 지원이 없어 다음 방법으로 구성했습니다.
이 방법은 시스템 pip가 준비된 Python 3.10 환경의 대안입니다.

```bash
python3 -m venv --without-pip --system-site-packages .venv
python3 -m pip install --target .venv/lib/python3.10/site-packages -r requirements.txt
```

모델 변환과 기준 렌더는 `.venv/bin/python`, 로봇팔의 ROS 자동 검증은
`source ./manipulation_env.sh`를 적용한 시스템 `python3`로 실행합니다.

### 로컬 ROS 런타임과 모델 빌드

검증한 deb 버전을 프로젝트 안에 내려받고, 종료 수정 런타임과 모델별 패키지를 빌드합니다.

```bash
mkdir -p external runtime/debs
cd runtime/debs
apt download libglfw3 libglfw3-dev \
  ros-humble-mujoco-vendor=0.1.1-1jammy.20260907.141308 \
  ros-humble-mujoco-ros2-control=0.1.2-1jammy.20260908.125234 \
  ros-humble-mujoco-ros2-control-msgs=0.1.2-1jammy.20260907.213024 \
  ros-humble-mujoco-ros2-control-plugins=0.1.2-1jammy.20260907.225305 \
  ros-humble-ros2-control-cmake=0.2.1-1jammy.20260304.202614
cd ../..
bash build_runtime.sh
bash build_robot_packages.sh
```

`build_runtime.sh`는 deb를 `runtime/root`에 추출하고 `mujoco_ros2_control` 0.1.2에
로컬 패치를 적용해 `ros_ws/install`에 빌드합니다. `/opt/ros`와 NVIDIA 드라이버는 변경하지 않습니다.
고정 deb 버전을 apt에서 구할 수 없으면 대체 버전 조합은 별도 검증이 필요합니다.

`build_robot_packages.sh`는 고정 커밋의 제조사 소스를 가져와 다음 패키지만 빌드하고
FR5·FR10·UR5e 모델을 생성합니다.

- `fairino_description`, `fairino5_v6_moveit2_config`, `fairino10_v6_moveit2_config`
- `ur_description` 2.14.0과 프로젝트의 `ur5e_moveit_config`

기존 소스의 커밋이 다르면 덮어쓰지 않고 중단합니다. 런타임 패치를 적용할 수 없는
변경이 있어도 중단합니다. 종료 수정의 근거와 적용 범위는 [문제 해결](troubleshooting.md)에 있습니다.

빌드가 끝나면 [README의 빠른 실행](../README.md#빠른-실행)으로 모델을 실행합니다.

## 모델 생성과 재빌드

전체 모델 패키지를 다시 빌드할 때는 `bash build_robot_packages.sh`,
공통 런타임을 다시 빌드할 때는 `bash build_runtime.sh`를 사용합니다.
이미 빌드된 환경에서 프로필만 수정했다면 선택한 모델만 생성합니다.

```bash
.venv/bin/python prepare_robot.py --list
.venv/bin/python prepare_robot.py --robot ur5e
```

`home_rad`, `servo.kp`, `servo.kv`와 검증 목표 배열은 프로필의 `joints` 순서와 길이를 따릅니다.
변환기는 관절 이름으로 MuJoCo 초기 자세와 구동기를 연결합니다.
프로필 수정 후 모델을 재생성해야 하며, 실행 중인 모델은 종료하고 다시 실행해야 합니다.

| 생성 위치 | 내용 |
|---|---|
| `models/<모델>/<모델>.xml` | MuJoCo 형상·관성·충돌·위치 서보 |
| `models/<모델>/<모델>.ros2_control.urdf` | ROS 모델과 MuJoCo 하드웨어 인터페이스 |
| `runtime/generated/<모델>/controllers.yaml` | 모델의 관절·제어기 이름을 반영한 ros2_control 설정 |
| `runtime/generated/<모델>/moveit_controllers.yaml` | 같은 프로필에서 생성한 MoveIt 제어기 연결 |
| `runtime/generated/ur5e/visual_meshes/` | 원본 DAE에서 변환한 재질별 OBJ |
| `models/<모델>/source.json` | 제조사 출처와 모델 생성 정보 |

생성 XML/URDF에는 해당 PC의 경로가 포함됩니다.
[.gitignore](../.gitignore)는 `.venv`, `external`, `runtime`, `ros_ws`, 생성 XML/URDF와
로컬 실행 로그를 제외합니다. 소스·프로필·패치·검증 요약·스크린샷·출처 기록은 Git에 포함합니다.
제조사 메시와 다운로드한 deb는 빌드 과정에서 별도로 준비합니다.

## 공통 코드 구성

| 파일 | 역할 |
|---|---|
| [robot_config.py](../robot_config.py) / [config/robots](../config/robots) | 프로필 검사, 생성 모델 해시 확인과 제어기 설정 생성 |
| [prepare_robot.py](../prepare_robot.py) / [collada_visuals.py](../collada_visuals.py) | URDF/MJCF 변환과 UR 시각 모델 변환 |
| [run_sim.sh](../run_sim.sh) / [manipulation_env.sh](../manipulation_env.sh) | 모델 선택과 공통 ROS 환경 |
| [manipulation.launch.py](../manipulation.launch.py) | MuJoCo·ros2_control·MoveIt·RViz 및 종료 처리 |
| [controller_defaults.yaml](../config/controller_defaults.yaml) | 공통 위치 명령·궤적 제어 설정 |
| [verify_robot.py](../verify_robot.py) / [tests](../tests) | 실제 ROS 동작·종료와 모델 변환 검사 |
| [capture_robot_images.py](../capture_robot_images.py) / [docs/robots](robots/README.md) | 기준 렌더·GUI 촬영과 모델별 자료 |
| [build_robot_packages.sh](../build_robot_packages.sh) / [build_runtime.sh](../build_runtime.sh) | 제조사 패키지와 패치 런타임 빌드 |
| [ur5e_moveit_config](../packages/ur5e_moveit_config) | 공식 Xacro, `tool0` 계획 체인, KDL·RViz 설정 |
| [종료 패치](../patches/mujoco_ros2_control-0.1.2-shutdown.patch) / [플러그인 수명 처리](../native/moveit_plugin_lifetime.c) | UI 자원 해제 순서와 MoveIt 종료 우회 처리 |

`manipulation_env.sh`는 기존 다른 워크스페이스의 환경 대신 `/opt/ros/humble`과
프로젝트의 로컬 런타임·빌드 패키지를 로드합니다. 셸 초기 설정 파일은 수정하지 않습니다.
공통 도메인은 `MANIPULATION_ROS_DOMAIN_ID`로 지정하며 기본값은 89입니다.

FR5의 기존 `prepare_fr5.py`, `run_fr5.sh`, `fr5.launch.py`, `fr5_env.sh`,
`verify_fr5.py`, `build_fr5_runtime.sh`는 호환 진입점으로 유지합니다.
`fr5_env.sh`는 기존 `FR5_ROS_DOMAIN_ID`도 지원합니다.

## 소스 버전과 출처

| 구성 요소 | 고정한 소스 |
|---|---|
| FAIRINO FR5·FR10 | [frcobot_ros2](https://github.com/FAIR-INNOVATION/frcobot_ros2/tree/fcf0c7f0d60d949d8a9a4238f929a44d07f60379), `fcf0c7f0d60d949d8a9a4238f929a44d07f60379` |
| Universal Robots Description | [Humble 소스](https://github.com/UniversalRobots/Universal_Robots_ROS2_Description/tree/65fa221f6d9e1904b30b6a05ae39cb24a0c40fac), `65fa221f6d9e1904b30b6a05ae39cb24a0c40fac` |
| mujoco_ros2_control | [태그 0.1.2](https://github.com/ros-controls/mujoco_ros2_control/tree/178b1c39e5010185116bc89b4e94cbb25b825f61), `178b1c39e5010185116bc89b4e94cbb25b825f61` |

제조사 소스는 `external/`에 두고 원본 파일을 수정하지 않습니다.
UR Description은 BSD-3-Clause입니다. 프로젝트의 UR5e MoveIt 설정 출처와
Apache-2.0 라이선스는 [NOTICE](../packages/ur5e_moveit_config/NOTICE)와
[LICENSE](../packages/ur5e_moveit_config/LICENSE)에 기록했습니다.
종료 패치의 상류 소스 라이선스는 [Apache-2.0](../patches/Apache-2.0.txt)입니다.
연동 구조는 [ros2_control 공식 문서](https://control.ros.org/humble/doc/mujoco_ros2_control/doc/index.html)를
기준으로 구성했습니다. 실제 로봇용 드라이버는 이 시뮬레이션 launch에 로딩하지 않습니다.
