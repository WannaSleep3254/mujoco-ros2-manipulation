# 패키지·빌드·작업 환경 구조

[README](../README.md) · [설치](setup.md) · [검증](verification.md) · [공통 모듈 경계](module_boundaries.md)

2026-10-19 개편은 단일 로봇 실행을 유지하면서 코드·빌드 정의·작업 환경을 분리합니다.
기존 루트 명령은 호환 진입점입니다. 구현을 바꿀 때는 `src/`의 정식 모듈을 수정합니다.

```text
pyproject.toml                   Python 패키지·CLI·빌드 도구
requirements.txt                 실행 의존성의 단일 명세
src/mujoco_ros2_manipulation/
  paths.py                       명시적인 작업공간 경로
  profiles.py                    로봇 프로필·제어기 설정
  scenes.py                      작업 환경 검증·고정 URDF 형상·설치 자세
  build.py                       소스·deb 검증·빌드 순서
  model_build/                    변환·DAE·생성물 무결성
  verification/                   ROS 검사·수명주기·보고서·이미지
  workcells/                      두 로봇 합성·제어 설정·공동 검증
  examples/single_joint/          직접 토픽 브리지 예제
config/
  build.yaml                     제조사/런타임 커밋·패키지 경로·deb 잠금
  robots/                        로봇의 관절·게인·ROS 이름·소스 참조
  scenes/                        단일 로봇 설치 자세·고정 물체
  workcells/                     여러 로봇 인스턴스·공통 물리 조건
launch/manipulation.launch.py     ROS launch 어댑터
scripts/check_repository.py       구문·문서 링크·설정 검사
packages/ur5e_moveit_config/       ament/MoveIt 패키지
native/ + patches/               네이티브 종료 처리·스텝 계측·접촉/시계 정지
examples/single_joint/            예제 안내
tests/unit/                     ROS 없이 실행하는 검사
tests/model/                    제조사 모델이 필요한 회귀 검사
tests/integration/              실제 ROS 검사 절차
.github/workflows/ci.yml          패키지 검사 + ROS 5개 조합
```

## 패키지와 작업공간

wheel은 Python 코드와 CLI를 배포합니다. 제조사 메시·ROS 빌드·작업 환경 설정을 포함하는 독립 실행 번들은 아닙니다.
CLI는 저장소를 작업공간으로 사용합니다. 저장소 밖에서는 `MUJOCO_ROS2_WORKSPACE`를 지정합니다.
`manipulation_env.sh`는 이 경로와 checkout의 `src` 경로를 설정하므로 ROS 시스템 Python에서도
editable 설치 없이 같은 코드를 사용합니다. 기존 `prepare_robot.py`, `verify_robot.py`,
`run_sim.sh`, FR5 명령과 단일 관절 명령은 계속 동작합니다.

패키지 설치만으로 ROS Python 바인딩·MoveIt·네이티브 런타임이 설치되지는 않습니다.
`manipulation_env.sh`는 로컬 검증과 같은 Cyclone DDS를 기본으로 선택합니다.
별도 검증 환경은 `MANIPULATION_RMW_IMPLEMENTATION`으로 변경할 수 있으며 검증 보고서에는
실제 `rmw_implementation`을 기록합니다. 호스트의 암묵적인 DDS 선택을 상속하지 않습니다.

`requirements.txt`의 MuJoCo·NumPy 버전과 Dependabot 정책은 유지합니다.

## 빌드 정의의 단일 출처

`config/build.yaml`에서 제조사 및 런타임 저장소 URL·커밋, ROS 패키지 경로,
종료 패치 경로, 검증한 amd64 deb의 패키지·버전·SHA-256을 관리합니다.
로봇 프로필의 `source.repository`가 이 저장소 정의를 참조합니다.
선택한 로봇의 MoveIt 패키지와 메시 패키지로 colcon 대상을 도출하므로 셸의 모델 목록을 수정할 필요가 없습니다.
새 로봇의 별도 ROS 패키지는 빌드 명세의 `packages`에 등록합니다.

빌드는 `build_workspace.py` 또는 설치한 `manipulation-build`로 실행합니다.
`--dry-run`은 파일이나 네트워크를 변경하지 않고 선택을 확인합니다.
`--fetch-only`는 잠긴 소스만 준비합니다. 기존 체크아웃의 커밋이 다르면 중단합니다.
제조사 추적 파일 변경도 모델 빌드 전에 거부합니다.
소스에서 재빌드하는 `mujoco_ros2_control` 본체의 바이너리 deb는 사용하지 않습니다.
캐시의 추가 deb는 무시하고, 잠긴 전체 deb를 검증한 후 추출합니다.
`--download-debs`는 빠진 파일만 정확한 버전으로 내려받아 해시를 검증합니다.
배포 서버에서 그 버전이 사라지면 준비 단계가 실패합니다. 자동으로 다른 버전으로 대체하지 않습니다.

이 명세는 전체 Ubuntu apt 환경의 완전한 lockfile은 아닙니다. 시스템 ROS/MoveIt과 C++ 도구의
패치 버전까지 모두 고정하지 않으며 CI의 시스템 의존성도 배포 저장소에서 설치합니다.
생성 기록의 Python 환경과 실제 CI 결과로 호환성을 확인합니다.

## 모델과 작업 환경

`config/robots`는 관절·기구학 소스·서보·기본 자세·ROS 연결을 관리합니다.
`config/scenes`는 `robot_pose`의 xyz[m]/rpy[rad]와 고정 물체의 형상·크기·위치를 관리합니다.
물체의 xyz/rpy는 world 기준이며 box 크기는 전체 변 길이[m], cylinder 크기는 반지름·전체 높이[m]입니다.

- `default`: 기존 설치 원점, 바닥의 윗면 z=-0.005m.
- `pedestal`: 0.3×0.3×0.4m 받침대, 로봇 설치 z=0.4m.

같은 작업 환경을 FR5·FR10·UR5e와 조합할 수 있습니다. box/cylinder 고정 형상을 지원합니다.
이 단일 로봇 작업 환경 형식은 그대로 유지합니다. 두 로봇은 별도 [공동 작업 환경](workcells.md) 형식을 사용합니다.
mesh/CAD·동적 물체·레일·그리퍼는 아직 지원하지 않습니다.

작업 환경 구성 도구는 제조사 URDF 내부의 변환을 보존하면서 world 고정 관절을 추가합니다.
원본에 world 루트가 있으면 `workcell_source_root`로 이름을 바꾸고 새 world 아래 설치 자세를 적용합니다.
고정 물체에는 `workcell_` 접두사를 사용하고 이름 충돌을 거부합니다.
합성 URDF의 동일한 collision 형상을 MuJoCo 변환과 MoveIt에 사용합니다.
따라서 별도 비동기 Planning Scene 발행자 없이 바닥·받침대를 로봇의 고정 충돌 링크로 공유합니다.
작업 환경의 고정 물체와의 충돌은 MoveIt에서 self-collision 계열로 보고될 수 있습니다.
실행 중 환경을 이동시키려면 재생성 대신 별도의 동적 환경 설계가 필요합니다.

기본 출력 경로는 유지합니다. 추가 작업 환경은 아래에 분리됩니다.

| 내용 | 기본 작업 환경 | 추가 작업 환경 |
|---|---|---|
| XML·URDF·OBJ·source.json | `models/<robot>/` | `models/scenes/<scene>/<robot>/` |
| 제어기 YAML | `runtime/generated/default/<robot>/` | `runtime/generated/<scene>/<robot>/` |
| 최근 검증 | `<robot>_<mode>.json` | `<robot>_<scene>_<mode>.json` |

검증 파일은 모두 `runtime/validation` 아래에 있고 실행별 이력은 `runs/`에 보관합니다.
프로필 해시와 환경 해시를 별도로 기록하며, 빌드 명세·작업 환경 YAML·패키지 생성 코드까지 입력 해시로 검사합니다.
프로필·환경·생성기가 바뀌면 해당 조합을 재생성해야 합니다. 실행 중에는 재생성하지 않습니다.

```bash
.venv/bin/python prepare_robot.py --robot fr5 --scene pedestal
bash run_sim.sh robot:=fr5 scene:=pedestal
# 별도 터미널 / GUI를 닫은 뒤 독립 headless 검증
source ./manipulation_env.sh
python3 verify_robot.py --robot fr5 --scene pedestal --domain 97
```

## CI와 검증 경계

PR과 main push에서 다음 작업을 실행합니다. 권한은 contents:read이며 자동 병합은 설정하지 않습니다.

| 작업 | 검사 |
|---|---|
| Python | wheel·sdist 빌드, 설치한 CLI를 저장소 밖에서 호출, 단위 검사, 구문·문서 링크·설정 |
| ROS Humble | FR5·FR10·UR5e default 및 FR5 pedestal 및 FR5·UR5e 공동 작업 환경, 잠긴 런타임 빌드, 선택 모델 빌드, 실제 궤적·MoveIt·TF·시계·종료 |
| 산출물 | 배포 패키지, 성공/실패의 ROS 보고서와 colcon 로그 보존 |

ROS 작업은 공식 `ros:humble-ros-base-jammy` 컨테이너에서 실행합니다.
UR5e는 CI에서 3회 연속 실행하며 한 번이라도 실패하면 해당 작업이 실패합니다.
GUI/GPU 성능·실물 로봇·작업 환경별 모든 충돌 상황은 자동 검증 범위가 아닙니다.
로컬 통과와 GitHub 실행 결과는 구분해 보고합니다. 기존 GUI 자료는 이전 버전의 보관 기록입니다.

후속 구현으로 로봇 인스턴스별 이름 접두사, 합성 SRDF·팔별 제어기와 공통 물리 시뮬레이션을 추가했습니다.
[공동 작업 환경](workcells.md)과 [확장 계획](plans/base_and_multi_robot.md)에 검증한 범위와 남은 항목을 구분했습니다.

공동 작업 환경의 접촉 인터록은 네이티브 물리 고정과 별도 ROS 감시 노드의 양팔 제어기 비활성화를 결합합니다. [동작·복구·검증](contact_stop.md)을 참고합니다.
