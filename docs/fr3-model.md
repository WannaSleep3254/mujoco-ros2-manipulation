# FAIRINO FR3 V6 모델 추가

FAIRINO FR3 V6의 6축 모델을 기존 로봇 프로필 구조에 추가했습니다. 프로필 상태는
`experimental`입니다. 제조사 URDF·STL·SRDF를 변환한 모델 검사와 ROS 실행 검사를
구분하며, 모델 생성만으로 실행·취소·뷰어 연동을 검증했다고 판단하지 않습니다.

제조사 소스는 `config/build.yaml`의 FAIRINO 커밋
`fcf0c7f0d60d949d8a9a4238f929a44d07f60379`로 고정합니다.
`fairino3_v6.urdf`의 SHA-256은
`ba9266ba6d32391b8c68020bea2533f6070dc3be654607abe7ed1b7f25e13021`입니다.
`fairino3_v6`의 STL 7개와 제조사 SRDF의 충돌 제외 9쌍을 사용합니다.
SRDF의 `Never` 제외도 제조사 원본에서 이어받으며 새 제외 쌍은 추가하지 않습니다.

| 항목 | 설정 |
|---|---|
| 로봇·계획 그룹 | `fairino3_v6_robot` · `fairino3_v6_group` |
| 궤적 제어기 | `fairino3_controller` |
| 관절·끝단 프레임 | `j1`~`j6` · `wrist3_link` |
| MoveIt 패키지 | `fairino3_v6_moveit2_config` |
| 초기 위치 이득 | `[2000, 4000, 3000, 800, 600, 400]` |
| 초기 속도 이득 | `[80, 120, 100, 20, 15, 10]` |
| 물리 설정 | 2 ms, `implicitfast`, 시뮬레이션 배속 1 |

이득은 FR5 프로필에서 차용한 초기 시뮬레이션 설정입니다. 제조사의 실제 제어기
이득이나 FR3 전체 작업 범위에서 검증된 이득을 뜻하지 않습니다. 관절 한계와
액추에이터 힘 제한은 FR3 제조사 URDF에서 가져옵니다. 그리퍼는 포함하지 않습니다.

2026-10-28 격리 모델 검사에서는 MuJoCo Python 3.14.0으로 생성 후 2,000 step을
적분했습니다. 정착 후 마지막 상태의 접촉 수는 0이고, 홈 자세의 최대 관절 오차는
약 0.002703 rad입니다. 홈·direct 목표·MoveIt 목표의 끝단 FK를 독립 URDF 계산과
대조했습니다. 이 결과는 ROS native 3.12.0의 실행·취소·정지 결과와 구분합니다.
원본은 로컬 `runtime/w09-fr3/model-a/`와 `model-a-summary.json`에 보존합니다.

모델 생성과 변환 검사는 ROS를 실행하지 않고 수행할 수 있습니다.

```bash
PYTHONPATH=src .venv/bin/python -m unittest \
  tests.unit.test_fr3_profile tests.model.test_fr3_profile -v
```

새 ROS overlay가 필요하면 기존 환경을 source한 뒤 아래와 같이 패키지와 출력
경로를 한정할 수 있습니다. 이 명령은 ROS 노드를 실행하거나 native runtime을
빌드하지 않습니다. 환경을 source한 뒤 `PYTHONPATH`를 덮어쓰지 않습니다.

```bash
source ./manipulation_env.sh
colcon --log-base runtime/w09-fr3/ros-log build \
  --base-paths external/frcobot_ros2/fairino_description \
    external/frcobot_ros2/fairino3_v6_moveit2_config \
  --build-base runtime/w09-fr3/ros-build \
  --install-base runtime/w09-fr3/ros-install \
  --packages-select fairino_description fairino3_v6_moveit2_config \
  --cmake-args -DBUILD_TESTING=OFF
```

위 overlay를 실제 launch에 연결하는 절차와 실행·취소·뷰어 검사는 별도로 필요합니다.
격리 생성본의 ros2_control URDF는 해당 출력 폴더의 모델 경로를 가리키므로 단순
복사로 운영 모델로 옮기지 않습니다. 운영 출력이 필요하면 그 위치에서 모델을
다시 생성합니다. 또한 공통 `config/build.yaml` 변경은 기존 모델의 생성 입력
SHA에도 영향을 주므로 기존 모델을 다시 사용할 때 무결성 검사와 재생성을 거칩니다.

## 실제 ROS 실행·취소와 복귀

구현은 [고정 커밋 330b44a](https://github.com/WannaSleep3254/mujoco-ros2-manipulation/commit/330b44a)에 연결합니다.
최종 로컬 실행 `20261028T055751.995183Z-f651c336`은 direct 궤적과 MoveIt 계획·실행,
실제 이동을 관측한 뒤 자체 목표 취소, Action CANCELED, 신선한 정지 관측과 새 홈 목표의
성공·정착, TF·시계와 소유 프로세스 정리를 통과했습니다. native MuJoCo 3.12.0의
5,040스텝에서 접촉·비정상 값·시계 불연속은 0이며 접촉 인터록이 활성화된 상태였습니다.

취소 뒤 41개 관측이 약 0.400초 동안 전진했습니다. 이 구간의 최대 위치 변화는
약 0.000180 rad, 최대 속도는 약 0.01104 rad/s입니다. 선언한 관측 조건은
0.4초 이상·5표본 이상, 위치 변화 0.002 rad 이하·속도 0.02 rad/s 이하입니다.
정확한 영속도나 실물 안전 정지를 판정한 수치가 아닙니다. 별도 홈 Action 성공 이후에도
새 source/clock 관측과 목표 정착을 요구하며, 관측 창의 최대 홈 오차는 약 0.002707 rad입니다.

첫 실행은 ROS UUID의 uint8 JSON 저장 실패로 중단했습니다. 두 번째 실행에서는
취소·복귀가 완료됐지만 전용 단일 로봇 launch의 인터록이 활성화되지 않아 엄격한 native
최종 검사가 실패했습니다. UUID를 정확한 16바이트 Python 정수로 정규화하고 전용 검증 환경에
인터록을 명시한 뒤 세 번째 새 실행을 확인했습니다. 공통 최종 검사와 기본 launch 정책을
완화하지 않았으며 실패 a·b의 원본과 정상 정리 결과도 보존했습니다.

일반 사용 환경에서는 다음과 같이 필요한 모델·패키지를 생성한 뒤 별도 도메인으로 검사합니다.
이 명령의 models 구성 요소는 native runtime을 다시 빌드하지 않습니다.

```bash
python3 build_workspace.py --component models --robots fr3
source ./manipulation_env.sh
python3 verify_robot_cancel.py --robot fr3 --domain 162 \
  --output runtime/validation/fr3-cancel-new
```

CI는 기존 공동 작업 환경 작업에서 FR3를 추가로 빌드하고 같은 명령을 실행합니다.
실험 프로필과 초기 이득은 유지합니다. 큰 동작 범위·다른 설치 환경·FR3의 접촉 실패 주입,
실물 로봇과 그리퍼 통합은 검증하지 않았습니다.

## 뷰어 표시와 실행 대상

기존 콘솔에서 FR3는 읽기 전용 모델 미리보기입니다. FR3를 선택해도 실제 실행 대상은
FR5·UR5e 공동 환경이며, 미리보기 중 명령·계획·Pose·시작·연결·시연 API는 차단됩니다.
FR3 단일 실행 검증과 기존 양팔 콘솔의 실행 대상을 혼동하지 않습니다.
실제 HTTP/ROS 실행 `20261028T054434.622822Z-3d4e9017`은 4개 표시 모델과 차단·복귀,
native 검사·정리를 통과했습니다. 브라우저에서는 FR3 형상 선택·맞춤·확대·충돌 표시와
기존 양팔 보기 복귀를 확인했습니다. 390 px 내비게이션 넘침도 줄바꿈으로 수정해
본문 폭 375 px, 320 px에서는 305 px임을 확인했습니다.
