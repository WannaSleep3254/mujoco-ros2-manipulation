# 공통 표시 데이터와 ROS 없는 기록 뷰어

현재 Three.js 화면은 물리 계산이나 로봇 명령을 수행하지 않고, 공급원이 만든 형상과 world 변환을 표시합니다. MuJoCo 공급원과 ROS 없는 기록 공급원이 같은 `LocalSceneView`를 사용합니다. 실제 다른 시뮬레이터·센서·로봇을 제어하는 범용 플러그인을 제공하는 단계는 아닙니다.

## 공통 형식

`display-scene-v1`은 `schema_version: 2`인 manifest입니다. 기존 `compiled-scene-v1`도 브라우저에서 계속 읽을 수 있습니다. 현재 콘솔의 MuJoCo 작업자는 기존 형식을 만들고, `DisplayHostAdapter`가 공통 형식으로 변환합니다.

| 데이터 | 규칙 |
|---|---|
| 모델·장면 | `model_sha256`, 전체 manifest 내용에서 계산한 `scene_sha256`과 HTTP 본문의 SHA를 구분합니다. 자산은 내용 해시로 참조합니다. |
| 좌표·단위 | world, 오른손 좌표계, Z up, m·rad, world 위치와 row-major 3×3 회전 행렬을 사용합니다. 반사·비직교 행렬을 거부합니다. |
| 기본 형상 | box는 반 길이, cylinder·capsule은 반지름과 반 길이, sphere는 반지름입니다. 메시 정점은 이미 로컬 m로 변환되어 있으며 원점 보정을 다시 적용하지 않습니다. 무한 plane의 브라우저 표시 범위는 유한합니다. |
| 관절 | 중복 없는 이름과 정확히 대응하는 전체 상태를 사용합니다. v1 기록 예제는 회전 관절(rad)만 사용하며 직선 관절 확장에는 관절 종류·단위 필드가 필요합니다. |
| 출처 | 공급원 이름과 입력 SHA는 `provenance`에 기록합니다. 엔진 이름·버전·컴파일 SHA는 선택적인 `provenance.engine` 안에 있으며 엔진이 없으면 null입니다. |
| 수신 나이 | 수신 프로세스의 monotonic 시계로 계산한 age에 브라우저의 로컬 경과 시간을 더합니다. 원천 시각을 수신 시계에서 빼지 않습니다. |
| 세션·순번 | console/session/generation binding은 선택한 표시를 식별합니다. 기록 표본의 source session·clock domain·시각·순번은 별도로 전달합니다. 같은 binding에서 원천 세션 변경·시각 역행·순번 재사용을 거부합니다. |

형상 목록·자산 크기·전체 바이트·표본 수를 제한하고, 전체 검증이 끝나기 전에 현재 장면을 교체하지 않습니다. 잘못된 모델·오래된 binding의 응답은 적용하지 않습니다. 같은 표본을 반복 조회해도 수신 나이를 갱신하지 않습니다. 이 검증은 형식·표시 일치 검사이며, 기록의 물리적 정확성이나 경로의 충돌 회피를 보증하지 않습니다.

## ROS 없는 연결 실행

```bash
PYTHONPATH=src python3 -m mujoco_ros2_manipulation.operator_console.recorded_server --demo --port 8896
```

이 예제는 수학적으로 만든 2축 링크의 기록입니다. 제조사 모델이나 물리 시뮬레이션 결과로 표시하지 않습니다. Python 표준 라이브러리와 저장소의 로컬 JS만 사용하며, ROS·MuJoCo·NumPy가 없어도 실행할 수 있습니다.

직접 준비한 기록은 시작할 때 명시합니다.

```bash
PYTHONPATH=src python3 -m mujoco_ros2_manipulation.operator_console.recorded_server \
  --recording /path/to/recording.json --port 8896
```

`display-recording-v1` JSON은 `schema_version`, `kind`, `label`, `manifest`, `frames`를 포함합니다. 각 frame은 `time_sec`, 전체 `positions_rad`, 공통 pose를 갖습니다. 처음은 0초이고 원본 표본 시각은 엄격히 증가해야 합니다. 메시 바이너리는 인접한 `assets/<sha256>.bin`에서 읽습니다. 기록은 최대 64 MiB·20,000표본·3,600초이며 시작할 때 읽은 내용과 자산을 메모리에 고정합니다.

재생은 다음 표본을 표시할 뿐 FK나 물리 적분을 하지 않습니다. 되감기·seek·기록 교체는 새 binding을 만들고 이전 요청을 무효화합니다. 일시 정지와 EOF에서는 마지막 표본의 나이가 증가하여 stale로 표시됩니다. 카메라는 계속 움직일 수 있습니다. 연결 중단도 마지막 자세와 구분해 표시하며 새 응답을 받으면 복구합니다.

서버는 loopback에만 열리고 고정한 표시 자산만 제공합니다. 재생·정지·seek 요청은 같은 origin의 토큰을 요구합니다. `/api/command` 같은 로봇 명령 경로는 없습니다. 실행 중 임의 파일을 업로드하거나 다른 로봇 제어기로 바꾸는 기능도 없습니다.

## 실행 전 계획 검토

일반 관절 조작의 기본 흐름은 **계획 생성 → 경로 미리보기 → 승인·실행**입니다. 기존 자동 계획·실행 API는 호환성을 위해 유지하며 시연도 기존 방식으로 실행합니다.

1. `/api/plan`이 기존의 완전한 이름 기반 목표를 받습니다. 서버가 계획·거리·MoveIt 검사를 마치고 `plan_ready`를 알립니다. 이 단계의 궤적 전송 수는 0입니다.
2. `/api/view/review`로 현재 계획을 표시하고 재생합니다. 서버가 보존한 정확한 review payload의 SHA와 표시할 목표·초기 상태·궤적을 다시 대조합니다. 성공한 과거 실행의 궤적 재생과 구분합니다.
3. `/api/plan/execute`가 새 실행 ID와 계획 ID·SHA·서버 세션을 전달합니다. 서버가 승인을 한 번만 소모하고 저장한 동일 궤적을 다시 검사한 뒤 전송합니다. 미리보기 재생 자체는 실행 승인이 아닙니다.

계획 유효 시간은 발급 후 wall 60초입니다. 최종 전송 직전까지 만료를 검사합니다. 초기 관절 차이가 어느 하나라도 0.002 rad 이상이거나 모델·정책·실제 MoveIt world/attached object·충돌 행렬·고정 변환·서버 세션이 달라지면 다시 계획해야 합니다. 잘못된 해시, 취소, 이미 사용한 승인과 시계 역행도 거부합니다. 승인 뒤 검사나 실행이 실패해도 같은 토큰을 다시 사용할 수 없습니다.

이동이 필요하지 않은 목표는 `plan_not_required`로 종료하며 승인 토큰을 발급하지 않습니다. 이미 완료한 요청 ID를 다시 보내도 새 궤적을 전송하지 않습니다. 실물 안전 기능으로 검증한 흐름은 아닙니다.

## 검증과 남은 범위

공통 형식·기록 로더·세션 전환은 [단위 검사](../tests/unit/test_display_recording.py), [브라우저 경계 검사](../tests/js/test_viewer.mjs)에서 확인합니다. 계획 승인은 [저장소 검사](../tests/unit/test_motion_review.py), [미리보기 검사](../tests/unit/test_view_plan.py), [실제 ROS 검증기](../src/mujoco_ros2_manipulation/workcells/verify_motion_review.py)를 사용합니다.

다른 시뮬레이터의 실제 연결, 센서·점군, 임의 CAD assembly, 가져온 환경을 포함한 실제 회피 실행은 후속 작업입니다. 현재 형식에 엔진 출처를 옮긴 것만으로 이런 연결이 완료되지는 않습니다. [움직이는 기록의 성능·안정성 평가](viewer_display_benchmark.md)도 생산용 전체 제어 성능과 구분합니다.

[조작 화면](operator_console.md) · [공통 모듈 경계](module_boundaries.md)
