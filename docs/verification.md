# 동작과 종료 검증

[프로젝트 README](../README.md) · [설치와 빌드](setup.md) · [모델별 자료](robots/README.md)

[verify_robot.py](../verify_robot.py)는 선택한 모델의 실제 ROS 노드를 별도 프로세스로 실행해
궤적·MoveIt·상태·TF·시계와 종료를 검사합니다. 아래 명령은 저장소 루트에서 실행합니다.

## 2026-10-27 야간 로컬 검증

이 절은 2026-10-27의 W01~W06 진단·수정과 대표 실제 실행을 격리한 로컬 작업 환경에서 확인한 기록입니다. 요청한 1~4단계의 로컬 구현과 대표 검증을 마쳤으며, 원격 CI와 공개 반영 상태는 [PR #52](https://github.com/WannaSleep3254/mujoco-ros2-manipulation/pull/52)와 후속 승격 PR에서 확인합니다. 원본 실패 자료와 새 결과는 각각 보존하며, 단위 검사·실제 ROS 실행·브라우저 확인을 구분합니다.

### W01~W03 취소 정리와 접촉 원인

- **W01:** 정리 단계별 시작·완료, 취소 지연 시 Python 스택, 중첩 작업 ID와 소유 프로세스 대기를 기록하도록 보강했습니다. 일반 시연·취소 `20261027T103325.195004Z-20a5139e`와 native 접촉 인터록 뒤 취소 `20261027T113253.316711Z-1d903aaf`에서 전체 정리를 통과했습니다. 별도로 확인한 접촉 후 취소 경로는 수정 후 검사했지만, 과거의 간헐적인 종료 지연은 같은 증상으로 재현되지 않았습니다. 그 대기 위치와 근본 원인을 해결했다고 판정하지 않습니다.
- **W02:** 보존한 817점 경로를 재고정하거나 변경하지 않고 재실행해 접촉을 재현했습니다. 첫 접촉 스텝에서 적분 전 명목 간격 약 2.002 mm, 실제 간격 약 -0.146 mm로 차이는 약 2.1477 mm였습니다. 같은 창의 명목 표본 최소 간격은 약 1.749 mm입니다. 접촉 응답 적분 전의 차이를 확인한 단일 사례이며 특정 관절의 원인이나 모든 실행의 추종 오차 상한을 증명하지 않습니다. 실패한 동작과 정상 정리 판정도 함께 보존했습니다.
- **W03:** 팔 사이 거리 정책과 별도로 이동 로봇·고정 설비 사이의 native 간격을 검사합니다. 현재 최소 5 mm + 추종 여유 10 mm는 합계 15 mm의 명시적인 시뮬레이션 설정입니다. 실물 안전 거리나 측정으로 증명한 최악 오차가 아닙니다. 보존한 작은 간격의 경로는 전송 전에 거부하며 접촉·시계 감시와 최종 경로 검사를 유지합니다. 고정 suite 20개 중 선정한 6개를 한 번씩 실행해 정상 동작 4개와 예상한 잘못된 목표 거부 2개를 확인했습니다. 미실행 14개는 결과에 합산하지 않습니다.

로컬 원본은 `runtime/w01-reproduction/`, `runtime/w02-night/attempt-01-summary.json`, `runtime/w02-night/native-contact-analysis.json`, `runtime/w03-environment/fixed-ci-six/fixed-benchmark.json`에 보존했습니다. W02 첫 수집의 UUID 배열 처리 문제로 누락된 액션 상태 30개와 잘린 수동 관측도 한계로 남깁니다. 수집기 수정으로 그 과거 누락 자료가 복원되지는 않습니다. 정확한 native 스텝의 기하 자료와 시각으로 연결한 표준 제어기 관측을 구분합니다.

### W04 작업·계획·실행 분리

로컬 커밋 `9a65e73`에서 계획 요청 어댑터, 궤적 실행 어댑터·요청별 ledger와 추가 관측 규약을 연결했습니다. 기존 v1 명령과 호환 자동 실행, 일회 계획 승인, 상태·거리 검사와 중복 전송 방지는 유지합니다. 취소 전송·수신·요청 종료, 물리 감시의 정지와 프로세스 정리는 서로 다른 판정입니다.

실제 ROS 도메인 218의 계획 검토·승인 `20261027T121926.842852Z-c7e7ef6d`, 도메인 220의 재계획 `20261027T122157.550007Z-f52bd6a7`를 통과했습니다. 현재 MuJoCo·ROS 2·MoveIt 조합의 분리 경계를 검증했으며 새 엔진이나 모든 로봇 플러그인의 교체 호환성을 확인한 결과는 아닙니다. 필드·시간·수락·취소 책임은 [작업·계획·실행 규약](operation_contract.md)에 정리했습니다.

### W05 이미지 수신·분석과 객체 계층

도메인 221에서 실제 `sensor_msgs/Image` RGB8 발행자→수신 콜백→별도 JPEG 작업자→HTTP 조회를 확인했습니다. 잘못된 인코딩 거부, 발행자가 남아 있는 상태의 stale, 실제 로봇 명령과 요청 분석, 재연결 후 세대 변경·이전 이미지 410·명령 재전송 없음을 검사했습니다. `runtime/w05-image-transport/result.json`은 `status=passed`, `cleanup_clean=true`입니다.

도메인 222의 실제 브라우저에서는 64×48 이미지의 디코딩과 표시, 객체 계층과 계획 검토·승인·실행 성공을 확인했습니다. 대상 모델의 CPU 대조에서는 world 포함 body 27개·geom 46개, 가져온 환경의 body ID 26과 시각 geom ID 44·충돌 geom ID 45를 확인했습니다. 44와 45는 형상 식별자이며 개수가 아닙니다. 이미지는 진단용 합성 입력이며 하드웨어 카메라·보정·world 변환·센서 정확도를 검증한 것은 아닙니다. 공구·TCP 역할도 이름으로 추측하지 않습니다.

원천 acquisition stamp의 기본 시계 영역은 `unknown`이며 명시적으로 선언한 경우에만 `ros_simulation`입니다. `/clock` 존재로 원천 시계를 추정하지 않습니다. 원천 시각, 수신 순번과 monotonic 수신 나이를 분리하며 현재 공급원 관측을 과거 요청 시점의 값으로 동기화하지 않습니다. 반복 stamp는 나이를 갱신하지 않고 시계 역행·공급원 변경은 이전 세대 이미지를 무효화합니다. 입력·변환·보존 상한과 읽기 전용 패널은 [이미지 입력과 상태 분석](sensor_analysis.md)을 따릅니다.

### W06 설치 환경 실행 연결

대표 합성 STEP 환경을 별도의 실행 타깃으로 선택해 MuJoCo·MoveIt·명령 서버와 native 거리 검사에 연결했습니다. 기본 Workcell ID와 관절·제어 계약은 유지하며 뷰어의 표시 모델 선택으로 실행 타깃이 바뀌지 않습니다.

| 실제 로컬 조건 | 도메인 | 확인한 결과 |
| --- | --- | --- |
| 같은 설치 환경의 정상 실행 | 228 | 계획만 생성할 때 전송 0회, 승인한 계획 실행, 자동 명령·취소·홈 복귀와 정착 |
| 잘못된 manifest SHA | 224 | 실행 시작 거부, ROS 엔티티·물리 스텝·목표 전송 0회 |
| 검토 뒤 타깃 파일 변경 | 229 | 승인 실행 거부와 목표 전송 0회 |
| 기본 모델 실행과 overlay 타깃의 불일치 | 226 | 실행 정체성 불일치 차단과 목표 전송 0회 |
| 별도 배치의 native 접촉 정지 | 231 | 가져온 환경 body의 실제 접촉, action·감시 정지와 정상 정리 확인. 정상 실행 228과 배치가 달라 동일 타깃 결과로 합산하지 않음 |
| 같은 타깃·SHA의 정상 동작부터 진단 접촉 정지까지 | 223 | 최종 실행에서 일반 계획·승인·홈 복귀·이동 중 취소·홈 복귀 후 별도 진단 궤적의 실제 접촉 정지, native 검사와 전체 정리 통과 |

로컬 결과는 `runtime/w06-environment/normal-domain228-session/review-verification.json`, `changed-domain229-session/review-verification.json`, `mismatched-runtime-session/review-verification.json`과 각 소유 실행의 종료 결과에 남깁니다. 잘못된 SHA 실행은 `20261027T124156.605573Z-c7286370`입니다. 초기 실패·수정 전 실행도 최종 통과 자료와 별도로 보존합니다.

최종 실행 `20261027T130258.401995Z-c3b4d68a`의 schema 2 결과는 `status=passed`, `motion_checks_passed=true`, `physics_checks_passed=true`, `shutdown_clean=true`, `process_group_gone=true`입니다. 환경 manifest SHA `e9a661c87bcf392cb74f989f115c17b963df2000dba4d5fa354686bcdcb5d03b`를 바꾸지 않고 일반 경로의 목표 8개와 마지막 독립 진단의 목표 2개를 구분해 기록했습니다.

진단 경로는 `--allow-simulated-contact`를 명시한 시뮬레이션 검증기에서만 사용합니다. 첫 궤적점과 FK 시작 관절, 관절 이름·순서, 끝점과 상대 시각의 정확한 일치를 검사했습니다. 일반 계획·실행 API의 검사 완화나 접촉 주입은 하지 않았습니다. native 7,691스텝 중 가져온 box와 손목 3의 접촉 1회, 약 -0.11218 mm 간격, 감시의 `stopped`와 양팔 제어기 `inactive`, 두 action의 CANCELED 상태 `[5,5]`를 확인했습니다. 이후 시계가 약 0.6초 계속 진행하는 동안 관절 위치 변화와 속도는 0이었고 전체 정리도 통과했습니다. 시뮬레이션의 위치 고정 검증이며 실물 제동 시험이나 일반 플래너가 접촉 경로를 허용한다는 뜻은 아닙니다.

현재 충돌 형상은 보수적인 box 근사이며 샘플링한 거리·추종 여유는 연속 시간 충돌 없음이나 실물 안전을 보장하지 않습니다. 시작 시 고정한 파일과 ROS 설명·모델 경로를 대조하며 native 시뮬레이터 메모리 전체의 동일성을 증명하는 기능은 아닙니다. 절차와 제한은 [설치 환경 실행](environment_execution.md)에 정리했습니다.

### 전체 회귀 검사

같은 로컬 작업 환경의 Python 소스로 단위 852개를 54.759초에 통과했습니다. 진단 궤적의 FK 시작점과 토픽·이미지 stamp 경계 회귀를 포함합니다. 모델 38개와 저장소 문법·로컬 링크·build manifest·로봇 프로필 3종·scene 검사, JavaScript 10개 검사 파일도 통과했습니다. 이미지 수신 모듈과 선택 기능의 순수 import 검사를 포함하며, 선택 기능을 가져오는 것만으로 ROS·MuJoCo·Pillow를 시작하지 않습니다.

최근 단위 검사 로그는 `runtime/night-release-clock-allowed-unit.log`이며 이전 전체 검사 로그는 `runtime/w05-w06-full-unit.log`, `runtime/w05-w06-full-model.log`, `runtime/w05-w06-full-repository.log`입니다. 의도한 NaN 제어 입력의 모델 시험이 생성한 경고는 `runtime/w05-w06-model-negative-mujoco.log`에 별도로 보존했습니다. 새 CI workflow에는 설치 환경 4조건과 실제 RGB8 전송 검사를 추가했으며 동일 명령을 로컬 도메인 218~222에서 모두 통과했습니다. 위 수치는 로컬 결과이며 원격 workflow 결과는 해당 PR의 CI에서 확인합니다. 최종 시각 수정 후 도메인 222의 `runtime/validation/image-transport-final-clock/result.json`도 다시 통과했으며, 실제 HTTP의 원천 이미지와 `/clock` 나노초가 십진 문자열로 전달되고 정리 후 서버가 종료되는 것을 확인했습니다. epoch 시각의 1ns 역행 검사는 실제 콜백을 사용하는 단위 회귀로 구분합니다. 이 검사 수와 대표 실행 결과를 전체 suite·모든 로봇·모든 센서의 검증 완료로 확대하지 않습니다. W01의 간헐 지연 원인, 실제 카메라 보정과 복잡한 CAD는 [작업리스트](development_worklist.md)의 후속 항목으로 유지합니다.

## 2026-10-21 후속 검증

[시뮬레이터 제작 상태](simulator_status.md)와 [탐색 A/B·GUI 리포트](reports/2026-10-21-search-optimization/index.html)에 현재 PC에서 확인한 범위를 정리했습니다.

- 실제 ROS 상시 명령 회귀: `20261021T093445.292780Z-941dbff7`, 공동/출발 조정·중복/동시 요청·충돌 계획 실패·취소/복구·CLI·동작 중 종료 통과.
- 실제 GUI 공동 작업 환경: `20261021T093837.684967Z-e087f7f3`, MuJoCo·RViz GPU 프로세스, 12관절·공유 시계/상태, 계획·충돌 목표 거부·양팔 취소, 접촉 0·정상 종료 통과. 원본 PNG와 해시 보존.
- 오프라인 탐색 A/B: 고정 입력 6개 × 두 방식 × 5회, 워밍업 별도. 선택 팔·지연·전체 spline·선택 경로 거리 평가 정확히 일치, 네이티브 호출 수 증가 없음.

```bash
source ./manipulation_env.sh
python3 verify_motion_commands.py --domain 138
python3 verify_scene.py --gui --screenshot-dir runtime/validation/새캡처폴더 --performance-seconds 5 --domain 142
PYTHONPATH=src .venv/bin/python scripts/compare_search_optimization.py --repeats 5
runtime/report-tools/bin/python scripts/import_search_optimization_report.py --check docs/reports/2026-10-21-search-optimization
```

GUI·ROS 검증과 오프라인 성능 측정은 순서대로 실행합니다. GUI의 약 1.000 RTF는 동작 검사 후 정지 유지 5wall초 구간으로,
최대 처리량·이동 성능·GUI FPS와 구분합니다. 상시 명령은 실제 ROS에서 재검사했으며,
새 탐색의 전체 지령→이동 지연 효과는 다음 랜덤 평가에서 별도 측정합니다.

## 전용 조작 화면 검증

[통합 화면 안내](operator_console.md) · [원본 검증 자료](operator/validation.json.gz)

실제 HTTP→ROS→관절 이동·양팔 취소·출발 조정 복구·정상 종료는 `verify_operator_console.py`로 검사합니다.
`20261021T104643.608766Z-21a701d4`에서 라이브 EGL 렌더러를 함께 실행해 통과했으며 매 물리 스텝 접촉 0을 확인했습니다.
브라우저 버튼·목표 파일·이력/리포트·데스크톱/모바일·재시작 후 실제 동작은 별도 검증했습니다.
CI는 같은 API/ROS 검증을 `--no-render`로 실행합니다. 네이티브 GUI 창·실물 검증과 구분합니다.

```bash
source ./manipulation_env.sh
python3 verify_operator_console.py --domain 143 --no-render
```

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

각 실행은 `runtime/validation/runs/<run_id>/result.json`과 `launch.log`를 보존합니다.
GUI GPU 관측은 해당 실행 디렉터리의 `nvidia_smi.txt`에 저장합니다.
기존 `runtime/validation/<모델>_<headless|gui|gui_window>.json`은 가장 최근에 시작한 실행의 결과 사본이고,
같은 이름의 `.log`는 해당 실행 로그를 가리키는 링크입니다. 실행 시작 시 `running`으로 갱신하므로 이전 성공을 최신 결과로 오인하지 않습니다.
뒤늦게 끝난 이전 실행은 더 최신 실행의 결과를 덮어쓰지 않습니다. 기존 형식의 최신 결과는 첫 실행 전 `runs/legacy-*/`에 보존합니다.

사전 생성물 검사·노드 초기화·동작·GUI·종료 실패도 JSON에 남깁니다. 인자 구문 오류처럼 실행 이력이 시작되기 전의 CLI 오류는 제외합니다.
상태는 `running`, `passed`, `failed`, `interrupted`로 구분합니다. 강제 종료나 전원 차단으로 마무리하지 못한 실행은 `running`으로 남으며 성공으로 판정하지 않습니다.

| JSON 필드 | 의미 |
|---|---|
| `schema_version` / `run_id` / `status` | 결과 형식 버전 2, 고유 실행 ID, 실행 상태 |
| `stage` / `failure_stage` | 현재 단계와 실패가 발생한 단계 |
| `error` / `cleanup_errors` | 최초 오류의 타입·메시지와 정리 중 추가 오류 |
| `started_utc` / `completed_utc` | 실행 시작·완료 시각 |
| `model_manifest_sha256` | 검증한 모델 무결성 명세의 해시 |
| `motion_checks_passed` | 선택한 검증기의 동작 검사 통과. 크래시 전용 검증은 양팔 이동 전제만 포함하며 전체 정상 동작 검증과 구분 |
| `direct_tracking_error_rad` / `moveit_tracking_error_rad` | 상태 확인을 마친 시점의 관절별 오차 |
| `cleanup_diagnostics` | 노드 해제, ROS 종료, launch 정리, 추가 진단(finalize 콜백)의 시작·완료 시각과 소요 시간. 진단 기록이며 정상 종료 판정을 대신하지 않음 |
| `shutdown_clean` | 종료 검사 통과 |
| `launch_return_code` | launch 프로세스의 종료 코드, 정상값 0 |
| `process_group_gone` | 검증이 실행한 프로세스 그룹의 잔류 없음 |
| `shutdown_failures` / `shutdown_timed_out` | 예상하지 않은 비정상 노드와 시간 초과 |
| `expected_process_exits` | 크래시 전용 검증에서만 허용한 정확한 노드·PID·종료 코드. 일반 검증은 빈 목록 |
| `robot_profile_sha256` | 검증에 사용한 프로필의 해시 |

성공은 종료 코드 0, 사전 검사·동작·정리 실패는 1, 사용자 중단은 130을 반환합니다.
동작 검사 완료 후 종료 검사에서 오류가 나면 동작 결과를 보존하되 전체 상태를 `failed`로 기록합니다.
정리 오류가 발생해도 최초 실패 원인을 덮어쓰지 않습니다. SIGINT로 끝나지 않으면 SIGTERM·SIGKILL로 단계적으로 정리하며 강제 정리는 정상 종료로 판정하지 않습니다.
촬영 자료 수집기는 최신 상태가 `passed`이고 모델 명세까지 일치하는 결과만 사용합니다. 기존 형식의 로컬 촬영 결과는 재검증해야 합니다.
종료 오류의 원인과 로컬 수정은 [문제 해결](troubleshooting.md)에 정리했습니다.


## 취소와 정리 지연 진단

중첩 시연의 취소 경로에서는 정지 요청, 검증 작업 종료, 소유 프로세스 정리를 나누어 확인합니다. 공통 `run_verification` 수명주기를 사용하는 실행의 `cleanup_diagnostics.steps`에는 `destroy_node`, `ros_shutdown`, `launch_stop`, `finalize` 순서로 시작 시각을 먼저 저장합니다. 호출이 끝나지 않으면 해당 항목이 `running`으로 남습니다. 완료 항목에는 UTC 시각, 단조 시계 값, 소요 시간과 추가 오류를 기록합니다. 최초 동작 오류는 기존 `error`에 보존합니다.

시연 worker는 첫 SIGINT 수신 때 10초 후 자신의 작업 폴더의 `cancel-stacks.log`에 Python 스레드 프레임을 한 번 기록하도록 예약합니다. 작업 처리가 끝나면 예약을 취소합니다. 네이티브 C/C++ 스택을 수집하는 기능은 아니며, 수집 자체가 worker를 종료하지 않습니다. 정상적으로 빨리 끝난 취소에는 지연 스택이 생성되지 않을 수 있습니다. 진단 파일 생성 실패도 원래 취소 결과를 바꾸지 않습니다.

상위 조작 화면의 `events.jsonl`에는 `demo_worker_wait_started`와 `demo_worker_wait_finished`를 남깁니다. 기존 worker 대기의 PID, 제한 시간, 대기 구간의 경과 시간, 반환 코드와 시간 초과 여부를 확인할 수 있습니다. 45초는 `process.wait`의 제한이며, audit 기록과 후속 정리를 포함한 전체 기한은 아닙니다. 시간 초과 기록은 강제 정리를 시작하기 전에 저장합니다.

`verify_operator_demos.py`는 종료 확인 후 소유 console 세션을 검증 실행 폴더의 `console-session`에 먼저 복사하고 audit을 검사합니다. 정리 실패나 audit 오류 때문에 작업 상태·로그·스택이 버려지지 않도록 한 것입니다. 결과의 `console_evidence.copied`와 `audit_valid`를 구분해 확인합니다. 자료 복사 성공이 정상 종료를 뜻하지 않습니다.

```bash
source ./manipulation_env.sh
python3 verify_operator_demos.py --domain 201 --port 8911 --no-render
```

위 도메인과 포트는 실행 전 비어 있는지 확인하고 선택합니다. 검증기는 자신의 프로세스만 정리하며, 강제 정리·프로세스 잔류가 발생한 실행을 정상 종료로 판정하지 않습니다. 이 진단 추가만으로 과거의 간헐적 정리 지연 원인이 해결됐다고 판단하지 않습니다.

## 접촉 전후 상태와 제어기 관측

`MANIPULATION_STEP_WITNESS=1`로 검증기를 실행하면 각 소유 실행 폴더에 `step-witness.json`을 추가합니다. 재빌드한 네이티브 플러그인이 필요합니다. 기존 `physics.json`과 정상 종료·접촉 판정은 유지합니다.

네이티브 기록은 최근 512개 물리 스텝의 계산 직전 `qpos/qvel/ctrl`, 계산 직후 상태와 스텝 ID를 보존합니다. 첫 접촉이나 정지에서 기록 창을 고정하고, 인터록 적용 후 상태를 별도로 남깁니다. `ctrl`은 그 스텝에 적용된 액추에이터 입력이며 관절 목표 라디안이라는 일반 의미를 부여하지 않습니다. 파일 쓰기는 종료 시 수행합니다. 상태·접촉 수와 용량 제한, 잘린 기록, 오류를 확인한 뒤 분석해야 합니다. 현재 실증 대상은 모델을 바꾸지 않는 headless 실행입니다.

제어기의 표준 상태는 별도 관측기로 수집합니다. 실행 전에 독립 도메인을 정하고, 새 파일의 부모 폴더를 준비합니다.

```bash
source ./manipulation_env.sh
python3 capture_controller_witness.py --domain 202 \
  --output runtime/새진단폴더/controller-witness.json --duration 120 --capacity 20000
```

이 도구는 두 제어기의 `controller_state`, 액션 상태와 `/clock`을 구독합니다. 위치·속도·오차의 현대 필드와 호환 필드를 보존하고, 원천 stamp와 수신 단조 시각을 분리합니다. 표준 메시지에는 native 스텝 ID가 없어 같은 2ms 스텝과 정확히 일치한다고 판단하지 않습니다. 액션 UUID를 요청 ID로 추정하지도 않습니다. 실행의 모델·도메인·시계·요청 자료와 함께 연결해야 합니다. 용량 때문에 제거된 표본과 잘못된 입력은 계수에 남으며, DDS 전송 유실을 모두 측정한 값은 아닙니다.

접촉 전 취소와 접촉 후 취소는 별도로 실행합니다.

```bash
MANIPULATION_STEP_WITNESS=1 python3 verify_operator_demos.py \
  --domain 201 --port 8911 --no-render --cancel-after-latch
```

접촉 후 취소 검사는 고정 장애물의 인터록이 걸리고 두 제어기가 비활성인 상태에서 취소합니다. 완료 자료의 run ID, native 원본 해시와 내용, 스텝 coverage, 모니터 소스, timestep과 접촉 쌍을 대조합니다. 정상 취소와 물리 시연 전체 완료를 구분하며, 과거의 지연이 매번 발생한다고 가정하지 않습니다.

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
PYTHONPATH=src .venv/bin/python -m unittest discover -s tests -t . -v
```

2026-10-19 검증 보고서·생성물 검사 보강 후 **34개 통과·건너뜀 0개**를 확인했습니다.
FR5·FR10·UR5e의 실제 headless ROS·MoveIt 실행과 정상 종료, 잘못된 모델 요청의 실패 JSON도 확인했습니다.
GUI 경로는 모의 검사이며 실제 데스크톱 GUI 재실행과 구분합니다. [검증 결과](validation_hardening.json)에 실행 ID·모델 명세·검사 코드 해시를 보관했습니다.

기존 11개 검사(2026-10-07 모두 통과)는 다음을 포함합니다.

- 프로필 선택, 관절 수·이름 변경, 잘못된 배열과 미준비·오래된 모델의 실행 차단.
- 관절 순서를 변경해도 유지되는 MuJoCo 이름 매핑과 세 모델의 끝단 순기구학 일치.
- DAE Scene 변환·단위·색상·법선, 세 자세의 UR5e 메시 배치와 물리 모델 보존.

변환 검사에는 제조사 소스와 생성 모델이 필요합니다. 해당 자료가 없으면 관련 검사를
건너뛰므로 결과의 건너뜀 개수도 확인합니다.
신규 회귀 검사는 입력·출력 변경, 제조사 소스 변경, 잘못된 명세, 단계별 생성 실패와 게시 중단을 검사합니다.
검증 이력 검사는 ROS 연결을 모의하여 실패·중단·정리 오류·동시 실행의 최신 결과·촬영 수집 거부를 재현합니다.
이는 실제 ROS·GUI 통합 검사와 구분합니다.
코드는 [무결성 검사](../tests/unit/test_model_integrity.py), [실행 이력 검사](../tests/unit/test_verification_reports.py), [프로필 검사](../tests/unit/test_robot_profiles.py)와 [시각 모델 검사](../tests/model/test_ur5e_visuals.py)에 있습니다.

기존 `verify_fr5.py`는 FR5 호환 진입점입니다. 신규 모델은 공통 검증기를 사용합니다.
[초기 1관절 브리지 검증](single_joint_demo.md)은 별도 데모와 ROS 도메인을 사용합니다.

## 구조 개편 후 검사

단위 검사는 `tests/unit`, 제조사 모델 검사는 `tests/model`, 실제 ROS 절차는
[tests/integration](../tests/integration/README.md)로 분리했습니다. 전체 검사는 위의
`PYTHONPATH=src ... discover -s tests -t .`로 실행합니다.
`verify_robot.py --scene pedestal`은 기본 모델 검증을 덮어쓰지 않고 작업 환경별 보고서를 생성합니다.
실제 MoveIt의 robot_description 충돌 링크와 작업 환경의 고정 TF도 확인합니다.
검증 조합과 CI 실행 경계는 [구조 설명](architecture.md#ci와-검증-경계)에 있습니다.

2026-10-19 구조 개편 후에는 **44개 통과·실패 0개·건너뜀 0개**,
세 모델 기본 작업 환경과 FR5 받침대 작업 환경의 실제 ROS 연동·충돌 모델·TF·종료를 확인했습니다.
[구조 개편 검증 기록](structure_validation.json)에 코드 해시·실행 ID와 미실행 항목을 보관합니다.

공동 작업 환경 확장 후 **56개 통과·실패 0개·건너뜀 0개**를 확인했습니다.
기존 4개 모델·작업 환경 ROS 실행과 공동 작업 환경 2회 연속 실행도 통과했습니다.
[공동 작업 환경 검증 기록](workcell_validation.json)에 좌우 격리·공동 계획·충돌 거부·상호 취소·종료와 남은 검사를 구분했습니다.

## 접촉 원인 식별을 위한 무장애물 대조 검증

[11초 비교 영상과 재현 명령](contact_comparison.md)은 같은 8초 왕복 명령을 두 독립 실행에 적용합니다.
접촉 실행은 모션 약 2.012초에 양팔이 취소·고정되고, 무장애물 대조군은 남은 약 5.988초 동안 계속 이동·복귀하여 양팔 SUCCEEDED로 끝납니다.
명령 해시·모델이 같고 두 실행 모두 자동 정지 활성화를 유지합니다. 양팔 동시 정지 정책은 변경하지 않았습니다.
전체 검사 82개(실패·건너뜀 0개), 두 조건의 실제 ROS headless·GUI와 비교 게이트가 통과했습니다.
[비교 기록·원본 영상·보고서·해시](contact_comparison.md#검증과-ci)를 함께 보관합니다.
이전 2차 시연 `d515e41`의 ROS 5조합·Python CI는 6개 모두 통과했으며, 원본 공동 작업 환경 아티팩트까지 확인했습니다.
새 비교 커밋에서는 CI에 장애물 없음 실행과 동일 명령·성공/취소 결과 비교를 추가했습니다.

## 고정 장애물 접근과 양팔 자동 정지 검증

[새 시연 영상 (약 7초)](workcells/fr5_ur5e/contact_stop/visible_approach/approach-contact-stop.mp4) ·
[초기·접근·접촉 화면과 실행 명령](contact_stop.md#고정-장애물-접촉-시연)

고정 기둥을 시작부터 배치한 별도 시뮬레이션에서 양팔을 각각 약 56° 이동했습니다.
FR5 손목 경로는 약 0.65m, 관측 최고 속도는 약 0.50rad/s였고,
손목이 기둥 표면에 닿자 양팔 목표가 자동 취소되고 두 제어기가 비활성화됐습니다.
접촉 주입 요청·검증기의 목표 취소 요청은 모두 0회입니다.
새 검증기는 이동량·속도가 너무 작으면 실패하며, 주입 서비스 부재·의도한 접촉 쌍·표면 접촉 깊이·정지 유지도 확인합니다.

실제 GUI 실행 `20261020T035225.895962Z-17bf9e00`에서 접촉 직전·직후 프레임과 정지 구간을 검토했습니다.
첫 접촉 스텝의 활성 접촉점은 4개이며 이후 추가 적분·최대 관절 변화는 각각 0회·0rad입니다.
영상의 정지 표시는 약 4.17초에 나타나며, 이는 영상 내 시점이고 제어기 반응 시간 측정값이 아닙니다.
[원본 GUI 결과](workcells/fr5_ur5e/contact_stop/visible_approach/validation.json)와
[자료·소스 해시 및 회귀 결과](workcells/fr5_ur5e/contact_stop/visible_approach/assets.json)에 근거를 보관합니다.

전체 단위·모델 검사 78개 통과(건너뜀 0개), 새 headless·GUI 시연 및 기존 일반·일시정지 주입 시험,
정상 공동 제어 회귀 검사가 통과했습니다. CI에는 `verify_contact_demo.py --domain 113`을 추가했고,
GUI 녹화와 영상의 시각 검토는 로컬에서 수행했습니다. 기존 단일 로봇 ROS 실행은 이번 변경에서 별도로 재실행하지 않았습니다.
이전의 시각적 접촉이 없는 자료는 아래에 과거 기록으로 보존합니다.

## 시험용 접촉 주입 후 양팔 정지 기록

[시연 영상 (MP4, 약 11초)](workcells/fr5_ur5e/contact_stop/demo/contact-stop.mp4) ·
[정지 화면](workcells/fr5_ur5e/contact_stop/demo/contact-stop.png) ·
[시연 해석과 시험 절차](contact_stop.md#시연-영상-2026-10-20)

2026-10-20 실제 FR5·UR5e GUI 실행 `20261020T031658.580272Z-135895a1`에서
시험 접촉 1건, 양팔 Action 자동 취소(CANCELED), 양팔 제어기 비활성화,
정지 후 최대 관절 변화 0rad·추가 물리 적분 0회와 정상 종료·잔류 프로세스 없음을 확인했습니다.
[원본 실행 결과](workcells/fr5_ur5e/contact_stop/demo/validation.json),
[물리 기록](workcells/fr5_ur5e/contact_stop/demo/physics.json),
[영상·화면·기록의 해시](workcells/fr5_ur5e/contact_stop/demo/assets.json)를 함께 보관합니다.

**영상 130프레임과 정지 이미지에는 눈으로 확인할 수 있는 접촉 장면이 없습니다.**
두 로봇은 떨어져 있으며, 시험 구체를 FR5 전완 내부로 순간 배치한 뒤 정지 표시가 나타납니다.
이 자료는 시험용 접촉 주입에 대한 정지 상태 기록으로 분류합니다.
물리 기록의 접촉 쌍과 정지 스텝은 기능 검증 근거이며 시각적 접촉 장면의 증거로 사용하지 않습니다.
촬영용 이동 대기가 포함된 `guard_observed_wall_sec`는 반응 시간으로 사용하지 않습니다.
이 시연은 자동 정지 검증이며 실제 로봇의 제동 시험이나 고속 충돌 시연을 포함하지 않습니다.

## 핵심 프로세스 크래시 검증

[실행 방법과 범위](process_crash.md)에 두 대상의 독립 실행·실패 판정·보고서 해석을 정리했습니다.
전체 프로세스 종료 검증은 접촉 시 물리 위치 고정 검증과 구분합니다.

## 시계 정지 감시와 실패 주입

[시계 검증](clock_stop.md)은 메시지가 끊기는 경우와 같은 값이 반복되는 경우를 각각 실행합니다.
네이티브 벽시계 감시·양팔 위치 고정·취소/비활성화·명령 차단·의도한 일시정지의 오탐 없음을 확인합니다.
`stop_reason=clock_stall`, `clock_stop_test=true`로 접촉 정지와 구별하고 전체 물리 스텝 보고서도 검사합니다.

## 양팔 간섭 회피 전용 사례

[교차 사례](interference_avoidance.md)는 독립 계획 두 개의 단독 안전성·합성 간섭·전송 차단과
공동 계획의 양팔 성공·실제 수렴·전체 물리 스텝 접촉 0을 확인합니다.
20ms 이하 샘플과 실제 제어기의 선형/3차/5차 보간을 사용합니다.
일반 MoveIt 계획만 성공했다는 이유로 위험한 병렬 궤적을 전송하지 않습니다.


## 단독 정상·무회피 실제 접촉·공동 회피 비교

[세 조건 비교](interference_comparison.md)는 같은 궤적 묶음을 네 번의 새 launch에서 재사용합니다.
FR5·UR5e 각각 단독 성공, 무회피 동시 실행의 실제 손목 접촉·양팔 자동 정지, 공동 회피의 양팔 성공을 비교합니다.
정지 후 관절 변화 0rad·남은 명령 3.24초를 확인했으며, 실제 GUI 원본을 같은 지령 시점에 맞춰 확대했습니다.
첫 회피 시도의 참조 경로 차단·정적 렌더와 구분하며 원본 자료를 교체하지 않습니다.


## 거리·경로 평가와 출발 타이밍 비교

[타이밍 검증](timing_coordination.md)은 같은 원본 묶음의 기존 공동 회피와 출발 지연을 각각 새 launch에서 실행합니다.
네이티브 MuJoCo 3.12.0 거리/FK, 관절·끝단 경로 길이, 계획·수신 자세 최소 간격, 양팔 성공·수렴·접촉 0을 확인합니다.
원래 이동 spline은 보존하며 후보가 없거나 입력·거리 정책이 맞지 않으면 지령 전에 실패합니다.
시뮬레이션의 이산 최소 간격 검사이며 실제 로봇이나 연속 시간 전체 간격을 인증하지 않습니다.


## 상시 명령·요청 구간 계측

[상시 명령 안내](motion_requests.md)의 `verify_motion_commands.py`는 새 공동 시뮬레이션 한 번에서
연속 명령·실제 계획 실패·중복/동시 요청 거부·이동 전/중 취소·취소 후 복구·별도 CLI 프로세스를 검증합니다.
상세 기록은 `runtime/validation/command-sessions/`, 실행·종료 판정은 기존 검증 이력에 저장합니다.
이미 도달한 목표는 관절 명령을 보내지 않으며 최초 이동 지연을 null로 유지합니다.

## 1차 성능평가 리포트와 모션 플래닝 검토

[리포트 안내](reports/2026-10-21-phase1/README.md)에서 독립 실행 HTML·측정 JSON·CSV·시계열을 확인합니다.
같은 양팔 교차 사례 1개의 기존 GUI/headless 자료와 오프라인 고정 입력 5회 재측정입니다.
오프라인 총 중앙값 2.133초는 새 MoveIt 계획·ROS 검사·지령·실행 시간을 제외하며,
명령된 궤적 8.622초는 sim 시간입니다. 전체 지령→실제 이동 wall 지연과 랜덤 목표 성능은 아직 미계측입니다.
[세션 대조](reports/2026-10-21-phase1/review.md)는 이전 소스 사본과 최신 공동 작업 환경의 차이,
후보 유형·지연 의미·접촉 제어 도구 및 다음 평가 순서를 정리합니다.
이번 반영은 보고서·문서만 변경하며 로봇 실행 검사를 새로 통과한 것으로 기록하지 않습니다.


## 공통 리포트 검증과 목록

[생성·게시 안내](reporting.md)의 스키마로 단위·wall/sim 시계·표본·미측정 값을 검증합니다.
[리포트 목록](reports/index.html)과 CSV·파일 해시는 동일 입력에서 생성하며,
`python scripts/reports.py build --check`는 출력의 누락·변경을 쓰기 없이 확인합니다.
새 단위 검사는 자료 해시 변경·잘못된 표본·경로 이탈·잘못된 링크·출력 폴더 보존을 확인합니다.
발행 리포트 생성은 ROS 동작을 다시 실행하거나 새 성능을 측정하는 과정이 아닙니다.

## 랜덤 목표·계산 예산 회귀 검사

[랜덤 평가 절차](random_motion_benchmark.md)에 같은 목표·실측 시작 기준·전략 순서 교대와 원본 검증을 정리했습니다. CI 공동 작업 환경에서는 근거리 목표 2개의 두 전략 실행과 0.01wall초 예산 소진 시 이동 지령 금지를 별도 새 launch에서 검사합니다. 계획 실패와 평가 실행 실패를 구분하며, 접촉·복귀 실패·시작 자세 불일치는 평가를 중단합니다.

## 고정 설비 간격과 취소 진단 회귀

팔 사이 간격과 주변 설비의 간격은 별도로 검사합니다.
[환경 간격 정책](../config/planning/environment_clearance.yaml)은 시뮬레이션의 초기 설정으로
최소 간격 5mm와 추종 여유 10mm를 더해 계획 표본에서 15mm 이상을 요구합니다.
현재 모델의 홈 자세와 설비 간격은 약 74mm이며, 장착부별 임의 예외를 추가하지 않았습니다.
이 설정은 연속 시간의 충돌 회피나 실물 로봇의 안전을 보증하지 않습니다.

[보존한 고정 기둥 경로](../tests/fixtures/environment-post-grazing.json.gz)는
[과거 CI 실패](https://github.com/WannaSleep3254/mujoco-ros2-manipulation/actions/runs/40445409751)의
817개 궤적 점을 그대로 사용합니다. 계획 경로의 설비 최소 간격은 표본에서 약 1.75mm였습니다.
같은 경로를 별도 실행한 진단에서는 첫 접촉 계산 직전의 실제 자세가 기둥과 약 0.146mm 겹쳤습니다.
그 시각의 계획 자세와 실제 적용한 위치 입력의 가상 자세는 약 2.002mm 떨어져 있었습니다.
접촉 반응 전에 약 2.148mm의 간격 차이가 생긴 사실을 확인했으며, 한 관절만의 원인이나 발생 빈도까지 확정하지 않습니다.

상시 명령 서버는 시작·목표, 계획 후보와 최종 전체 경로를 검사하고, 승인한 동일 궤적도 실행 전에 다시 검사합니다.
실제 관측 간격이 최소값보다 작거나 관절 상태 표본이 부족하면 실행을 성공으로 기록하지 않습니다.
환경 정책 파일이 바뀌면 기존 계획을 실행하지 않고 재시작과 새 계획을 요구합니다.
접촉·시계 인터록은 계속 유지합니다.

환경 거리는 움직이는 로봇 형상과 고정된 주변 형상 사이에서 계산합니다. 충돌 마스크와 body 제외 설정을 반영하고,
로봇에 속한 정적 장착 형상은 제외합니다. 로봇 밖의 움직이는 물체, flex, hfield/SDF와 명시적 geom 접촉 쌍은
이 검사에서 지원하지 않아 실행 준비를 거부합니다. 과거 재현 자료가 고정한 기본 거리 계산 소스는 보존하고,
환경 확장은 별도 소스 해시로 검사합니다.

취소한 시연은 전체 동작이 완료된 것으로 기록하지 않습니다.
`physics_verification.native_checks_passed`는 중단할 때까지 실제로 관측한 native 자료의 검사 결과이며,
중단된 실행의 `physics_checks_passed`와 `contact_demo_complete`는 `false`로 남깁니다.
원래 취소 원인, 노드 해제·ROS 종료·소유 launch 종료·최종 집계의 결과를 함께 확인합니다.
과거에 발생한 간헐적인 취소 대기 지연은 최근 재현되지 않았으므로 원인 해결로 처리하지 않습니다.
