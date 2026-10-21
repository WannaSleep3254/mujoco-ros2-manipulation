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
- DAE 장면 변환·단위·색상·법선, 세 자세의 UR5e 메시 배치와 물리 모델 보존.

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
`verify_robot.py --scene pedestal`은 기본 모델 검증을 덮어쓰지 않고 장면별 보고서를 생성합니다.
실제 MoveIt의 robot_description 충돌 링크와 장면의 고정 TF도 확인합니다.
검증 조합과 CI 실행 경계는 [구조 설명](architecture.md#ci와-검증-경계)에 있습니다.

2026-10-19 구조 개편 후에는 **44개 통과·실패 0개·건너뜀 0개**,
세 모델 기본 장면과 FR5 받침대 장면의 실제 ROS 연동·충돌 모델·TF·종료를 확인했습니다.
[구조 개편 검증 기록](structure_validation.json)에 코드 해시·실행 ID와 미실행 항목을 보관합니다.

공동 작업장면 확장 후 **56개 통과·실패 0개·건너뜀 0개**를 확인했습니다.
기존 4개 모델·장면 ROS 실행과 공동 장면 2회 연속 실행도 통과했습니다.
[공동 장면 검증 기록](workcell_validation.json)에 좌우 격리·공동 계획·충돌 거부·상호 취소·종료와 남은 검사를 구분했습니다.

## 접촉 원인 식별을 위한 무장애물 대조 검증

[11초 비교 영상과 재현 명령](contact_comparison.md)은 같은 8초 왕복 명령을 두 독립 실행에 적용합니다.
접촉 실행은 모션 약 2.012초에 양팔이 취소·고정되고, 무장애물 대조군은 남은 약 5.988초 동안 계속 이동·복귀하여 양팔 SUCCEEDED로 끝납니다.
명령 해시·모델이 같고 두 실행 모두 자동 정지 활성화를 유지합니다. 양팔 동시 정지 정책은 변경하지 않았습니다.
전체 검사 82개(실패·건너뜀 0개), 두 조건의 실제 ROS headless·GUI와 비교 게이트가 통과했습니다.
[비교 기록·원본 영상·보고서·해시](contact_comparison.md#검증과-ci)를 함께 보관합니다.
이전 2차 시연 `d515e41`의 ROS 5조합·Python CI는 6개 모두 통과했으며, 원본 공동 장면 아티팩트까지 확인했습니다.
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
[세션 대조](reports/2026-10-21-phase1/review.md)는 이전 소스 사본과 최신 공동 장면의 차이,
후보 유형·지연 의미·접촉 제어 도구 및 다음 평가 순서를 정리합니다.
이번 반영은 보고서·문서만 변경하며 로봇 실행 검사를 새로 통과한 것으로 기록하지 않습니다.


## 공통 리포트 검증과 목록

[생성·게시 안내](reporting.md)의 스키마로 단위·wall/sim 시계·표본·미측정 값을 검증합니다.
[리포트 목록](reports/index.html)과 CSV·파일 해시는 동일 입력에서 생성하며,
`python scripts/reports.py build --check`는 출력의 누락·변경을 쓰기 없이 확인합니다.
새 단위 검사는 자료 해시 변경·잘못된 표본·경로 이탈·잘못된 링크·출력 폴더 보존을 확인합니다.
발행 리포트 생성은 ROS 동작을 다시 실행하거나 새 성능을 측정하는 과정이 아닙니다.

## 랜덤 목표·계산 예산 회귀 검사

[랜덤 평가 절차](random_motion_benchmark.md)에 같은 목표·실측 시작 기준·전략 순서 교대와 원본 검증을 정리했습니다. CI 공동 장면에서는 근거리 목표 2개의 두 전략 실행과 0.01wall초 예산 소진 시 이동 지령 금지를 별도 새 launch에서 검사합니다. 계획 실패와 평가 실행 실패를 구분하며, 접촉·복귀 실패·시작 자세 불일치는 평가를 중단합니다.
