# 실제 ROS 연동 검사

ROS 2 Humble와 로컬 런타임·모델 생성물이 필요합니다. 일반 단위 검사에서 ROS를 자동 실행하지 않습니다.
CI의 `ros` 작업은 동일 명령을 실제로 실행하며 실패·성공 로그를 모두 업로드합니다.

```bash
source ./manipulation_env.sh
python3 verify_robot.py --robot fr5 --domain 94
python3 verify_robot.py --robot fr10 --domain 95
python3 verify_robot.py --robot ur5e --domain 96
python3 verify_robot.py --robot fr5 --scene pedestal --domain 97
```

GUI와 실제 하드웨어는 CI 범위에 포함하지 않습니다. 새 PC는 잠긴 deb의 배포 중단 등으로
빌드 준비가 실패할 수도 있으며, 이 경우 ROS 검증 성공으로 보고하지 않습니다.

## 네이티브 종료 순서

새 런타임을 빌드한 뒤, 비어 있는 도메인과 새 출력 폴더를 사용합니다.
다음 검사는 시험 전용 interposer로 실행 순서를 관측하거나 지연합니다. 성능 표본으로 사용하지 않습니다.

```bash
source ./manipulation_env.sh
task_native_binary="$PWD/ros_ws/install/mujoco_ros2_control/lib/mujoco_ros2_control/ros2_control_node"
python3 tests/integration/verify_native_shutdown_clock.py \
  --binary "$task_native_binary" --domain 201 --output runtime/validation/shutdown-clock-new
python3 tests/native/shutdown_probe/build_probe.py --output runtime/validation/shutdown-probe-new
python3 tests/integration/verify_native_shutdown_race.py \
  --expect fixed --domain 206 \
  --expected-native-sha256 "$(sha256sum "$task_native_binary" | cut -d' ' -f1)" \
  --probe runtime/validation/shutdown-probe-new/update_probe.so \
  --output runtime/validation/shutdown-race-new
```

첫 검사는 로봇·하드웨어 없이 실제 네이티브 노드의 시계 미수신·정지와 초기 SIGTERM을 확인합니다.
두 번째는 자체 MuJoCo 작업 환경에서 실제 JTC update를 붙잡아 둔 채 자체 native PID에만 SIGINT를
보냅니다. update 반환 → RT join → Context 종료 → 제어기 정리 순서와 전체 물리 기록·프로세스 정리를
요구합니다. 기본적으로 어떤 로봇 목표도 전송하지 않습니다. 설치한 CM 2.54.3/JTC 2.54.1 ABI가
달라지면 검사를 멈추고 interposer를 재검토해야 합니다.

`--expect legacy`는 격리된 구형 런타임의 잘못된 종료 순서를 재현하는 진단 모드입니다.
성공한 안전 실행으로 집계하지 않으며, CI는 `fixed`만 검사합니다.
원인·실패 원본과 검증 범위는 [네이티브 종료 기록](../../docs/native_shutdown.md)에 있습니다.

## FR3 이동 중 취소와 신선한 정지 관측

```bash
source ./manipulation_env.sh
python3 verify_robot_cancel.py --robot fr3 --domain 162 --output runtime/validation/fr3-cancel-new
```

출력은 새 runtime 폴더여야 하며 명시한 도메인을 사용 중이면 거부합니다.
기존 direct·MoveIt·TF·시계 검사 뒤 실제 이동 중 자체 목표만 취소하고, 취소 응답·Action CANCELED·
전진하는 신선한 관절/시계의 정지 구간·새 홈 Action 성공과 정착·native/정리를 각각 확인합니다.
[FR3 문서](../../docs/fr3-model.md)에 실험 프로필과 판정 한계를 기록했습니다.

## TCP Pose 목표

```bash
source ./manipulation_env.sh
python3 -m mujoco_ros2_manipulation.workcells.verify_pose_goal --domain 151
```

기존 장면의 플랜지 원점 참조를 사용합니다. 별도의 Pose 계획 요청, IK/FK 확인,
다른 팔의 전체 spline 유지, 일회 승인한 경로 실행과 관측 TCP 오차를 확인합니다.
잘못된 공구 SHA·도달 불가·검토 후 환경 변경·대기 중 취소에는 목표 전송 0회를 요구합니다.
Action 성공과 TCP 완료 조건을 각각 검사하며, 이 검사는 그리퍼가 있는 ROS 장면을 검증하지 않습니다.
CI의 기존 공동 작업 환경 작업에서 같은 명령을 실행합니다.

FR5·평행 그리퍼·자유 블록의 독립 물리 실행과 실제 3D 화면은
[기본 집기 실험](../../docs/grasping.md)에서 설명합니다. 이 실행은 Python MuJoCo 3.14.0이며
기존 ROS native 3.12.0의 검사와 결과를 합산하지 않습니다.

## 계획 캐시·플래너 비교

기존 고정 suite와 별도 실험이며 기본 플래너와 캐시 설정은 바꾸지 않습니다.
각 명령은 사용하지 않는 도메인과 새 출력 폴더를 지정합니다. 소싱한 ROS의 PYTHONPATH를 덮어쓰지 않습니다.

```bash
source ./manipulation_env.sh
python3 benchmark_planner_cache.py --list
python3 benchmark_planner_cache.py --mode functional --domain 160 --output runtime/validation/cache-functional-new
python3 benchmark_planner_cache.py --mode performance --domain 161 --output runtime/validation/cache-performance-new
```

[실험 규약](../../docs/planner_cache.md)에 고정 입력·계산 예산·시도·실패·미실행과 원본 보존 규칙을 명시합니다.
기능 검사의 실제 입력 5조건과 제어된 주입 5조건을 구분합니다. 성능은 네 조건 각 3회이며,
p95나 범용 우열을 판정하지 않습니다. 각 실제 실행의 native 물리와 소유 프로세스 정리도 통과해야 합니다.

## 공동 작업 환경

공동 작업 환경 검증은 `python3 verify_scene.py --workcell fr5_ur5e --domain 93`을 실행합니다.
`build_workspace.py --workcell fr5_ur5e`로 준비하며 CI에서는 두 번 연속 실행합니다.

공동 작업 환경 검증은 현재 실행 ID·감시 코드 해시를 가진 네이티브 `physics.json`을 필수로 요구합니다.
전체 물리 스텝 수와 일치하지 않거나 접촉이 발견되면 검증이 실패합니다.
CI는 5초 준비 후 기본 10초 성능 구간과 20회 양팔 계획을 포함합니다.
로컬 60초·GPU GUI 검증 절차는 [공동 작업 환경 진단](../../docs/workcell_diagnostics.md)을 참고합니다.

접촉 자동 정지 시험은 `python3 verify_contact_stop.py --domain 111`과
`python3 verify_contact_stop.py --domain 112 --paused-contact`입니다.
검증기가 직접 목표를 취소하지 않고 네이티브 정지·감시 노드의 취소/비활성화를 관찰합니다.
주입 서비스는 시험용 launch에만 있으며, 일반 검증은 해당 서비스가 노출되지 않는지도 확인합니다.

핵심 프로세스 크래시의 전체 종료 검증은 각각 새 launch로 실행합니다.

```bash
python3 verify_process_crash.py --target move_group --domain 115
python3 verify_process_crash.py --target ros2_control_node --domain 116
```

CI의 공동 작업 환경 작업에도 같은 명령을 포함합니다. 이 시험은 실행에서 소유한 프로세스 하나에만
SIGKILL을 주입합니다. 정상 동작·접촉 정지의 물리 커버리지 검사를 대체하지 않습니다.
자세한 성공 조건과 미검증 범위는 [크래시 검증](../../docs/process_crash.md)에 있습니다.

시계 실패 주입은 새 launch 두 번으로 확인합니다.

```bash
python3 verify_clock_stop.py --mode missing --domain 119
python3 verify_clock_stop.py --mode repeated --domain 120
```

각 실행에 1.5초 의도한 일시정지·복귀, 양팔 이동, 시계 오류 관측, 물리 고정·양팔 목표 취소와
제어기 비활성화·새 명령/리셋/재활성화 차단 검사를 포함합니다.
CI의 공동 작업 환경 작업도 동일 명령을 실행합니다. [범위·판정](../../docs/clock_stop.md)을 참고합니다.

전용 양팔 간섭 회피 사례는 다음을 새 launch에서 두 번 실행합니다.

```bash
python3 verify_interference.py --domain 126
python3 verify_interference.py --domain 127
```

양팔 시작·목표는 안전하고 팔별 계획도 상대 팔 고정 시 안전해야 합니다.
합성 궤적의 교차 간섭을 전송 전에 차단한 뒤, 같은 조건의 `both_arms` 계획만 실행합니다.
두 Action 성공·실제 목표 수렴·네이티브 전체 스텝 접촉 0까지 검사합니다.
[엄격한 후처리 설정과 재현 자료](../../docs/interference_avoidance.md)를 참고합니다.

## 고정 홈 복귀 입력과 실패 경로 재현

```bash
source ./manipulation_env.sh
python3 reproduce_motion.py --mode witness
python3 reproduce_motion.py --mode live --domain 154
```

과거 기하 표본은 고정 자산·정책으로 재평가하며 새 ROS 요청의 경로는 별도로 저장합니다.
기존 사용자 시뮬레이션과 다른 빈 도메인을 사용합니다. 새 경로가 통과하는 경우도 결과에 보존하며, 과거와 같은 실패를 강제하지 않습니다.
[입력·완료 기준·한계](../../docs/reproductions/home-return-2026-10-23/README.md)를 함께 확인합니다.


웹 뷰어의 실제 비교 시연과 중단·단독 소유권은 다음 명령으로 검사합니다.

```bash
python3 verify_operator_demos.py --domain 156 --port 8879 --no-render
```

고정 장애물/무장애물의 동일 명령, 새 공동 계획에서 만든 출발 조정 비교, 접촉 시연 중단과
중첩 launch의 종료를 실제 HTTP·ROS 경로로 확인합니다. CI는 같은 검사를 도메인 155에서 실행합니다.
로컬에서 `--no-render`를 빼면 각 새 실행의 job·run·모델 해시에 맞는 최신 뷰어 프레임도 요구합니다.
원본 결과·계획·물리 기록은 `runtime/validation`과 해당 콘솔 세션에 보존하며 소개 사이트에 자동 게시하지 않습니다.


## 일반 요청의 공간 후보 재선택

```bash
source ./manipulation_env.sh
python3 verify_motion_replanning.py --domain 159
```

실제 명령 토픽과 MuJoCo 양팔 실행을 사용해 정상 계획, 시험용 실패 응답 뒤의 같은 요청 경로 재선택,
후보 1개 상한, 후보 계산 중 취소 토픽 수신, 0.01 wall초 계산 예산, 이후 정상 복구를 확인합니다.
시험용 계획 응답과 실제 MoveIt 호출을 구분해 기록하며 성공률·속도 평가로 사용하지 않습니다.
실패·취소에는 전송 수 0과 최초 이동 null을, 성공에는 양팔 Action 완료와 최종 검사·전체 물리 스텝 접촉 0을 요구합니다.
현재 사용 중인 도메인이면 실행을 거부합니다. 자세한 동작과 기록 항목은 [명령 안내](../../docs/motion_requests.md#일반-요청의-제한된-공간-재계획)에 있습니다.

최종 Action 전송 직전 시험용 취소 플래그를 설정하는 회귀 사례도 포함합니다.
이미 검사를 통과한 경로라도 첫 비동기 전송이 차단되고, 두 팔이 정지하며, 다음 정상 요청이 실행되는지 확인합니다.
이 플래그 주입은 OS 신호나 취소 토픽의 전달 지연을 측정하지 않습니다.


## 고정 입력의 후보 수 비교

```bash
source ./manipulation_env.sh
# 기존 단일 모델 생성 후, 별도 기둥이 있는 작업 환경만 추가 생성합니다.
PYTHONPATH=src .venv/bin/python prepare_workcell.py --workcell fr5_ur5e_eval_post
python3 benchmark_fixed_motion.py --list
python3 benchmark_fixed_motion.py --repeats 3 --domain 171
```

[고정 사례](../../config/evaluations/fixed_motion_v1.json)는 근거리, 기존 랜덤 목표 5개,
양팔 교차, 홈 복귀, 환경 기둥, 잘못된 목표의 10개입니다. 후보 1개와 최대 12개를
같은 5초 계산 예산으로 비교하며, 순서를 번갈아 3회 반복합니다. 각 행은 새 시뮬레이션과
별도 빈 ROS 도메인을 사용합니다. 기본 실행은 도메인 171~230을 사용하며, 사용 중이면 중단합니다.

준비·워밍업·실제 시작 자세 보정·끝점 사전 검사는 채점 시간에서 제외합니다.
보정은 정상 명령 경로로 0.03 rad 이하, 최대 3회만 수행하며 실제 시작 오차는 0.0015 rad 이내여야 합니다.
기둥은 URDF와 MJCF에 함께 포함합니다. 실제 MoveIt Planning Scene에서 시작·목표의 유효성과
직선 관절 보간의 기둥 충돌을 먼저 확인합니다. 이는 회피 경로의 존재나 최단 경로를 보장하지 않습니다.

정상적인 후보 소진·예산 소진은 실패 결과로 보존하고 다음 행을 평가합니다. 준비·전송·실행·물리·정리 오류는
평가를 중단하며, 남은 행은 미실행으로 남깁니다. 잘못된 목표는 동작 성공률과 별도로 집계합니다.
부분 경로에서 발견한 최소 거리를 전체 경로의 최솟값으로 취급하지 않습니다.
`--cases local_small environment_post invalid_goal --repeats 1`은 CI 확인용 부분 실행이며,
전체 사례와 미실행 행도 결과에 보존합니다. 원본은 `runtime/validation/fixed-suites`에 저장합니다.


측정이 종료된 뒤 `scripts/export_fixed_motion.py --source <fixed-benchmark.json> --output <새-evidence.json.gz>`로
원본 bytes와 전체 요약을 다시 대조한 공개 근거를 만들 수 있습니다. 출력 파일이 이미 있으면 덮어쓰지 않습니다.
세션의 구현 SHA·모델·환경이 행 사이에 달라졌거나 원본이 누락되면 추출을 거부합니다.
`scripts/import_fixed_motion_report.py --evidence <공개-evidence.json.gz> --output <새-리포트-디렉터리>`는
공개 근거만 읽어 예비 리포트를 만듭니다. 원본 경로·프로세스 환경·식별값·원문 오류는 공개하지 않습니다.

2026-10-23의 [60행 예비 평가](../../docs/reports/2026-10-23-fixed-motion/index.html)는 성공 47회,
계획 실패 7회, 의도한 무효 목표 거부 6회를 포함합니다. 성공은 모두 첫 후보였으며 홈 복귀는 6회 모두
Action 전송 없이 거부됐습니다. 후보 상한 증가에 의한 회복 성공이나 홈 복귀 해결을 주장하지 않습니다.
보존된 공개 근거와 보고서의 일치는 다음 명령으로 확인합니다.

```bash
PYTHONPATH=src runtime/report-tools/bin/python scripts/import_fixed_motion_report.py --check docs/reports/2026-10-23-fixed-motion
```


## 로컬 3D 표시와 읽기 전용 미리보기

```bash
source ./manipulation_env.sh
python3 verify_operator_views.py --domain 231 --port 8890 --with-demos
```

별도 빈 도메인과 포트에서 콘솔을 시작해 준비된 FR5·FR10·UR5e 모델, 실제 ROS 관절
상태, 이 콘솔에서 완료한 요청의 저장 계획을 확인합니다. 모델이나 계획을 미리보는 동안
명령·시뮬레이션 시작·시연 시작을 거부하고, 계획 재생이 실제 관절을 움직이지 않는지도
확인합니다. CPU 3D 표시 worker는 물리를 진행하거나 ROS 명령을 보내지 않습니다.
정리 검사는 이 실행에서 소유한 콘솔과 worker만 대상으로 합니다.

이 검사는 WebGL 브라우저 검사가 아닙니다. 브라우저에서는 드래그 회전, 확대·이동,
모델 전환, 저장 경로의 재생·스크럽, 오래된 상태 표시와 반응형 배치를 따로 확인합니다.
`tests/js/*.mjs`는 GPU를 만들지 않고 식별자·형상·카메라·비동기 UI 경계를 검사합니다.

기본 표시는 로컬 3D입니다. 기존 `verify_operator_console.py`와 `verify_operator_demos.py`는
`--no-render`가 없으면 서버 이미지 방식을 명시적으로 선택해 JPEG 호환 동작을 검사합니다.
`--no-render`는 두 표시 worker를 모두 끄며 기존 ROS 실행·취소 검사를 유지합니다.

`--with-demos`를 추가하면 접촉 비교 2회와 경로·출발 시점 비교 3회의 새 실행에서도 모델·실행 식별자, 진행하는 실제 자세, 종료 후 마지막 자세와 증가하는 수신 나이를 확인합니다. CI도 이 옵션을 사용합니다.


## 로컬 설계환경의 기하와 읽기 전용 미리보기

```bash
.venv/bin/python verify_environment_overlay.py --native --catalog-output runtime/validation/environment-preview-catalog.json
source ./manipulation_env.sh
python3 verify_environment_views.py --catalog runtime/validation/environment-preview-catalog.json --domain 148 --port 8895
```

첫 명령은 합성 메시 fixture로 새 작업 환경을 만들고 단위·축·설치, 두 형식의 변조 거부와 native 3.12 기하를 검사합니다.
로컬에 FreeCADCmd가 설치돼 있으면 `--with-freecad`로 직접 만든 STEP의 실제 변환과 단위 오류 거부도 추가합니다.
둘째 명령은 빈 도메인의 소유 콘솔에서 미리보기·명령 차단·기본 작업 환경 불변과 프로세스 정리를 확인합니다.
설비가 포함된 ROS 실행·환경 여유 거리 검증을 수행하는 명령은 아닙니다. 결과·메시는 `runtime/`에만 보존합니다.
같은 catalog 출력은 덮어쓰지 않으므로 재실행에는 새 경로를 지정합니다. [자세한 절차](../../docs/environment_design_import.md)를 참고합니다.
