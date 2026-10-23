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

공동 장면 검증은 `python3 verify_scene.py --workcell fr5_ur5e --domain 93`을 실행합니다.
`build_workspace.py --workcell fr5_ur5e`로 준비하며 CI에서는 두 번 연속 실행합니다.

공동 장면 검증은 현재 실행 ID·감시 코드 해시를 가진 네이티브 `physics.json`을 필수로 요구합니다.
전체 물리 스텝 수와 일치하지 않거나 접촉이 발견되면 검증이 실패합니다.
CI는 5초 준비 후 기본 10초 성능 구간과 20회 양팔 계획을 포함합니다.
로컬 60초·GPU GUI 검증 절차는 [공동 장면 진단](../../docs/workcell_diagnostics.md)을 참고합니다.

접촉 자동 정지 시험은 `python3 verify_contact_stop.py --domain 111`과
`python3 verify_contact_stop.py --domain 112 --paused-contact`입니다.
검증기가 직접 목표를 취소하지 않고 네이티브 정지·감시 노드의 취소/비활성화를 관찰합니다.
주입 서비스는 시험용 launch에만 있으며, 일반 검증은 해당 서비스가 노출되지 않는지도 확인합니다.

핵심 프로세스 크래시의 전체 종료 검증은 각각 새 launch로 실행합니다.

```bash
python3 verify_process_crash.py --target move_group --domain 115
python3 verify_process_crash.py --target ros2_control_node --domain 116
```

CI의 공동 장면 작업에도 같은 명령을 포함합니다. 이 시험은 실행에서 소유한 프로세스 하나에만
SIGKILL을 주입합니다. 정상 동작·접촉 정지의 물리 커버리지 검사를 대체하지 않습니다.
자세한 성공 조건과 미검증 범위는 [크래시 검증](../../docs/process_crash.md)에 있습니다.

시계 실패 주입은 새 launch 두 번으로 확인합니다.

```bash
python3 verify_clock_stop.py --mode missing --domain 119
python3 verify_clock_stop.py --mode repeated --domain 120
```

각 실행에 1.5초 의도한 일시정지·복귀, 양팔 이동, 시계 오류 관측, 물리 고정·양팔 목표 취소와
제어기 비활성화·새 명령/리셋/재활성화 차단 검사를 포함합니다.
CI의 공동 장면 작업도 동일 명령을 실행합니다. [범위·판정](../../docs/clock_stop.md)을 참고합니다.

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
# 기존 단일 모델 생성 후, 별도 기둥 장면만 추가 생성합니다.
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
기둥은 URDF와 MJCF에 함께 포함합니다. 실제 MoveIt 장면에서 시작·목표의 유효성과
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
