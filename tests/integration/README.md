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
