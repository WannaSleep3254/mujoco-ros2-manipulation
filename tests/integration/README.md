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
