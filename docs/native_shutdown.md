# 네이티브 종료 순서 수정

PR #58의 첫 CI에서 `environment_post / single` 요청은 판정을 마쳤지만, SIGINT로
시뮬레이션을 정리하던 중 `ros2_control_node`가 SIGSEGV로 종료됐습니다.
종료 진단 파일도 완성되지 않아 해당 실행을 실패로 유지했습니다.
[원본 CI](https://github.com/WannaSleep3254/mujoco-ros2-manipulation/actions/runs/40644539205)의
실행 ID는 `20261028T064051.774229Z-70b01253`입니다.

## 원인

기존 네이티브 노드는 ROS Context가 종료된 뒤 제어 스레드를 join했습니다.
그런데 ControllerManager는 Context의 **pre-shutdown** 단계에서 제어기를 비활성화하고
내부 상태를 해제합니다. 이 단계에서는 `rclcpp::ok()`가 아직 참일 수 있으므로,
제어 스레드의 활성 상태 확인과 실제 `update()` 사이에 정리가 끼어들 수 있었습니다.

- controller_manager 2.54.3은 pre-shutdown에서 `shutdown_controllers()`를 호출합니다.
  활성 제어기의 `deactivate()`와 `release_interfaces()`가 실행됩니다.
- joint_trajectory_controller 2.54.1의 `on_deactivate()`는 관절 인터페이스와
  `traj_external_point_ptr_`를 해제합니다. 동시에 진행 중인 `update()`는 이 상태를 사용합니다.
- 실패 스택은 네이티브 제어 루프 → ControllerManager update → JointTrajectoryController
  update로 이어졌습니다. 정리 로그와 RT 스택을 함께 보존했습니다.

확인한 공식 소스는
[ControllerManager 2.54.3](https://github.com/ros-controls/ros2_control/blob/2.54.3/controller_manager/src/controller_manager.cpp)과
[JointTrajectoryController 2.54.1](https://github.com/ros-controls/ros2_controllers/blob/2.54.1/joint_trajectory_controller/src/joint_trajectory_controller.cpp)입니다.
로컬과 CI의 rclcpp는 16.0.21입니다. 라이브러리가 메모리에서 내려가지 않게 하는 기존
MoveIt 플러그인 유지 장치는 제어기 내부 상태의 해제를 막지 않으므로 이 경쟁을 해결하지 못합니다.

## 변경 동작

SIGINT·SIGTERM과 MuJoCo 창 닫기는 정지 요청만 남깁니다. 노드의 소유 스레드는
executor를 취소하고 제어 스레드가 끝날 때까지 기다린 다음 ROS Context를 종료합니다.
진행 중인 update는 끝날 수 있지만, 정지 요청 이후 다음 read/update/write 단계로 넘어가지 않습니다.

시뮬레이션 시계가 시작되지 않거나 멈춘 경우에도 종료할 수 있도록, 시계 대기는
정지 요청을 확인하는 짧은 실제 시간 대기로 바꿨습니다. 시계가 멈춘 동안 같은 제어 주기를
반복하지 않으며, 시계 역행과 읽기 오류는 명시적인 실패로 처리합니다.
executor가 시작되기 전에 도착한 종료 요청도 놓치지 않도록 취소를 반복합니다.

종료 로그에는 컴파일한 헤더의 SHA256과 다음 순서가 남습니다.

1. `UPDATE_LOOP_JOINED`: 제어 스레드 종료 확인
2. `CONTEXT_SHUTDOWN_BEGIN`: ROS Context 정리 시작
3. ControllerManager의 제어기·하드웨어 정리
4. `CONTEXT_SHUTDOWN_DONE`: Context 정리 완료

새 패치는 잠긴 upstream 소스에 적용하는 다섯 번째 패치입니다. 새 설치뿐 아니라
기존 네 개 패치가 적용된 소스도 같은 결과로 갱신하며, 사용자가 수정한 소스는 덮어쓰지 않습니다.

## 검증과 범위

수정 전 같은 조건의 로컬 6회가 통과했지만, 이것으로 간헐 CI 실패를 무효화하지 않았습니다.
고정한 실제 JTC의 가상 update 호출을 시험 전용 interposer로 붙잡아 둔 뒤,
자체 소유 native PID에만 SIGINT를 보내 두 순서를 비교했습니다.

| 검사 | 확인한 결과 |
|---|---|
| 원본 CI | 종료 중 SIGSEGV(-11), native 최종 물리 파일 누락. 실패 원본 유지 |
| 구형 런타임의 강제 일정 | 실제 JTC 호출을 붙잡은 동안 제어기 정리가 시작됨. 최종 b 실행에서 반환 없이 SIGSEGV(-11)와 원본과 같은 JTC update null 접근 스택 재현 |
| 수정 런타임의 같은 일정 | JTC 반환 → RT join → Context 종료 → 제어기 정리. native 물리 검사와 소유 프로세스 정리 통과 |
| 수정 후 `environment_post` 단일·제한 후보 각 3회 | 6회 모두 동작·물리·종료 검사와 새 종료 로그 순서 통과 |
| 실제 무하드웨어 노드의 시계·신호 5조건 | 시계 미수신/정지 각각 SIGINT·SIGTERM, 핸들러 설치 직후 SIGTERM 모두 정상 종료 |
| GUI 종료 helper | 실제 rclcpp 기반 4개 gtest 통과. 실제 창을 닫는 GUI 시험은 아님 |

시계 정지 검사에서는 실제 CM read/update/write 58주기 후 시계 발행을 멈췄으며,
추가 0.300초 관측 창에서 카운터가 증가하지 않았습니다. 제어기·로봇이 없는 이 검사와
위 JTC·MuJoCo 검사는 별도로 판정합니다. interposer는 타이밍에 영향을 주므로 모든 주입 결과를
성능 표본에서 제외합니다. 과거 W08 성능 수치는 새 네이티브 코드의 성능 측정으로 바꾸지 않습니다.

시험 장치의 첫 a 실행은 종료 순서 겹침을 관측했지만 SIGSEGV는 재발하지 않았습니다.
이후 이벤트를 검증한 PID·스레드·실행 ID에 직접 연결하고, 진입 기록을 원자적으로 게시하도록
시험 장치를 보완했습니다. 엄격한 새 시각 규약을 만족하지 못한 a 기록은 원본과 재검토 결과를
보존했고, 최종 판정에는 같은 b 장치로 실행한 구형·수정 런타임 한 쌍을 사용했습니다.

[근거 목록](native_shutdown_evidence.json)에 코드·바이너리·결과 해시와 로컬 원본 위치를 기록합니다.
새 CI는 시계 5조건과 실제 JTC의 수정 순서를 필수 공동 작업 환경 검사에 포함합니다.
재현 명령은 [실제 ROS 검사 안내](../tests/integration/README.md#네이티브-종료-순서)에 있습니다.

이번 수정은 위 CI의 **제어기 정리와 update 경쟁**을 다룹니다. 과거 W01의 간헐적인
취소 후 정리 지연은 같은 원인으로 확정하지 않았으며 열린 진단 항목입니다.
외부 플러그인이 직접 Context를 종료하는 추가 경로, 잘못된 모델 토픽 입력으로 초기화가
끝나지 않는 경우, 실물 로봇의 안전 정지는 이번 수정의 검증 범위에 포함하지 않습니다.
