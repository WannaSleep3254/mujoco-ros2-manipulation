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

SIGINT·SIGTERM과 MuJoCo 창 닫기는 종료 요청만 남깁니다. 노드는 executor의 새 callback 실행을
취소하고 정상 제어 주기를 중단합니다. 진행 중인 update는 끝날 수 있으며 이후 write는 차단합니다.
이미 시작된 ControllerManager 관리 요청이 남아 있으면, 그 요청이 필요로 하는 read/update만
출력 없이 처리합니다. 모든 executor worker가 끝난 뒤 제어 스레드를 join하고 ROS Context를 종료합니다.
이 추가 처리는 아래의 초기 활성화 중 종료 실패를 확인한 뒤 보완한 동작입니다.

시뮬레이션 시계가 시작되지 않거나 멈춘 경우에도 종료할 수 있도록, 시계 대기는
정지 요청을 확인하는 짧은 실제 시간 대기로 바꿨습니다. 시계가 멈춘 동안 같은 제어 주기를
반복하지 않으며, 시계 역행과 읽기 오류는 명시적인 실패로 처리합니다.
executor가 시작되기 전에 도착한 종료 요청도 놓치지 않도록 취소를 반복합니다.

종료 로그에는 컴파일한 헤더의 SHA256과 다음 순서가 남습니다.

1. `UPDATE_LOOP_JOINED`: 제어 스레드 종료 확인
2. `CONTEXT_SHUTDOWN_BEGIN`: ROS Context 정리 시작
3. ControllerManager의 제어기·하드웨어 정리
4. `CONTEXT_SHUTDOWN_DONE`: Context 정리 완료

기존 종료 순서 수정은 잠긴 upstream에 적용하는 다섯 번째 패치로 보존하고,
관리 요청을 마무리하는 변경은 여섯 번째 패치로 분리했습니다. 새 설치와 기존 네 개·다섯 개
패치가 적용된 소스를 정확히 대조해 갱신하며, 사용자가 수정한 소스는 덮어쓰지 않습니다.

## 최초 수정의 검증과 범위

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

최초 종료 수정 런타임의 시계 정지 검사에서는 실제 CM read/update/write 58주기 후 시계 발행을 멈췄으며,
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

## 초기 활성화 중 종료의 후속 진단

위 SIGSEGV 수정은 PR #59와 #58의 필수 CI를 각각 통과해 main에 반영됐습니다.
이후 같은 소스 트리의 [PR #60 첫 CI](https://github.com/WannaSleep3254/mujoco-ros2-manipulation/actions/runs/40658625725)에서
다른 종료 지연이 관측됐습니다. 실제 JTC update는 반환했지만 제어 스레드 join과 Context 종료가
완료되지 않았고, 상위 launch가 최종 SIGKILL(-9)로 종료했습니다. 당시 joint_state_broadcaster의
활성화 요청은 아직 진행 중이었습니다. 원래 시험은 두 팔의 활성화만 기다렸습니다.

ControllerManager 2.54.3의 관리 요청은 제어기 목록을 교체한 뒤 RT 루프가 이를 확인하기를
기다릴 수 있습니다. RT를 먼저 멈추면 관리 callback은 목록 확인을, executor는 callback 완료를,
소유 스레드는 executor 반환을 기다리게 됩니다. 원래의 정리·update 경쟁과 별개의 대기 문제입니다.

후속 수정은 executor 취소를 확인한 뒤 관리 callback group이 사용 중일 때만 read/update를
처리합니다. ROS 시계가 멈춰 있어도 진행하도록 실제 시간으로 대기하고, ROS 시각을 임의로
전진시키지 않습니다. group이 일시적으로 비어 있어도 executor worker가 모두 끝나기 전에는
RT를 끝내지 않습니다. 아직 서비스가 만들어지지 않은 상태도 명시적으로 구분합니다.

정상 준비 상태의 시험은 broadcaster와 두 팔의 활성화, 세 spawner의 정상 종료를 먼저 확인합니다.
별도 회귀 시험에서는 broadcaster를 비활성 상태로 준비하고, 실제 JTC update를 붙잡은 뒤
실제 활성화 요청을 보냅니다. 요청이 RT 응답을 기다리고 있다는 로그와 미완료 future를 확인한
다음 SIGINT를 보냅니다. 같은 최종 B 시험으로 첫 수정 런타임은 JTC 반환 후에도 8초 이상
응답이 없고 SIGKILL(-9)이 필요했으며, 후속 수정은 활성화 응답 `ok=true`를 받은 뒤 정상 종료했습니다.
후속 수정의 물리 기록은 401스텝, 접촉·비정상 수치 0이며 내부·외부 강제 정리가 없었습니다.

시계 미수신·정지·초기 신호 5조건과 세 제어기가 모두 활성화된 실제 JTC 종료 검사도
후속 바이너리에서 다시 통과했습니다. 첫 후속 실행의 모델 입력 무결성 차단과 시험 도구 경로
오류는 native가 시작되지 않은 사전 실패로 보존했습니다. 재생성한 모델은 입력 metadata만
변경됐고 기존 XML·URDF 바이트가 동일함을 확인했습니다.
[후속 근거 목록](native_shutdown_followup_evidence.json)에 최종 B 입력 일치, 코드·바이너리 해시와
검증 원본을 연결합니다. 최초 수정의 근거 목록과 성능 표본은 변경하지 않습니다.

`write` 차단은 새 물리 명령 적용을 막는다는 뜻입니다. MuJoCo가 이미 받은 제어값으로 즉시
움직임을 멈춘다는 보장은 아닙니다. 현재 고정한 MuJoCo·JTC·joint_state_broadcaster 구성을
검증하며, 임의 플러그인의 callback 내부 정지를 안전하게 중단하는 기능은 아닙니다. 처리가
끝나지 않으면 불완전 진단을 남기고 Context 정리를 강행하지 않습니다. 실제 회귀 fixture는
write 호출 수를 직접 계측하지 않으며, write를 받지 않는 helper와 호출 경로의 단위 검사로
출력 금지를 확인합니다.

## 별도로 확인한 경로 샘플링 한도

[main 병합 후 CI](https://github.com/WannaSleep3254/mujoco-ros2-manipulation/actions/runs/40658567190)의
3D 연동 검사는 내부 출발 시점 비교에서 실패했습니다. 이 실행의 native 종료와 프로세스 정리는
정상이었습니다. 34.022849689초·1,703점의 후보는 경로 형식 범위 안에 있었지만, 균일 검사 시점
3,404개와 원래 경로 시점 1,703개를 합치고 중복 2개를 제외하면 5,105개로 검사 상한 5,000개를 넘었습니다.

샘플 용량 초과만 명시적인 후보 거부로 기록하고, 남은 후보와 요청 예산 안에서 탐색을 계속하도록
보완했습니다. 5,000개 상한, 검사 간격과 거리 기준은 유지합니다. 취소·시간 예산 초과와 실제
평가 오류는 계속 즉시 전파하며, 검사를 생략한 후보는 실행하지 않습니다. 이 실패는 네이티브
종료 문제나 3D 렌더링 오류로 집계하지 않습니다.
