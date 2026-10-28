# 작업·계획·실행 관측 규약

`operation-observation-v1`은 기존 요청 기록을 화면과 분석 도구가 읽기 위한 추가 규약입니다. [순수 변환 함수](../src/mujoco_ros2_manipulation/workcells/operation_contract.py)의 `describe_operation(record)`가 원래 단계와 요청 식별자를 보존하며 관측 단계를 정리합니다. 명령 수락, 계획 승인, 궤적 전송이나 취소를 수행하지 않습니다. 기존 v1 명령의 네 필드와 검사는 그대로 사용합니다.

## 관측 단계

| 단계 | 현재 원본 단계의 예 | 의미 |
| --- | --- | --- |
| `admission` | `received`, `state_snapshot` | 요청 수신과 시작 상태 확인입니다. 실행 허가를 뜻하지 않습니다. |
| `planning` | `pose_ik`, `planning_both_arms`, `departure_search`, `safety_checks` | IK, 경로 생성과 검사입니다. 실제 로봇 이동이 아닙니다. |
| `review` | `plan_ready`, `approved_plan_recheck` | 가상 경로 검토 또는 승인한 계획의 재검사입니다. |
| `dispatch` | `final_state_recheck`, `goal_sent`, `goal_acceptance` | 전송 준비, 전송과 수락 관측입니다. 관절 이동 확인과 구분합니다. |
| `executing` | `first_motion`, `goal_result`, `actions_done` | 관절 이동 또는 실행 응답 관측입니다. 응답 완료가 정착 완료를 뜻하지 않습니다. |
| `settling` | `settling`, `settled` | 관절 정착 확인 단계입니다. 최종 요청 결과는 별도로 봅니다. |
| `terminal` | `terminal`, `plan_not_required` | 이 요청의 종료 단계입니다. 프로세스 종료나 물리적 정지와 다릅니다. |
| `unknown` | `cancel_received`, 미등록 단계 | 취소 수신만으로 중단한 작업 단계를 알 수 없습니다. 새 단계도 이름의 일부로 추정하지 않습니다. |

명시적인 `stage`가 있으면 사용하고, 없으면 제공한 이벤트의 마지막 이름을 사용합니다. `source_stage`는 원본을 보존합니다. 종료 상태를 받았더라도 미등록 단계를 `terminal`로 바꾸지 않습니다. `request_terminal`은 `succeeded`, `failed`, `cancelled`, `rejected`만 구분합니다. `unknown`은 종료 결과를 확인하지 못한 상태입니다.

`mode`는 기존 `automatic`, `plan_only`, `approved_plan`을 구분합니다. 생략된 옛 기록은 `automatic`으로 표시하되 `mode_origin=legacy_default`를 남깁니다. 새 모드는 원문과 `mode_known=false`를 함께 전달합니다. 계획 요청과 승인 실행 요청은 각각의 ID로 연결합니다. 실행 요청 ID가 있다는 이유만으로 궤적 전송이나 이동을 판정하지 않습니다.

기존 v1 실행 명령은 완전한 이름 기반 관절 목표입니다. TCP Pose는 별도 plan-only 요청으로 받아 등록한 기준점과 IK/FK를 확인한 뒤 관절 경로로 계획합니다. 원문·TCP 정의·정확한 궤적을 검토 digest에 묶고, 승인 뒤에는 IK를 다시 계산하지 않습니다. `coordinated`와 `departure_timing`은 계획 전략이며 실행 모드와 별개입니다. 향후 장치 명령, 캐시나 Servo를 추가하더라도 이 분류를 혼합하지 않습니다.

## 시간과 취소

이벤트의 `wall_sec`는 기록 프로세스의 monotonic 경과 시간이며 `simulation_sec`는 ROS 시뮬레이션 시각입니다. 두 값을 빼서 지연으로 해석하지 않습니다. 원천 관절 stamp와 수신 나이도 별도 필드입니다. 보존한 수신 나이는 기록 당시의 값이며 현재 상태의 신선도를 보증하지 않습니다.

이벤트는 마지막 128개만 투영하고 전체 개수와 생략한 앞부분의 개수를 표시합니다. 반복 조회하는 명령 상태 응답에는 가장 최근 이벤트 한 개만 포함하며, 전체 원본 이력은 trace에 남깁니다. 이력 자체가 없는 요약은 `history_provided=false`입니다. 취소 수신이 보이지 않더라도 이력이 없거나 일부를 생략했다면 `server_acknowledged=null`로 남깁니다. 이 규약은 원본 기록을 대체하지 않습니다.

취소의 전송 중, 전송 결과 미확인, 서버 수신과 요청의 취소 종료를 구분합니다. 이 상태들은 물리적 정지를 뜻하지 않습니다. 전달한 감시 보고의 latch, 단계와 사유는 별도 `physical_guard`에 투영합니다. 현재 알려진 사유인 `contact` 또는 `clock_stall`에서 `latched=true`, `phase=stopped`일 때만 `reported_stopped`로 표시합니다. 이는 감시 보고의 내용이며 실물 안전 인증이 아닙니다.

소유 프로세스의 `cleanup_clean`, `shutdown_clean`, `process_group_gone`도 별도로 보존합니다. 요청의 성공·실패·취소와 프로세스 정리는 서로 대체할 수 없습니다. 미제공 값은 null이며 성공으로 추정하지 않습니다.

## 데이터와 실행 책임

관절 목표와 상태는 이름으로 대응합니다. 전송용 배열은 제어기별 관절 순서와 정확히 맞아야 하며, 현재 회전 관절의 위치·속도는 rad·rad/s입니다. 계획의 상대 시간과 ROS 예약 시작 시각을 구분합니다. 새 직선 관절에는 m·m/s와 관절 종류를 명시해야 합니다.

표시 형상은 현재 공통 표시 규약의 world, 오른손 좌표계, Z-up과 m 단위를 사용합니다. 표시용 가상 자세, 검토 경로와 실제 수신 상태를 혼합하지 않습니다. 모델·환경·관절 대응·제어 모드·시계 영역·수신 신선도의 실제 검사는 기존 명령 서버와 어댑터가 담당합니다. 이 관측 변환 함수는 그 검사를 통과시킬 권한이 없습니다.

계획 요청은 [MoveItPlanningAdapter](../src/mujoco_ros2_manipulation/workcells/planning_adapter.py)가 전달하고, 실행은 [TrajectoryExecutionAdapter](../src/mujoco_ros2_manipulation/workcells/execution_adapter.py)가 요청별 `ExecutionLedger`로 관리합니다. 명령 서버는 요청 수락, 상태·거리 검사, 계획 승인과 재검사를 담당합니다. 계획 어댑터는 전달받은 가상 시작 상태를 실시간 상태로 교체하지 않으며, 실행 어댑터는 전달받은 궤적을 변경하지 않습니다.

현재 MoveIt 요청 대기 25초, 각 팔의 목표 수락 5초, 실행 결과 대기 45초, 취소 응답과 취소 결과의 각 5초 대기를 보존합니다. 이 값은 현재 구현의 대기 한도이며 전체 작업의 최대 지연 보장이 아닙니다. 계산 예산, 상태 신선도 검사, 승인 유효 시간과 소유 프로세스 종료 한도도 별도 책임입니다.

`execution_transport`는 이번 요청의 전송 진입, 반환받은 future, 목표 수락, 확인한 목표 UUID, 취소 응답과 action 종료를 최대 32개 목표에 대해 기록합니다. 누적 전송 수와 이번 요청의 전송 수는 별도입니다. `goal_sent` 이벤트는 기존 기록과 호환되는 전송 준비 이벤트이므로 실제 전송 성공을 증명하지 않습니다. `submission_started`와 `sent`를 함께 봅니다.

전송 호출이 예외를 내거나 future를 반환하지 않으면 실제 전송 여부가 불명확할 수 있습니다. 이때 재전송이나 다른 목표 탐색을 하지 않고 `cleanup_uncertain=true`로 남겨 서버를 차단합니다. 늦게 수락된 목표는 해당 요청의 ledger에 소유권을 유지하고 비동기 취소합니다. 취소 응답만 받았거나 정상적인 action 종료를 확인하지 못했다면 정리 완료로 바꾸지 않습니다. 이 상태도 물리 감시의 정지와 별개입니다. 진단 저장에서는 비정상 future를 다시 읽지 않아 최초 오류를 보존합니다.

[관측 규약 검사](../tests/unit/test_operation_contract.py)는 명시적 단계 대응, 원본 단계 보존, ID 연결, 시계 구분, 취소와 정지·정리의 분리, 이력 크기 제한과 입력 불변성을 확인합니다. 실제 계획·실행·취소와 자원 정리는 기존 ROS 회귀 검증을 함께 사용합니다.

[공통 모듈 경계](module_boundaries.md) · [명령과 시간 기록](motion_requests.md) · [공통 표시와 계획 검토](viewer_sources.md) · [작업리스트](development_worklist.md)
