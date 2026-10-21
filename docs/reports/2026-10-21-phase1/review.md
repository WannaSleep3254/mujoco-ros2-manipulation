# 모션 플래닝 세션과 1차 리포트 대조 검토

검토일: 2026-10-21. 원본 기준: `0c31c469f08f853c775222a098706bef3f154bb7`.
비교 자료는 ‘모션 플래닝 질문 정리’ 대화와 그 대화에서 작성한 개선안입니다.
측정 수치를 새로 만들거나 실행 설정을 바꾸지 않고, 서로 다른 소스 기준과 평가 범위를 대조했습니다.

## 판정

MoveIt 2·ros2_control을 유지하고 계측 후 작업별 계획을 확장한다는 방향은 타당합니다.
다만 다른 세션이 검사한 일진 프로젝트의 이전 소스 사본에는 최신 공동 장면·계측 구현이 반영되지 않았습니다.
그 검토의 ‘미구현·미확인’을 현재 원본의 상태로 그대로 적용하면 중복 작업과 잘못된 병목 판단이 생깁니다.

| 쟁점 | 최신 원본의 근거 | 판단 |
|---|---|---|
| 약 2초 계산 | 오프라인 총 중앙값 2.133431초, 출발 시점 탐색 1.890811초 | 두 중앙값의 비율 약 89%. 이 반복 측정은 MoveIt·ROS 서비스·지령을 제외하므로 이들 병목의 증거가 아님 |
| 계획 시간 기록 | [공동 검증기](../../../src/mujoco_ros2_manipulation/workcells/verify.py)는 `plan()`에서 monotonic wall·그룹·오류 기록 | 왕복 시간은 이미 있음. 서버 내부 시간과 지령→최초 이동까지의 단일 요청 계측은 아직 필요 |
| RRTConnect 미확정 | [공동 설정 생성](../../../src/mujoco_ros2_manipulation/workcells/prepare.py)의 팔별·양팔 그룹은 `RRTConnectkConfigDefault`만 정의 | 요청 `planner_id`는 빈 값이나 구성 유형은 `geometric::RRTConnect`. 요청/서버 선택 ID도 기록하도록 보완할 대상 |
| 시도·시간 설정 | 요청 `num_planning_attempts=3`, `allowed_planning_time=5.0` | 현재 기준을 보존하고 단일 시도·짧은 예산을 별도 실험. 허용 시간이 전체 응답의 엄격한 상한은 아님 |
| 바닥 충돌 모델 | [장면 YAML](../../../config/workcells/fr5_ur5e.yaml)의 바닥·받침대를 같은 생성기가 URDF 충돌 링크와 MuJoCo geom으로 만듦 | 최신 공동 장면에 바닥이 없다는 판단은 해당하지 않음. 별도 PlanningScene CollisionObject인 것은 아님 |
| TCP·파지 형상 | 현재 경로 지표의 끝단은 `left_wrist3_link`, `right_tool0` 원점 | 실제 그리퍼 TCP·파지 물체·빈 내부가 구현됐다는 의미는 아님. 빈피킹과 조립에는 추가 모델 필요 |

이 판정은 `fr5_ur5e` 공동 장면 기준입니다. 다른 단일 로봇 장면까지 모두 동일하다고 일반화하지 않습니다.
일진 소스 사본과 원본의 동기화는 별도이며 이번 작업에서는 수행하지 않았습니다.

## 후보 수와 시간의 의미

| 구분 | 현재 값·역할 | 비교 시 주의 |
|---|---|---|
| [출발 지연 후보](../../../src/mujoco_ros2_manipulation/workcells/timing.py) | 이 고정 입력에서 58개 평가, FR5 2.3초 지연 선택 | 이미 생성된 두 경로의 순서·출발 지연. 58개 파지 목표나 OMPL 경로가 아님 |
| OMPL 요청 내부 시도 | 요청당 3회 설정 | 58개 출발 후보와 다른 계층 |
| [공동 경로 후보](../../../src/mujoco_ros2_manipulation/workcells/plan_selection.py) | 안전 기준을 만족할 때까지 최대 12개 제안 | 실제 매번 12개 실행했다는 뜻이 아님 |
| 파지·IK 후보 | 현재 고정 관절 목표 평가에 없음 | 저비용 필터는 향후 빈피킹 설계이며 현재 지연 개선 효과는 미측정 |

4개 MoveIt 서비스 응답은 준비·FR5·UR5e·공동 회피 요청을 한 번씩 관측한 값입니다.
5회 오프라인 반복과 별개의 실행이며, 지령부터 동작까지의 합산 시간으로 사용하지 않습니다.
실행 전체 약 41초에는 동일한 관측 구간과 준비·정리가 포함됩니다.
2.3초 지연은 의도된 로봇 대기이며 CPU 계산 지연이 아닙니다.
관측된 1개 고정 사례의 경로 단축을 랜덤 목표 성공률이나 전역 최적해로 해석하지 않습니다.

## 도구 추천 보완

- 현재 요구인 작은 우회와 빠른 반응에는 우선 기존 출발 시점 탐색을 세분 측정합니다.
  그 다음 동일한 랜덤 목표에서 MoveIt/OMPL 기준선과 Tesseract/TrajOpt를 비교합니다.
  TrajOpt는 초기 경로·비용에 영향을 받는 최적화이며 일정 시간·전역 해를 보장하지 않습니다.
  [공식 안내](https://tesseract-robotics.github.io/tesseract_python/_source/modules/tesseract_motion_planners_trajopt/tesseract_motion_planners_trajopt.html)
- GPU 대안 cuRoboV2는 모델·서로 간 충돌·VRAM·초기 준비 비용을 포함해 별도 평가합니다.
  이 PC에서 성능을 측정하지 않았습니다. [공식 계획 흐름](https://nvlabs.github.io/curobo/latest/getting-started/motion_planning.html)
- MJPC는 MuJoCo 기반 반복 최적제어이며 예측 샘플링·CEM·미분 기반 방법을 포함합니다.
  접촉 작업 실험 후보로 두며 전역 빈피킹 경로 생성기의 즉시 대체안으로 삼지 않습니다.
  [공식 개요](https://github.com/google-deepmind/mujoco_mpc/blob/main/docs/OVERVIEW.md)
- CasADi는 자동미분·수치 최적화·최적제어 구성 도구이며 오프라인 전용이 아닙니다.
  Crocoddyl은 DDP 계열 로봇 접촉 최적제어 라이브러리로 보행 전용이 아닙니다.
  [CasADi](https://web.casadi.org/docs/), [Crocoddyl](https://github.com/loco-3d/crocoddyl)
- Pilz는 지정 PTP/LIN/CIRC 동작, Ruckig는 속도·가속도·저크 거동을 다룹니다.
  임의 장애물을 우회하는 전역 경로와 공통 충돌 검증은 별도입니다.
  [Pilz](https://moveit.picknik.ai/humble/doc/examples/pilz_industrial_motion_planner/pilz_industrial_motion_planner.html), [Ruckig](https://docs.ruckig.com/)

## 다음 구현의 우선순위와 완료 조건

1. **상시 명령·구간 계측:** 같은 요청 ID에 장면 준비·계획 왕복·서버 planning_time·출발 탐색·안전검사·Action 수락·최초 이동·완료를 연결합니다.
   단조 증가 wall 시계와 sim 시계를 분리하고 요청/선택 planner ID, 상태 나이, 실패·취소도 보존합니다.
2. **출발 탐색 비용 분석:** 후보 수·샘플 수·보간·네이티브 FK/거리 호출·Python 경계 비용을 측정한 뒤 모델 재사용·재계산 절감·배치를 실험합니다.
   지연에 따른 충돌 가능성이 단조라고 가정한 이진 탐색은 사용하지 않습니다.
   거친 사전 필터를 쓰더라도 최종 상세 검사와 기존 계획 30mm·관측 20mm 기준·2ms 접촉 감시를 보존합니다.
3. **랜덤 비교:** HTML 06절의 파일럿 후 고정된 사례·seed·시간 예산에서 성공률·실패 지연·경로 길이·최소 간격을 함께 집계합니다.
   반복 사이 시작 상태 복구, cold/warm, GUI/headless를 구분합니다. 짧은 계획 예산의 실패를 통계에서 빼지 않습니다.
4. **작업 특화:** 빈피킹은 빈·TCP·그리퍼·파지 후 attached object·접근/퇴피를 추가합니다.
   조립은 비접촉 접근과 지원되는 힘/순응 제어를 분리하고 FT 피드백·힘 제한·실패 복구를 검증합니다.
5. **실로봇 계약:** 제조사 위치/속도/토크 인터페이스·주기·지터·중단 동작을 확인합니다.
   `use_sim_time` 변경만으로 하드웨어 전환·안전·접촉 품질이 검증되지는 않습니다.

서비스 왕복에서 `planning_time`을 뺀 값은 순수 DDS 비용이 아닙니다.
`longest_valid_segment_fraction`을 0.05로 늘려 검사 지점을 줄이는 최적화도 일괄 적용하지 않습니다.
검사 간격이 커지면 충돌을 놓칠 수 있으므로 해상도·보수적 충돌 형상·최종 검사를 함께 평가합니다.
[MoveIt Humble OMPL 공식 설명](https://moveit.picknik.ai/humble/doc/examples/ompl_interface/ompl_interface_tutorial.html)

측정 원본과 시각 자료는 [리포트 안내](README.md), 대조 결과는 [HTML 08절](phase1-performance.html#review)에 있습니다.
