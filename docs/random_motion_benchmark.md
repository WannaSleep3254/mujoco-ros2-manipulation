# 랜덤 관절 목표와 출발 탐색 비용 비교

FR5·UR5e 공동 시뮬레이션을 한 번 실행하고, 저장된 같은 관절 목표를 두 전략으로 반복합니다.
`coordinated`는 양팔 공동 MoveIt 계획, `departure_timing`은 팔별 계획 후 출발 지연 탐색입니다.
새 로봇 모델·작업장·플래너를 추가하는 성능평가의 첫 입력 규약입니다.

## 실행과 재현

```bash
source ./manipulation_env.sh
python3 benchmark_motion.py --seed 20261021 --count 5 --radius 0.25 --compute-budget 5 --domain 139
```

CLI `manipulation-benchmark-motion`도 제공합니다. 기존 명령 노드를 띄운 도메인과 구분합니다.
새 MuJoCo·MoveIt 실행과 명령 노드를 자동으로 준비하고 검증 후 정상 종료합니다.
`runtime/validation/runs/<실행 ID>/result.json`과 `runtime/validation/random-benchmarks/<세션>/`에 결과를 보존합니다.

| 입력 | 의미 |
|---|---|
| seed | Python `random.Random`에 전달하는 uint32 |
| count | 목표 1~200개. 각 목표를 두 전략에 적용 |
| radius | 명목 홈 ±0.05~1rad와 관절 한계의 교집합에서 관절별 균등 표본 |
| compute-budget | 두 전략에 같은 0.01~60wall초 계획·검증 예산 적용 |
| workload | 이전 `workload.json` 재사용. seed/count/radius와 함께 지정하지 않음 |

```bash
python3 benchmark_motion.py --workload runtime/validation/random-benchmarks/세션/workload.json \
  --compute-budget 5 --domain 139
```

충돌 목표나 계획 실패를 미리 제거하지 않습니다. 입력 파일은 목표 전체, 모델·관절 한계,
명목 홈, seed, 실행 순서와 해시를 포함합니다. 모델·입력·순서를 바꾼 파일은 같은 입력으로 재생하지 않습니다.
목표 난수는 고정하지만 OMPL 내부 난수열·OS 스케줄링까지 고정하는 것은 아닙니다.

## 시작 상태와 실행 순서

1. 첫 정밀 복귀 후 안정된 실측 관절 상태를 기준으로 보존합니다.
2. 각 전략을 한 번씩 워밍업하고, 매 요청 전 같은 홈 목표로 복귀합니다.
3. 사례별 전략 순서를 번갈아 적용합니다. 두 전략은 같은 목표 파일과 계산 예산을 사용합니다.
4. 각 요청의 실측 시작 자세가 기준에서 0.0015rad 이내인지 검사합니다. 초과하면 평가를 중단합니다.

위치 서보의 중력 부하 때문에 명목 홈과 안정된 실측 자세가 완전히 같지는 않습니다.
복귀 계획의 관절 목표 허용오차는 0.00001rad이며, 채점할 계획에는 기존 0.003rad를 적용합니다.
명목 홈·실측 기준·모든 요청의 초기 상태를 함께 보존합니다. 복귀와 워밍업의 요청 원본도 남기되
채점 표본에서는 분리하고 30wall초 예산을 사용합니다. 이 과정도 정상 완료해야 평가가 유효합니다.

## 계산 예산과 실패

예산 구간은 초기 정지 상태 스냅샷 후부터 계획·출발 탐색·최종 거리/MoveIt 검사·상태 재확인까지입니다.
MoveIt 요청의 허용 시간도 남은 예산에 맞추며, 두 번째 팔은 첫 팔 계산 후 남은 예산을 사용합니다.
RPC 전후와 네이티브 샘플 사이에서 예산을 검사합니다. 소진되면 `PlanningBudgetExceeded`로 기록하고
Action을 보내지 않습니다. 안전검사를 줄이거나 검사 실패를 성공으로 바꾸지 않습니다.

협력식 제한이므로 진행 중인 RPC나 네이티브 호출을 선점하지 않습니다. 실제 초과 시간이 원본에 남으며
지정한 초에 반환되는 hard timeout을 보장하지 않습니다. 시뮬레이터 시작·워밍업·최초 상태 스냅샷·
1초 sim 예약·물리 동작·동작 후 검사는 이 계산 예산과 구분합니다. 수신→동작 지연은 이 구간들을 별도로 포함합니다.

계획·예산 실패는 전체 표본의 실패로 남기고 다음 사례를 진행할 수 있습니다.
물리 실행 실패, 접촉/시계 인터록, 시작 상태 불일치, 복귀 실패는 평가를 중단하고 남은 사례를 `not_run`으로 유지합니다.
평가 실행 자체가 정상 종료했다는 뜻의 `passed`와 각 목표의 동작 성공률은 서로 다른 판정입니다.

## 출발 탐색의 비용 기록

요청 `departure_search.profiling`과 `candidates`에 아래를 기록합니다. 해가 없거나 중단된 탐색도 보존합니다.

| 값 | 범위 |
|---|---|
| elapsed_wall_sec | 전체 출발 탐색 |
| candidate_wall_sec | 후보별 평가 구간의 합 |
| native_distance_wall_sec | 후보 평가 안에서 네이티브 거리 계산 함수에 머문 시간 |
| native_distance_calls | 실제 함수 호출 수. 중단된 후보에서 수행한 호출도 포함 |
| other_wall_sec | 전체 탐색에서 후보 평가를 뺀 나머지 |

네이티브 시간은 후보 평가 시간에 포함됩니다. 세 시간을 모두 더해 총 계산 시간으로 사용하지 않습니다.
후보의 `complete: false`는 간격 미달, `complete: null`과 `status: interrupted`는 계산 중단을 뜻합니다.
측정 래퍼와 시계 조회 자체의 비용도 포함된 wall 계측입니다.

## 검증과 발행

```bash
# 예산 소진 시 두 전략 모두 실제 Action 0개인지 강제 확인하는 부정 시험
python3 benchmark_motion.py --count 1 --radius 0.15 --compute-budget 0.01 \
  --expect-budget-exhaustion --domain 140

# 정상 비교의 전체 원본을 새 평가 ID로 발행
runtime/report-tools/bin/python scripts/import_random_motion_report.py \
  --run runtime/validation/runs/실행_ID/result.json --report-id 새날짜-random-motion --date YYYY-MM-DD
runtime/report-tools/bin/python scripts/reports.py build
```

CI의 근거리 정상 시험은 `--require-all-success`로 모든 채점 동작 성공을 요구합니다.
부정 시험은 일반 랜덤 성능 표본으로 발행하지 않습니다. 새 발행본의 `evidence.json.gz`는
물리 검증·명령 세션·전체 workload·채점 요청·복귀·워밍업 원본을 무손실 gzip으로 보존합니다.
발행기는 실패 누락, 목표·예산·시작 자세 불일치, 비정상 종료와 접촉을 거부합니다.
기존 리포트는 덮어쓰지 않으며 `--check docs/reports/평가_ID`로 보존 원본과 대조합니다.

소규모 결과에는 중앙값·최댓값·유효 표본 수를 표시합니다. 최초 이동과 경로 수치는 성공한 유효 표본,
계산 예산·최종 결과 시간은 실패를 포함한 유효 표본입니다. 사용하지 않은 구간은 null입니다.
홈 주변 관절 목표 결과를 전체 작업 공간, TCP/Cartesian 작업, 반복 충돌 회피 또는 실물 성능으로 일반화하지 않습니다.

[명령과 계측 경계](motion_requests.md) · [출발 조정과 거리 정책](timing_coordination.md) · [리포트 운영](reporting.md)
