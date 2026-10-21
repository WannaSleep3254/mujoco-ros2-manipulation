# 출발 탐색 중복 계산 제거와 결과 일치 검증

원래 탐색은 왼팔 지연과 오른팔 지연을 각각 0초부터 검사하고, 팔별 첫 성공 중 작은 지연을 선택합니다.
새 탐색은 같은 규칙을 유지하면서 아래 두 경우의 계산을 줄입니다.

- 두 팔의 0초 지연은 같은 spline·시작 상태·거리 정책을 평가하므로 한 번의 결과를 재사용합니다.
- 왼팔의 성공 지연을 찾은 뒤, 오른팔의 더 작은 지연만 검사합니다. 같은 지연은 기존 왼팔 우선 동률 규칙을 바꾸지 못하고, 더 큰 지연은 기존 최선을 개선하지 못합니다.

0초에 이미 성공하면 한 번의 전체 거리 검사로 선택이 결정됩니다.
선택한 궤적과 모든 거리 표본·보간 규칙, 계획 30mm·관측 20mm 기준은 유지합니다.
각 요청의 최종 MuJoCo/MoveIt 안전검사와 네이티브 접촉·시계 감시도 계속 수행합니다.

반올림한 상태 캐시나 요청 간 캐시는 사용하지 않습니다. 공유 0초 결과는 한 번의 탐색 안에서만 사용합니다.
새 요청·다른 모델·다른 거리 정책에서는 다시 계산합니다. 계산 예산 확인과 중단 후보 기록을 유지합니다.

## 계측 의미

`candidates`와 `profiling.candidate_count`는 실제 평가한 후보입니다.
`search_decisions`에 공유 0초 재사용과 현재 최선에 의한 생략 근거를 보존합니다.
`reused_zero_delay_evaluations`는 재사용 횟수, `grid_values_pruned_by_bound`는 설정된 전체 그리드에서
최선보다 나을 수 없어 제외한 값의 수입니다. 후자는 이전 알고리즘이 실제로 평가했을 횟수와 다릅니다.
성능 비교에는 실제 실행한 후보·거리 호출·wall 시간을 사용합니다.

## 같은 입력 A/B 비교

```bash
PYTHONPATH=src .venv/bin/python scripts/compare_search_optimization.py --repeats 5
```

기본 입력은 [기존 랜덤 평가](reports/2026-10-21-random-motion/index.html)의 고정 목표 5개에서 보존한
독립 spline과 별도 교차 입력 1개입니다. 현재 모델·네이티브 라이브러리·정책이 원본과 같아야 합니다.
입력 모델이 바뀌면 조용히 대체하지 않고 실패합니다.

다른 환경에서는 현재 모델에서 정상 검증·고정한 묶음을 사용합니다.

```bash
source ./manipulation_env.sh
python3 scripts/compare_search_optimization.py \
  --bundle runtime/validation/interference-comparison-bundle.json --case crossing --repeats 1 \
  --output runtime/validation/search-optimization-ci.json
```

`choose_delay(..., optimize=False)`는 기존 두 순서 탐색을 참조 경로로 유지합니다.
같은 구현·모델에서 `optimize=True`와 비교하며, 오래된 실행의 시간을 새 실행과 직접 비교하지 않습니다.
각 입력·방식의 워밍업 1회는 별도 보존하고, 측정 순서를 번갈아 적용합니다.
각 반복의 선택 팔·지연·전체 spline·선택 경로의 거리 평가가 정확히 같은지 검사합니다.
원본 입력 변경이나 네이티브 호출 수 증가도 실패로 처리합니다.

출력은 새 `runtime/validation/search-comparison/<ID>/result.json`입니다.
소스 커밋·구현/모델/라이브러리/입력 해시, 전체 후보와 선택 결과·반복 순서를 보존합니다.
실패 시 부분 결과·중단 후보·오류를 같은 파일에 남기며 성공 결과로 덮어쓰지 않습니다.

이번 계측 범위는 **오프라인 출발 탐색**입니다. 모델 로딩·ROS 왕복·최종 MoveIt 검사·예약 대기·물리 실행과
구분합니다. 입력의 난이도에 따라 이득이 달라지며 실시간 재계획이나 최소 공간 우회 최적화는 후속 기능입니다.

[랜덤 목표 평가](random_motion_benchmark.md) · [거리·타이밍 정책](timing_coordination.md)
