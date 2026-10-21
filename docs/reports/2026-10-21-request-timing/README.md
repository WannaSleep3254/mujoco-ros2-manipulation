# 상시 명령·지령 → 동작 구간 계측 검증

[HTML 요약](index.html) · [표준 입력](report.json) · [지표 CSV](metrics.csv) · [요청 원본 스냅샷](evidence.json)

평가 구현은 `c67565f958da9c6ddade2bdc393e227b4acb48d2`입니다.
실행 `20261021T063206.805570Z-c9214995`는 한 MuJoCo 공동 장면에서 연속 요청과 별도 CLI,
중복·동시 요청, 잘못된 목표, 실제 충돌 목표 계획 실패, 이동 전/중 취소, 복구 및 SIGINT 종료를 검사했습니다.
소스·환경·모델·거리 평가기 해시와 11개 요청을 원본 스냅샷에 보존합니다.

의도된 실패·거부·취소를 포함한 회귀 검증이며 랜덤 모션 성공률 평가가 아닙니다.
최초 이동은 명령 노드가 `/joint_states`에서 0.001rad 초과 변화를 받은 시점입니다.
1초 sim 예약 대기를 포함하고, 계획·안전검사·수락 시간은 별도로 표시합니다.
이미 도달한 목표는 Action을 보내지 않고 최초 이동을 null로 유지합니다.

[실행·명령 계약](../../motion_requests.md) · [공통 리포트 운영](../../reporting.md)

```bash
runtime/report-tools/bin/python scripts/import_motion_request_report.py \
  --check docs/reports/2026-10-21-request-timing
```
