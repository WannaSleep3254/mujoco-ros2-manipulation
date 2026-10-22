# 성능평가 리포트

[공개 HTML 목록](https://wannasleep3254.github.io/mujoco-ros2-manipulation/) · [로컬 HTML 목록](index.html) · [목록 JSON](catalog.json) · [생성·검증·게시 방법](../reporting.md)

공식 자료는 원본 `mujoco-ros2-manipulation` 저장소에서 관리합니다.
일진 프로젝트의 소스 사본에는 원본의 고정 커밋·리포트 링크를 연결합니다.

각 평가 폴더의 `report.json`이 공통 요약의 입력입니다.
공통 생성기는 `index.html`, `metrics.csv`, `report-manifest.json`과 전체 목록을 생성합니다.
새 측정은 새 ID로 추가하고, 정정은 새 폴더의 `supersedes`로 이전 보고서를 연결합니다.

첫 등록 자료는 [2026-10-21 1차 평가](2026-10-21-phase1/index.html)입니다.
고정 FR5·UR5e 교차 1사례의 오프라인 5회와 GUI/headless 비교이며,
기존 [상세 HTML](2026-10-21-phase1/phase1-performance.html)과 그 원본 해시는 보존합니다.
표준 요약은 수치를 새로 측정하지 않고 [기존 근거](2026-10-21-phase1/evidence.json)에서 변환했습니다.
후속 [상시 명령·요청 구간 계측](2026-10-21-request-timing/index.html)은 11개 요청의 성공·실패·취소·거부와 별도 CLI·SIGINT 검증을 보존합니다.
이 회귀 결과는 랜덤 목표 성능평가와 구분합니다.
[랜덤 목표·계산 예산 비교](2026-10-21-random-motion/index.html)는 같은 목표 5개 × 두 전략과 별도 고정 교차 탐색 비용을 보존합니다. 복귀·워밍업과 실패를 포함한 전체 원본은 무손실 gzip입니다.

[출발 탐색 최적화·실제 GUI 점검](2026-10-21-search-optimization/index.html)은 고정 입력 6개 × 두 방식 × 5회의 정확한 선택 결과 일치와 계산 절감을 비교하고, 별도 GUI 실행·원본 화면을 보존합니다. 오프라인 탐색 시간과 전체 지령 지연은 구분합니다.

HTML 목록과 `catalog.json`은 새 평가 입력에서 자동 생성됩니다.
목록에서 **요약 보기**와 **상세 리포트**를 바로 선택할 수 있고, 요약 상단에도 상세 링크가 표시됩니다.
별도 영상 자료가 등록되어 있으면 **동작 영상** 링크도 표시됩니다.

GitHub 파일 화면은 HTML 소스를 표시합니다. 로컬 브라우저로 열거나 게시용 정적 사이트를 사용합니다.
공개 사이트는 main의 리포트 관련 변경 후 검증된 파일을 GitHub Pages에 게시합니다.
배포 상태와 공개 파일 검증은 [게시 안내](../reporting.md#html-게시와-운영)에서 확인합니다.

## 기능 시연

공개 목록 아래의 **기능 시연**에서 [뷰어 상태·피킹 접근·후퇴 비교](https://wannasleep3254.github.io/mujoco-ros2-manipulation/operator/picking-comparison-2026-10-22/index.html)를 엽니다.
[별도 등록 목록](operator-demos.json)과 시연 폴더의 manifest로 게시 파일을 검증합니다.
성능평가 `catalog.json`의 보고서·표본에는 합산하지 않습니다. [등록 방법](../reporting.md#기능-시연의-등록과-게시)을 따릅니다.

## 설계·개발 리포트

[뷰어 개발 방향성](../operator/viewer-direction-2026-10-22/index.html)은 이기종 복수 로봇의 통합 조작 방향과 현재 구현·검증 범위, 대안·개선 계획을 정리합니다.
[모션플래닝 개발 방향성](../operator/motion-planning-direction-2026-10-22/index.html)은 빈피킹 우선 요구와 현재 양팔 계획, 지연·최소 우회 개선, 기술 비교·실물 전환·검증 계획을 정리합니다.
게시 목록에서는 **설계·개발 리포트**로 구분하며 성능평가 표본에는 합산하지 않습니다.
새 방향성 리포트는 [작성 기준 v1.9](../reporting/report-style-guide.md)와 [HTML 템플릿](../reporting/templates/engineering-report-v1.9.html)을 기본으로 사용합니다.
