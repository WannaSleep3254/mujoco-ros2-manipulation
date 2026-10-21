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
HTML 목록과 `catalog.json`은 새 평가 입력에서 자동 생성됩니다.

GitHub 파일 화면은 HTML 소스를 표시합니다. 로컬 브라우저로 열거나 게시용 정적 사이트를 사용합니다.
공개 사이트는 main의 리포트 관련 변경 후 검증된 파일을 GitHub Pages에 게시합니다.
배포 상태와 공개 파일 검증은 [게시 안내](../reporting.md#html-게시와-운영)에서 확인합니다.
