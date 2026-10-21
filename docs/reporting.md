# 리포트 데이터·생성·검증·게시

공통 구현은 [reporting 패키지](../src/mujoco_ros2_manipulation/reporting),
데이터 계약은 [JSON Schema 2020-12](../src/mujoco_ros2_manipulation/reporting/report.schema.json)입니다.
ROS·물리 엔진을 실행하거나 import하지 않고 기존 측정 파일에서 HTML·CSV·목록을 생성합니다.
기존 단일 로봇·공동 장면 검증기의 실행 결과 형식은 유지합니다.
그 원본을 발행 리포트로 옮길 때 어댑터를 사용합니다.

## 위치와 파일 역할

```text
docs/reports/
  README.md                  # 사용 안내
  index.html                 # 자동 생성: 전체 목록·검색
  catalog.json               # 자동 생성: 목록 API용 JSON
  site-manifest.json         # 자동 생성: 게시 파일 전체 해시, 자기 자신 제외
  <날짜>-<평가ID>/
    report.json              # 작성·검토할 공통 입력
    index.html               # 자동 생성: 표준 요약
    metrics.csv              # 자동 생성: 동일 지표와 단위·시계·n
    report-manifest.json     # 자동 생성: 파일 해시·크기·생성기 버전/지문
    ...                      # report.json에 선언한 근거·그림·기존 상세 HTML
runtime/report-site-<ID>/     # 새 폴더에만 내보내는 게시용 파일; Git 제외
```

발행 폴더에는 검토한 자료를 보존합니다. 반복 시험 중간 결과는 `runtime`에 두고,
대표 결과를 발행할 때만 가져옵니다. 새 측정은 새 ID, 정정은 새 ID와 `supersedes`를 사용합니다.
이는 PR 운영 규칙이며 과거 입력을 Git에서 수정하지 못하게 잠그는 기능은 아닙니다.
스키마가 바뀌면 `schema_version`을 추가하고 별도 변환기를 작성합니다.

## 데이터 계약 v1

| 필드 | 필수 정보 |
|---|---|
| 식별·판정 | ID, 발행일, 제목, preliminary/validated/superseded, 범위, 결론, 한계 |
| provenance | 저장소, 실제 평가 코드 40자리 commit, 작업장면, 모델, 시나리오, 입력 종류 |
| environment | OS·CPU·Python·ROS·native MuJoCo 필수. 모르는 값은 ‘미확인’으로 명시 |
| sample_groups | 조건별 고유 사례·실행 수, 성공·실패 또는 둘 다 null, seed 정책과 설명 |
| stages | 지령→계산→안전검사→전송→동작의 측정/부분/추정/미측정 범위와 시계 |
| metrics | 단위, wall/simulation/not_applicable, 범위, 조건별 값·n·집계 방식·해석 |
| assets | 폴더 내부 상대 경로, 역할, 설명, SHA-256. 이미지·데이터·기존 HTML 등 |
| references | 관련 자료의 HTTPS URL. supersedes는 같은 목록의 이전 ID만 허용 |

`unique_cases`와 반복 수는 다릅니다. 여러 그룹이 같은 사례를 공유할 수 있어 그룹별 사례 수를 합산하지 않습니다.
지표의 n은 집계에 쓴 실행/반복 수이고, 한 실행의 관절 상태 2763개와 구분합니다.
성공·실패가 알려져 있으면 합계가 그룹 실행 수와 같아야 합니다. 실패를 제외한 성능 분포는 별도로 명시합니다.

미측정 값은 **`value: null`, `sample_count: 0`, `statistic: not_measured`**로 표현합니다.
0초로 채우지 않습니다. 시간 단위에는 wall 또는 simulation 시계가 필요하며 거리·각도에는 not_applicable을 씁니다.
작은 고정 표본의 p95/p99 표기를 막기 위해 생성기는 n<20의 꼬리 분위수를 거부합니다.
20개 이상이라는 조건은 충분한 통계적 신뢰도를 뜻하지 않습니다. 사례별 상관·신뢰구간과 표본 설계는 별도 검토합니다.
임의로 새 성능을 추정하거나 서로 다른 실행의 wall/sim 시간을 더하지 않습니다.

## 실행

Python 3.10 이상. 기존 고정 MuJoCo·NumPy 버전은 변경하지 않습니다.
선택 의존성 `reports`는 `jsonschema>=4.18,<5`이며 JSON Schema 2020-12 검증에 사용합니다.
구버전 jsonschema를 쓰는 ROS 환경은 유지하고 별도 도구 환경에 설치할 수 있습니다.

```bash
python3 -m venv --system-site-packages --without-pip runtime/report-tools
runtime/report-tools/bin/python -m pip install --upgrade 'pip>=24,<26' 'setuptools>=64,<80'
env -u PYTHONPATH runtime/report-tools/bin/python -m pip install '.[reports]'
runtime/report-tools/bin/python scripts/reports.py validate
runtime/report-tools/bin/python scripts/reports.py build
runtime/report-tools/bin/python scripts/reports.py build --check
runtime/report-tools/bin/python scripts/reports.py site --output runtime/report-site-review
python3 -m http.server 8000 --bind 127.0.0.1 --directory runtime/report-site-review
```

브라우저에서 `http://127.0.0.1:8000/`을 엽니다. `index.html`을 직접 열어도 기본 열람·검색·조건 필터가 동작합니다.
위 준비 명령은 이 PC의 `ensurepip` 부재·구형 시스템 pip를 고려한 Ubuntu 방식입니다.
시스템 pip를 읽을 수 있는 전용 venv를 만들고 업그레이드는 그 안에만 설치합니다.
기존 `.venv`와 ROS 의존성을 수정하지 않습니다. 일반적인 ensurepip 제공 환경에서는 보통의 `python3 -m venv`도 가능합니다.
설치된 명령 `manipulation-report`도 같은 기능이며 체크아웃 밖에서는
`MUJOCO_ROS2_WORKSPACE`를 지정하거나 전역 옵션 `--reports-dir /절대경로`를 명령 앞에 둡니다.

`build --check`는 파일을 쓰지 않고 오래된 출력·누락을 실패로 판정합니다.
`site`는 기존 출력 폴더를 지우지 않습니다. 매번 새 이름을 사용합니다.
선언한 자료와 생성 파일만 복사하며, 링크·앵커·자료 해시·중복 ID·경로 이탈·심볼릭 링크를 검사합니다.
기존 상세 HTML도 내보내면 같은 정적 파일 집합에서 링크를 검사합니다.
공통 HTML은 입력 문장을 이스케이프하며 CSV의 텍스트 수식도 이스케이프합니다.
과거 상세 HTML의 JavaScript는 재작성하지 않으므로 포함 여부는 PR에서 검토합니다.

새 리포트는 [미측정 예제](reporting/example-unmeasured/report.json)를
`docs/reports/<새ID>/report.json`으로 복사해 시작합니다. 폴더명과 ID를 맞추고 환경·측정·자료를 채웁니다.
예제의 null을 측정값으로 바꿀 때 표본·성공/실패·시계도 함께 기록합니다.
자료 파일의 SHA-256은 `sha256sum`으로 계산합니다.

## 기존 1차 리포트 변환

```bash
runtime/report-tools/bin/python scripts/import_phase1_report.py --check
```

[어댑터](../src/mujoco_ros2_manipulation/reporting/phase1.py)는 기존 근거 JSON의 수치와 원본 자산 해시를 검사합니다.
첫 실행 시 `report.json`이 없으면 생성하고, 기존 파일과 다르면 덮어쓰지 않고 실패합니다.
공통 HTML·CSV는 여기서 만든 표준 입력으로 재생성합니다. 기존 상세 HTML·JSON·CSV·그림·manifest는 바꾸지 않습니다.
새 랜덤 모션 평가에는 새 어댑터를 추가하거나 측정기가 공통 스키마를 직접 출력하게 합니다.

## HTML 게시 방식 검토 — 2026-10-21

당일 GitHub 저장소 조회의 `has_pages=false`를 확인했습니다. Pages 설정·배포·도메인은 변경하지 않았습니다.

| 방법 | 용도·평가 |
|---|---|
| 로컬 파일 / HTTP | 개발·검토용. 추가 서비스 없이 현재 지원 |
| CI Artifact | PR의 생성 결과 다운로드·검토용. 장기 공개 링크로 사용하지 않음 |
| GitHub Pages + Actions | **권장**. 공통 생성·검증 후 선택한 정적 파일만 발행. 별도 서버 없이 목록과 HTML 열람 |
| main의 /docs 직접 Pages 발행 | 가능하지만 문서 전체가 대상이고 생성 검증·파일 선택 통제가 약해 현재 구조에는 비추천 |
| GitHub Releases | 대용량 로그·전체 시계열·영상의 버전별 내려받기 보관. HTML 웹 서비스와 역할이 다름 |

이번 CI는 사이트를 만들고 Artifact로 보관하며 **배포하지 않습니다**. 기존 ROS 필수 검사 이름과 보호 규칙은 유지합니다.
기존 ROS 진단 Artifact는 14일 보존입니다. 중요한 원본을 CI 링크에만 의존하지 않습니다.
현재 규모의 자료는 Git에 보관하되 랜덤 평가가 커지면 Git에는 요약·작은 그림·파일 해시를,
Release에는 압축 원본을 두고 HTTPS 참조로 연결합니다. 웹용 HTML에 매번 대형 영상을 내장하지 않습니다.

실제 Pages 도입 시:

1. Settings → Pages → Source에서 GitHub Actions 선택.
2. [검토용 workflow 예시](reporting/report-pages.workflow.yml)를 `.github/workflows/report-pages.yml`로 옮기는 별도 PR 작성.
3. 기본 브랜치의 `github-pages` environment·배포 권한을 확인. 예시는 main에서 수동 실행만 허용합니다.
4. 생성·검증된 폴더만 업로드하고 Pages 배포 실행. 이 작업 브랜치에서는 실제 배포하지 않습니다.
5. 발급된 URL에서 목록·조건 전환·상대 경로·CSV·기존 HTML·모바일·다운로드를 확인한 뒤 README에 실제 주소 추가.

예상 프로젝트 주소는 `https://WannaSleep3254.github.io/mujoco-ros2-manipulation/`이며 **아직 게시된 주소가 아닙니다**.
루트 고정 경로 대신 상대 링크를 사용해 프로젝트 하위 경로와 로컬 열람 모두 지원합니다.

공식 근거:

- [JSON Schema 검증·FormatChecker](https://python-jsonschema.readthedocs.io/en/stable/validate/)
- [Pages 게시 소스 선택](https://docs.github.com/en/pages/getting-started-with-github-pages/configuring-a-publishing-source-for-your-github-pages-site)
- [Pages Actions·배포 권한·environment](https://docs.github.com/en/pages/getting-started-with-github-pages/using-custom-workflows-with-github-pages)
- [Actions Artifact 보존·다운로드](https://docs.github.com/en/actions/how-tos/manage-workflow-runs/download-workflow-artifacts)
- [Release 자료 보관](https://docs.github.com/en/repositories/releasing-projects-on-github/about-releases)
