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
목록에는 **요약 보기**와 함께 등록된 `legacy_report`의 **상세 리포트**, `video`의 **동작 영상** 링크를 표시합니다.
같은 링크는 요약 상단에서도 열 수 있습니다. 자료가 없으면 해당 링크를 만들지 않으며,
같은 역할의 자료가 여러 개이면 자료명을 함께 표시해 구별합니다.
`catalog.json`의 기존 `url`은 표준 요약 주소를 유지하고, 추가 `resources` 배열에 역할(`kind`)·자료명(`label`)·상대 주소(`url`)를 기록합니다.
상세 HTML 안에 포함된 영상은 해당 **상세 리포트**를 열어 확인합니다.
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

## HTML 게시와 운영

[공개 리포트 목록](https://wannasleep3254.github.io/mujoco-ros2-manipulation/) ·
[게시 workflow](../.github/workflows/report-pages.yml) ·
[배포 상태](https://github.com/WannaSleep3254/mujoco-ros2-manipulation/actions/workflows/report-pages.yml)

GitHub Pages의 Source는 GitHub Actions, 배포 환경은 `github-pages`입니다.
배포 환경은 `main` 브랜치만 허용하고, workflow의 build/deploy도 main에서만 실행합니다.
커스텀 도메인이나 별도 서버는 사용하지 않습니다.

| 보관·열람 방법 | 운영 용도 |
|---|---|
| 로컬 파일 / HTTP | 개발·검토용. 같은 정적 파일과 상대 경로 사용 |
| CI `report-site-preview` Artifact | PR 생성 결과를 다운로드해 검토. 14일 보관 |
| GitHub Pages + Actions | 공통 생성·검증을 통과한 파일만 공개 HTML로 게시 |
| GitHub Releases | 평가 원본이 커질 때 압축 로그·전체 시계열·영상을 버전별 보존 |

### 자동 게시

1. PR에서 기존 필수 CI 6개와 생성 결과를 검토하고 main에 병합합니다.
2. main의 리포트 입력·생성기·의존성·게시 workflow 변경이 있으면 `Publish reports`를 실행합니다.
3. JSON 스키마·원본 자료 해시·출력 일치·리포트 단위 검사를 수행합니다.
4. `site`로 선택한 파일만 새 폴더에 내보내고 Pages Artifact를 올립니다.
5. Pages 배포 후 공개 파일 전체를 해당 커밋의 `site-manifest.json`과 대조합니다.
   HTML 200 응답만으로 성공 처리하지 않습니다. 이전 내용·누락·해시 불일치는 실패입니다.

일반 ROS·단위 CI의 필수 검사 이름과 main 보호 규칙은 유지합니다.
Pages workflow는 리포트 게시를 담당하며 실시간 로봇 시험이나 새 성능 측정을 실행하지 않습니다.
보고서의 평가 코드 커밋은 배포 코드 커밋과 다를 수 있으며, 보고서의 provenance가 평가 기준입니다.

### 수동 실행과 게시 확인

Actions → Publish reports → Run workflow에서 **main**을 선택합니다.
배포 결과의 page URL과 이후 공개 파일 검증 단계를 함께 확인합니다.
다른 브랜치에서는 build/deploy가 실행되지 않습니다.

```bash
python3 scripts/check_published_reports.py \
  --url https://wannasleep3254.github.io/mujoco-ros2-manipulation/ \
  --attempts 12 --delay 5
```

검사기는 표준 라이브러리만 사용하며 토큰을 보내지 않습니다. 공개 사이트의 manifest와 선언된 모든 파일의
바이트 수·SHA-256을 로컬 커밋과 비교합니다. CDN 반영 중에는 제한된 횟수만 재시도하고,
계속 다르면 성공으로 보고하지 않습니다. 과거 체크아웃으로 최신 사이트를 검사하면 실패할 수 있습니다.
비교할 발행 버전의 커밋을 사용합니다.

목록 검색, 조건 필터, JSON/CSV 다운로드, 기존 상세 HTML·영상과 모바일 화면은 최초 게시·템플릿 변경 시
브라우저에서 추가로 확인합니다. GitHub Pages 설정은 Git 외부 상태이므로 저장소 관리 권한으로도 확인합니다.

### 변경·복구

자료 수정은 새 평가 ID 또는 정정 리포트로 PR을 작성합니다. 게시 설정·검증기는 일반 코드 PR로 변경합니다.
실패한 배포는 Actions의 단계별 로그와 공개 파일 검증 메시지로 진단합니다.
main을 수정한 뒤 다시 게시하거나, 검토한 이전 커밋의 설정·자료를 새 PR로 복원합니다.
브랜치 보호나 파일 해시 검사를 해제하여 실패를 통과시키지 않습니다.

중요한 원본을 14일 CI 링크에만 의존하지 않습니다. 현재 규모는 Git에서 보존하고,
랜덤 평가가 커지면 요약·작은 그림·파일 해시는 Git, 압축 원본은 Release 링크로 분리합니다.
[단순 수동 workflow 예시](reporting/report-pages.workflow.yml)는 참고용이며,
실제 운영 설정은 `.github/workflows/report-pages.yml`입니다.

공식 근거:

- [JSON Schema 검증·FormatChecker](https://python-jsonschema.readthedocs.io/en/stable/validate/)
- [Pages 게시 소스 선택](https://docs.github.com/en/pages/getting-started-with-github-pages/configuring-a-publishing-source-for-your-github-pages-site)
- [Pages Actions·배포 권한·environment](https://docs.github.com/en/pages/getting-started-with-github-pages/using-custom-workflows-with-github-pages)
- [Actions Artifact 보존·다운로드](https://docs.github.com/en/actions/how-tos/manage-workflow-runs/download-workflow-artifacts)
- [Release 자료 보관](https://docs.github.com/en/repositories/releasing-projects-on-github/about-releases)
