# 개발과 검증

Python 3.10 / Ubuntu 22.04 / ROS 2 Humble가 기준입니다. 먼저 [설치](docs/setup.md)와
[구조 설명](docs/architecture.md)을 확인합니다. 구현은 `src/mujoco_ros2_manipulation`에서 수정하고,
루트의 호환 진입점을 유지합니다. 작업장면은 `config/scenes`, 빌드 버전은 `config/build.yaml`을 수정합니다.

```bash
PYTHONPATH=src .venv/bin/python scripts/check_repository.py
PYTHONPATH=src .venv/bin/python -m unittest discover -s tests/unit -t . -v
# 제조사 모델을 생성한 환경
PYTHONPATH=src .venv/bin/python -m unittest discover -s tests/model -t . -v
# Python 배포 검사: 별도의 패키징 가상환경에서 build를 설치한 후
python -m build
```

모델 입력 변경 후에는 선택한 장면 조합을 재생성합니다. ROS 동작 변경은
[실제 연동 절차](tests/integration/README.md)로 검증합니다.
실행하지 않은 GUI·실물 검사를 통과한 것으로 기록하지 않습니다.

PR에는 변경 동작, 호환성 영향, 실행한 검사와 생략한 검사를 구분합니다.
실행 중인 모델은 종료하고 생성합니다. `external`, `runtime`, `ros_ws`, `.venv`와
자격 증명을 커밋하지 않습니다. 소스 사본 동기화는 원본 변경과 별도 작업입니다.

공동 장면은 `config/workcells`와 `src/mujoco_ros2_manipulation/workcells`에서 관리합니다.
`python3 build_workspace.py --component models --workcell fr5_ur5e` 후 전체 모델 검사를 실행하고,
`source ./manipulation_env.sh` 후 `python3 verify_scene.py --domain 93`으로 실제 공동 제어를 확인합니다.

## 브랜치와 변경 반영

`main`과 `devel`을 상시 유지합니다. 저장소의 기본 브랜치와 공개·배포 기준은 `main`입니다.

| 브랜치 | 역할 | 시작점·병합 대상 | 보존 |
|---|---|---|---|
| `main` | 검증을 마친 공개 기준 | `devel`에서 단계별 승격 PR | 상시 유지 |
| `devel` | CI를 통과한 기능들의 개발 통합본 | 기능 PR을 받아 통합 | 상시 유지 |
| `codex/viewer-<작업명>` | 뷰어의 개별 기능 | 최신 `devel`에서 분기 → `devel` PR | 병합 후 정리 |
| `codex/motion-plan-<작업명>` | 모션 계획의 개별 기능 | 최신 `devel`에서 분기 → `devel` PR | 병합 후 정리 |
| `codex/<작업명>` | 모델·환경·빌드·문서 등의 개별 작업 | 최신 `devel`에서 분기 → `devel` PR | 병합 후 정리 |

미완성 실험은 기능 브랜치에서 진행합니다. 뷰어·모션별 고정 브랜치는 두지 않습니다.
`/`는 브랜치 이름을 분류하는 용도이며 변경을 자동 상속하지 않습니다.
`devel`과 `devel/<이름>`은 Git 참조 이름이 충돌하므로 함께 만들 수 없습니다.

### 일반 기능 개발

```bash
git fetch --prune origin
git switch devel
git pull --ff-only origin devel
git switch -c codex/motion-plan-home-repro
```

마지막 명령의 `home-repro`는 실제 작업 이름으로 바꿉니다.
PR의 대상(base)을 **`devel`로 명시**합니다. 기본 브랜치는 계속 `main`이므로 PR 생성 화면의 대상을 확인합니다.
변경 동작·완료 조건·검증 결과를 작성하고 필수 검사 6개와 대화 해결을 확인한 뒤 병합합니다.
기능 브랜치에 후속 미완성 작업을 계속 쌓지 않고, 통합된 최신 `devel`에서 다음 작업을 분기합니다.

### main 승격과 동기화

기능별 완료 조건, 기존 동작 회귀, 필요한 실제 ROS 검증이 충족된 묶음을 `devel` → `main` PR로 승격합니다.
검증하지 않은 GUI·실물·성능 범위는 구분하고, 리포트는 고정 입력·원본·코드 버전과 연결합니다.
기능·동기화·승격 PR에는 **merge commit**을 사용해 커밋의 포함 관계를 보존합니다.
main 동기화를 포함한 기능 PR에도 같은 방식을 적용합니다.
`devel`은 승격 후에도 유지하며 삭제하거나 초기화하지 않습니다.

다음 main 승격 전에는 최신 `main`이 `devel`에 포함되어 있어야 합니다.
승격의 병합 커밋이나 긴급 수정은 `codex/sync-main-<작업명>` 같은 동기화 브랜치를 `devel`에서 만들고,
최신 `origin/main`을 merge한 뒤 `devel` 대상 PR로 반영합니다. 동기화 PR도 동일한 필수 검사를 거칩니다.
파일 차이가 없는 동기화는 다음 기능 PR에서 함께 반영할 수 있으며,
보호된 브랜치에 직접 push하거나 강제 갱신해 동기화하지 않습니다.

공개 버전의 긴급 수정은 최신 `main`에서 `codex/hotfix-<작업명>`을 분기해 `main` PR로 반영할 수 있습니다.
병합 후에는 해당 main 변경을 `devel`에도 동기화합니다.
자동화가 `main` 대상으로 여는 의존성·보안 PR도 대상을 확인하고, main에 반영한 변경은 같은 방식으로 동기화합니다.

### 보호·CI·배포

- `main`과 `devel` 모두 PR 필수, 검사 6개, 대상 브랜치 최신 반영, 미해결 대화 해결을 요구합니다.
- 관리자에게도 보호 규칙을 적용하며, 두 상시 브랜치의 삭제·강제 push를 허용하지 않습니다.
- PR CI는 대상이 `main`·`devel`·`codex/**`인 경우 실행합니다. `codex/**` 대상 지원은 기존 중간 통합 PR의 호환용입니다.
- `devel`은 PR에서 통합 검증하며, main push에서는 병합된 공개 기준을 다시 검사합니다. 필요하면 `devel`에서도 수동 Verification을 실행합니다.
- 필수 CI 작업 이름을 바꾸면 두 브랜치 보호 규칙의 검사 이름도 함께 갱신합니다.
- Pages 배포는 `main`의 발행 대상 변경에만 연결합니다. `devel`과 기능 PR의 사이트 결과는 CI Artifact로 확인합니다.

최초 `devel` 도입을 위한 CI·운영 규칙 변경은 기존 main 대상 PR 절차로 반영한 뒤,
갱신된 main에서 `devel`을 생성하고 보호 규칙을 적용합니다.
초기 CI 도입의 [PR #9 통합 경로](docs/integration.md)와 #5~#8은 당시의 보관 이력입니다.

## 성능 리포트

선택 의존성 `pip install '.[reports]'`를 설치하고 [리포트 규칙](docs/reporting.md)을 따릅니다.
공통 입력 `report.json`과 선언한 근거 자료를 작성한 뒤 아래를 실행합니다.

```bash
python scripts/reports.py validate
python scripts/reports.py build
python scripts/reports.py build --check
```

생성 파일을 직접 편집하지 않습니다. 새 측정·정정은 새 평가 ID를 사용하고 원본을 보존합니다.
기존 1차 자료는 `python scripts/import_phase1_report.py --check`로 대조합니다.
Verification CI는 검증된 정적 폴더를 Artifact로 보관합니다. main의 발행 대상 변경은 별도 Publish reports 워크플로에서 검증·배포합니다.

## 전용 조작 화면

`src/mujoco_ros2_manipulation/operator_console`에서 로컬 HTTP 화면과 ROS 어댑터를 관리합니다.
기존 명령 계약·안전검사를 우회하지 않으며, 자체 소유 프로세스만 종료합니다.
`source ./manipulation_env.sh` 후 `python3 verify_operator_console.py --domain 143 --no-render`로
실제 HTTP→ROS 동작·양팔 취소·복구와 접촉·종료를 검사합니다. 화면·EGL 렌더러는 로컬 브라우저에서 별도로 확인합니다.


## 병합 후 브랜치 정리

`main`과 `devel`은 자동·수동 정리 대상에서 제외합니다.
GitHub의 병합 후 자동 브랜치 삭제를 사용하며, 두 상시 브랜치는 삭제를 금지하는 보호 규칙으로 유지합니다.
필수 검사를 통과해 대상 브랜치에 병합된 기능 브랜치는 자동 삭제합니다.
main으로 승격한 PR의 head인 `devel`은 유지합니다.

로컬 정리 전에는 원격을 갱신하고, 해당 tip이 병합 대상 `devel` 또는 `main`에 포함되는지 확인합니다.
미포함 커밋·열린 PR·사용 중인 worktree·진행 중인 작업이 있으면 보존합니다.
로컬 병합 대상 브랜치도 동기화한 뒤 그 브랜치로 이동합니다. 정리가 가능한 브랜치만 `git branch -d`로 삭제하고 원격 추적을 갱신합니다.
병합된 커밋과 PR 이력은 상시 브랜치에 남으므로 일반 작업 브랜치를 `archive/*`로 복제하지 않습니다.
재현할 완성본은 해당 시점의 태그·릴리스와 검증 원본으로 보존합니다.

[2026-10-23 실제 정리 기록](docs/maintenance/merged-branches-2026-10-23.json)은 devel 도입 전 main에 포함된 22개 브랜치 참조를 정리한 시점의 기록입니다.
[2026-10-22 검토 원본](docs/maintenance/merged-branches-2026-10-22.json)은 당시 권장안과 작업 브랜치를 보존한 과거 기록입니다.
