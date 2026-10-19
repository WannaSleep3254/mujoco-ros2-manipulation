# Python 의존성 업데이트 운영

[프로젝트 README](../README.md) · [설치 환경](setup.md) · [ROS 검증](verification.md) · [Dependabot 설정](../.github/dependabot.yml)

## 적용 범위와 활성화 시점

`.github/dependabot.yml`은 루트 `requirements.txt`의 pip 의존성을 매주 월요일 09:00, `Asia/Seoul` 기준으로 확인하도록 정의합니다.
이 설정이 기본 브랜치 `main`에 병합되기 전에는 정기 버전 업데이트가 활성화된 것으로 간주하지 않습니다.
예약 시각은 검사 일정이며 실제 시작 시각은 GitHub의 실행 상황에 따라 달라질 수 있습니다.

기존 Dependency graph·Dependabot alerts·Dependabot security updates 설정은 유지합니다.
자동 병합을 설정하거나 보안 설정을 변경하는 Workflow·API 호출은 추가하지 않습니다.
버전 업데이트 PR은 동시에 최대 5개입니다. 보안 업데이트 PR의 별도 상한은 GitHub에서 관리하며 이 값으로 제한되지 않습니다.
기본 브랜치를 그대로 사용하므로 `target-branch`는 지정하지 않습니다.

## 버전 제한과 그룹화

이번 설정 도입으로 `requirements.txt`의 버전·범위를 변경하지 않습니다.
pip 패키지 이름은 Dependabot의 정규화된 소문자 이름으로 지정합니다.

| 패키지 | 현재 요구사항 | 일반 버전 업데이트 정책 | 이유 |
|---|---|---|---|
| MuJoCo | `mujoco==3.14.0` | major·minor·patch 모두 수동 검토 | 모델 생성·Python 브리지의 검증 기준 유지 |
| NumPy | `numpy==1.26.4` | major·minor·patch 모두 수동 검토 | ROS Humble·네이티브 확장과 함께 검증한 조합 유지 |
| PyYAML | `PyYAML>=5.4,<7` | major·minor·patch 모두 수동 검토 | 버전 하한 상승으로 ROS 시스템 PyYAML 5.4.1이 제외되는 것을 방지 |
| pycollada | `pycollada==0.9.3` | minor·patch를 `python-helpers` PR로 그룹화 | DAE 변환 회귀 검사 후 검토 |
| Pillow | `Pillow>=9` | minor·patch를 같은 그룹으로 처리 | 렌더·화면 캡처 PNG 처리 검증 후 검토 |

전체 패키지의 일반 major 업데이트는 제외합니다. 새 패키지가 추가되면 해당 패키지의 호환성·그룹 정책도 검토합니다.
그룹의 `update-types`는 `minor`·`patch`, 제외 규칙은 `version-update:semver-*` 형식을 사용합니다.
`versioning-strategy: increase`로 보조 패키지 요구사항 변경을 PR로 제안하되, 검토자가 Python 3.10 지원을 확인한 후 수동 병합합니다.
허용할 minor·patch 릴리스가 없으면 주간 검사가 성공해도 PR이 생기지 않을 수 있습니다.

### MuJoCo·NumPy·ROS 호환성

현재 Python 환경은 Python 3.10, MuJoCo 3.14.0, NumPy 1.26.4입니다.
ROS 시뮬레이션은 C++ MuJoCo 3.12.0과 패치한 `mujoco_ros2_control` 0.1.2를 별도로 사용합니다.
pip MuJoCo 업데이트가 C++ 라이브러리·ROS vendor를 함께 갱신하지는 않습니다. 두 경로의 모델 로딩과 관절·좌표계·종료 동작을 각각 확인해야 합니다.
NumPy 2.0에는 C ABI 변경이 있으므로 NumPy 1.x 대상으로 빌드된 확장과의 호환성은 별도 검증 대상입니다.
[NumPy 2.0 마이그레이션 안내](https://numpy.org/doc/stable/numpy_2_0_migration_guide.html)

보안 업데이트는 위 일반 버전 정책과 구분합니다. `ignore`에는 `versions` 범위나 무조건 제외 규칙을 사용하지 않고 `update-types`만 지정했습니다.
Dependabot의 보안 전용 실행은 이 업데이트 유형 제한을 적용하지 않으므로, 보안 수정 PR의 생성 경로는 유지합니다.
이는 모든 취약점이 자동 수정된다는 보장은 아닙니다. 보안 PR이 MuJoCo·NumPy·PyYAML의 기준 버전을 변경하면 같은 호환성 검증 후 수동 검토합니다.
[Dependabot 제외 규칙 구현](https://github.com/dependabot/dependabot-core/blob/main/common/lib/dependabot/config/ignore_condition.rb)

## 변경 시 검증

설정 변경은 중복 키 없는 YAML 파싱과 Dependabot v2 JSON Schema 검증을 수행합니다.
스키마 통과는 GitHub에서의 실제 의존성 해석·업데이트 작업 성공과 구분합니다.
`requirements.txt`가 변경되지 않았는지 diff로 확인합니다.

의존성 버전을 검토하는 PR에서는 별도의 검증 환경에 후보 버전을 설치한 뒤 최소한 다음 회귀 검사를 수행합니다.
운영 가상환경을 바로 업그레이드하지 않습니다.

```bash
.venv/bin/python -B -m unittest discover -s tests -v
```

제조사 소스·생성 모델이 없어서 건너뛴 검사는 성공한 검사와 구분해 기록합니다.
MuJoCo·NumPy·PyYAML 변경 시에는 FR5·FR10·UR5e의 모델 생성, ROS 궤적·MoveIt 실행, TF·관절 수렴, GUI 종료와 SIGINT 종료도 확인합니다.
Pillow 변경은 기준 렌더·GUI 캡처의 PNG 저장을, pycollada 변경은 DAE 변환의 형상·색상·단위와 물리 모델 불변성을 확인합니다.
GitHub의 업데이트 PR 생성만으로 ROS 검증을 통과했다고 간주하지 않습니다.

## 일진 프로젝트의 소스 사본

원본은 `/home/sea21jk/projects/isaac_sim_setup`입니다.
`/home/sea21jk/projects/iljin-forging-inspection/src/mujoco-ros2-manipulation`은 Git 이력·가상환경·빌드 산출물 없는 소스 보관본입니다.
일진 프로젝트의 `README.md`, `docs/PROJECT_CONTEXT.md`, `collection_manifest.json`에 수집 범위·기준 커밋·파일별 SHA-256이 기록되어 있습니다.
현재 확인한 수집 방식은 추적 파일의 사본 보관이며, 자동 소스 동기화 스크립트는 확인되지 않았습니다. Notion 동기화와도 별개입니다.

2026-10-19 이번 변경 전 확인한 기준 커밋은 `7e9d387c711c6f0a0b20ea70f373da40c39b9068`이며, 수집된 78개 파일은 기록 해시·당시 원본 파일과 일치했습니다.
이 PR에서 사본과 수집 기록은 변경하지 않습니다. 사용자 작업 중인 다른 파일도 보존합니다.
일진 프로젝트 원격의 이전 DNS 오류·API 404가 해결되었는지는 이번 작업에서 재확인하지 않았으므로 원격 동기화 완료로 보고하지 않습니다.

병합 후 필요한 반영 절차는 다음과 같습니다.

1. 원본의 병합 커밋과 사본의 현재 상태를 확인하고, `collection_manifest.json`의 해시와 사본을 다시 비교합니다.
2. 이번 PR의 변경 파일인 `.github/dependabot.yml`, `docs/dependency_updates.md`, `README.md`를 기준 커밋 대비로 비교합니다. 사용자 변경이 있으면 보존하고 충돌을 수동 조정합니다.
3. 원본 추적 파일의 새 목록을 기준으로 사본과 파일별 크기·SHA-256을 갱신합니다. 전체 사본이 해당 커밋과 일치하는 경우에만 수집 커밋을 올립니다. 부분 반영이면 별도 반영 범위·미반영 파일을 기록합니다.
4. 일진 저장소의 미커밋 CAD·문서 등과 분리해 리뷰하고, 원격 접근·푸시 결과가 확인된 경우에만 원격 반영 완료로 기록합니다.

사본 내부의 `.github/dependabot.yml`은 보관 목적이며 일진 저장소 루트의 Dependabot 설정으로 작동하지 않습니다.
일진 저장소 자체의 의존성 자동화는 별도 범위입니다.

## 기본 브랜치 병합 후 확인

- GitHub의 기본 브랜치에서 `.github/dependabot.yml`을 읽을 수 있는지 확인합니다.
- Insights 또는 Security의 Dependabot 화면에서 pip `/` 업데이트 작업과 로그를 확인합니다.
- 주간 일정, 일반 버전 제한·그룹화, 버전 업데이트 PR 상한 5개가 적용되는지 확인합니다.
- Dependency graph·alerts·security updates와 자동 병합 비활성 상태가 유지되는지 확인합니다.
- PR이 없으면 업데이트 후보 부재와 작업 오류를 구분합니다. 필요 시 수동 업데이트 확인을 실행합니다.
- 첫 업데이트 PR에서 실제 diff, Python 3.10 요구사항, 회귀 검사 결과를 확인하고 수동 검토합니다.

설정 필드의 의미와 PR 상한·그룹 정책은 [GitHub Dependabot 옵션 안내](https://docs.github.com/en/code-security/reference/supply-chain-security/dependabot-options-reference)를 기준으로 합니다.
