# 보호된 main으로 CI 구성을 통합하는 경로

[README](../README.md) · [개발과 검증](../CONTRIBUTING.md) · [CI 정의](../.github/workflows/ci.yml)

## 초기 통합 PR

2026-10-20 기준 `main`에는 아직 CI 워크플로가 없고,
검증 보강 PR #5에도 검사 결과가 없습니다. 이후 PR #6에서 패키지 구조·빌드·CI를 도입했고,
PR #7부터 공동 장면을 포함한 현재의 6개 검사를 실행합니다.
따라서 필수 검사 6개를 유지한 상태에서 CI 도입 전 PR부터 하나씩 병합하는 경로는 사용할 수 없습니다.

**PR #9 → main을 초기 통합 경로로 사용합니다.** #9 브랜치에는 #5~#8의 커밋과 CI 구성이 모두 포함돼 있습니다.
커밋을 다시 쓰거나 보호 규칙을 완화하지 않고, 같은 소스 트리의 검증을 main 대상으로 수행합니다.
CI 파일은 현재 통합 PR 브랜치에 있으며, main 반영은 실제 병합 후에 이루어집니다.

| PR | 포함하는 변경 | 처리 경로 |
|---|---|---|
| [#5](https://github.com/WannaSleep3254/mujoco-ros2-manipulation/pull/5) | 실패 보고서·생성물 무결성 | #9에 커밋 포함, 변경별 검토 이력 유지 |
| [#6](https://github.com/WannaSleep3254/mujoco-ros2-manipulation/pull/6) | 패키지·빌드·작업장면 분리·CI | #9에 커밋 포함, 변경별 검토 이력 유지 |
| [#7](https://github.com/WannaSleep3254/mujoco-ros2-manipulation/pull/7) | 공동 장면·양팔 제어와 계획 | #9에 커밋 포함, 변경별 검토 이력 유지 |
| [#8](https://github.com/WannaSleep3254/mujoco-ros2-manipulation/pull/8) | 매 스텝 접촉·GUI·성능 검증 | #9에 커밋 포함, 변경별 검토 이력 유지 |
| [#9](https://github.com/WannaSleep3254/mujoco-ros2-manipulation/pull/9) | 위 변경 전체와 접촉 자동 정지·무장애물 대조 | main 대상 통합 PR |

#5~#8은 개별 병합하지 않습니다. #9의 대상 변경 전후 head 커밋이 같음을 확인했고,
`git merge-base --is-ancestor`로 선행 4개 PR의 head가 모두 통합 브랜치에 포함됨을 검사했습니다.

## main의 보호 조건

- PR을 통한 변경만 허용하며 관리자도 같은 규칙을 적용받습니다.
- 최신 main 반영과 아래 GitHub Actions 검사 6개 통과를 요구합니다.
- 강제 푸시와 브랜치 삭제를 차단하고, 미해결 리뷰 대화는 해결해야 합니다.
- 개인 저장소의 필수 승인 리뷰 인원은 0명입니다. 자동 병합은 설정하지 않습니다.

| 필수 검사 이름 | 범위 |
|---|---|
| `Python package and unit tests` | 패키지 빌드·구문/링크·단위 검사 |
| `ROS Humble (fr5, default)` | FR5 기본 장면 |
| `ROS Humble (fr10, default)` | FR10 기본 장면 |
| `ROS Humble (ur5e, default)` | UR5e 기본 장면 |
| `ROS Humble (fr5, pedestal)` | FR5 받침대 장면 |
| `ROS Humble (workcell, fr5_ur5e)` | 공동 제어·접촉 정지·무장애물 대조 |

이 이름은 [워크플로](../.github/workflows/ci.yml)의 작업 이름·행렬과 일치해야 합니다.
검사 이름을 변경하면 보호 규칙의 필수 검사 목록도 함께 정리해야 합니다.

## 병합 전후 절차

1. 통합 PR #9의 대상이 main이고, CI·빌드·모든 선행 변경이 diff에 포함되는지 확인합니다.
2. main 기준 새 PR 커밋의 필수 검사 6개가 모두 성공하고, 충돌·미해결 대화·뒤처진 main 상태가 없는지 확인합니다.
   과거의 다른 기준 브랜치에서 성공한 기록만으로 이번 통합의 성공을 판정하지 않습니다.
3. 통합 diff를 검토한 후 별도 병합 지시에 따라 병합합니다. 기존 개별 커밋을 main 이력에 남기려면 `Create a merge commit`을 사용합니다.
4. 병합 후 main push CI 6개의 완료와 README·워크플로의 기본 브랜치 반영을 확인합니다.
5. #5~#8의 커밋이 main에 포함됐는지 확인하고, 자동으로 닫히지 않은 참고 PR은 포함 완료로 닫습니다.
   원본 브랜치와 검토 이력은 이 단계에서 임의 삭제하지 않습니다.

병합 경로 정리 작업은 #9의 대상·설명과 이 문서를 준비하는 범위입니다.
main 직접 커밋·보호 우회·실제 PR 병합·참고 PR 닫기는 수행하지 않습니다.

초기 통합 후 새 기능 PR은 CI가 포함된 최신 main에서 분기해 main을 대상으로 작성합니다.
후속 작업에서도 위 필수 검사 이름과 보호 조건을 유지합니다.
