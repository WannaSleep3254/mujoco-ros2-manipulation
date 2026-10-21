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

## 보호된 main으로 변경 반영

초기 CI 도입과 선행 변경은 [PR #9 통합 경로](docs/integration.md)로 main에 반영했습니다.
#5~#8은 해당 PR에 커밋이 포함된 검토 이력이며 개별 병합하지 않습니다.
초기 통합 후 새 PR은 최신 main에서 분기해 main을 대상으로 작성합니다.
보호 규칙은 관리자에게도 적용하며, PR의 필수 검사 6개·최신 main 반영·미해결 대화 해결을 확인합니다.
CI 작업 이름을 바꾸면 보호 규칙의 필수 검사 이름도 함께 검토합니다.


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
CI는 검증된 정적 폴더를 Artifact로 보관하며 Pages를 배포하지 않습니다.

## 전용 조작 화면

`src/mujoco_ros2_manipulation/operator_console`에서 로컬 HTTP 화면과 ROS 어댑터를 관리합니다.
기존 명령 계약·안전검사를 우회하지 않으며, 자체 소유 프로세스만 종료합니다.
`source ./manipulation_env.sh` 후 `python3 verify_operator_console.py --domain 143 --no-render`로
실제 HTTP→ROS 동작·양팔 취소·복구와 접촉·종료를 검사합니다. 화면·EGL 렌더러는 로컬 브라우저에서 별도로 확인합니다.
