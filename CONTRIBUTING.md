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
