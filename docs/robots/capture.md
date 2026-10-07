# 모델 자료 촬영과 재생성

[모델 자료 목록](README.md) · [프로젝트 README](../../README.md)

모델마다 초기 자세 렌더 1장과 실제 GUI 스크린샷 2장을 보관합니다.
`reference_image.json`에는 초기 관절값·카메라·Python MuJoCo 버전을,
`gui_capture_validation.json`에는 촬영한 실행의 ROS 상태·계획·궤적·종료 결과를 기록합니다.
`assets.json`은 각 이미지의 출처, 촬영 시각과 SHA-256 해시를 연결합니다.

## 초기 자세 렌더

모델과 제조사 소스를 먼저 준비합니다. 명령은 저장소 루트에서 실행합니다.

```bash
.venv/bin/python capture_robot_images.py
.venv/bin/python capture_robot_images.py --robot ur5e
```

첫 명령은 세 모델을, 두 번째는 UR5e만 생성합니다. EGL로 MuJoCo Python
3.14.0을 실행해 `docs/robots/<모델>/images/home.png`에 저장합니다.
형상의 경계로 카메라를 배치하며 해상도는 1000×750입니다. 기준 렌더에서는
그림자를 끕니다. 모델의 물리·시각 형상을 변경하거나 이미지 픽셀을 보정하지 않습니다.
`home_rad`의 키프레임에서 렌더하므로 ROS 노드나 물리 정착을 실행한 사진으로
해석하지 않습니다. 오래된 프로필로 생성한 모델은 재생성해야 합니다.

## 실제 GUI 스크린샷

Ubuntu의 X11 데스크톱, `xprop`, `xwd`, `libX11` 및 Python Pillow가 필요합니다.
현재 PC에서는 약 1850×950 크기로 촬영합니다. 데스크톱 터미널에서
ROS 환경을 로드한 뒤 시스템 Python으로 실행합니다.

```bash
source ./manipulation_env.sh
python3 verify_robot.py --robot fr5 --domain 92 --gui \
  --screenshot-dir docs/robots/fr5/images
python3 verify_robot.py --robot fr10 --domain 90 --gui \
  --screenshot-dir docs/robots/fr10/images
python3 verify_robot.py --robot ur5e --domain 91 --gui \
  --screenshot-dir docs/robots/ur5e/images
```

명령은 모델을 하나씩 실행합니다. 제어기 활성화, 직접 궤적, MoveIt ROS API
계획·실행, 관절 오차 안정화, TF·시계를 검사한 뒤 MuJoCo와 RViz 창을 촬영하고
launch를 정상 종료합니다. `--screenshot-dir`는 `--gui`와 함께 사용해야 합니다.

촬영 함수는 이번 launch 프로세스 그룹의 PID와 X11 창의 PID를 대조합니다.
이 그룹에 속한 MuJoCo/RViz 창만 크기를 조정하고 앞으로 가져와 촬영합니다.
FR5의 카메라는 로봇 전체가 보이도록 줌을 조정합니다. 촬영 후 포인터 위치를
복구하며, 전체 데스크톱·터미널·브라우저를 캡처하지 않습니다.
MuJoCo 촬영 중에는 해당 실행의 RViz 창을 잠시 최소화하고, 촬영할 창을 활성화합니다.
`xwd`로 읽은 창 픽셀을 PNG로 인코딩하며 색 보정·외형 편집을 수행하지 않습니다.

GUI의 엔진은 로컬 ROS 런타임 MuJoCo 3.12.0입니다. 초기 자세 렌더와 버전·관절
자세·카메라가 다릅니다. RViz의 주황색 계획 요청 모델과 반투명 현재 상태가
함께 나타날 수 있습니다. 각도가 정확히 같은지는 `/joint_states`와 검증 기록으로 확인합니다.

## 검증 기록과 이미지 목록 갱신

검증기는 `runtime/validation/<모델>_gui.json`에 결과를 저장합니다. 촬영 후 다음을 실행합니다.

```bash
.venv/bin/python capture_robot_images.py --collect
.venv/bin/python capture_robot_images.py --robot ur5e --collect
```

첫 명령은 세 모델의 기록을, 두 번째는 UR5e만 모읍니다.
`motion_checks_passed`, `shutdown_clean`, `process_group_gone`이 모두 `true`이고
`launch_return_code`가 `0`인 결과를 모델 폴더의 `gui_capture_validation.json`으로 보관합니다.
`robot_profile_sha256`이 현재 프로필과 같은지, PNG 해시·해상도가 원본 촬영 기록과
같은지도 검사합니다. 검사가 실패하면 해당 모델 자료의 수집을 중단합니다.

이번에 정리한 `assets.json`에는 각 PNG의 경로·크기·해시와 출처 JSON을 기록했습니다.
사진을 다시 촬영하면 검증 JSON과 이미지 목록을 함께 갱신해야 합니다.
기존 `docs/fr5_shutdown_validation.json`, `docs/fr10_validation.json`,
`docs/ur5e_validation.json`의 과거 실행 시각을 새 촬영 날짜로 바꾸지 않습니다.

원본 제조사 메시·로컬 XML/URDF·ROS 빌드·실행 로그는 Git에서 제외하고,
모델별 안내·검증 요약·PNG·촬영 메타데이터를 저장소에 포함합니다.
