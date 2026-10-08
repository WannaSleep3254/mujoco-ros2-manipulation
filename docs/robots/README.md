# 모델별 문서와 시각 자료

[프로젝트 README](../../README.md) · [설치와 빌드](../setup.md) · [검증 안내](../verification.md) · [자료 재생성 방법](capture.md)

FR5 → FR10 → UR5e 순서로 모델 변환과 ROS 2 제어 연동을 확장했습니다.
각 모델 페이지에 실행 명령, 프로필·관절 순서, 끝단 좌표계, 검증 결과와 스크린샷을 모았습니다.

| 모델 | 문서 | 계획 그룹 | 계획 끝단 | 자료 |
|---|---|---|---|---|
| FAIRINO FR5 V6 | [FR5 안내](fr5/README.md) | `fairino5_v6_group` | `wrist3_link` | [이미지 목록](fr5/assets.json) · [검증](../fr5_shutdown_validation.json) |
| FAIRINO FR10 V6 | [FR10 안내](fr10/README.md) | `fairino10_v6_group` | `wrist3_link` | [이미지 목록](fr10/assets.json) · [검증](../fr10_validation.json) |
| Universal Robots UR5e | [UR5e 안내](ur5e/README.md) | `ur5e_manipulator` | `tool0` | [이미지 목록](ur5e/assets.json) · [검증](../ur5e_validation.json) |

## 초기 자세 미리보기

| FR5 | FR10 | UR5e |
|---|---|---|
| [![FR5](fr5/images/home.png)](fr5/README.md) | [![FR10](fr10/images/home.png)](fr10/README.md) | [![UR5e](ur5e/images/home.png)](ur5e/README.md) |

각 모델의 `home_rad`를 MuJoCo Python으로 렌더했습니다. 로봇마다 형상에 맞춰
카메라를 배치하므로 이미지의 크기는 실물 크기 비교 기준이 아닙니다.
실제 MuJoCo/RViz GUI 스크린샷은 각 모델 페이지에서 확인할 수 있습니다.

## 모델별 구현 차이

| 항목 | FR5 | FR10 | UR5e |
|---|---|---|---|
| 제조사 모델 | FAIRINO FR5 V6 URDF | FAIRINO FR10 V6 URDF | 공식 UR Humble Xacro |
| 관절 이름 | `j1`–`j6` | `j1`–`j6` | `shoulder_pan_joint` 등 공식 6개 이름 |
| MuJoCo 시각 모델 | 제조사 STL | 제조사 STL | 원본 DAE에서 변환한 색상별 OBJ |
| SRDF 충돌 제외 쌍 | 11개 | 14개 | 11개 |
| 끝단 프레임 | `wrist3_link` | `wrist3_link` | `wrist_3_link → flange → tool0` |
| 개별 조정 | 기준 모델·종료 수정 | 링크·관성에 맞춘 별도 게인 | DAE 변환·공식 끝단 체인·시뮬레이션 MoveIt 패키지 |

세 모델 모두 `MoveIt 2 → JointTrajectoryController → mujoco_ros2_control → MuJoCo`
구조로 연결됩니다. 실행 모델은 `robot:=fr5`, `robot:=fr10`, `robot:=ur5e`로 선택합니다.
공통 기본 도메인은 89이므로 현재 모델을 종료한 뒤 다음 모델을 실행합니다.

## 자료 구성

```text
docs/robots/
├── README.md                        # 모델 목록과 비교
├── capture.md                       # 촬영·재생성 방법
├── fr5/
├── fr10/
└── ur5e/
    ├── README.md                    # 모델별 실행·설정·검증 안내
    ├── reference_image.json         # 초기 자세와 렌더링 조건
    ├── gui_capture_validation.json  # 촬영한 실행의 ROS 검증 결과
    ├── assets.json                  # 이미지 출처·촬영 시점·SHA-256
    └── images/
        ├── home.png                 # 초기 자세 기준 렌더
        ├── mujoco_gui.png           # 궤적 실행 후 실제 MuJoCo 창
        └── rviz.png                 # 궤적 실행 후 실제 RViz 창
```

세 모델 폴더는 동일한 구성을 사용합니다. 기존 `docs/*validation.json`은
실행 모드별 검증 기록으로 보존합니다. 로컬 상세 로그는 `runtime/validation/`에 있습니다.
이전 UR5e 시각 모델 수정의 [비교 시점 이미지](../images/ur5e_mujoco_visual.png)도 유지했습니다.

## 구현 범위와 다음 작업

현재 자료는 모델 변환, 위치 서보, ROS 2 궤적 실행, MoveIt 계획·실행과 종료 검증을 다룹니다.
모델마다 예제 목표와 게인이 달라 관절 오차를 모델 간 성능 순위로 해석하지 않습니다.

| 후속 작업 | 현재 상태 |
|---|---|
| MuJoCo·MoveIt의 바닥·장애물 장면 일치 | 미구현 |
| 고정 베이스·FR5/UR5e 공동 장면·공동 충돌 회피 | [작업계획 작성](../plans/base_and_multi_robot.md), 구현 전 |
| 그리퍼와 도구 TCP, 집기·이동·놓기 | 미구현 |
| 공통 작업 목표의 모델별 비교 표·영상 | 후속 검증 필요 |
| 접촉 조립·힘/순응 제어 | 미구현 |
| 실제 로봇 연결 | 미검증 |
