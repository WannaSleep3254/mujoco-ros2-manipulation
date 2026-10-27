# 설치 환경을 실행 타깃으로 연결하기

설계 파일에서 만든 환경을 MuJoCo, MoveIt, 명령 서버가 함께 사용하도록 연결합니다. 로봇과 제어기의 기본 Workcell 계약을 유지하고, 설치 환경의 출력 디렉터리와 SHA를 별도의 실행 타깃으로 고정합니다. 뷰어의 모델 미리보기 선택은 실행 타깃을 바꾸지 않습니다.

현재 구현은 시뮬레이션 전용입니다. 아래 명령의 실제 실행·검토·승인·취소 결과는 별도 검증 보고서로 확인해야 합니다. 코드와 단위 테스트만으로 실제 ROS 실행이 검증되었다고 기록하지 않습니다.

## 시작할 때 환경과 SHA를 함께 지정합니다

먼저 [설계 파일 가져오기 절차](environment_design_import.md)에 따라 새 `environment-overlay-v1` 산출물을 만듭니다. 기존 기본 모델이나 실행 중인 환경을 덮어쓰지 않습니다. 생성 결과의 실제 `manifest_sha256`을 사용합니다.

```bash
# 새로 선택한 동일 환경으로 물리 시뮬레이터와 MoveIt을 시작합니다.
bash run_scene.sh workcell:=fr5_ur5e gui:=false rviz:=false \
  execution_environment:=/absolute/local/environment_directory \
  execution_manifest_sha256:=<overlay.json의 실제 SHA256>

# 같은 ROS domain에서 명령 서버에도 정확히 같은 쌍을 지정합니다.
python3 -m mujoco_ros2_manipulation.workcells.motion_server \
  --workcell fr5_ur5e \
  --execution-environment /absolute/local/environment_directory \
  --execution-manifest-sha256 <overlay.json의 실제 SHA256>
```

셸에서 `<...>`를 그대로 실행하지 말고 생성 결과의 64자리 소문자 SHA로 바꿉니다. ROS domain은 실행 중인 다른 작업과 겹치지 않는 값으로 선택합니다.

두 인자는 반드시 함께 지정합니다. 둘 다 생략하면 기존 기본 모델을 사용합니다. 웹 요청이나 명령 메시지에는 경로·SHA를 넣지 않으며, 실행 중 다른 환경으로 바꾸려면 기존 실행을 종료한 뒤 새 타깃으로 시작합니다.

## 같은 모델을 사용하는지 확인합니다

시작 시 다음 과정을 거칩니다.

1. 기존 기본 Workcell과 `source.json`을 검사합니다.
2. 명시한 SHA에 맞는 `overlay.json`, 로봇·관절 계약, 설치 변환, 전체 출력과 외부 메시 의존성을 검사합니다.
3. 소유한 별도 CPU worker에서 실제 형상과 home 접촉 여부를 확인합니다. worker의 nonce·소스 SHA·영수증이 일치해야 합니다.
4. 검사 전후의 동일성을 확인한 뒤 타깃을 고정합니다. 이후 재검사는 파일·의존성 해시 검사이며 CAD 변환이나 물리 적분을 반복하지 않습니다.

MuJoCo와 상태 발행자는 타깃의 URDF를 사용하며, URDF의 `mujoco_model`은 타깃 MJCF를 가리킵니다. MoveIt에도 같은 URDF·SRDF를 전달합니다. native 거리 검사기는 타깃 MJCF를 별도로 열어 두 팔 사이 거리와 고정 설비까지의 거리를 계산합니다.

명령 서버는 준비 시, 각 요청의 시작 시, 실제 전송 직전에 다음 값을 대조합니다.

- 타깃의 manifest·출력·메시 의존성 SHA
- 실제 ROS 하드웨어의 MuJoCo 모델 경로와 로봇 설명
- MoveIt의 로봇 설명과 SRDF
- 계획 장면의 형상·부착물·허용 충돌 행렬·패딩·고정 변환
- 환경 거리 정책과 검토 계획의 identity

환경이 달라지거나 파일이 바뀌면 새 목표를 전송하지 않고 서버를 차단합니다. 기존 검토 계획은 재사용할 수 없습니다. 타깃을 다시 검사하고 서버를 재시작해야 합니다. 환경을 명시하지 않은 기존 자동 명령은 기존 실행 흐름을 유지합니다.

현재 ROS identity 검사는 게시된 설명과 모델 경로, 고정된 파일의 일치를 확인합니다. native 시뮬레이터 메모리의 컴파일 모델 전체를 별도 서비스로 증명하는 기능은 아닙니다. 실행 중 모델 교체나 GUI 재로드는 이 절차에 포함하지 않습니다.

## 기록과 검증 범위

`ExecutionTarget.description()`은 로컬 파일 경로와 원본 설계 형상 없이 타깃 ID, 기본 Workcell ID, manifest·출력·의존성 identity를 반환합니다. 명령 세션·상태·요청 기록의 `execution_target`에 이를 추가합니다. 기존 Workcell 설명과 명령 메시지 v1은 유지합니다.

`overlay.json`에 기록한 생성 당시의 아티팩트 검증 범위는 수정하지 않습니다. 실제 실행 결과는 별도 보고서에 남깁니다. 가짜 `source.json`을 만들어 overlay를 기존 생성물로 취급하지 않습니다.

대표 검증은 직접 생성한 `synthetic_negative_box` STEP을 사용합니다. 이 형상은 크기 100×200×300 mm, 시작 좌표 [-50,-100,-150] mm인 상자입니다. `config/environments/synthetic_negative_box.import.json`의 Z-up 설정으로 실제 FreeCAD 변환을 수행합니다. 기본 환경 아티팩트 검증의 `synthetic_asymmetric_box_v1`은 별도의 축·원점을 가진 fixture입니다.

실제 실행 검증에는 다음 근거가 필요합니다.

- 설치 형상·단위·원점·시각/충돌 변환을 CPU와 native MuJoCo에서 대조
- home 및 계획 시작 상태가 환경 거리 정책을 충족
- 동일 타깃에서 계획·검토·승인·실행·취소·홈 복귀 결과와 관절 정착 확인
- 잘못된 SHA의 시작 거부와 실제 목표 전송 0회
- 검토 후 환경 변경, MuJoCo/MoveIt 환경 불일치 시 실제 목표 전송 0회
- native 접촉 기록, action 목표 UUID와 종료 관측, 소유 프로세스 종료 확인

현재 충돌 형상은 가져온 설계의 바운딩 박스를 감싼 보수적인 box입니다. 오목한 형상의 빈 공간까지 막을 수 있습니다. 별도의 환경 여유 거리 정책과 추종 오차 여유를 사용하며, 팔 사이 거리 정책으로 대체하지 않습니다. 샘플링한 거리와 여유값은 연속 시간 충돌 없음이나 실물 안전을 보장하지 않습니다.

실행 타깃은 접촉·시계 고장 주입 또는 별도의 시연용 모델과 함께 사용할 수 없습니다. 이러한 옵션으로 타깃 MJCF가 조용히 교체되는 것을 막기 위한 제한입니다.

## 실제 실행과 거부 사례

2026년 10월 27일 직접 생성한 STEP을 실제 FreeCAD로 변환해 검증했습니다. CPU MuJoCo 3.14.0과 native MuJoCo 3.12.0의 설치 형상 차이는 1 µm 이하였으며, 추가 box의 충돌 쌍도 native 평가에 포함되었습니다.

| 사례 | 결과 |
| --- | --- |
| 설치 환경에서 계획·검토·승인·홈 복귀·움직임 후 취소·다시 복귀 | 통과. 생산 경로로 8개 action 목표를 전송하고 종료·환경 거리·정착을 확인했습니다. |
| 잘못된 manifest SHA | ROS context나 시뮬레이터를 만들기 전에 거부했습니다. 목표 전송은 0회입니다. |
| 검토 후 환경 파일 변경 | 검증용으로 별도 합성한 타깃만 변경했습니다. 기존 검토를 무효화하고 서버를 차단했으며 목표 전송은 0회입니다. 원래 입력 환경은 보존했습니다. |
| base 시뮬레이터와 overlay 명령 서버의 불일치 | 준비 단계에서 거부했으며 목표 전송은 0회입니다. |

위 정상 실행의 native 접촉은 0회였고, 프로필의 0.01 rad 정착 기준과 소유 프로세스 정리를 확인했습니다. 원래 실패한 검증 기록도 보존했으며 생산 정책은 바꾸지 않았습니다.

## imported box의 실제 접촉 정지 진단

`verify_environment_contact`는 명시적으로 허용한 시뮬레이션에서만 실행하는 별도 진단 CLI입니다. 생산 계획기·승인 API·뷰어에는 충돌 경로를 허용하는 예외가 없습니다.

```bash
python3 -m mujoco_ros2_manipulation.workcells.verify_environment_contact \
  --domain <사용하지 않는 domain> \
  --execution-environment <별도로 만든 synthetic contact 환경> \
  --execution-manifest-sha256 <그 환경의 실제 SHA256> \
  --allow-simulated-contact
```

이 진단에서는 같은 STEP box를 팔의 이동 범위 가까이에 설치한 별도 타깃을 사용합니다. 해당 타깃의 SHA를 유지한 채 먼저 **모든 생산 검사**를 거치는 반대 방향의 작은 계획·검토·승인·홈 복귀와 움직임 후 취소·홈 복귀를 수행합니다. 마지막 접촉 단계에서만 왼팔 첫 관절 +0.6 rad, 오른팔 첫 관절 +0.08 rad를 4초에 걸쳐 움직이는 고정된 raw `FollowJointTrajectory` 두 개를 직접 보냅니다. 이 마지막 단계는 계획을 의도적으로 우회하는 접촉 진단입니다.

raw 전송 전 실제 시작 자세의 환경 여유가 15 mm 이상인지, 1,001개 native FK 표본에서 팔 사이 여유가 30 mm 이상인지, 지정한 imported box와 실제로 겹치는 경로인지를 확인합니다. 전송할 두 메시지의 관절 순서·첫/끝 자세·명목 시간 [0, 4]초·위치만 사용하는 선형 보간 조건을 FK 입력과 정확히 대조합니다. FK 표본은 **명목 궤적 시간**에 대한 계산이며, 지연 실측이나 ROS/native 물리 step의 정확한 대응을 의미하지 않습니다.

접촉 단계에서는 검증기가 먼저 취소하지 않습니다. native latch, ROS guard의 양 제어기 비활성화, 두 action의 취소/중단 종료, 정지 후 자세 변화와 속도 0, 계속 진행하는 ROS 시계를 각각 확인합니다. 마지막에는 기존 `check_stop_physics`로 전체 native step·접촉·hold 기록을 검사하고, 실제 접촉 body가 imported box인지 확인합니다. 생산 단계의 취소 요청과 raw 접촉 단계의 검증기 취소 횟수는 별도로 기록합니다. 종료·소유 프로세스 정리도 action 종료와 별도 항목입니다.

최종 검증에서는 **같은 타깃 SHA**에서 생산 목표 8개와 마지막 raw 목표 2개를 실행했습니다. 두 raw 메시지의 FK anchor 계약이 정확히 일치했고, native 7,691 step 중 `left_wrist3_link`와 `environment_contact_box` 사이에서 활성 접촉 1회가 발생했습니다. native latch 후 두 action은 취소 상태(5)로 종료했고 양 제어기는 inactive가 되었습니다. raw 접촉 단계의 검증기 취소 요청은 0회였습니다. 정지 후 자세 변화와 속도는 0으로 관측됐고 ROS 시계는 약 0.6초 계속 진행했습니다. 전체 native 검사와 소유 프로세스 정리도 통과했습니다.

이 결과는 생성한 단일 STEP box와 두 로봇의 해당 설치·고정 진단에 대한 것입니다. 임의 CAD 형상, 연속 시간 충돌 없음, 지연 상한이나 실물 안전을 검증한 결과로 확대하지 않습니다.
