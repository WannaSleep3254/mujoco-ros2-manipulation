# 이미지 입력과 상태 분석

현재 Three.js 뷰어를 유지하면서 이미지 입력, 요청별 시간축과 표시 모델의 객체 정보를 추가했습니다. 모델 렌더 이미지와 외부 이미지 토픽은 서로 다른 공급원입니다. 센서 입력은 선택 기능이며, 등록하지 않아도 기존 계획·승인·실행 흐름을 사용합니다.

## 실행과 입력

```bash
source ./manipulation_env.sh
python3 -m mujoco_ros2_manipulation.operator_console \
  --domain 89 --port 8897 --image-topic /camera/image
```

명시적으로 등록한 `sensor_msgs/Image` 토픽 하나의 RGB8 입력만 지원합니다. 원천 시계의 기본값은 `unknown`입니다. 발행자가 ROS 시뮬레이션 시각으로 acquisition stamp를 작성한다고 확인한 경우에만 `--image-clock-domain ros_simulation`을 지정합니다. `/clock`을 발견했다는 이유로 원천 시계를 추정하지 않습니다.

| 경계 | 현재 상한 |
| --- | --- |
| 입력 크기 | 640 × 480, stride 1,920 bytes, 원본 921,600 bytes |
| 대기 원본 | 최신 입력 1개와 변환 중 입력 1개 |
| JPEG 변환 | 최대 5 Hz, JPEG 하나 512 KiB |
| 이미지 보존 | 최대 64개, 합계 8 MiB |
| 수신 메타데이터 | 최대 256개 |

ROS 콜백은 입력 검사와 최신 원본 교체만 수행합니다. JPEG 변환은 별도 작업자가 담당합니다. 대기 중 덮어쓰기, 거부, 변환 실패와 보존에서 제거한 개수는 로컬 관측이며 DDS 손실률이 아닙니다. 수신 순번도 발행자의 원천 프레임 번호가 아닙니다.

이미지 acquisition stamp와 ROS `/clock` 수신은 음수가 아닌 ROS int32 초 범위인 0~2,147,483,647과 0~999,999,999 나노초를 검사합니다. 현재 epoch 시각도 수용하며 음수 시각은 이 입력 정책에서 거부합니다. ROS 경로는 정수 초·나노초를 직접 합치고, `/clock`은 `observe_clock_stamp(sec, nanosec)`를 사용합니다. 이 경로의 정확한 `source_stamp_ns`, `observed_clock_ns`는 JavaScript 정수 정밀도를 잃지 않도록 십진 문자열로 전달합니다. 초 단위 필드는 표시용 값입니다.

`observe_clock(seconds)`는 기존 호출을 위한 float 초 호환 API입니다. 입력이 이미 float로 반올림됐다면 원래 ROS 나노초를 복원할 수 없으므로 정수 구성 경로와 같은 정밀도를 보장하지 않습니다. ROS 콜백은 이 호환 경로로 초와 나노초를 합치지 않습니다. 어느 경로에서도 `/clock`의 존재만으로 이미지 원천 시계를 추정하지 않습니다.

발행자가 하나일 때만 이미지를 받습니다. 단절·중복 발행·endpoint 변경과 명시적인 새 실행은 수신 세대를 바꾸고 이전 이미지 요청을 거부합니다. 설치된 Humble의 한 인자 메시지 콜백과 discovery endpoint를 사용하므로 메시지마다 송신자를 증명하는 기능은 아닙니다. 같은 acquisition stamp의 반복 수신은 신선도를 갱신하지 않습니다. 시계 역행은 새 실행·공급원 세대가 확인될 때까지 입력을 유효하게 복원하지 않습니다.

## 읽기 전용 패널

`센서·상태 분석`에서 마지막 이미지와 보존한 수신 순번, 요청 이벤트, 현재 수신 상태, 표시 모델의 body·geom 관계를 확인합니다. 보존에서 제거된 이미지나 이전 세대 요청은 최신 이미지로 대체하지 않습니다. 패널과 탭이 숨겨졌을 때 이미지·계층을 추가 조회하지 않으며, 오래된 비동기 응답은 적용하지 않습니다.

- `GET /api/image-source`: 제한된 수신 메타데이터와 상태입니다.
- `GET /api/image-source/frame?generation=…&sequence=…`: 해당 세대·순번의 JPEG입니다. 보존 범위 밖이면 410입니다.
- `GET /api/analysis/<request_id>`: 이 콘솔에서 관측한 요청의 마지막 128개 이벤트와 현재 공급원 관측입니다.
- `GET /api/view/hierarchy?binding_id=…`: 현재 표시 모델과 binding에 묶인 객체 정보입니다.

일반 상태 조회에는 이미지 메타데이터 전체 이력을 반복해서 넣지 않습니다. 보존 이력은 패널이 보일 때 최대 초당 한 번 조회합니다. 요청 분석은 동시에 하나씩, 최소 1.5초 간격으로 조회합니다. 모든 경로는 읽기 전용이며 공급원 재연결이나 기록 선택으로 명령을 재전송하지 않습니다.

요청 경과 시간은 기록 프로세스의 monotonic 시간입니다. 이미지 acquisition stamp, 관절 원천 stamp와 `/clock`은 각자 보존합니다. 현재 공급원의 시각과 나이는 선택한 과거 요청 당시의 값이 아니며 자동으로 보간·동기화하지 않습니다. 이 관측으로 제어 지연이나 같은 native step을 측정했다고 해석하지 않습니다.

객체 계층은 화면을 만든 같은 MuJoCo 컴파일 결과의 `body_parentid`와 `geom_bodyid`에서 가져옵니다. 로봇 소유권은 명시한 정확한 root body의 하위 트리로 정합니다. 공구·TCP·센서 역할은 이름으로 추측하지 않고 `unknown`으로 남깁니다. 모델·binding 변경 시 이전 객체 선택을 무효화합니다.

## 검증과 현재 범위

`verify_image_transport`는 별도 도메인에서 진단 패턴을 실제 ROS Image로 발행하고, 콘솔의 수신·JPEG 변환·HTTP 조회를 확인합니다. 잘못된 인코딩 거부, 발행자가 남아 있는 상태의 stale, 실제 시뮬레이션 명령과 요청 분석, 발행자 재연결 후 세대 변경·이전 이미지 거부·명령 재전송 없음을 검사합니다. 소유한 콘솔·시뮬레이션을 종료하고 정리 결과도 별도로 기록합니다.

```bash
source ./manipulation_env.sh
python3 -m mujoco_ros2_manipulation.operator_console.verify_image_transport \
  --domain 221 --port 8915 --output runtime/image-transport-new
```

비어 있는 도메인과 사용하지 않는 포트, 존재하지 않는 출력 디렉터리가 필요합니다. 진단용 합성 이미지의 전송 검사이며 실제 카메라, 보정·센서 정확도, 하드웨어 시간 동기화, 장시간 부하 성능을 검증한 결과는 아닙니다. 점군·프로파일 입력과 공구 의미 정보는 후속 확장입니다.

[작업·계획·실행 규약](operation_contract.md) · [공통 표시 공급원](viewer_sources.md) · [설치 환경 실행](environment_execution.md) · [작업리스트](development_worklist.md)
