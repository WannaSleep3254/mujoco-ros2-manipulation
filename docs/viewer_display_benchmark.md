# 움직이는 기록을 이용한 뷰어 성능·안정성 검증

2026-10-26의 정지 자세 예비 비교는 그대로 보존합니다. 이 문서는 같은 해상도와 카메라에서 움직이는 기록을 재생하는 별도 검증 도구를 설명합니다. 원래의 JPEG 비동기 작업자나 ROS 제어 경로 전체의 성능을 측정하는 도구는 아닙니다.

## 재현 방법

MuJoCo, Pillow, NumPy가 설치된 Python과 Playwright를 사용할 수 있는 Node, Chrome이 필요합니다. 브라우저나 패키지를 자동으로 설치하지 않습니다.

```bash
python scripts/viewer_display_benchmark.py \
  --output /tmp/viewer-display-example \
  --node /path/to/node \
  --playwright-module /path/to/node_modules/playwright \
  --chrome /usr/bin/google-chrome \
  --repeats 3 --seconds 15 --soak-seconds 120
```

`--hardware-gl`을 추가하면 격리 Chrome에 `--use-gl=angle --use-angle=gl --disable-software-rasterizer`를 적용합니다. 소프트웨어 WebGL이 선택되면 시험을 실패 처리합니다. 사용자 Chrome 프로필이나 OS 설정을 바꾸지 않습니다. 이 장비에서는 해당 옵션으로 NVIDIA RTX 3050 WebGL을 확인했습니다. 다른 장비에서 같은 GPU를 보장하는 옵션은 아니므로 JSON의 실제 renderer를 확인해야 합니다.

모델을 지정하지 않으면 두 관절의 단순 도형으로 만든 시험 장면을 사용합니다. 이를 실제 로봇 결과로 표시하지 않습니다. 준비된 로봇 모델을 사용하려면 다음 두 인자를 함께 전달합니다.

```bash
  --model /path/to/generated/cell.xml \
  --recording docs/reports/2026-10-26-viewer-moving-display/recording.json
```

기록의 스키마는 `schema_version: 1`, `joint_names`, `samples`이며 각 표본은 증가하는 `time_sec`와 모든 관절을 포함한 `positions`를 갖습니다. 시각은 초, 회전 관절은 rad, 직선 관절은 m입니다. 기록은 조회 시점에 가장 가까운 이전 표본을 선택하여 반복 재생합니다. 새 로봇 동작을 요청하거나 물리를 적분하지 않습니다.

각 방식은 새 로컬 HTTP 서버, 새 Chrome 프로세스와 임시 프로필에서 실행합니다. 홀수 회차는 WebGL→JPEG, 짝수 회차는 JPEG→WebGL 순서입니다. 초기 준비 후 1초를 기다리고, 기본 15초 구간의 데이터를 측정합니다. GPU 드라이버와 파일시스템 캐시 또는 기존 사용자 작업은 초기화하지 않습니다.

초기 표시 측정은 서버의 모델 준비와 Chrome 실행이 끝난 뒤 시작합니다. 서버 모델 컴파일·EGL 초기화와 Chrome 프로세스 시작 비용을 포함한 전체 cold-start 측정은 아닙니다.

중단된 실행의 완료된 6개 시험을 보존하려면 같은 인자로 `--resume-trials`를 추가합니다. 기록·모델 해시, 해상도, 조회 주기와 구간 길이가 일치해야 기존 시험을 읽습니다. 안정성 시험은 새로 시작합니다. 최종 `evidence.json`은 이미 있으면 덮어쓰지 않습니다.

기본 비교와 안정성 시험을 나누어 실행하려면 먼저 `--baseline-only`를 추가하고, 종료 후 이 옵션을 빼고 `--resume-trials`를 추가합니다. 재개 시 하드웨어 선택 조건도 같아야 합니다.

## 통제한 조건과 측정의 범위

두 방식은 같은 XML, 관절 표본, `mj_forward`, 640×480 픽셀, DPR 1과 중앙 카메라 투영을 사용합니다. JPEG는 MuJoCo에서 요청마다 렌더링하고 품질 88로 인코딩합니다. WebGL은 현재 `LocalSceneView`에서 같은 기하와 자세를 표시합니다. 조명, 재질과 그림자 처리는 다릅니다.

| 지표 | 실제 측정한 대상 |
|---|---|
| WebGL FPS | CPU에서 제출한 표시 프레임. 같은 자세를 반복해서 그린 횟수 포함 |
| JPEG 표시 횟수 | 이미지 decode·canvas draw 후 다음 rAF에 도달한 횟수. 물리 화면의 paint 확인 아님 |
| 새 자세·이미지 | 성공적으로 적용한 서로 다른 조회 응답 |
| 첫 표시 | 장면 초기화부터 첫 WebGL 제출 또는 첫 JPEG decode·draw/rAF까지 |
| navigation→ready | 별도 Chrome에서 탐색 시작부터 시험 준비 확인까지. 자동화 확인 비용 포함 |
| 반복 본문량 | 측정 구간의 `/pose` 또는 `/frame` 응답 본문. 상태 API·HTTP 헤더·초기 자산 제외 |
| CPU | 시험 브라우저 또는 시험 서버와 자손 프로세스의 Linux CPU 시간. 코어 하나가 100% |
| RSS | 위 프로세스들의 RSS 합. 공유 페이지 중복 포함 |
| 휠 응답 | Chrome 자동화가 전달한 신뢰된 wheel 이벤트부터 WebGL CPU 제출 또는 JPEG decode·draw/rAF까지. 두 종료 지점이 달라 직접적인 paint 지연 비교 불가 |

`LocalSceneView`의 선택적 `onRenderFrame` 관찰 콜백은 제출 시각과 카메라 상태를 제공합니다. GPU 완료 시각이나 화면 출력 시각을 제공하지 않습니다. 콜백을 쓰지 않는 제품 경로에서는 카메라 배열 복사도 수행하지 않습니다.

## 안정성 확인

별도 WebGL 세션에서 기본 120초 동안 움직이는 기록을 표시한 뒤 다음 항목을 확인합니다.

- 소스를 멈추면 마지막 자세가 stale로 바뀌며 카메라 조작을 유지합니다.
- 새 표본을 받으면 현재 자세 표시로 복구합니다.
- 잘못된 자세와 모델 SHA가 일부 기하나 마지막 sequence를 덮어쓰지 않습니다.
- 같은 sequence로 신선도를 갱신하지 않습니다.
- 소유자가 새 epoch를 지정하면 이전 자세를 숨기고, 이전 epoch의 늦은 응답을 거절합니다.
- 새 epoch에서 기록을 되감고 sequence 1부터 다시 표시합니다.

새 epoch는 이 시험의 소유자가 명시적으로 지정합니다. 이 시험만으로 임의의 소스 시계 역행 감지나 실제 다른 로봇 모델의 제어 전환을 입증하지 않습니다. 서로 다른 모델 선택과 실제 공급원의 시계 처리는 별도 연결부의 검증 대상입니다.

## 2026-10-26 결과

하드웨어 조건을 맞춘 [NVIDIA WebGL 추가 비교](reports/2026-10-26-viewer-moving-hardware/results.md)를 우선 확인할 수 있습니다. [원본 근거](reports/2026-10-26-viewer-moving-hardware/evidence.json)에는 실행 옵션과 실제 renderer도 포함했습니다.

앞서 기본 headless 설정으로 측정한 [소프트웨어 렌더러 결과](reports/2026-10-26-viewer-moving-display/results.md), [그 원본 근거](reports/2026-10-26-viewer-moving-display/evidence.json)와 [고정 관절 기록](reports/2026-10-26-viewer-moving-display/recording.json)도 보존했습니다.

실제 FR5·UR5e 모델과 기존 출발 시점 조정 계획을 표본화한 8.622초 기록을 사용했습니다. 기록 재생은 표시 부하를 만들기 위한 것이며 새 계획·실행·접촉 판정은 하지 않았습니다. 기존 사용자 ROS 세션에는 연결하지 않습니다.

기본 headless Chrome은 SwiftShader 소프트웨어 렌더러를 선택했습니다. 별도 실행 옵션을 적용한 비교에서는 JPEG와 WebGL 모두 NVIDIA RTX 3050 사용을 확인했습니다. 두 결과를 섞지 않으며, 측정된 브라우저 CPU와 실제 렌더러를 함께 읽어야 합니다.

물리 마우스, Edge, Ubuntu의 간헐적인 입력 누락, 실제 input→display 지연, GPU 실행 시간·VRAM, 원격 네트워크와 수시간 이상의 연속 운용은 확인하지 않았습니다.
