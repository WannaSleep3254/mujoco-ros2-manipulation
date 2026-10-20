# 양팔 간섭: 단독 정상·무회피 접촉·공동 회피 비교

[README](../README.md) · [첫 회피 시도 원본](interference_avoidance.md) · [검증 안내](verification.md)

회피 대상은 **상대 로봇팔**입니다. 고정 장애물은 추가하지 않았습니다.
첫 영상에서 두 팔이 함께 움직여 어떤 대상을 피하는지 분명하지 않았던 점을 보완했습니다.
같은 장면·시작·목표·고정된 궤적 묶음으로 네 번 새 시뮬레이션을 실행하고 세 조건을 비교합니다.

| 조건 | 영상 위치 | 명령과 판정 |
|---|---|---|
| 1. 각각의 정상 동작 | 위 왼쪽 FR5 / 위 오른쪽 UR5e | 한 팔만 독립 궤적 실행, 상대 팔은 시작 자세 유지. 각각 목표 도달·접촉 0 |
| 2. 회피 없는 충돌 동작 | 아래 왼쪽 | 위 두 독립 궤적을 같은 시각에 실행. 손목 간 실제 접촉으로 양팔 자동 정지 |
| 3. 간섭 회피 동작 | 아래 오른쪽 | 같은 시작·목표를 사용하는 `both_arms` 공동 궤적. 양팔 완료·접촉 0 |

## 실제 비교 영상

[![세 조건 비교 — 실제 접촉 후 양팔 정지](interference_comparison/contact.png)](interference_comparison/comparison.mp4)

[비교 영상 재생 — 약 24.38초](interference_comparison/comparison.mp4) ·
[단독 목표 도달 시점](interference_comparison/normal-complete.png) ·
[회피 목표 도달 시점](interference_comparison/avoidance-complete.png)

왼쪽 흰색 로봇이 FR5, 오른쪽 파란 관절 캡 로봇이 UR5e입니다.
각 칸은 **독립 실행**이며, 지령 시작 시각을 맞춰 원래 속도로 재생합니다.
MuJoCo 창의 고정 영역을 확대하고 한글 설명을 추가했습니다. 로봇 픽셀의 생성·위치 변경이나 시간 배속은 없습니다.
영상 앞 0.5초는 시작 대기입니다. 상대 팔이 고정된 위 두 칸과 실제로 맞닿아 멈춘 아래 왼쪽을 먼저 비교한 뒤,
아래 오른쪽에서 팔을 들어 자세와 타이밍을 바꾸며 통과하는 동작을 확인할 수 있습니다.

### 이번 GUI 실행의 측정값

| 조건 | 원래 궤적 길이 | 실제 결과 | 물리 스텝 | 원본 보고서 / 영상 |
|---|---:|---|---:|---|
| FR5 단독 | 6.317초 | 성공(4), 접촉 0 | 18,001 | [JSON](interference_comparison/left.json) / [MP4](interference_comparison/left.mp4) |
| UR5e 단독 | 6.317초 | 성공(4), 접촉 0 | 18,016 | [JSON](interference_comparison/right.json) / [MP4](interference_comparison/right.mp4) |
| 무회피 동시 | 6.317초 | 약 3.08초에 접촉, 양팔 취소(5,5)·비활성화 | 6,528 | [JSON](interference_comparison/collision.json) / [MP4](interference_comparison/collision.mp4) |
| 공동 회피 | 21.898초 | 양팔 성공(4,4), 접촉 0 | 17,991 | [JSON](interference_comparison/avoidance.json) / [MP4](interference_comparison/avoidance.mp4) |

접촉 링크는 `left_wrist2_link`와 `right_wrist_2_link`입니다. 첫 접촉 스텝에서 활성 접촉점 3개,
최대 침투 약 0.31mm를 기록했고 이후 추가 물리 적분 없이 양팔 위치를 유지했습니다.
명령은 **3.24초 남아 있었으며**, 목표 완료로 멈춘 것이 아닙니다. 양팔 정지 후 최대 관절 변화는 0rad이고,
원래 두 궤적의 종료 시각과 가장 긴 회피 궤적 종료 뒤까지 계속 관찰했습니다.
검증기 목표 취소 요청·접촉 주입 요청은 0회입니다. 두 제어기의 정지는 기존 네이티브 감시·ROS guard가 처리합니다.

공동 회피 궤적은 경로와 시간이 달라 21.90초입니다. 독립 경로보다 빠르거나 최단이라는 주장은 하지 않습니다.
같은 목표를 향해 움직이지만 상대 팔을 고려해 **다른 경로와 타이밍**을 사용하는 첫 비교 사례입니다.

## 조건 일치와 원본 증거

- 시작 관절 명령, 목표 제약, 준비·팔별·공동 궤적의 모든 점·속도·가속도를 한 번 고정합니다.
  네 실행의 `bundle_sha256`, `command_hashes`, 사례·모델 명세 해시가 모두 같아야 비교가 통과합니다.
  [궤적 묶음](interference_comparison/bundle.json.gz) · [비교 판정·프레임 출처](interference_comparison/comparison.json)
- 무작위 계획을 네 번 다시 생성하지 않습니다. 각 실행은 같은 준비 궤적으로 같은 명령 시작 상태에 도달한 뒤 출발합니다.
  실제 측정 자세에는 중력·추종 오차가 있으므로 별도로 기록합니다. 단독 실행의 상대 팔 변화도 0.01rad 미만이어야 합니다.
- 매 실행마다 단독 궤적은 각각 안전, 병렬 합성은 실제 팔 간 간섭, 공동 회피 궤적은 안전임을 다시 검사합니다.
  실제 JTC spline 보간과 20ms 이하·관절 변화 0.025rad 이하의 샘플 검사를 사용합니다.
- 접촉 비교 전용 CLI만 MuJoCo 플러그인·시뮬레이션 시계·활성 자동 정지를 확인한 뒤 의도적 충돌 궤적을 전송합니다.
  일반 검증·제어 경로의 위험 지령 차단은 유지합니다. 실제 하드웨어에 사용하는 실행기가 아닙니다.
- 실제 실행은 매 물리 스텝을 감시합니다. 접촉 모드는 완료된 네이티브 보고서의 실행 ID·소스 해시·접촉 쌍·표면 접근·정지 스텝·고정 유지를 검사합니다.
  나머지 모드는 접촉 0과 래치 없음을 요구합니다. 새 launch마다 정지 상태를 초기화하며 실행 중 래치를 해제하지 않습니다.
- 촬영 원본은 각 623프레임·24fps입니다. 비교 영상은 원본의 12번 프레임부터 585프레임을 사용하며,
  원본 길이가 부족하면 정지 프레임을 임의로 채우지 않고 실패합니다. 시계 수신과 프레임 캡처 간 수십 ms 차이가 있어
  자막 시점은 제어기 반응 지연 측정값으로 사용하지 않습니다.
- [보관 명세](interference_comparison/published_manifest.json)에 영상·보고서·이미지·압축 로그의 해시가 있습니다.
  로그는 무손실 gzip으로 보관하며 압축 해제 원본 해시도 기록합니다.
  [FR5 로그](interference_comparison/left.log.txt.gz), [UR5e 로그](interference_comparison/right.log.txt.gz),
  [실제 접촉 로그](interference_comparison/collision.log.txt.gz), [회피 로그](interference_comparison/avoidance.log.txt.gz)

## 재현

```bash
source ./manipulation_env.sh
# 먼저 새로운 정상 계획·검사·실행을 완료합니다.
python3 verify_interference.py --domain 126
python3 verify_interference_comparison.py \
  --freeze-report runtime/validation/interference_dual_arm_crossing_fr5_ur5e_headless.json \
  --bundle runtime/validation/new-comparison-bundle.json
for mode in left right collision avoidance; do
  python3 verify_interference_comparison.py --mode "$mode" --domain 129 \
    --bundle runtime/validation/new-comparison-bundle.json
done
python3 scripts/compare_interference.py --summary-only \
  --output-dir runtime/validation/new-comparison-summary
```

촬영하려면 각 실행에 `--gui --record-dir runtime/validation/new-capture-<모드>`를 추가합니다.
그 뒤 `scripts/compare_interference.py --gui --output-dir runtime/validation/new-comparison-video`를 실행합니다.
출력 경로는 새 `runtime/` 하위 경로를 사용합니다. X11·시스템 OpenCV의 H.264·Pillow·Noto CJK 글꼴을 사용합니다.
로컬에서 GUI 네 실행과 별도의 headless 네 실행 및 비교 판정이 통과했습니다.
단위 82개·모델 18개, 총 100개 검사 통과. 패키지 빌드·설치 후 저장소 외부 CLI 도움말도 확인했습니다.
[headless 비교 판정](interference_comparison/headless-comparison.json)에 네 실행 ID와 보고서 해시를 보관합니다.
CI는 GUI 없이 네 조건과 비교 판정을 실행하고 원본 보고서를 보관합니다.

Python MuJoCo 정적 모델 검사 버전은 3.14.0, 실제 ROS 시뮬레이션 엔진은 3.12.0입니다.
이 자료는 시뮬레이션에서 상대 팔의 간섭을 피하는 관절 공간 사례이며, 끝단 Pose/IK·도구·접촉 조립과 실제 로봇 제동 검증은 남아 있습니다.
