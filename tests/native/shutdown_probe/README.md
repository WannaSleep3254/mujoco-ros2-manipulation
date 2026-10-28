# 실제 JTC 종료 순서 probe

설치된 Humble `joint_trajectory_controller` 2.54.1의 실제 `update(Time const&, Duration const&)`
심볼을 시험용으로 감쌉니다. `ControllerManager`가 active 상태를 확인하고 가상 호출에
진입한 뒤, 원본 JTC 함수의 시작 직전에서 단 한 번 대기합니다. 이후 소유 native PID에만
SIGINT를 보내며 private `release` 파일로 호출을 풀어 줍니다. 로봇 목표를 전송하거나
컨트롤러·모델 설정을 변경하지 않습니다. 실물과 운영 프로세스에는 사용하지 않습니다.

`build_probe.py --output runtime/<새 폴더>`는 shim만 빌드하며 실제 설치 JTC 심볼의
`RTLD_NEXT` 해석을 확인합니다. ROS나 시뮬레이션 런타임을 빌드·기동하지 않습니다.

```bash
source ./manipulation_env.sh
python3 tests/native/shutdown_probe/build_probe.py --output runtime/shutdown-probe-build-new
python3 -m unittest discover -s tests/native/shutdown_probe -p 'test*.py' -v
python3 tests/integration/verify_native_shutdown_race.py \
  --expect fixed --domain 219 --overlay /tmp/<격리 빌드>/install \
  --probe runtime/shutdown-probe-build-new/update_probe.so \
  --expected-native-sha256 <검토한 native 실행 파일 SHA256> \
  --output runtime/shutdown-race-fixed-new
```

`--expect legacy`는 원본의 대기 중 CM teardown을 요구합니다. 결과는
`expected_unsafe_overlap`로 보존하며 안전한 native 실행으로 판정하지 않습니다. 실제
SIGSEGV 발생 여부는 별도 필드입니다. `fixed`는 release → 실제 update 반환 → owner join →
context shutdown → CM teardown 순서, native 정상 종료, 엄격한 물리 검사와 소유 프로세스
정리를 모두 요구합니다. 단순히 프로세스가 종료됐다는 이유로 성공 처리하지 않습니다.

테스트 launch는 생산 launch의 Node 생성 동안 native Node의 `additional_env`만 감싸
기존 lifetime guard를 보존하고 `probe:JTC:guard`를 명시합니다. `run_scene.sh`를 경유하지
않고 기본 환경을 source한 다음 지정 overlay를 마지막으로 source합니다.

게이트 디렉터리는 0700, 파일은 0600이며 새 run ID와 소유권을 확인합니다. 대기는 최대
10초, 관찰 창은 0.25~2초입니다. 항상 gate를 해제한 후 자체 launch만 정리합니다.
완성된 arm/release 파일은 원자적으로 게시합니다. entered 이벤트를 먼저 fsync한 뒤 동일한
PID·TID·run ID·시각의 entered 파일을 게시하며, 모든 이벤트를 검증한 소유 PID와 SIGINT
시각에 연결합니다. 누락되거나 외부 작업의 이벤트, 중복·역행 시각은 거부합니다.
성공·실패와 원본 해시는 새 runtime 폴더에 남습니다. 이 제어된 시험이 통과해도 과거
W01의 다른 간헐 정리 지연까지 해결됐다고 판정하지 않습니다.
