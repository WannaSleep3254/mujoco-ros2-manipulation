# 설계파일로 주변 설비를 구성하는 절차

로컬 STEP 파일을 m 단위 자산으로 변환하고, 설치 위치를 지정해 기존 로봇 작업 환경과 함께 확인합니다. 새 작업 환경은 별도 출력 폴더에 만들며 기존 로봇 모델과 실행 중인 시뮬레이션을 재생성하지 않습니다.

현재 연결한 기능은 **읽기 전용 3D 미리보기**입니다. 새 환경의 URDF·MJCF·MoveIt 설명을 함께 생성하고 기하를 검사하지만, 이 환경에서 ROS 명령을 실행하거나 설비 회피 경로를 검증하는 기능은 후속 작업입니다.

## 1. 단위와 좌표계를 확인합니다

입력은 로컬 STEP의 유효한 닫힌 단일 solid입니다. assembly, 외부 참조, 가동 부품과 임의 CAD 형식은 지원하지 않습니다. FreeCADCmd는 별도 자식 프로세스에서 실행하고, 변환 시간과 입력·출력 크기를 제한합니다.

정규화 설정에는 다음 값을 명시합니다.

- STEP에서 의도한 길이 단위와 실제 치수(m)를 기록합니다. 기대한 치수를 확인하지 않은 채 단위를 추정하지 않습니다.
- 원본 좌표계 이름과 up 축, 자산 좌표계로 옮기는 회전행렬을 지정합니다. 행렬은 오른손 좌표계를 유지하며 원본 up 축을 +Z로 옮겨야 합니다.
- 자산은 m 단위, 오른손 좌표계, Z-up으로 저장합니다. 원점은 보존합니다.
- FreeCAD가 STEP 단위를 해석한 내부 mm 좌표에 0.001을 한 번 적용합니다. STEP 선언 단위로 다시 배율을 곱하지 않습니다.

예제 설정은 [synthetic_negative_box.import.json](../config/environments/synthetic_negative_box.import.json)입니다. 이 설정은 검증용 100×200×300 mm 상자에만 맞습니다. 자신의 설계파일을 사용할 때는 실제 치수와 좌표계에 맞게 새 설정을 작성합니다.

```bash
.venv/bin/python import_environment_step.py \
  --source /local/path/fixture.step \
  --spec config/environments/my_fixture.import.json \
  --output runtime/environments/my_fixture_asset_v1
```

`/usr/bin/freecadcmd`가 기본 변환기입니다. 다른 로컬 설치는 `--freecad`로 지정합니다. FreeCAD 설치는 이 명령이 자동으로 수행하지 않습니다. 결과 JSON의 `manifest_sha256`은 다음 단계에서 사용합니다.

원본 STEP은 출력 자산에 복사하지 않지만 **파생 메시에도 설계 형상이 담깁니다**. 자산·환경 설정·로그는 로컬 `runtime/` 아래에 두고 공개 Git이나 보고서에 추가하지 않습니다. 출력 디렉터리는 소유자 전용 0700으로 새로 만들며, 기존 폴더는 비어 있어도 덮어쓰지 않습니다.

## 2. 설치 위치를 지정하고 새 작업 환경을 만듭니다

로컬 JSON 설치 목록을 작성합니다. `bundle` 상대 경로는 이 JSON 파일을 기준으로 해석합니다. `manifest_sha256`에는 앞 단계의 실제 값을 넣습니다.

```json
{
  "schema_version": 1,
  "assets": [{
    "bundle": "my_fixture_asset_v1",
    "manifest_sha256": "앞 단계에서 확인한 64자리 SHA256",
    "installation": {
      "name": "fixture",
      "parent_frame": "world",
      "xyz_m": [0.0, 0.65, 0.2],
      "rpy_rad": [0.0, 0.0, 1.5707963267948966],
      "rgba": [0.5, 0.7, 0.8, 1.0]
    }
  }]
}
```

```bash
.venv/bin/python prepare_environment_overlay.py \
  --workcell fr5_ur5e \
  --assets runtime/environments/installation.json \
  --output runtime/environments/fr5_ur5e_fixture_v1 \
  --overlay-id fr5_ur5e_fixture_v1
```

설치 이동(m)과 RPY 회전(rad)은 world 좌표계에서 한 번 적용합니다. 같은 설치 정보로 MJCF와 URDF를 생성하고, MoveIt 설명에도 같은 URDF·SRDF를 넣습니다. 기존 작업 환경의 모델 폴더 내부에는 새 작업 환경을 만들 수 없습니다.

시각 형상은 정규화 STL입니다. 충돌 형상은 가져온 CAD의 바운딩 박스를 감싼 **보수적인 box 하나**입니다. 예제는 각 면에 5 mm를 더합니다. 빈 공간도 막힐 수 있으며 오목한 설계 형상을 그대로 충돌 계산에 사용하는 기능은 아닙니다.

생성 시 Python MuJoCo에서 `mj_forward`만 수행해 설치된 시각 형상의 world bbox, box 중심·크기·회전과 URDF 모서리를 대조합니다. home에 접촉이 있으면 출력을 게시하지 않습니다. 물리 시간은 진행하지 않습니다.

## 3. 검증한 작업 환경을 로컬 뷰어에 등록합니다

새 작업 환경 생성 명령이 출력한 실제 `manifest_sha256`으로 로컬 catalog를 작성합니다. 목록은 최대 8개이며, 웹에서는 이 목록의 ID만 선택할 수 있습니다.

```json
{
  "schema_version": 1,
  "overlays": [{
    "directory": "fr5_ur5e_fixture_v1",
    "manifest_sha256": "새 작업 환경 생성 결과의 64자리 SHA256"
  }]
}
```

```bash
source ./manipulation_env.sh
python3 operator_console.py --workcell fr5_ur5e --domain 156 --port 8890 \
  --view-scene-catalog runtime/environments/catalog.json
```

도메인·포트는 실행 중인 다른 세션과 겹치지 않는 값으로 선택합니다. catalog 상대 경로는 catalog 파일을 기준으로 해석합니다. 시작 시 별도 CPU worker가 전체 작업 환경을 확인하며, 부모 ROS 프로세스는 이후 파일·해시 검사만 수행합니다.

뷰어에서 **모델 미리보기**를 선택하면 새 환경이 포함된 정지 모델을 자유 카메라로 볼 수 있습니다. 이 선택으로 제어 대상이 바뀌지는 않습니다. 미리보기 중에는 명령·새 실행·시연 시작을 거부하며, 실시간 보기로 돌아온 뒤 기존 기본 작업 환경을 사용합니다. 파일이 바뀌면 기존 검증은 무효가 되므로 새 출력과 catalog를 만들어 다시 시작합니다. 웹 업로드·임의 URL·실행 중 CAD 재변환은 제공하지 않습니다.

## 검증과 모듈 경계

```bash
# 합성 메시 fixture → 새 overlay → Python CPU 기하 검사
.venv/bin/python verify_environment_overlay.py

# 직접 생성한 STEP을 실제 FreeCAD로 변환하고 native 3.12 기하까지 대조
.venv/bin/python verify_environment_overlay.py --with-freecad --native
```

기본 검사는 실제 STEP 변환을 수행했다고 기록하지 않습니다. `synthetic-fixture` 출처를 가진 합성 메시를 사용합니다. `--with-freecad`에서만 직접 만든 STEP과 실제 FreeCAD·OCCT 버전 및 변환기 SHA를 기록합니다. 검증 결과와 생성물은 `runtime/`에 남습니다.

| 모듈 | 담당 기능 |
| --- | --- |
| `workcells/environment_assets.py` | 단위·좌표계·정규화 자산·명시적 충돌 근사와 SHA 검사 |
| `workcells/import_step.py`, `freecad_step_worker.py` | 소유한 FreeCAD 자식에서 로컬 STEP 변환 |
| `workcells/environment_overlay.py` | 기존 base를 보존하는 새 작업 환경 합성, 순수 파일 검사와 CPU 기하 검사 분리 |
| `operator_console/environment_sources.py` | 시작 시 로컬 allowlist 검증, 읽기 전용 작업 환경 등록 |
| `operator_console/scene_host.py`, `scene_worker.py` | ROS와 분리한 표시용 모델·관절 상태 처리 |
| `workcells/environment_native.py` | native MuJoCo 3.12에서 설치 기하와 home을 별도로 대조 |

새 산출물은 `environment-overlay-v1`과 `overlay.json`을 사용합니다. 기존 `source.json`을 가진 Workcell v1로 가장하지 않습니다. 기존 생성기의 전체 파일 해시를 바꾸지 않으며, 새 생성기·base·메시·설치·전체 출력·외부 메시 참조의 SHA를 고정합니다.

native 검사는 이번 설치 형상의 기하 일치를 확인합니다. 현재 거리 평가기는 두 로봇 사이의 충돌 형상만 계산하므로, 팔 사이 30 mm 정책을 새 설비와의 여유 거리로 해석하면 안 됩니다. 새 설비를 포함한 MoveIt 상태 검사, 시뮬레이터와 명령 서버의 정확한 모델 일치, 실제 계획·실행·취소는 다음 단계에서 함께 연결하고 검증해야 합니다.
