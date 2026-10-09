# POSIM-Control

[English](README.md)

수상 선행·추종 차량의 Gazebo 제어기와 속도·파도·안정성·복구 정책을 제공합니다. 제어기는 물리 모델에 추진력을 가합니다. WWW-POSIM은 충돌을 검사한 경로를 제공하고 실측 진행 상태를 확인합니다.

## Python 정책

Python 3.10 이상이 필요합니다.

```sh
git clone https://github.com/IOES-Lab/POSIM-Control.git
cd POSIM-Control
python -m venv .venv
. .venv/bin/activate
python -m pip install -e .
```

Windows에서는 `.venv\Scripts\activate`로 가상환경을 활성화합니다.

```python
from posim_control.speed import target_speed
from posim_control.stability import tilt_degrees

speed = target_speed(near_port=False, canal=False, clearance=5.)
tilt = tilt_degrees({"x": 0., "y": 0., "z": 0., "w": 1.})
```

## 네이티브 플러그인

CMake 3.22 이상·C++17·Gazebo Jetty 개발 패키지(`gz-sim`, `gz-plugin`)가 있는 POSIM 환경에서 빌드합니다.

```sh
cmake -S native -B build -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$PWD/install"
cmake --build build -j 2
ctest --test-dir build --output-on-failure
cmake --install build
export GZ_SIM_SYSTEM_PLUGIN_PATH="$PWD/install/lib${GZ_SIM_SYSTEM_PLUGIN_PATH:+:$GZ_SIM_SYSTEM_PLUGIN_PATH}"
```

월드에 `wwos::PromotionPilot`, 추종 모델에 `wwos::PromotionFollower`를 연결합니다. 플러그인 파일은 `libwwos-promotion-pilot`, `libwwos-promotion-follower`이며 운영체제에 맞는 라이브러리 확장자를 사용합니다.

현재 제어기는 `wamv`, `wamv_follower` 모델과 좌우 추진기 조인트 토픽을 사용합니다. 호출 앱이 모델·지형을 준비하고 명령 경로 전체의 충돌을 검사합니다. 모델과 파도 시스템은 POSIM과 설정된 의존성에서 제공합니다.

`/wwos/promotional/path`에 `gz.msgs.Double_V`로 `[mode, speed_mps, count, x0, y0, ..., origin_epoch]`를 보냅니다. 좌표 단위는 로컬 m입니다. 유효한 계획을 벽시계 기준 3초 안에 다시 보내야 합니다. 빈 계획은 추력을 중립으로 만듭니다. 상태 토픽은 `/wwos/promotional/pilot`, 선행 차량 오도메트리는 `/model/wamv/odometry`입니다.

- 모드 1: 경로를 따라 추진
- 모드 2: 추진기를 사용한 복구
- 모드 3: 힘을 제한한 물리적 예인 보조
- 최대 경유점 수: 256
- 선행 차량 순항 상한: 10.5 m/s
- 추종 차량 속도 상한: 11.9 m/s
- 추종 목표 거리: 38 m
- 거친 파도 속도 상한: 7 m/s
- 얕은 수역·운하 속도 상한: 4 m/s

추종 차량은 선행 차량의 실측 위치를 따라갑니다. 시계·진행·기울기·파도 정책은 실제 관측을 사용합니다. 복구 함수는 지형을 검사한 이동 후보를 제안합니다. 호출 앱이 보조 복구를 승인하고 표시합니다.

## 구조

| 경로 | 내용 |
|---|---|
| `src/posim_control/speed.py` | 수심·운하 속도 제한과 파도 이력 |
| `stability.py` | 기울기 계산과 전복 정책 |
| `recovery.py` | 시계·진행 검사와 탈출 후보 |
| `arrival.py` | 실측 진행에 따른 도착시간 추정 |
| `native/Pilot.cc`, `Follower.cc` | Gazebo 추진 제어 시스템 |
| `native/Following.hh`, `AttitudeSafety.hh`, `VoyageLimits.hh` | 추종 제어와 안전 한도 |
| `native/tests/`, `tests/` | 제어기·정책 시험 |

## Gazebo 없이 시험

```sh
python -m unittest discover -s tests
cmake -S native -B build-policy -DPOSIM_CONTROL_BUILD_PLUGINS=OFF -DCMAKE_BUILD_TYPE=Release
cmake --build build-policy -j 2
ctest --test-dir build-policy --output-on-failure
```

코드: [Apache-2.0](LICENSE). [출처 고지](NOTICE.md)를 확인하세요.
