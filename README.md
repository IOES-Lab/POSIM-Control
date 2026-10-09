# POSIM-Control

[한국어](README.ko.md)

Native Gazebo controllers for a surface leader and follower, plus Python policies for speed, rough water, stability and recovery. Controllers apply propulsion to physical models. WWW-POSIM supplies collision-checked paths and observes measured progress.

## Python policies

Python 3.10 or later is required.

```sh
git clone https://github.com/IOES-Lab/POSIM-Control.git
cd POSIM-Control
python -m venv .venv
. .venv/bin/activate
python -m pip install -e .
```

On Windows, activate `.venv\Scripts\activate` instead.

```python
from posim_control.speed import target_speed
from posim_control.stability import tilt_degrees

speed = target_speed(near_port=False, canal=False, clearance=5.)
tilt = tilt_degrees({"x": 0., "y": 0., "z": 0., "w": 1.})
```

## Native plugins

Build in a POSIM environment with CMake 3.22+, C++17 and Gazebo Jetty development packages (`gz-sim` and `gz-plugin`).

```sh
cmake -S native -B build -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX="$PWD/install"
cmake --build build -j 2
ctest --test-dir build --output-on-failure
cmake --install build
export GZ_SIM_SYSTEM_PLUGIN_PATH="$PWD/install/lib${GZ_SIM_SYSTEM_PLUGIN_PATH:+:$GZ_SIM_SYSTEM_PLUGIN_PATH}"
```

Attach `wwos::PromotionPilot` at the world level and `wwos::PromotionFollower` to a follower model. Plugin files are `libwwos-promotion-pilot` and `libwwos-promotion-follower`, with the platform library suffix.

The current controller uses models named `wamv` and `wamv_follower` and their left/right propeller joint topics. The caller creates those models, loads terrain and checks every commanded corridor. Models and wave systems come from POSIM and its configured dependencies.

Send `gz.msgs.Double_V` to `/wwos/promotional/path` as `[mode, speed_mps, count, x0, y0, ..., origin_epoch]`. Coordinates are local metres. Re-send a valid plan within three wall seconds. Empty plans neutralize thrust. Status is published on `/wwos/promotional/pilot`; leader odometry is on `/model/wamv/odometry`.

- Mode 1: propulsion along the path
- Mode 2: propulsion recovery
- Mode 3: bounded physical towing assistance
- Maximum waypoint count: 256
- Leader cruise cap: 10.5 m/s
- Follower speed cap: 11.9 m/s
- Follower target separation: 38 m
- Rough-water speed cap: 7 m/s
- Shallow-water and canal cap: 4 m/s

The follower follows measured leader poses. Clock, progress, tilt and rough-water policies use actual observations. Recovery helpers propose terrain-checked motion; the caller authorizes and reports assisted recovery.

## Structure

| Path | Contents |
|---|---|
| `src/posim_control/speed.py` | Depth/canal caps and rough-water hysteresis |
| `stability.py` | Tilt measurement and capsize policy |
| `recovery.py` | Clock/progress checks and escape candidates |
| `arrival.py` | Arrival estimate from measured progress |
| `native/Pilot.cc`, `Follower.cc` | Gazebo propulsion systems |
| `native/Following.hh`, `AttitudeSafety.hh`, `VoyageLimits.hh` | Formation law and safety limits |
| `native/tests/`, `tests/` | Controller and policy tests |

## Tests without Gazebo

```sh
python -m unittest discover -s tests
cmake -S native -B build-policy -DPOSIM_CONTROL_BUILD_PLUGINS=OFF -DCMAKE_BUILD_TYPE=Release
cmake --build build-policy -j 2
ctest --test-dir build-policy --output-on-failure
```

Code: [Apache-2.0](LICENSE). See [source notices](NOTICE.md).
