# Headless Gazebo 매핑

Ubuntu 24.04 / ROS 2 Jazzy에서 실행한다. Windows에서는 WSL Ubuntu를 사용한다.
Windows에서 Ubuntu가 없으면 PowerShell에서 `wsl --install -d Ubuntu-24.04`를
실행하고 Ubuntu 사용자 설정을 마친다. 아래 명령은 Ubuntu 터미널에서 실행한다.
ROS 2 Jazzy가 없다면 먼저 [공식 설치 안내](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)의
locale 및 ROS apt 저장소 설정을 완료한다. 저장소 설정 후 필요한 패키지를 설치한다.

```bash
sudo apt update
sudo apt install -y git ros-jazzy-ros-base ros-jazzy-ros-gz \
  ros-jazzy-slam-toolbox python3-numpy python3-opencv python3-pytest
mkdir -p ~/dev
cd ~/dev
git clone https://github.com/livsbittt/move_control.git
cd move_control
source /opt/ros/jazzy/setup.bash
python3 -m pytest test/ -q
bash tools/gz/test_planning.sh
```

이미 clone했다면 `cd ~/dev/move_control && git pull --ff-only` 후 실행한다.
이 전용 Gazebo runner는 소스에서 Python 노드를 시작하므로 colcon 빌드나
Pinky 실물 bringup 패키지는 필요하지 않다. headless도 GPU lidar용 EGL 렌더링은
필요하므로 `/scan`이 없으면 먼저 `/tmp/gztest/gz.log`에서 렌더링 오류를 확인한다.

기본은 GUI 없는 `gz sim -s -r --headless-rendering`이다.
GUI가 필요하면 `RIG_GUI=1 bash tools/gz/test_planning.sh`로 실행한다.
대시보드는 http://localhost:28161, 로그는 `/tmp/gztest/`에 있다.
동일 ROS domain 13 / Gazebo partition `pinky_rig13`으로 두 번 실행하지 않는다.
종료는 실행 터미널에서 Ctrl+C를 누른다.

별도 WSL 터미널에서 실제 지도 수신·저장·검증:

```bash
cd ~/dev/move_control
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=13 ROS_LOCALHOST_ONLY=1
mkdir -p artifacts/headless
python3 tools/gz/map_run_monitor.py --out artifacts/headless --timeout 3600
python3 tools/gz/save_map.py artifacts/headless/maze 15
python3 tools/gz/check_map.py --map artifacts/headless/maze.yaml \
  --out artifacts/headless/overlay.png
```

첫 실행에서 스캔·odom 흐름을 짧게 확인하려면 같은 환경에서
`python3 tools/gz/rig_health.py`를 실행한다. `artifacts/headless/maze.yaml`,
`maze.pgm`, `overlay.png`가 저장 결과이며, `check_map.py`의 출력과 종료 코드를
함께 확인한다. monitor timeout은 전체 탐색 완료를 의미하지 않는다.

`maze.pgm` / `maze.yaml`은 Gazebo 라이다 → ROS bridge → slam_toolbox의
`/map`을 저장한 결과다. 사전 제작된 지도나 synthetic rig를 사용하지 않는다.
이 시뮬레이션은 ground-truth odometry를 사용하고 scan matching을 끈다.
실물 로봇의 odometry 오차나 실물 주행 성능까지 검증하는 구성은 아니다.

판정은 분리한다: 지도 수신·저장 성공과 전체 탐색 완료는 다르다.
`check_map.py`는 지도 밖 미로 영역도 미탐색으로 계산한다. 현재 품질 게이트의
미탐색 허용치는 40%이므로 이 도구의 통과도 100% 매핑 완료를 뜻하지 않는다.
전체 완료 보고에는 미탐색 비율, 벽 검출률, 모니터 종료 상태를 함께 기록한다.

저장 PGM의 첫 행은 북쪽이며, ROS OccupancyGrid의 남쪽부터 시작하는 행을
뒤집어 저장한다. 회귀 테스트:

```bash
source /opt/ros/jazzy/setup.bash
python3 -m pytest test/ -q
python3 -m pytest tools/gz/test_map_monitor.py -q
```
