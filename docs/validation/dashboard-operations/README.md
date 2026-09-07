# 대시보드 동작 검증 — 2026-09-08

감지 / 관측 / 조작의 기존 구조를 유지하면서 배터리, 보정, 위치, 동작 요약과 지도 수집·영상 수신 스위치를 추가했다. 주행·비상정지는 HTTP 성공만으로 완료 처리하지 않고 요청 이후 ROS 상태 수신을 확인한다. 지도 수집 중지는 자동 주행 중지 요청 후 SLAM 일시정지 상태를 다시 읽으며 지도를 보존한다.

배터리는 `/battery_state`의 실제 값을 사용한다(`battery_topic`으로 변경 가능). 유효하지 않은 잔량을 전압으로 추측하지 않으며 오래된 메시지·중복 타임스탬프를 최신 상태로 표시하지 않는다. 카메라 JPEG 생성 번호가 상태에 전달되지 않던 문제도 수정했다. 영상 스위치는 브라우저의 영상 요청을 제어하며 센서 전원을 끄지 않는다. 과거 지도 QA 파일은 `metrics_file`을 명시한 경우에만 표시한다.

## 확인 결과

- ROS 환경의 전체 순수 로직 및 웹 회귀 테스트: **353 passed** (`backend-tests.log`).
- 기존 대시보드 Node 테스트: **22 passed**.
- Chromium → 실제 WebNode HTTP → ROS 토픽/서비스 → 화면 상태 확인: **PASS**, 페이지 오류 없음 (`browser-test.json`).
- 시작/정지, 비상정지/해제, 보정 중단/재시도, 지도 수집 중지/재개 및 거절 응답, 배터리 정상/미상/만료, 영상 요청 중지/재개와 설정 유지, 모바일 가로 넘침, 연결 단절을 확인했다.
- ROS Jazzy `colcon build --packages-select move_control`: **1 package finished**.

ROS 도메인 231의 격리된 테스트 노드가 배터리·영상·지도·상태 및 SLAM 서비스를 제공한다. 화면의 72%와 영상은 테스트 데이터이며 실기 배터리 측정, 모터 구동, Gazebo 완주 또는 실기 배포 증거가 아니다. 실기에서는 BatteryState를 발행하는 드라이버가 필요하다. 이 작업은 이전 Gazebo 검증을 대체하지 않는다.

## 화면

- [개선 전 main 2001ee0](main-before.png)
- [개선 후 데스크톱](desktop-after.png)
- [지도·영상 조작](controls-after.png)
- [모바일](mobile-after.png)

`before.png`는 기존 실행 서버의 더 오래된 화면이다. 직접 비교 기준은 같은 테스트 노드를 사용한 `main-before.png`다.

## 재현

ROS 2 Jazzy와 slam_toolbox가 설치된 Linux/WSL에서 저장소 루트 기준:

```bash
bash tools/run_dashboard_operations.sh
```

별도 터미널에서 Python Playwright와 Chromium이 설치된 Windows 또는 Linux 환경으로 실행:

```bash
python tools/test_dashboard_operations_browser.py
```

Windows에서는 Ubuntu WSL을 통해 테스트 시나리오를 전달한다. 화면은 http://localhost:28361 이며 테스트 도메인 231을 다른 장비에 사용하지 않아야 한다.

```bash
source /opt/ros/jazzy/setup.bash
python3 -m pytest test/ tools/test_web_battery_ros.py tools/test_web_map_control.py tools/test_web_calibration.py tools/test_web_map_raster.py tools/test_web_pose_freshness.py -q
node --test tools/test_dashboard_calibration.cjs tools/test_dashboard_mapping.cjs tools/test_dashboard_rotation.cjs
```
