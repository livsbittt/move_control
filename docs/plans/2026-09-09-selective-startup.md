# 선택적 프로세스 실행

`robot.launch.py`는 실행 프로파일 결정과 ROS 프로세스 생성을 분리한다. 순수 함수 `control/startup_profile.py`가 옵션을 검증하고, launch의 `_processing_actions`가 선택한 프로세스만 생성한다. LCD가 꺼져 있으면 `lcd_control` 패키지 경로도 조회하지 않는다.

| 프로파일 | 기본 실행 | 기본 미실행 |
|---|---|---|
| `full` (호환 기본값) | IMU, camera, safety, wander, LCD, web, watch, calibration | 없음 |
| `sensing` | camera, safety, web, 정지 partial calibration | IMU, wander, LCD, watch |

`ros2 launch rosy_control robot.launch.py profile:=sensing start_imu:=false`로 별도 IMU 서비스와 중복 없이 정지 센싱을 실행한다. 센서 드라이버, 모터/TF bringup, ADC는 외부 선행 구성이다. SLAM(`map.launch.py`)과 goal은 별도 실행이며 이 launch에서 켜지 않는다.

`start_imu`, `start_camera`, `start_wander`, `start_lcd`, `start_web`, `start_watch`, `start_calibration`은 `auto|true|false`를 받는다. auto는 프로파일 기본값이다. safety는 끌 수 없다. `calibration_sensing_only`도 auto이며 sensing에서는 true, full에서는 false이다. sensing에서 partial calibration을 해제하거나 calibration 프로세스를 끄는 옵션은 오류로 거절한다.

시작 로그는 선택/미선택 목록과 partial calibration 값을 기록한다. 기존 safety 1.5초, wander 3초, 화면/감시 3.5초, calibration 4초 순서를 유지한다. 이 지연은 준비 완료 판정이 아니다. 센서 신선도, 비상정지, calibration ready 등 기존 실제 데이터 기반 게이트가 그대로 적용된다. 모든 선택 노드는 respawn한다.

`watch_node`의 필수 그래프는 현재 전체 하드웨어 구성(IMU, camera, wander 포함)으로 고정되어 있다. sensing에서는 기본적으로 끄며, 명시적으로 켜면 선택하지 않은 노드를 missing으로 보고한다. 프로파일별 감시 필수 목록은 별도 개선 대상이다.

프로파일은 프로세스 선택이지 센서 정상 보증이 아니다. partial baseline은 기존 non-IMU 센서 기준을 유지하므로 SLAM이 꺼져 map/map_tf가 없으면 `partial_baseline_ready=false`가 유지된다. 카메라는 기존 advisory eligible 정책을 유지한다. 프로파일별 필수 센서 집합 분리는 추후 작업이며, 현재 IMU 이외의 기준을 완화하지 않는다. partial ready가 true여도 주행 가능 상태는 항상 false이다.

로컬 검증은 순수 프로파일과 실제 launch 구성 함수를 가짜 ROS action 객체로 실행해 선택 목록, bool 전달, 지연 순서, optional 패키지 조회 생략을 확인한다. 실제 ROS 프로세스 실행 및 장치 센서 준비 확인은 별도이다.
