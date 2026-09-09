# Rosy Control (Pinky Pro)

Rosy Control은 맵·위치 추정·주행·안전 제어를 개발하는 프로젝트입니다.
상위 `ROS/Rosy`의 OS 개발, `ROS/Rosy Fleet`의 관제 개발과 독립적으로 유지합니다.
로컬 저장소 폴더명은 `Rosy Control`, ROS/Python 패키지명은 `rosy_control`입니다.
이전 이름 `move_control`에서 전환했습니다(2026-09-09). 로봇에 이미 배포된 장비는
`tools/deploy/migrate_to_rosy_control.sh`를 한 번 실행한 뒤 새 릴리스를 설치해야 합니다.
전환 배경과 절차는 `docs/plans/2026-09-09-rosy-control-package-rename.md`를 보십시오.
공백이 있는 로컬 경로를 쉘에서 사용할 때는 따옴표로 감쌉니다.

- `calib_node` — 자동 캘리브 `/calib/step auto`: 안정 바닥 IR(4095 무시) → 느린 전진 부호+라이다 요 → 절벽 IR. 상태 `/calib/status` `/calib/phase`.
- `camera_detect_node` — 전면 OV5647. 바닥 대비 허공(절벽 앞)과 장애물. `/camera/cliff` `/camera/blocked` `/camera/side`
- `safety_node` — 라이다 3cm, 초음파 2.5cm, IR 절벽, IMU 기울기. 맵 크기로 거리를 정하지 않음. 기울기=`/safety/tilt`(후진). 들어올림=`/safety/pickup`(정지). 벽=`/safety/blocked`
- `wander_node` — 전진 / IR 절벽이면 정지→IR이 풀릴 때까지만 후진→회전 / 벽·카메라 허공·장애물이면 정지→회전. `/wander/cmd` stop|start
- `control_node` — `/goal_distance`, `/goal_rotate`

LCD / LED 화면은 별 패키지 `lcd_control` (`ros2 launch lcd_control lcd.launch.py`).

자세한 실행은 `STEPS.txt`.
