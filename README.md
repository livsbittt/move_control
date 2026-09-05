# move_control (Pinky Pro)

- `calib_node` — 자동 캘리브 `/calib/step auto`: 안정 바닥 IR(4095 무시) → 느린 전진 부호+라이다 요 → 절벽 IR. 상태 `/calib/status` `/calib/phase`.
- `camera_detect_node` — 전면 OV5647. 바닥 대비 허공(절벽 앞)과 장애물. `/camera/cliff` `/camera/blocked` `/camera/side`
- `safety_node` — 라이다 3cm, 초음파 2.5cm, IR 절벽, IMU 기울기. 맵 크기로 거리를 정하지 않음. 기울기=`/safety/tilt`(후진). 들어올림=`/safety/pickup`(정지). 벽=`/safety/blocked`
- `wander_node` — 전진 / IR 절벽이면 정지→IR이 풀릴 때까지만 후진→회전 / 벽·카메라 허공·장애물이면 정지→회전. `/wander/cmd` stop|start
- `control_node` — `/goal_distance`, `/goal_rotate`

LCD / LED 화면은 별 패키지 `lcd_control` (`ros2 launch lcd_control lcd.launch.py`).

자세한 실행은 `STEPS.txt`.
