# map_260905.world 재점검 및 Gazebo 실행

기준 코드: `7e3d416`. 원본 `map/map_260905.world` SHA-256은 `8475464857322d71f6e492171419bf285e7bd034089b90ad59d6f106b0941072`이다. 원본의 16개 충돌 벽, 대각 벽 2개, 좌표와 1:1 축척을 유지하고 기존 테스트 로봇만 추가했다. 시작점은 벽 여유가 가장 큰 격자 후보인 (-0.20, 0.27)m이며 중심 여유는 0.347m다.

## 실측 결과

| 실행 | 보정 | 이동 | 판정 |
|---|---|---|---|
| wheel: 기존 바퀴·접촉 물리 | 첫 3cm 전진 중 제한 시간 초과, ready=false | 순변위 약 1.23cm, 전방 성분 약 1.01cm | HOLD: 바퀴 모델 응답이 부족함 |
| velocity: 이상적인 모터 응답 | 전후 2회 왕복 + 좌우 회전 8단계 + safety 적용 확인 성공 | 약 180초 관찰, 누적 궤적 0.682m, 최소 중심–벽 거리 0.164m | 보정 및 제한 시간 내 주행 검증 통과; 전체 매핑 미완료 |

velocity 실행의 좌/우 회전 배율은 0.99924 / 1.01004이고, 마지막 실시간 상태의 `settings_applied=true`, `ready=true`다. `/cmd_vel` 발행자는 safety_node 하나였다. 사용한 안전 반경은 0.105m다. 최소 거리는 0.5초 간격으로 기록한 2D 위치와 회전된 충돌 박스 사이의 거리이며, 물리 접촉 센서 검증을 대신하지 않는다.

탐색 상태 표본 244개 중 방향 정렬 151개, 전진 90개였다. 첫 탐색 목표까지의 경로는 약 0.79m에서 0.18m로 줄었지만 관찰 종료까지 도달하지 않았다. 특히 대각 벽 주변에서 정렬과 전진을 자주 전환한다. 다음 점검은 경로 재계산에 따른 추종 목표 변화, 짧은 lookahead, 곡률과 정렬 전환 조건을 기록해 원인을 분리해야 한다. 안전 여유를 줄여 통과시키지는 않았다.

## 이번에 수정한 검증 도구

- 기존 Gazebo 어댑터가 새 `calibration_atomic` 모듈에 시뮬레이션 시계를 주입하지 않는 누락을 수정했다. 실제 로봇 코드의 안전 입력 제한이나 타임아웃은 변경하지 않았다.
- 기존 `check_map.py`는 다른 정방향 미로의 벽 이름과 범위를 가정하여 이번 월드에 사용할 수 없다. 별도 월드 준비·관찰·그림 도구를 추가하고 대각 박스 거리 계산을 테스트했다.
- 원본 월드, 기존 저장 지도, 제품 주행·보정 파라미터는 변경하지 않았다.

검증: `python -m pytest test/ -q` — 319 passed, 1 skipped(ROS 전용). 새 shell runner의 `bash -n`도 통과했다.

## 재실행과 파일

ROS 2 Jazzy / Gazebo가 설치된 Linux 소스 폴더에서:

```bash
RIG_PLANT=wheel bash tools/gz/run_track260905.sh
RIG_PLANT=velocity RIG_DURATION=180 bash tools/gz/run_track260905.sh
python3 tools/gz/report_track_run.py /tmp/pinky-calmap227
```

ROS domain 227, Gazebo partition `pinky_calmap227` 전용이며 기존 calibration mapping runner와 동시에 실행할 수 없다. 결과는 `/tmp/pinky-calmap227`에 저장된다. Git 메타데이터가 없는 복사본에서는 `RIG_SOURCE_COMMIT`에 복사 기준 커밋을 지정한다. 후속 실행은 이전 결과를 archive에 보존한다.

`wheel/`과 `velocity/`에 결과, 상태, 궤적, 지도 PGM/YAML/NPZ, 원본 월드 identity 및 그림을 보존했다. 각 실행의 source manifest는 제품 코드와 설정 해시를 기록한다. 첫 실행 당시 runner는 보정 실패를 JSON에 기록하고도 shell 종료 코드 0을 반환했으며, 저장 후 runner에 실패 시 비정상 종료 검사를 추가했다. 결과 JSON을 판정 기준으로 삼는다.

두 실행 모두 Gazebo LiDAR와 ground-truth odometry를 사용했다. IMU는 GT 기반, IR/영상은 가상 입력, US는 LiDAR 기반이다. velocity 모델은 바퀴 접촉과 슬립을 검증하지 않는다. 따라서 실제 기기 보정·무충돌 운용·완성 지도·현장 위치 복구의 증거로 사용하지 않는다. 이번 지도 파일은 **부분 지도**이며, 기존 완성 지도를 대체하지 않는다.
