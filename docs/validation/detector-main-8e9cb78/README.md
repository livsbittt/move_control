# main 8e9cb78 장애물 감지 및 맵 검증

판정: HOLD. 최신 main을 새 Gazebo 실행으로 검증했으나 전체 맵핑은 완료되지 않았다.

- 실행 ID: b5c0b55765e24489ae98ed1898ce48d9
- 원본 월드: map/map_260905.world, 벽 16개, 원본 크기, 바퀴 물리 모델
- 원본 월드 SHA256: 8475464857322d71f6e492171419bf285e7bd034089b90ad59d6f106b0941072
- 실제 Gazebo 렌더링 카메라 및 LiDAR. IR은 합성, IMU는 GT 파생, US는 LiDAR 파생이다.
- 관련 테스트: 50 passed. 카메라, 작은 장애물, 불량 영상, 추적, 위험 판단, 임시 장애물 지도.
- 새 실행 관측: 20.761 simulation seconds, 이동 경로 0.065895 m.
- 중단: 보정 첫 왕복 복귀 중 `Sensor data became stale or invalid: map_tf: valid=False, age=0.000s`.
- 최종 명령: 선속도/각속도 0, wander stop, goal stopped. 최종 속도 발행자는 safety_node 한 개.
- 지도 수신 및 시간/좌표 검사는 통과했으나 mapping_complete=false.
- 원본 월드 내부 미관측 65.4447%. 시작점과 연결된 점 공간의 미관측 64.7186%. 후자는 로봇 크기를 고려한 통과 가능 면적 판정이 아니다.
- phantom 0%는 관측 완료를 의미하지 않는다. 여러 벽의 표면 검출률은 0%다.
- 장애물 시나리오는 waiting 상태로 종료, 생성 이벤트 0개. 이번 실행에서 등장/횡단/제거 후 주행 재개는 검증되지 않았다.

## 카메라 감지 범위

실행 중 /camera/observation에서 floor_foreground_v2, quality.valid=true, 전경 영역을 확인했다. `acceptance-live.json`에 실제 수신 메시지를 저장했다. `camera-replay.json`은 실행 도중 저장된 11개 프레임의 부분 재생 결과이며 전체 실행 프레임을 대표하지 않는다.

기존 docs/validation/dynamic-obstacles/obstacle-camera.png를 최신 코드로 재생하면 영역 [0,0,320,162] 하나를 반환한다. 박스와 벽이 연결된 전경으로 합쳐지며 개별 물체 인스턴스가 아니다. near_path=false, blocked=false이며 해당 영상의 물체 하단은 가까운 경로 ROI 앞에 있다. 거리와 움직임은 unknown이다. 사람/의자 등의 의미 기반 모델은 아직 연결되지 않았다.

## 저장 결과

track_map.yaml + track_map.pgm은 실제 SLAM 부분 맵이다. 원본 월드로 미관측 공간을 채우지 않았다. track_result.png는 원본 벽(빨강), SLAM 맵, 실제 궤적의 비교 그림이다. 그림을 직접 확인했으며 좌우 영역이 상당 부분 미관측이다.

이 결과는 실패를 기록한 검증이며 보정 실패 원인의 수정, 개별 물체 detector 연결, 동적 장애물 통과, 전체 탐색 완료를 증명하지 않는다. 기존 결과를 덮어쓰지 않고 별도 폴더에 저장했다.