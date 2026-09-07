# Rotation calibration implementation plan

> **For Claude:** Use executing-plans to implement and verify each task.

**Goal:** 시작 시 정지 검사, 전후 왕복, 좌우 회전 보정과 독립 반복 검증을 모두 통과해야 주행 준비를 표시한다.

**Architecture:** ROS 없는 스캔 정합과 회전 상태 머신을 추가한다. 시작 노드는 입력과 안전 조건을 연결하고, 기존 safety 노드만 최종 속도를 발행한다. 기존 직진 결과는 유지하며 회전 결과 및 미수행 장착 보정을 별도로 표시한다.

**Tech Stack:** ROS 2 Jazzy, Python/numpy, unittest/pytest, 기존 HTML/JavaScript 대시보드.

## 설계

- 정지 스캔을 기준으로 point-to-line ICP를 수행한다. odom/IMU를 정합 초기값이나 정답으로 사용하지 않는다. 서로 다른 방향의 면, 대응점 비율, 잔차, 단계별 이동 한계를 검사한다. 라이다 위치 오프셋은 정합의 평행이동으로 흡수하며 장착 보정값을 변경하지 않는다.
- 기본 ±20도, 0.10rad/s로 `왼쪽 → 원점 → 오른쪽 → 원점`을 수행한다. 정지 후 각도를 읽고 명령 적분과 비교하여 양방향 저속 배율을 얻는다. 0.75~1.25 밖의 배율은 거부하고, 보정 후 전체 동작을 반복한다. 각 leg 및 전체 시간, 차체 이동량, 복귀 오차, IMU/odom 일치, 센서 신선도를 검사한다.
- 회전 시작/진행 시 전체 스캔의 주변 여유 공간과 기존 위험 신호를 확인한다. E-stop은 자동 해제하지 않는다. 정합 실패/정체/센서 유실/취소 시 즉시 raw zero, ready false, 배율 비활성화.
- 새 회전 배율은 기존 직진 배율과 별도 토픽으로 보낸다. 전체 완료·신선한 결과가 있을 때 safety에서 저속 제자리 회전에만 적용한다. 바퀴 기하, 라이다 외부 파라미터, IMU 내부 전체 보정 완료를 주장하지 않는다.

## 작업과 검증

1. `test/test_scan_rotation.py`에 회전·평행이동·노이즈·단일 벽 퇴화·불충분 데이터·주변 공간 테스트를 먼저 추가한다. 실패 확인 후 `move_control/sensing/scan_rotation.py` 구현.
2. `test/test_rotation_calibration.py`에 양방향 응답, 별도 반복, yaw wrap, 정체, 반대 방향, drift, 센서 불일치 테스트를 먼저 추가한다. 실패 확인 후 `move_control/control/rotation_calibration.py` 구현.
3. `tools/test_startup_calibration_ros.py`와 safety 통합 테스트에 회전 전 ready 차단, 완료 저장, E-stop/유실/취소 및 배율 사용 조건을 추가한 후 노드·공유 설정 연결.
4. `tools/test_dashboard_calibration.cjs`를 먼저 확장하고 `web/dashboard.html`의 단계·배율·미수행 항목 표시 수정. 실제 브라우저로 고정 데이터 상태 렌더링 확인.
5. 격리 ROS Docker(외부 네트워크 차단)에서 `python3 -m pytest test/ -q`와 관련 `tools/` 테스트, Node 대시보드 테스트 실행. 문서·차이 검토 후 작업 브랜치 커밋 및 안전한 로컬 통합. 실제 ARM64/센서/모터 검증은 별도 증거로 남긴다.

## 초기 환경

Windows Python은 sensor_msgs가 없어 기존 test_scale 수집이 실패한다. 기존 ROS base 이미지는 cv2가 없어 수집이 실패한다. 로봇과 연결하지 않고 ROS+OpenCV 검증 이미지를 준비하여 전체 기준 테스트를 실행한다.
