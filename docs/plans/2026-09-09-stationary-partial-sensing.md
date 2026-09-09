# IMU 제외 정지 센싱

`/calibration/cmd`에 `sensing_only`를 보내면 IMU를 제외한 정지 센서 기준값과 상태를 수집한다. 시작 시 읽기 전용 매개변수 `calibration_sensing_only:=true`로도 선택할 수 있다. 기본값은 false이다.

이 모드는 주행 보정이 아니다. 기존 보정 인증서를 무효화하고 wander 정지 및 속도 0 명령을 유지한다. `/calibration/ready`, `calibration_verified`, `rotation_verified`, `motion_allowed`는 항상 false이다. 비상정지와 기존 안전 게이트는 그대로 적용된다. `validate_motion`은 거절한다.

상태는 `phase: sensing_only`, `mode: sensing_only`, `partial: true`, `excluded_sensors: [imu]`, `exclusion_reason: operator_requested_imu_exclusion`을 제공한다. `partial_baseline_ready`는 나머지 센서의 기존 기준 충족 여부이며 주행 허가가 아니다. IMU 항목은 `status: excluded`, `eligible: false`이고 IMU 기준값은 저장하지 않는다. 중단 및 파일 저장 실패는 기존 aborted/failed 상태로 표시되며 주행 금지는 유지된다.

`retry` 명령은 이 모드를 종료하고 IMU를 포함한 전체 보정을 새로 시작한다. 기존 자동 이동 보정 설정이 true이면 정상 센서와 안전 조건을 모두 충족한 뒤 제한된 보정 이동을 수행할 수 있으므로 안전한 바닥에서 실행한다. IMU 수신이 회복되어도 자동으로 이 모드를 종료하지 않는다.

로컬 테스트는 ROS 없는 순수 판정 및 실제 어댑터 메서드의 정지/명령 분기를 검증한다. 장치 배포, 센서 실측, 모터 정지 실증은 별도 확인이 필요하다.
