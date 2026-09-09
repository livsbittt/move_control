# 기존 설정 및 일부 센서 운행

`/calibration/cmd`의 `use_existing_settings`는 새 보정 이동 없이 기존 안전 설정으로 운행 준비를 확인한다. `use_limited_sensors`는 IMU를 명시적으로 제외하고 저속 제한 운행을 선택한다. 선택 자체가 이동 명령이나 비상정지 해제는 아니다. 기존 정지 전용 `sensing_only`와 구분한다.

두 운행 모드는 `calibration_complete=true`, `completion_source=operator_override`로 보정 절차를 완료 처리한다. 실제 측정 결과는 만들지 않으므로 `calibration_verified=false`, `rotation_verified=false`, `calibration_skipped=true`를 유지하고 기존 인증서를 무효화한다. phase/mode는 각각 `existing_settings`, `limited_sensors`이다.

운행 준비는 신선한 LiDAR 원시 거리, odom, IR, US(기존 no-echo advisory 정책), 센서 TF와 안전 geometry, 비상정지 해제, 신선한 안전 hazard 상태를 요구한다. 기존 설정 모드는 IMU도 요구하고, 일부 센서 모드만 IMU를 제외한다. 3초 정지 baseline이나 벽 fitting은 요구하지 않는다. `localization_required=true`일 때는 map/map_tf도 요구한다. 지도 탐색의 지도 요구 조건은 별도이다.

준비가 1초 유지되고 최종 안전 게이트의 현재 설정 적용 응답이 도착해야 `ready=operating_ready=motion_allowed=true`가 된다. 그 전에는 `waiting_reasons`를 표시한다. 센서/안전 상태를 잃으면 즉시 준비를 해제하고 정지 명령을 낸다. 준비가 되어도 별도 사용자 이동 명령을 기다린다.

일부 센서 모드는 `degraded=true`, `excluded_sensors=[imu]` 및 `speed_limits={linear_mps:0.005, angular_rad_s:0.05}`를 보고한다. atomic profile에도 명시적 limited-sensors 정보가 포함되며 실제 속도 제한과 IMU 제외 허용은 최종 안전 게이트의 유효 lease로만 적용한다. 프로파일이 만료되면 제외 허용도 사라진다. 다른 센서나 비상정지 보호를 제외하지 않는다.

기존 설정은 새 보정 이득을 추정하지 않고 항등 보정 이득을 사용한다. 최종 안전 노드에 이미 적용된 방향·형상·거리 설정은 유지된다. 기존 측정 인증서의 이득을 복원하는 기능과는 구분한다.

`sensor_check`는 선택한 운행 모드를 유지하며 준비를 다시 확인한다. `retry`(stay/return_origin 변형 포함)는 선택 모드를 종료하고 전체 보정을 시작한다. `abort`는 정지 및 준비 해제 상태를 유지한다. 어떤 명령도 자동 비상정지 해제를 하지 않는다.

## 실제 장치 검증

2026-09-09 `local-20260909-limited-v2`에서 IMU 서비스를 실제 중지하고 일부 센서 모드를 선택했다. 보정 완료 플래그와 IMU 제외를 확인한 뒤, 남은 센서 검사·설정 적용 응답·명시적 비상정지 해제를 거쳐 2초 직선 명령을 보냈다. 최종 안전 노드의 최대 출력은 0.005m/s, 오도메트리 이동량은 0.00905m였다. 시험 후 0 명령과 비상정지를 적용하고 IMU 서비스를 다시 시작했다. 증거는 장치 `~/.local/state/rosy_control/limited-motion-evidence.json`에 저장했다.

ROS 환경의 모드·lease·안전 어댑터·HTTP 집중 검증 31개가 통과했다. 비상정지 중 비영(非零) teleop 요청을 API도 409로 거절하도록 수정했으며, 정지용 0 명령은 계속 허용한다. 이 운행 확인을 실제 전체 센서 보정 인증서로 취급하지 않는다.
