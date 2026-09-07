# Auto calibration 안전 기반 구현 상태

작업 브랜치: `codex/rotation-calibration`, 기준 main: `6f6e461`.
상위 계획: `2026-09-07-auto-calibration-safety-profile.md`.

## 구현 및 로컬 검증 완료

- 센서별 수신 시각, 원본 시각, generation, validity를 관리한다. 오래된 원본과 중복 원본은 관측을 갱신하지 않는다. stamp가 없는 IR/카메라는 수신 시각만 검증할 수 있음을 표시한다.
- 라이다 필터, 초음파 hit 누적, IR median, 복도 표본은 새 관측에만 갱신한다. 필요한 센서가 유효하지 않으면 최종 출력을 정지한다.
- 기존 calibration 파일의 작은 반경/정지 거리 값이 bootstrap 하한을 축소하지 못한다. 실측 완료를 의미하지 않으며 profile은 `commissioned=false`이다.
- 모든 최종 명령에 선속도/각속도 상한을 함께 적용한다. 장애물 판정이 곡선 명령의 한 성분만 제거하면 정지하여 재계획을 요구한다. 직선 시험 gain은 곡선 명령에 적용하지 않는다.
- 초음파 무응답으로 기존 접촉 latch를 해제하지 않는다. 너무 좁거나 미확인인 복도를 열린 공간으로 표시하지 않는다.
- `/safety/profile`, `/safety/observation`, `/safety/decision`에 유효 적용값과 관측 상태 및 최종 제한 사유를 공개한다. observation은 아직 전체 공간 snapshot이 아니라 센서 상태 메타데이터이다.

격리 Docker ROS Jazzy 환경에서 `test/`와 safety/startup/wander calibration ROS 테스트 **224개 통과**. `colcon build --packages-select move_control` 통과. 독립 리뷰에서 발견한 US latch, 좁은 복도, 곡선 gain 문제를 회귀 테스트로 확인했다.

## 남은 소프트웨어 작업

- 실제 거리/coverage/frame을 포함하는 일관된 관측 snapshot과 wander/UI 소비 연결.
- 현재 운동 상태를 포함한 stopping sweep shadow 계산 및 공간에 따른 공통 속도 제한.
- 양방향 회전 관측성 검사, 회전 응답 추정, calibration profile의 원자적 검증·활성화·rollback.
- 진행량/왕복 진동/탈출 회전 종료 판단 개선.

## 실기 검증 HOLD

차체 및 지지 footprint, 센서 위치, 회전 응답, 제동 거리, 지연, ARM64 실행 시간은 실측되지 않았다. bootstrap 상한은 안전 인증이나 좁은 통로 통과 보장이 아니다. 새 validity 요구로 센서 누락/워밍업 시 기존보다 자주 정지할 수 있다. 실제 센서 stamp 및 IMU 단위 계약을 실기에서 확인해야 한다.

main 병합, 배포, 로봇 구동은 수행하지 않았다. 다른 세션 변경과 통합할 때 최종 설정 및 전체 회귀를 다시 확인한다. `lcd_control` 및 on-robot smoke는 이 격리 패키지 빌드의 검증 범위 밖이다.
