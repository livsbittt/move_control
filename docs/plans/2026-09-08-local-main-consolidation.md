# 로컬 main 통합

기준 main e87707f, 병합 대상 rotation 8e51b75. 나머지 세션 커밋은 main의 조상이다. 기존 워크트리의 미커밋 파일은 보존한다.

1. 부모 담당: Git 이력, 미커밋 문서/증거 비교, 주행·계획 충돌. main의 도착 통지, 수동 목표, 접근 가능한 영역, 위치 복구를 보존하고 정지 시간/순진동 진행 판정을 보완한다.
2. 안전 담당: safety 디렉터리의 충돌을 현재 mesh/방향별 형상과 localization 게이트를 기준으로 해결한다. 센서 관측 세대/신선도, 유한 명령, 속도 쌍 제한, 보정 프로파일 lease/ack를 추가한다. 현재 동작 반경을 옛 고정 원형 정책으로 바꾸지 않는다.
3. 보정 담당: startup_calibration_node, calibration_rotation, certificate를 현재 WallTracker/가변 거리/저장 증거/일시 센서 hold에 맞춰 통합한다. 좌우 회전 증거를 별도로 보존하고 이전 직진 인증서에 회전 성공을 만들어 넣지 않는다.

공통 인터페이스: safety는 `/safety/profile`에 유효한 현재 형상 revision 및 effective.turn_clear를 발행한다. startup은 기존 make_profile/ProfileLease schema로 `/calibration/profile`을 발행하고 `/calibration/applied`의 같은 revision 확인 후 ready를 허용한다. durable certificate와 live lease는 별개다. 기존 split gain/ready 토픽은 호환 표시용이며 safety의 gain 권한은 atomic lease만 사용한다.

검증: 순수 테스트 전체, ROS 어댑터 테스트, Gazebo localization 회귀 및 변경된 보정의 격리 검증. 각 작업은 지정 파일만 수정하고 커밋은 통합 담당자가 일괄 수행한다. 완료 후 현재 main HEAD/WIP 재확인하여 병합한다.
