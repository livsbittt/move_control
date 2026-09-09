# 필요할 때 실행하는 사용자 세션

로봇의 기존 `rosy-session-*` 사용자 서비스를 관리합니다. 서비스 설치, 자동 활성화, 전체 시작은 제공하지 않습니다. 로봇 사용자 계정에서 실행합니다.

부팅 설정을 적용하기 전에 배포 담당자가 `rosy-session-control.service`의 실제 `ExecStart`와 실행 스크립트를 확인하고, 다음 인자를 사용하는지 확인해야 합니다.

```text
ros2 launch rosy_control robot.launch.py profile:=sensing start_imu:=false calibration_sensing_only:=true
```

`boot-minimal`은 control의 실행 명령을 수정하지 않습니다. 위 설정을 확인한 뒤 다음 명령을 실행합니다.

```bash
bash tools/deploy/rosy-session.sh boot-minimal
```

map, led, imu만 각각 `disable --now` 처리하여 현재 실행과 부팅 자동 실행을 해제합니다. 각 서비스의 처리 결과와 활성화/실행 상태를 출력하며, 일부 실패해도 나머지 서비스를 확인한 뒤 실패 종료 코드를 반환합니다. bringup, adc, control 안전 코어와 기존 종료 순서는 변경하지 않습니다.

실행·중지·부팅 설정 변경 명령은 각각 20초, 상태 조회는 5초로 대기를 제한합니다 (`timeout` 필요). 개별 명령의 시간 초과 종료 코드는 124이며 `boot-minimal`은 일부 실패 시 1을 반환합니다. 시간 초과는 systemctl 응답 대기가 끝났다는 뜻입니다. 커널의 D 상태 프로세스가 종료되었다는 증거가 아니며, 서비스 작업은 계속 대기할 수 있습니다. 출력된 상태와 `systemctl --user list-jobs`, 해당 서비스의 프로세스 상태를 별도로 확인해야 합니다.

필요한 기능 하나를 명시적으로 선택합니다. `start`는 다음 부팅의 자동 실행을 활성화하지 않습니다.

```bash
bash tools/deploy/rosy-session.sh start map
bash tools/deploy/rosy-session.sh status map
bash tools/deploy/rosy-session.sh stop map
bash tools/deploy/rosy-session.sh start imu
bash tools/deploy/rosy-session.sh stop imu
bash tools/deploy/rosy-session.sh start led
bash tools/deploy/rosy-session.sh stop led
```

지도 기반 주행에는 SLAM과 `goal_node`가 모두 필요하다. `map.launch.py`는
기본적으로 정지 모드의 경로 계획 노드도 실행한다. SLAM만 필요한 호출자는
`start_goal:=false`를 지정한다. 계획 노드 실행만으로 주행을 시작하지 않는다.

2026-09-09 실기 적용 시 기존 SLAM 지도를 유지하기 위해 실행 중인 map 서비스는
재시작하지 않았다. 누락된 계획 노드는 임시 `rosy-session-goal` 사용자 유닛으로
추가했다. 이 유닛은 `BindsTo=rosy-session-map.service`로 지도 서비스 정지 시 함께
종료되고, 이후 지도 서비스를 새로 시작하면 갱신된 launch가 계획 노드를 실행한다.

부분 센싱은 IMU를 교정 판정에서 제외한 정지 관측입니다. IMU 서비스를 켜도 주행 허가나 전체 교정 완료로 바뀌지 않습니다. 전체 보정 복귀는 대시보드의 **부분 센싱 종료 · 전체 보정 다시 시작**으로 별도 요청합니다.
