# 초기화 진단과 적용 상태

## 확인한 실행 구조

기존 `robot.launch.py`는 IMU·카메라를 바로 시작하고 안전(1.5초), 주행(3초), LCD·웹·감시(3.5초), 보정(4초)을 시작했다. 고정 지연은 준비 완료 확인이 아니다. 하드웨어 bringup·ADC·LED·SLAM은 별도 서비스이므로 launch 파일만 변경해서는 부팅 실행 구성이 바뀌지 않는다.

`control/startup_profile.py`는 프로세스 선택, `launch/robot.launch.py`는 선택된 ROS 프로세스 생성, `control/sensing_only.py`는 IMU 제외 관측 결과, `control/startup_diagnostics.py`는 상태 전환 로그를 담당한다. 비활성 패키지는 조회하거나 실행하지 않는다. 기존 수동 실행 호환을 위해 기본 프로파일은 `full`이며 실제 장치의 부팅 명령에는 `profile:=sensing`을 명시해야 한다.

## 로그 확인

```bash
journalctl --user -u rosy-session-control -b -o short-iso
journalctl --user -u rosy-session-imu -b -o short-iso
ros2 topic echo /calibration/status
```

시작 로그의 `selected processes`와 `disabled`로 실행 선택을 확인한다. 보정 로그의 JSON `event=startup_state`는 단계·모드·준비 여부·센서 상태가 변할 때 출력한다. 센서 표본 수 변화마다 반복 출력하지 않는다. `stale`은 수신 부족, `excluded`는 운영자가 선택한 진단 제외이며 정상 센서 판정과 구분한다.

IMU 드라이버는 장치 접근, 초기화, 샘플 해석, ROS 발행을 분리했다. 장치 로그의 `stage=probe reg=0x0 actual=-1 errno=121`은 칩 ID를 읽지 못한 지점이며, 이 경우 reset 쓰기를 실행하지 않는다. 샘플을 받지 못한 상태를 보정 완료로 처리하지 않는다.

## 장치 검사 증거와 한계

사용자가 실행한 독립 검사에서 I²C 컨트롤러를 해제하고 GPIO0/1에서 저속으로 주소 0x28·0x29를 조회했지만 두 주소 모두 ACK=False였다. 유휴 SDA/SCL은 모두 1이었고 검사 후 컨트롤러와 핀 설정이 복구되었다. 따라서 ROS 노드 동시 실행만으로 현재 무응답을 설명할 수 없다. 센서 전원·배선·센서 내부 상태 중 어느 것이 원인인지는 아직 확정하지 않았다.

재연결 후 `local-20260909-selective-v1`을 장치에서 빌드했고 ROS 환경의 집중·HTTP 시험 26개가 통과했다. 기존 서비스/실행 명령은 migration backup의 `selective-startup/`에 보관했다. control 실행 인자를 센싱 구성으로 변경했고 부팅 자동 실행은 bringup·ADC·control만 유지했다. map·LED·IMU는 disabled이다.

실제 시작 로그에서 safety·camera·web·calibration 선택, IMU·wander·LCD·watch 제외를 확인했다. API는 `mode=sensing_only`, `ready=false`, `motion_allowed=false`, `estop=true`이며 `/teleop`과 주행 시작 요청은 HTTP 409로 차단되었다. 대시보드는 HTTP 200이다. 지도 서비스가 꺼져 있어 map/map_tf가 준비되지 않았고 LiDAR 정밀 기준도 invalid로 보고되어 부분 기준값 전체 준비는 완료하지 않았다.

10초 관측에서 `/scan` 100개, `/camera/front` 80개, `/cmd_vel` 601개를 받았다. 관측한 모든 모터 속도 명령은 0이었다. LiDAR 수신과 정밀 보정용 벽 추적 기준 충족은 별개이다. 장치의 `~/.local/state/rosy_control/selective-startup-evidence.json`에 측정 결과와 API 상태를 저장했다.

이번 부팅 21:01:24에는 IMU 첫 유효 샘플 로그가 있었다. 이후 프로세스에서 `D / i2c_dw_xfer`를 관측했으나 지속 정체는 입증하지 못했다. 21:16:57 중지 요청 직후 정상 종료되어 영구 커널 멈춤으로 판단하지 않는다. 21:19 재시작 검사에서는 칩 ID를 읽고 소프트 리셋을 쓴 후 chip boot 단계가 errno 121로 실패했고, 20초간 IMU 수신은 0개였다. 이는 리셋 직후 실패의 재현 증거이며 정확한 전기적 원인까지 확정한 것은 아니다.
