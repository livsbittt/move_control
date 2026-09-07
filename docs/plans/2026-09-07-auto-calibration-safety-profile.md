# Auto calibration 기반 주행·안전 파라미터 계획

> 상태: 계획 초안. 구현·파라미터 적용·로봇 이동은 수행하지 않는다.
> 기준: 2026-09-07, main `6f6e461cc3ce3ac67b6c1c6343738c8ea556d2c8`의 로컬 코드.
> 동시 수정 대응: 구현 착수 전에 변경된 소비 경로와 설정을 다시 대조한다.

**Goal:** 기기 형상, 센서 관측 성능, 실제 이동·정지 응답을 근거로 주행 파라미터를 산출하고, 동일한 검증 프로파일을 safety·wander·planner·UI가 사용하게 한다.

**Architecture:** 고정 기기 모델, 측정 증거, 파라미터 산출, 후보 검증, 프로파일 활성화를 분리한다. 의사결정은 ROS 없는 subject에 둔다. safety는 `/cmd_vel`의 유일한 발행자로 남는다.

**Tech Stack:** ROS 2 Jazzy, Python/numpy, 기존 safety/wander/goal 노드, 기존 HTML dashboard, ROS 격리 테스트 및 실제 기기 증거.

이 문서는 이전 worktree의 `2026-09-07-rotation-calibration.md`보다 우선한다. 이전의 ±20도·0.10rad/s·회전 배율 도입은 확정 사양이 아니라 검토 후보다. 해당 worktree의 미구현 테스트는 현재 코드에 통합하지 않는다.

## 1. 결정: 부팅 때 모든 값을 새로 학습하지 않는다

| 방안 | 장점 | 한계 | 결정 |
|---|---|---|---|
| 기존 짧은 왕복에 회전만 추가 | 작은 변경, 방향별 응답 검사 가능 | 형상·제동·파라미터 소유권 문제 미해결 | 단독 해법으로 부족 |
| 매 부팅마다 바퀴 기하·센서 장착·안전거리 전체 추정 | 이상적인 환경에서는 폭넓은 자동화 | 관측 부족, 긴 시험, 보정 전 안전 보장 순환 문제 | 채택하지 않음 |
| 설치 시 정밀 측정 + 부팅 시 재검증 + 운용 중 상태 감시 | 물리 한계를 유지하면서 실제 조건 반영 | 증거·프로파일 관리 필요 | 권장 |

자동 보정은 파라미터 결정의 중심이 되되, 관측할 수 없는 물리량을 임의로 추정하지 않는다. 산출 불가 값에는 이유와 필요한 증거를 남긴다. 명시적 검증 없이 학습값으로 안전 하한을 축소하지 않는다.

## 2. 현재 코드에서 확인한 출발점

아래는 코드와 함수 계산의 결과이며 실행 중인 로봇 파라미터 readback 결과는 아니다.

| 현재 사실 | 주행·보정에 미치는 의미 | 근거 |
|---|---|---|
| 시작 보정은 정지 검사 + 3cm 왕복 + 배율 적용 후 반복 | 전후 저속 명령 응답을 다룸. 회전·제동·물리 형상을 식별하지 않음 | [round_trip.py](../../move_control/control/round_trip.py), [startup_calibration_node.py](../../move_control/startup_calibration_node.py) |
| 왕복 계수는 명령 적분 / 라이다 전방 sector 거리 변화 | 실제 gate 출력이 제한된 시행, 벽 각도·sector 대상 변화·필터 지연을 분리해야 함 | `RoundTrip.update`, `CalibrationRangeFilter` |
| IMU 정지 추정치는 기록하고, TF 검사에서는 같은 TF로 nose를 역산 | 추정 기록은 보정 적용이 아님. TF 자기일관성은 실제 장착각 정확도 증거가 아님 | `StartupCalibrationNode.report/read_tf` |
| 기본 반경 0.076m, 라이다 x=-0.017m | 약 11cm라는 제품 표현과 15.2cm 외접원 지름은 같은 치수가 아님. 실제 돌출부를 대조해야 함 | [body.py](../../move_control/sensing/body.py) |
| `use_radius`는 0.040~0.150m 범위 값을 수용 | 기기 형상보다 작은 반경도 범위 검사만 통과할 수 있음. 물리 하한 검증 필요 | `use_radius` |
| safety는 robot→safety→cliff_calib→auto_calib 순서로 설정을 읽음 | 오래된 자동 파일이 공유 설정을 덮어씀 | [robot.launch.py](../../launch/robot.launch.py) |
| 공유 stop/clear=0.12/0.14m, auto 파일은 0.018/0.028m | 현 함수에 auto 값을 넣으면 실제 내부 하한은 0.111/0.121m. 설정값과 유효값을 구분해야 함 | [robot.yaml](../../config/robot.yaml), [auto_calib.yaml](../../config/auto_calib.yaml), [lidar_guard.py](../../move_control/control/lidar_guard.py) |
| auto 파일 전방 반각 8도는 safety에서 최소 45도로 보정 | ROS 파라미터 조회만으로 내부 유효값을 알 수 없는 사례 | `Bumper._refresh_distances` |
| wander 회전 공간 기준은 기본 0.086m, safety 회전 하한은 0.103m | 동일한 라이다 거리에도 행동 제안과 최종 허가 기준이 다름 | `Senses._turn_clear`, `lidar_can_rotate` |
| wander slow_front=0.04m, slow_rear=0.03m, stop_front=0.12m | stop보다 slow가 작아 의도된 보간 구간이 성립하지 않음. 개별 숫자 튜닝보다 순서 불변식 필요 | [wander.yaml](../../config/wander.yaml), `Motion._blend_speed` |
| turn은 odom 목표각 OR 시간 경과, 열린 방향이면 조기 전진 | 회전 gain만으로 성공의 의미를 바로잡을 수 없음 | `Motion._tick_turn` |
| escape에는 정렬·시간·복구 상태에 따른 여러 전진 복귀 경로가 있음 | 모든 경로가 정지 확인 및 새 관측 검사를 공유해야 함 | `Motion._tick_escape/_resume_forward` |
| planner clear_m=0.12m는 별도 설정, UI는 자기 파라미터를 표시 | safety 유효 한계와 planner/UI의 일치를 보장하는 계약이 없음 | [goal.yaml](../../config/goal.yaml), `GoalNode.grid_clearance`, `WebNode.read_limits` |
| `auto_escape`는 운용 중 민감도를 조절하고, `auto_map`은 L+R로 범위를 조절 | 전자는 행동 적응, 후자는 관측 범위 적응. 기체 크기 또는 안전 한계 학습으로 취급하지 않음 | `Motion._commit_escape_params`, [scale.py](../../move_control/safety/scale.py) |

추가 선행 확인: startup IMU는 단위 파라미터를 사용하지만 safety IMU는 값 크기로 단위를 추측한다. sensor validity와 정지 조건을 분리하고 각 소비자의 단위·시각을 일치시켜야 한다. 현재 safety의 IR 유실 처리, cliff/tilt 시 회전·후진 정책은 별도 경로에 있으므로 calibration 전용 정지 조건을 최종 gate에서도 보장해야 한다.

## 3. 파라미터를 네 종류로 구분한다

| 종류 | 대표 값 | 결정 근거 | 자동 변경 범위 |
|---|---|---|---|
| 기기 고정 모델 | 차체 footprint, 외접 반경, 바퀴·캐스터 접지 위치, 센서 TF, 돌출부 | 실제 조립 상태 실측 + 설치 URDF/TF 대조 | 부팅 때 변경 금지. 구조 변경 시 새 기기 모델 revision |
| 측정·추정 모델 | IMU bias, 전후/좌우 응답, deadband, 제동 잔여량, 데이터 지연·노이즈 | 관측 가능한 시험과 반복 검증 | 증거가 있는 속도·방향·표면·하중·전원 조건 내 후보 생성 |
| 파생 주행 한계 | stop/clear/slow, vmax/vback/wturn, 회전 sweep 여유, settle 시간, planner clearance | 고정 모델 + 측정 모델 + 독립 최소 여유 | 고정 하한 내 산출, 후보 재검증 및 profile 활성화 후 사용 |
| 독립 정책/사용 의도 | E-stop, 절벽 활성화, 최대 허용 속도, 최소 여유, 목표 경로·목표 회전각 | 시스템 안전 요구·운용 목적 | 보정으로 해제·완화하지 않음. 목표각은 응답 보정값과 구별 |

### 파라미터별 첫 적용 범위

| 기존/예정 항목 | 처리 방침 | 소비자 |
|---|---|---|
| `robot_radius` | 검증 footprint로부터 계산한 외접 반경의 호환 값. 환경의 통로 폭에서 추정하지 않음 | safety, wander, planner, UI |
| `lidar_yaw_offset` 및 센서 x/y/yaw | 설치 보정의 후보. 전체 scan 변위와 알려진 차체 운동이 필요. TF 단일 소유자에서 배포 | sensing, safety, calibration, UI, SLAM |
| `cmd_linear_sign`, 회전 부호 | 하드웨어 설치 시험으로 검증. 주행 도중 자동 반전 금지 | 드라이버/최종 gate 계약 |
| 전후 저속 gain | 직진 오차와 gate 제한을 분리한 시험에서 추정 | 적용 소유자 한 곳만 지정 |
| 회전 gain | 각도 관측·정지 검증이 먼저. 필요할 때만 속도 feed-forward 보정 도입 | 최종 gate 또는 드라이버 중 한 곳 |
| `imu_roll0/pitch0`, gyro bias | 알려진 수평·정지 상태에서 후보 생성. 경사진 바닥을 수평으로 학습하지 않음 | 공통 IMU 정규화 및 safety |
| `cliff_raw_max/clear/hits` | 일반 바닥 기준만으로 확정 금지. 채널별 바닥/비지지 표본 및 반응거리 검증 필요 | safety |
| `us_scale`, range offset | 알려진 여러 거리·면 각도에서 검증. lidar-US 차이 하나를 offset으로 저장하지 않음 | sensing/safety |
| `stop_distance/clear_distance/slow_*` | 같은 좌표·같은 여유거리 정의에서 파생. 이전 센서 원점 파라미터는 명시적 adapter | safety, wander |
| `front_half_width_deg` | 고정 좁은 cone에 의존하지 않도록 실제 이동 footprint와 beam 교차로 대체 계획 | safety |
| `turn_clear_m`, `clear_m`, `retry_clear_m`, `escape_clear_m` | 같은 기기 모델을 사용하되 로컬 sweep/지도 오차의 목적 차이는 유지 | safety, wander, planner |
| `align_deg`, pause/settle | 통로 여유·예상 다음 이동거리·회전 관측/정지 오차에서 검증 | wander |
| `sensor_timeout`, filter/hits | 실제 지연과 허용 정지 거리에서 제한. 유실을 통과시키려 timeout을 늘리지 않음 | sensing, safety |

## 4. 기체 반경과 안전거리의 계산 계약

### 4.1 거리 원점을 통일한다

센서 거리 r을 바로 차체 여유거리로 읽지 않는다. 스캔 각도는 기존 `robot_yaw()/wrap_pi()`를 거쳐 처리하고, 실제 센서 TF의 회전·평행이동으로 obstacle point를 `base_link`에 옮긴다.

`p_base = R_base_sensor · p_sensor + t_base_sensor`

차체 footprint를 F라 할 때 벽 충돌 검사는 p_base와 F 또는 예상 이동 중 F의 합집합 사이 거리로 정의한다. 원형 근사 시에는 `||p_base|| - R_body`를 사용할 수 있으나 직진 경로 전체 충돌은 별도로 검사한다. 초음파는 한 점이 아닌 빔 영역과 유효 측정 범위를 고려한다.

`R_body = max(||vertex|| for vertex in verified_footprint)`

외형 오차는 별도 `geometry_uncertainty`로 둔다. 기기 모델에는 카메라 지지대·케이블·바퀴·캐스터의 최외곽을 포함한다. 현재 `body.py`는 URDF를 실행 시 읽는 코드가 아니라 URDF 출처라고 주석된 상수 계산이므로 실제 설치 파일과 대조해야 한다.

초기 버전은 검증된 보수적 외접원으로 유지한다. 이 때문에 통로가 막히면 반경을 줄여 통과시키지 않는다. 더 좁은 통로 지원은 실측 polygon과 orientation별 경로 검증을 완료한 후 별도 확장한다.

### 4.2 직진·후진 안전 여유

다음은 설계식이며 현재 기기에서 확인된 제동 성능값이 아니다.

`margin_required(v, direction, condition) = margin_min + uncertainty + travel_during_delay + braking_travel`

단순 초기 모델은 `travel_during_delay=|v|·T_delay`, `braking_travel=v²/(2·a_brake_lower_bound)`다. 정지 응답이 비선형인 저속 모터는 실측 상한 envelope/table을 우선한다. 제동 감속 하한이 확인되지 않았으면 식에 임의 숫자를 넣어 안전거리를 축소하지 않는다.

- T_delay에는 센서 취득/scan 시간·대기열·필터·판단·명령 전달·구동기 반응을 포함하고, 정상/지연/노드 유실 정지를 구분한다.
- 독립 실제 속도의 보수적 상한과 최종 출력 이력을 사용한다. 보정하려는 odom만으로 실제 속도·제동을 증명하지 않는다.
- end-to-end 측정 잔여거리가 이미 포함한 지연은 중복 가산하지 않는다.
- 작은 표본의 최댓값이나 P99를 최악 조건 보장이라고 부르지 않는다. 표본 수, 조건, 범위, 불확실성과 추가 margin을 함께 명시한다.
- 범위 밖 속도·하중·바닥에는 외삽하지 않는다. 검증된 속도로 제한하거나 해당 모드를 보류한다.

현재 호환 scalar를 유지하는 동안에는 보수적 라이다 원점 임계값을 `R_body + ||t_lidar|| + margin_required`로 변환할 수 있다. 모든 방향에 같은 scalar를 쓰는 보수적 근사이며, 센서점을 base로 변환한 뒤 offset을 다시 더하면 안 된다.

현재 코드 예: `76mm + 17mm + 18mm = 111mm`. 이는 함수 하한 산술이지 정지거리 실측 결과가 아니다. 120mm와의 차이가 확인되었다고 어느 쪽을 새 정답으로 확정하지 않는다.

불변식: 같은 기준의 `stop < clear <= slow`, 전후 각 방향 별 적용, `margin_required >= margin_min`, 허용 v가 증가할 때 검증된 정지 envelope가 감소하지 않음. 히스테리시스 폭은 안정성·노이즈와 재출발 여유를 반영한다. 남은 여유가 부족하면 think-speed를 강제 최솟값으로 보장하지 않고 0까지 낮춘다.

### 4.3 회전 및 곡선 주행

회전은 시작각에서 정지 후 예상 잔여각까지 footprint가 훑는 영역을 검사한다. 회전축 이동·미끄러짐·센서 TF 불확실성도 포함한다. 완전 원형이 중심에서 제자리 회전하면 sweep은 같은 원이므로 무조건 `R + Rθ`로 팽창시키지 않는다.

`theta_stop ≈ |omega|·T_delay + omega²/(2·alpha_brake_lower_bound)`는 후보 모델이며 실측 종료 오차로 검증한다. 곡선 주행은 v와 omega를 함께 넣어 trajectory sweep을 검사한다. 제자리 회전 계수를 곡선 주행에 검증 없이 재사용하지 않는다.

통로 내 방향 오차는 다음 이동거리 L에서 대략 `L·sin(|yaw_error|)`의 옆방향 오차를 만든다. 여기에 footprint 방향 변화와 tracking 오차를 더한 값이 실제 측면 여유 안에 있어야 한다. 따라서 `align_deg`를 모든 통로에서 한 값으로 성공 처리하는 방식도 검토 대상이다.

### 4.4 절벽·바닥 지지는 별도 안전영역이다

수평 라이다의 빈 공간은 바닥이 있다는 증거가 아니다. 현재 IR 세 채널이 정상이어도 다음 회전·후진에서 바퀴와 캐스터가 지나갈 바닥 전체를 증명하지 않는다.

- obstacle footprint와 바퀴/캐스터 support footprint를 분리한다. IR 위치·지향·관측 바닥 영역과 접지부의 궤적을 대조한다.
- 절벽 감지 위치에서 접지부가 경계에 도달하기 전 남는 거리 ≥ 감지/판단 지연 이동 + 정지 잔여거리 + 오차 여유여야 한다. 전진·후진·양방향 회전별로 확인한다.
- 증명되지 않은 책상 경계·후방 지지는 자동 보정 동작 허가 근거로 쓰지 않는다. 정밀 측정은 충분히 넓은 지지면 또는 검증된 보호 fixture에서 수행한다.
- cliff 표본 수집을 위해 로봇이 스스로 책상 밖으로 진행하게 하지 않는다. 4095는 ADC 포화/유효성 별도 상태이며 cliff 학습 표본으로 사용하지 않는다.
- 먼/가까운 무효 거리, scan blind zone, 가려진 방향은 unknown이다. `inf`를 여유 공간으로 변환하지 않는다.

### 4.5 공간에 따른 실시간 속도 결정

자동 보정은 조건별 이동·제동 모델과 검증된 속도 상한을 정하고, 운용 중 속도 제어는 현재 공간에 맞는 `(v, omega)`를 그 범위 안에서 매 주기 선택한다. 환경이 바뀔 때마다 calibration gain이나 고정 profile을 다시 쓰지 않는다.

현재 `Motion._safe_speed`에도 전후 거리, 통로 여유에 따른 narrow factor, `gap/think_horizon`, odom 속도를 활용하는 기초 구조가 있다. 이를 단순 비례 속도에서 검증된 정지·관측·footprint 제약을 만족하는 속도 선택으로 확장한다. 현재 `_spin_wz`의 고정 wturn도 회전 공간과 남은 목표각에 따라 제한한다.

| 주행 상황 | 속도 선택 기준 |
|---|---|
| 넓고 관측된 직선 | 검증 최대 속도 범위 내에서 부드럽게 가속. 시야 끝까지의 정지 가능 거리로 상한 제한 |
| 좁은 통로 | 좌우 각각의 차체 여유, 중심 편차, 방향·tracking 오차로 감속. 통로 폭만 같다고 같은 속도를 주지 않음 |
| 전방 장애물·막다른 길 | 실제 이동 방향에서 정지할 수 있는 속도로 낮춤. 여유가 없으면 0 |
| 모서리·곡선 진입 | v와 omega를 함께 평가하여 차체 sweep과 측면 충돌을 확인. 회전반경만으로 통과 가능 판정하지 않음 |
| 제자리 회전 | 전체 sweep과 지지면, 회전 제동 잔여각, 목표까지 남은 각도로 각속도 제한 |
| 후진 | 후방 관측·지지면과 후진 전용 제동 모델 사용. 전방이 넓다는 이유로 후진 가속하지 않음 |
| 가려진 구역·센서 지연·관측 불확실 | 확실히 관측된 공간 안에서 정지 가능한 속도로 제한. 필요한 관측이 없으면 정지 |

설계 개요는 `v_selected <= min(v_profile, v_stopping, v_lateral, v_curve, v_observation)`다. 각 상한은 임의 상수가 아니라 해당 제약을 만족하는 속도이며, 최종 `(v, omega)` 쌍은 가속·제동 중 궤적까지 검사한다. v와 omega를 독립적으로 잘라 곡률을 바꾸었으면 변경된 궤적을 다시 평가한다.

단순 직진 모델에서는 최소 여유와 오차를 뺀 사용 가능 이동거리 d에 대해 `v*T + v²/(2*a) <= d`를 만족시킨다. a>0, T>=0, d>=0일 때 예시 상한은 `v_stopping = sqrt((a*T)² + 2*a*d) - a*T`다. d<0이면 0이며, a/T의 검증이 없으면 이 식으로 속도를 확대하지 않는다. 실제 적용은 검증된 제동 envelope를 역으로 조회하는 방식이 우선이다. 현재 속도가 이미 새 상한보다 높다면 즉시 제동하고 상한 변경만으로 안전해졌다고 판단하지 않는다.

안정적인 운용을 위해 가속은 rate limit과 공간 회복 확인을 적용한다. 위험 방향의 감속/정지에는 일반 표시 smoothing이나 재가속 hysteresis를 지연 요소로 넣지 않는다. 실제 모터 제동 한계는 정지 모델에 포함한다. 하한 속도보다 낮은 명령에서 구동이 불안정하면 더 빠른 creep을 강제하지 않고 정지/재계획한다.

UI에는 요청 속도·최종 허용 속도·실제 관측 속도와 제한 이유(전방 거리/측면 여유/곡선/지연/미관측)를 구분한다. acceptance에는 통로 폭 변화, 한쪽 벽 접근, 코너 진입, 좁은 곳에서 넓은 곳으로 나오는 경우, 급격한 관측 유실을 포함한다. 공간이 줄거나 불확실성이 늘 때 허용 속도가 커지지 않는 성질을 검증한다.

## 5. 측정과 관측 가능성

| 실험 | 관측/추정할 값 | 독립 기준·조건 | 부족하면 |
|---|---|---|---|
| 수평 정지 | IMU bias/노이즈, IR 바닥 분포, timestamp/jitter, 센서 안정성 | 고정 평면·기체 정지, 센서별 새 표본 | 재수집 또는 해당 센서 보류 |
| 전진/후진 여러 짧은 구간 | 속도 응답, deadband, 직진 편향, 반복성 | 알려진 벽/외부 표식 또는 관측 가능한 scan 변위 | gain 유지, 미검증 범위 속도 금지 |
| 좌/우 회전 | 부호, 실제 각속도, pivot 이동, 종료 잔여각 | raw scan 상대 자세 + IMU/odom 비교, 설치 시 외부 각도 기준 | 회전 준비 미완료 |
| 정지 명령 시험 | 실제 정지 시각·거리·잔여각 | 위험물에 접근하지 않는 넓은 지지면, 실제 출력·외부 운동 관측 | 안전 여유 축소 금지 |
| 직진+회전 혼합 | 바퀴 기하 또는 lidar x/y/yaw의 설치 보정 | 실제 바퀴 피드백, 동기화, 충분한 장면 구조 | 설치 모델 유지/설치 검증 요구 |
| 실제 주행 회귀 | 회전 후 전진, 좁은 통로, 모서리, 후진 탈출 | 동일 profile로 전체 루프, 안전 gate 출력 기록 | 후보 프로파일 거부 |

라이다 정합 후보는 point-to-line ICP 또는 벽/코너 특징 기반 상대 pose다. 구현 선택 전에 로봇 raw scan replay로 비교한다. 정합 성공 flag 하나가 아니라 overlap, residual, 관측 가능한 방향, 추정 불확실성, 처리 지연을 함께 판정한다. 단일 벽·평행 복도·대칭 구조·움직이는 물체·저점수에서는 추정을 보류한다. scan 취득 중 회전 왜곡과 clock/TF 정렬도 측정한다.

관측 가능성 해석: 평면 센서 변위는 `R(-phi)[d + (R(theta)-I)t]`로 쓸 수 있다. 직진에서는 t가 사라지므로 센서 장착 x/y를 식별하지 못한다. 차체 실제 직진 방향을 알면 yaw는 추정 가능하다. 순수 회전만으로 모든 장착 변수를 분리할 수도 없으므로 필요한 경우 혼합 운동을 사용한다. [선행 조사](../research/2026-09-07-calibration-motion-observability.md)

센서 독립성 계약: map TF는 SLAM이 같은 scan/odom을 사용한 결과일 수 있으므로 별개의 ground truth로 세지 않는다. odom에 IMU가 융합돼 있다면 IMU-odom 일치도 독립 검증 두 개가 아니다. 본 저장소 밖의 실제 odom 생성 경로를 먼저 확인한다.

항상 `/cmd_vel_raw` 요청, safety가 허가한 semantic 속도 및 제한 이유, 실제 `/cmd_vel` 출력, 독립 이동 관측을 함께 기록한다. 제한·충돌 정지·명령 경합이 발생한 시행으로 gain을 키우지 않는다. 이 시행은 진단 증거로 저장하고 parameter fitting에서 제외한다.

IMU 내부 전체 보정과 주행 yaw 응답 검증은 별개다. BNO055 내부 calibration status와 드라이버 단위 계약은 설치 단계에서 확인한다. 부팅 정지 샘플을 내부 9축 전체 보정 완료로 표시하지 않는다.

## 6. 세 가지 실행 모드

### 설치·기기 변경 시 commissioning

1. robot ID, 조립 revision, 실측 footprint/support geometry, TF·드라이버·odom 출처를 확정한다.
2. 보수적인 검증 bootstrap profile로 시험 공간·지지면·관측 가능성을 확인한다. 미검증 후보로 자기 시험의 안전거리를 줄이지 않는다.
3. 정지 → 전후 응답 → 좌우 회전 → 정지 응답 → 필요한 혼합 운동을 수행한다. 시험 거리는 신호 대 잡음과 허가된 이동영역이 함께 만족하는 범위에서 정하며, 공간이 부족하면 자동으로 더 멀리 움직이지 않는다.
4. calibration dataset에서 후보를 계산하고, 별도로 수집한 validation dataset/주행으로 검증한다.
5. 조건별 불확실성과 한계를 붙여 versioned profile을 발행한다.

### 매 부팅 auto calibration / revalidation

`PROFILE_CHECK → STATIONARY → SPACE_CHECK → LINEAR_CHECK → ROTATION_CHECK → STOP_SETTLE_CHECK → VALIDATE → STAGE/APPLY → READY`

- 우선 기존 검증 프로파일의 유효성을 확인한다. 기본 동작은 정밀 보정 전부의 반복이 아니라 해당 기기·환경에서의 재검증이다.
- 작은 전후 및 양방향 회전으로 방향·응답·정지 오차가 검증 범위 내인지 확인한다. 수치·동작 크기는 commissioning 증거 후 확정한다.
- 후보 변경이 필요하면 bounds 안에서도 별도 재검증한다. 확대된 속도·새 반경·외부 파라미터 변경은 commissioning으로 보낸다.
- 자동 E-stop 해제, 실패 후 자동 재시도, 보정 미완료 시 일반 주행 허가는 하지 않는다.
- calibration에는 제한된 command lease가 필요하다. wander stop 확인만으로 다른 teleop/control의 동시 raw 명령을 배제했다고 간주하지 않는다. 최종 gate가 허가된 소유자와 허용 동작 범위를 검증한다.
- motion 중 노드 사망·lease 유실·센서 유실·기울어짐·cliff·pickup·정합 상실·budget 초과 시 zero와 ready 취소. 최종 gate와 모터 watchdog까지 실제 정지 검증에 포함한다.

### 운용 중 감시

기존 profile 범위 내 응답과 관측 성능을 감시한다. 작은 이상이 계속되면 속도 제한 또는 재검증 요청으로 전환한다. 주행 도중 footprint 축소, 모터 부호 반전, 절벽 비활성화, 안전거리 완화는 하지 않는다. 안전 증거가 사라졌을 때 오래된 정상 profile로 자동 복귀해 계속 달리지 않는다.

## 7. 주행 로직과 연결할 계약

회전 종료를 `목표 방향 관측 → 감속/zero → 실제 정지 확인 → 정지 이후 새 scan/IR/위험 상태 확인 → 경로·공간 재평가 → 전진`으로 정의한다. 고정 시간이 경과했음은 성공이 아니라 목표 미도달/관측 불가의 실패 원인이다.

- `turn`, `escape`, `wall`, `backup`의 전진 복귀 경로가 같은 재출발 검사를 공유한다.
- 회전 중 열린 방향이 감지되더라도 바로 전진하지 않고 정지/재관측 과정을 거친다.
- angle feedback가 유효하지 않으면 시간만으로 성공 처리하지 않는다. 실제 angle source와 timestamp를 결과에 남긴다.
- 제동/응답 모델은 feed-forward 감속 시점을 도울 수 있지만 실제 도달/정지를 대체하지 않는다.
- 현재 `auto_escape` 민감도 적응은 유효 profile envelope 안에서만 허용하고 safety 한계를 바꾸지 못한다.
- planner는 기기 형상·지도 위치오차·grid 해상도를 반영해 경로를 검사한다. `clear_m`에 센서 원점 offset을 무조건 더하지 않는다. 방향 없는 원형 A*는 보수성을 명시하고 좁은 경로를 위해 body 크기를 축소하지 않는다.
- occupancy inflation이 unknown을 확장하지 않는 현재 동작에서는 차체가 unknown 영역 위로 걸치는 경우도 별도 검토한다. 경로 중심 cell이 free라는 것과 footprint 전체가 알려진 공간 안에 있다는 것은 다르다.

## 8. 파라미터 활성화는 하나의 프로파일 단위

파일 load 순서가 권한을 결정하지 않게 한다. `robot.yaml`은 고정 기본 정책의 단일 출처로 유지하고, 검증된 runtime profile은 명시적 overlay로만 사용한다. 기존 `auto_calib.yaml`은 schema/version 없는 구형 입력으로 표시해 migration 검증 전 묵시 적용하지 않는 방향으로 전환한다.

프로파일에는 `schema_version`, `profile_revision`, `robot_id`, geometry/TF/driver hashes, algorithm version, 측정 조건, 데이터 참조, 추정값·불확실성, 파생값, 유효 속도/방향, 검증 결과, 만료/재검증 조건을 포함한다. `nominal`, `candidate`, `effective`, `applied` 값을 구별한다.

활성화 순서:

1. 일반 주행 revoke, zero와 정지 확인, 이전 활성 profile 식별.
2. candidate 생성 및 불변식 검증. 별도 반복에서 허가된 candidate를 사용하되 일반 ready는 false 유지.
3. 필요한 소비자(safety, wander, 활성 planner)에 동일 revision을 stage. 각 소비자가 내부 계산까지 완료한 effective 값과 revision을 응답한다.
4. 전 소비자 준비 확인 후 파일을 원자 교체하고 active revision을 commit한다. 분산 ROS 서비스 호출 자체는 원자적이지 않으므로 **서로 다른 revision 동안 주행 불가**라는 barrier를 보장한다.
5. 활성 revision readback 일치 후 ready를 발행한다. 소비자 재시작·부분 실패·파일 실패·시간 초과는 ready false. 동일 revision 일치 복구 전 주행 재개 금지.

동적 envelope는 속도에 따라 tick마다 평가하되 식과 측정 모델의 revision은 고정한다. 토픽의 별도 gain과 ready가 서로 다른 부팅/프로파일에서 섞이지 않게 식별자·시각·유효 조건을 전달한다. 최종 gate에는 geometry 하한과 최대 허용 범위의 독립 검사가 남는다.

gain 적용 전 요청값만 검사하지 않는다. gain 이후 실제 출력 속도도 검증 범위와 safety envelope 안에 있어야 한다. 명령 부호 변환과 gain 적용 순서·소유자를 고정하여 startup/driver/safety에서 중복 보정하지 않는다. gain이 커졌다는 이유로 검증된 실제 속도 상한을 초과하지 못하게 한다.

기존 수동 `calib_node.compute()`는 고정 stop 값도 쓰며 비동기 parameter 응답 검증 없이 완료를 알린다. 이 경로도 같은 stage/apply 계약을 통하도록 정리해야 한다. 새 자동 노드만 고치고 구형 writer가 활성 파라미터를 덮어쓰게 두지 않는다.

## 9. 화면과 진단 결과

UI는 safety가 공개한 effective profile을 기준으로 표시한다. 현재처럼 UI 자체 파라미터만으로 실제 정지 한계를 설명하지 않는다.

| 화면 항목 | 표시할 내용 |
|---|---|
| 기체 | 검증 외형/반경, 기준 원점, geometry revision |
| 안전 여유 | 차체 외곽 기준 전후 거리, 회전 가능 공간, 실제 센서 원점 거리와의 구분 |
| 측정 | 정지/전후/회전/정지 응답 각각 검사·추정·적용·재검증 상태 |
| 적용 | 활성 profile revision과 safety/wander/planner의 적용 일치 |
| 미완료 | 관측 부족, 지지면 미확인, 좁은 시험 공간, 센서 유실 등 구체적 이유 |
| 범위 | 검증 속도·방향·조건, 장착 보정 및 IMU 내부 보정 미수행 여부 |

`sensors_valid`, `geometry_verified`, `linear_verified`, `rotation_verified`, `stopping_verified`, `profile_applied`, `navigation_ready`를 구별한다. 처음에는 기존 ready를 요구 항목의 conjunction으로 유지하고, 임의의 부분 성공으로 일반 주행을 열지 않는다. 전진만 가능한 제한 모드 등은 별도 운용 계약이 있을 때 추가한다.

## 10. 구현 순서와 완료 조건

| 단계 | 예정 파일/책임 | 완료 조건 |
|---|---|---|
| P0 현재 계약 정리 | config/launch, `body.py`, `lidar_guard.py`, `startup_calibration_node.py`, `calib_node.py`, `web_node.py` 조사 및 profile schema | nominal/effective/소비자 mapping, 구형 override 처리, 데이터 단위·좌표, actual odom 출처 문서화 |
| P1 기기 형상과 안전 정책 | 새 pure `sensing/footprint.py`, `control/safety_envelope.py`, `control/calibration_profile.py` | 회전/직진 sweep, 물리 하한·blind zone·unknown·불변식 테스트; 실측 geometry와 대조 |
| P2 측정과 추정 | 새 `sensing/motion_observation.py`, `control/calibration_estimation.py`; 기존 startup I/O 얇게 연결 | raw/final/observed 기록, 시각 정렬·독립성, 관측 불가 거부, 정지·직진·회전 응답 모델 |
| P3 후보 검증과 활성화 | `control/calibration_session.py`, startup, safety, 필요한 소비자 | 보정 전 safe bootstrap, lease, 후보 별도 반복, revision barrier, readback 및 rollback 정지 |
| P4 주행 소비 | wander Motion/Senses/Contact, goal/planning, safety | timeout 성공 제거, settle→fresh observation 후 전진, 하나의 effective envelope 준수 |
| P5 UI·증거·배포 | dashboard/web, 테스트·운영 문서 | 단계별 결과·유효값·이유 표시, 격리 통합과 실제 기기 시험 결과 분리 |

각 단계는 실패 테스트 → 최소 구현 → 회귀 확인 순으로 진행한다. 기존 public imports/ROS topic 호환은 migration 중 유지한다. 목표는 독립 module 수를 늘리는 것이 아니라 geometry·측정·정책·상태·I/O 책임을 분리하는 것이다.

### 필수 시험 행렬

- Geometry: 실제 최외곽 포함, 장착 180/190도, offset 양/음, 원점 변환, 작은 radius 후보 거부, 센서 offset 중복 가산 금지, 회전 끝뿐 아니라 중간 sweep 충돌.
- Safety envelope: 전후·좌우·곡선, 속도 증가, delay 증가, brake 성능 악화, unknown/blind zone, 좁은 통로. 위험 증가가 안전 여유 감소로 이어지지 않는 성질 검증.
- Estimation: 부호 반대, deadband, 미끄러짐, gate 제한, sensor clock skew, yaw wrap, IMU 단위, 동일 센서 재사용, 단일 벽 퇴화, 잘못된 TF, 외부 물체 이동.
- Lifecycle: 취소/E-stop/유실/프로세스 재시작/파일 실패/일부 소비자 적용 실패/구형 latched 결과/동시 teleop. ready false와 실제 zero 검증.
- Behavior: 목표각 미도달 timeout, 회전 중 열린 방향, 관성 잔여각, 정지 전 취득 scan의 지연 도착, backup/escape의 전진 전환, 반복 복구.
- Hardware: 보호된 지지면에서 여러 배터리·하중·바닥 조건, 모서리/측면/후방 장애물, 센서 유실 시 모터 정지, 물리 표시와 대조한 거리/각도/잔여 이동. 고의 충돌이나 낙하를 시험 방법으로 사용하지 않는다.

검증 실행은 저장소의 `python3 -m pytest test/ -q`, 관련 `tools/test_*_ros.py`, Node dashboard tests 및 실제 브라우저 확인으로 구성한다. ROS 시험은 로봇과 통신하지 않는 별도 네트워크/도메인에서 실행한다. 실제 기기 검증은 별도 단계다.

## 11. 수치 확정 전에 필요한 증거

| 미확정 항목 | 확보 방법 | 없을 때의 처리 |
|---|---|---|
| 실제 footprint·접지부·IR 위치 | 설치 URDF + 치수 측정/위에서 본 기준 사진 | 현재 반경은 코드 nominal. 새 물리 모델 승인 불가 |
| 실제 lidar/camera/US/IMU TF와 단위 | 설치 파일·driver 코드·topic readback | 외부 파라미터 자동 변경 금지 |
| odom의 encoder/IMU/명령 의존 | 실제 bringup와 구동기 생성 경로 추적 | 독립 ground truth로 취급 금지 |
| 모터 정지/통신 유실 동작 | final command와 실제 운동 동기 기록 | 정지 margin 축소/속도 확대 금지 |
| 조건별 지연·잔여거리·잔여각 | 보호된 fixture/지지면에서 반복 기록 | 보수 bootstrap 범위만 검토 |
| scan 관측 가능성과 처리시간 | ARM64 raw scan replay 및 실제 주기 측정 | ICP 방식·보정 동작 크기 미확정 |
| 배포 후 일관된 유효값 | 실제 active revision/readback | 코드 테스트를 배포 성공으로 간주하지 않음 |

문서 작성 시점은 **설계 검토 GO / 신규 파라미터·자동 동작 적용 HOLD**다. 기존 로봇의 운영 상태를 판정한 것은 아니다. 다음 구현의 첫 작업은 회전 코딩이 아니라 P0 계약 정리와 기기 모델 증거 확보다.

## 12. 외부 1차 근거

- [Nav2 Jazzy footprint](https://docs.nav2.org/jazzy/configuration_and_development/first_time_robot_setup_guide/footprint/setup_footprint/): base_link 기준 물리 외형과 원형/polygon 표현의 구분. 본 계획은 기존 패키지에 적용할 개념이며 Nav2 전체 도입 결정이 아니다.
- [Nav2 Jazzy inflation](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/core_servers/costmap_2d/costmap_plugins/inflation/): obstacle 주변 비용장과 물리 외형을 구별한다. 이 패키지의 binary grid inflation과 Nav2 비용장 구현을 동일시하지 않는다.
- [Nav2 Jazzy Collision Monitor](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/core_servers/collision_monitor/configuring_collision_monitor_node/): 속도별 영역·TTC, 입력 timeout, 측정 시점 이후 이동 보정이 독립 안전 monitor 설계 참고다.
- [Nav2 Jazzy Velocity Smoother](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/core_servers/configuring_velocity_smoother/): 명령 기반 추정과 실제 odom feedback을 구분하며 closed-loop의 주기/지연 조건을 명시한다.
- [ROS 2 Jazzy diff_drive_controller](https://control.ros.org/jazzy/doc/ros2_controllers/diff_drive_controller/doc/userdoc.html): 바퀴 반경과 간격의 역할 및 correction multipliers 구분. 실제 Pinky driver가 이 controller라는 증거는 아니다.
- [Bosch BNO055 datasheet](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bno055-ds000.pdf): 내부 센서별 calibration 절차와 상태. 부팅 평면 운동만으로 전체 IMU calibration을 주장하지 않는다.

본문의 제동식·sweep·profile 활성화는 운동학과 현재 코드에 근거한 설계 제안이다. 외부 문서가 이 기기의 수치나 안전 성능을 보증한다는 의미가 아니다.

## 13. 추가 코드 검토: 보정값을 신뢰하기 위한 논리 개선

2026-09-07 같은 main revision을 다시 확인했다. 아래는 로컬 코드 추적 및 일부 함수 재현 결과이며 로봇에서 발생한 사건 보고가 아니다. 기존 반경·안전거리·회전 종료 계획에 다음 항목을 추가한다.

### 13.1 가장 먼저: 새 관측, 유효 관측, 안전한 관측을 구분

**수신 시각과 측정 시각:** `is_robot_scan()`은 beam 수/range_max/wall-clock 형식을 검사하지만 현재 시각 대비 age를 검사하지 않는다. safety `Bumper.on_scan()`은 처리 후 `last_scan_time=self.now()`를 저장한다. startup에는 별도 stamp 검사가 있지만 최종 gate 자체의 관측 age 보장은 다르다. 오래된 scan 재수신·queue backlog를 새 증거로 처리하지 않도록 source timestamp, receive timestamp, generation, frame/source identity를 분리한다. 중복·역순·미래 시각과 허용 clock skew를 정책으로 정의한다.

**반복 읽기와 새 표본:** `Judge._look_accum()`은 callback 갱신 여부와 무관하게 tick마다 샘플을 더한다. safety filter/IR median/US hits/통로 통계도 timer에서 같은 최근 값을 다시 처리할 수 있다. 그러므로 샘플 수는 독립 관측 수와 같지 않다. 측정 갱신 시에만 통계를 갱신하고, 시간 유지 조건은 별도 duration으로 표현한다. 정지 후 관측은 단순 receive 시각이 아니라 source 취득 시각이 정지 완료 이후인지 검사한다.

**정보 없음과 넓은 공간:** `Scale._scale_update()`에서 양측 통로 폭을 얻지 못하면 narrow=-1, `Senses.on_narrow()`는 이를 inf로 바꾸고 `narrow_factor()`는 열린 공간 계수 1로 해석한다. 실제 개방 공간과 한쪽 sensor invalid를 구별할 수 없다. `observed_open / observed_constrained / unknown / stale` 상태와 방향별 coverage를 유지한다. unknown 때문에 자동으로 속도 상한이 올라가지 않게 한다.

**위험 없음과 위험 센서 유실:** safety의 IR stale은 이전 cliff 상태를 유지하고, IMU stale은 tilt/pickup=false로 변환한다. 카메라 cliff/block은 하나의 `last_cam_time`을 공유한다. 한 스트림의 생존으로 다른 스트림이 새 관측인 것처럼 보일 수 있다. 스트림별 시각·validity를 보존하고 `hazard=false`와 `hazard=unknown`을 분리한다. 최종 gate의 동작별 필수 센서 조건에 반영한다.

예정 acceptance: 같은 packet 반복·일부 stream만 유지·측정은 오래됐지만 receive는 새로움·sensor 유실 상태에서 허용 동작이나 sample 신뢰도가 상승하지 않는다.

### 13.2 모든 주행 모드가 동일한 공간 기반 속도 정책 사용

`WanderNode.tick()`은 navigation_mode가 있으면 `_tick_navigation()`으로 분기한다. 이 경로는 `follow_path(...speed=self.vmax, turn=self.wturn)`를 호출하며 일반 `Motion._safe_speed()`의 narrow factor나 전후 gap cap을 거치지 않는다. `follow_path`는 경로와 각도 오차에 따라 속도를 정하고 hazard boolean은 받지만 측면 여유·정지 모델은 받지 않는다.

따라서 일반 wander만 수정하면 explore/coverage 경로 주행은 같은 개선을 얻지 못한다. wander·지도 경로 추종·control·teleop·calibration이 만드는 요청을 공통 pure envelope 정책과 최종 safety 제한으로 연결한다. calibration은 허가된 별도 시험 범위를 쓰되 geometry/위험 정지를 우회하지 않는다.

최종 gate에서 omega만 0으로 변경하면 원래 곡선 명령은 직선으로 바뀐다. 반대로 v만 0이면 제자리 회전이 된다. 변경한 `(v, omega)` 궤적의 안전성을 다시 확인한다. 명령 gain 이후 출력에도 같은 상한을 적용한다.

예정 acceptance: 같은 공간·같은 요청에 대해 모드가 달라도 같은 물리 제한 적용. 회전 불가 시 곡선 명령이 검증 없이 직진으로 바뀌지 않음.

### 13.3 회전 목표 도달 시 계속 회전하는 계약 정리

`ExitSteer.steer()`는 목표 deadband에 들어가 `desired==0`이어도 기존 `self.sign`을 반환한다. 상위 resume gate가 승인할 때까지 계속 회전하려는 현재 의도다. 그러나 각도 목표를 지나치고 다시 반대로 돌아가는 진동 원인이 될 수 있으므로 실제 제어 trace로 확인할 필요가 있다.

`turn_sign` 하나 대신 `tracking / aligned / observation_invalid / no_target` 결과를 반환하게 설계한다. aligned에서는 zero와 settle/reobserve로 전환하고, timeout은 성공이 아니다. 재탐색이 필요하면 명시적 각도·시간 budget을 가진 다음 동작으로 분리한다. 작은 매 tick 반전이 정렬 전략이 되지 않게 한다.

### 13.4 정체의 의미를 실제 목적 달성으로 구분

`ProgressGuard.check()`는 5mm 위치 변화 또는 0.05rad 각도 변화로 타이머를 갱신한다. 위치가 고정된 좌우 0.06rad 진동을 반복하면 진행 중으로 볼 수 있다. 일반 wander의 stuck 계산은 현재 요청 속도×경과시간을 쓰므로 가변 속도 이력·gate가 막은 구간도 구분하기 어렵다.

진행 측정은 목적별로 나눈다: 직진은 허가된 출력 적분 대비 이동량, 회전은 목표 heading error 감소, 지도 주행은 route arc-length/목표까지의 잔여거리 감소, recovery는 해당 장애물 영역 이탈을 확인한다. 단순 회전량은 경로 진전이 아니다. gate가 의도적으로 정지시킨 상태는 actuator stall과 분리하고 이를 gain 증가나 escape 민감도 학습의 자료로 넣지 않는다.

기존 `RecoveryBudget`의 제한은 유지한다. 새 route·일시 정지·상태 전환·좌우 진동만으로 budget을 초기화하지 않는다. map correction jump를 실제 이동으로 오인하지 않도록 odom/scan 증거와 함께 비교한다. 막힌 상태가 오래 지속되면 무한 대기가 아니라 제한된 재계획 또는 원인 표시 정지로 전환한다.

### 13.5 미세한 여유에서 최소 동작량을 강제하지 않음

`backup_limit_m()`은 후방 spare가 3mm여도 최소 10mm를 반환한다. 최종 gate가 추가로 제동할 수 있지만 제안된 후진 한도 자체가 공간을 초과한다. 현재 think-speed 하한과 같은 종류의 문제다.

허가된 거리/속도가 실측 deadband·정지 오차보다 작으면 동작 불가로 판정한다. 최소 후진량·최저 속도를 확보하려고 공간 한계를 늘리지 않는다. unknown 거리도 기본 2cm 후진 허가로 바꾸지 말고 해당 helper의 반환 의미를 명시한다.

### 13.6 파라미터 추정 모델과 정상 운용 검사 분리

startup `on_imu()`는 gyro<0.15rad/s 조건을 sample validity에 넣고, ready 후에도 `baseline.fresh()`로 그 validity를 감시한다. 정지 baseline 조건과 정상 회전 중 유효 센서 조건이 섞여 있다. 회전 속도를 확장하면 정상 움직임을 sensor invalid로 분류할 수 있다.

`sensor_validity`(단위·범위·시각·규격), `stationary_quality`, `motion_consistency`, `profile_domain`을 나눈다. 센서의 유효 범위를 넓힌다는 이유로 정지 검사를 느슨하게 하지 않고, 정지 조건을 운용 중 모든 샘플에 적용하지 않는다.

`corrected_drive_speed`는 |v|<=0.014에서만 gain을 적용한다. gain=1.25이면 요청 0.014의 출력은 0.0175, 요청 0.0141의 출력은 0.0141이 되어 속도 요청과 출력이 경계에서 역전된다. 검증 범위를 표현하는 연속적·단조적인 command-to-response 매핑 또는 명시적인 범위 제한을 검토한다. 모델 간 연결 구간도 실측 검증하며 검증 밖 extrapolation으로 해결하지 않는다.

### 13.7 마지막 명령 경계의 최소 유효성 계약

`Gate.on_cmd()`는 메시지를 보관하고 age를 갱신하며 현재 구현에는 finite 검사·발행자 소유권·속도 범위 검사 자체가 없다. 상위 UI가 제한하더라도 모든 ROS raw 요청이 그 UI를 거친다는 보장은 없다. NaN/inf, 범위 밖 값, 다른 owner의 명령은 최종 gate에서 zero/거부 사유로 처리한다. 허용된 profile revision과 lease, 최종 v/omega의 물리 범위 검증을 한 곳에 둔다.

### 13.8 재현 결과와 우선순위

다음은 기존 순수 함수 또는 `_look_accum`의 원문 AST를 메모리에 읽어 실행한 결과다. 새 구현/테스트 파일을 쓰거나 로봇에 명령하지 않았다.

| 입력 | 현재 결과 | 해석 범위 |
|---|---|---|
| 동일 cached F/L/R로 `_look_accum` 8회 | front sample count=8 | 통계상 새 측정 8개라는 증거는 없음 |
| `backup_limit_m(0.123, 0.12, 0.12)` | 0.010m | 남은 0.003m보다 큰 helper 결과. 실제 이동 보장은 아님 |
| ExitSteer target=0.5rad, 현재 yaw=0.5rad | sign=+1 | 상위 재출발 판정까지 회전 유지하는 현 계약 |
| 위치 0 고정, yaw 0/0.06rad 교대, 20초 | ProgressGuard stalled=false | 각도 진동이 progress로 인정됨 |
| 2023년 wall-clock stamp, 720 beams, rmax=40 | is_robot_scan=true | source-shape 필터가 freshness 검사는 아님. startup의 별도 검사까지 통과했다는 뜻 아님 |
| `narrow_factor(NaN, 0.1)` | 1.0 | unknown이 개방 공간 속도 계수로 해석됨 |
| 직선 map route, 정상 pose/age | 요청 (0.014, 0, forward) | follower API에 측면/정지 모델 입력이 없음. final safety 우회는 아님 |

관련 기존 회귀 `test_path_follow`, `test_recovery_budget`, `test_recover`, `test_calibration`, `test_round_trip`는 **59 passed**였다. 이 결과는 기존 계약의 통과이며 위 개선 요구 충족 증거가 아니다. `front_block/guard_speed/escape_open`의 permissive invalid 정책은 현재 실제 wander 호출이 아니라 gz driver/테스트 사용으로 확인했으므로 실기 gate 결함으로 분류하지 않았다.

우선순위는 **P0: 관측 validity/시각·명령 gate·profile 계약 → P1: 모든 모드 공통 envelope·정지/재관측·유효 progress → P2: 증거 기반 추정 모델과 연속적 속도 매핑**으로 조정한다. 회전 보정 알고리즘은 이 기반 위에서 검증한다.

## 14. 구조 개선안: 동적 안전영역의 소유권과 입출력

2026-09-08 로컬 main `6f6e461` 재확인 기준. 새 ROS 안전 노드를 추가하는 대신 기존 safety 노드 안의 ROS 없는 정책 모듈로 시작한다. 최종 제어 권한은 한 곳에 유지하고, 프로세스 간 의존이나 새로운 명령 발행자를 늘리지 않는다.

| 책임 | 현재 연결에서의 한계 | 목표 계약 |
|---|---|---|
| 센서 관측 | 여러 Float32/Bool에 시각·유효성·측정 묶음 정보가 소실됨 | source stamp, generation, frame, coverage, unknown과 hazard를 보존하는 관측 snapshot |
| 기체 모델 | 반경과 라이다 offset·여유가 여러 함수에 분산 | 검증 geometry revision으로부터 footprint/support footprint 파생 |
| 자동 보정 | 주로 배율을 발행하며 유효 조건을 묶어 전달하지 않음 | 조건별 응답·정지·불확실성 모델을 포함한 검증 profile |
| 주행 제안 | wander와 map follower가 서로 다른 제한 사용 | 각 모드는 목표/원하는 v·omega를 제안하고 제한 사유를 받아 재계획 |
| 최종 안전 판단 | 전후 정지와 회전 허가를 분리한 scalar 판정 중심 | 요청과 현재 실제 운동 상태에서 정지까지의 영역을 검사한 최종 명령 |
| UI/진단 | 별도 설정값과 bool로 해석 | 최종 gate가 사용한 profile·영역·속도 제한·원인을 표시 |

### 14.1 SafetyDecision 입력과 출력 초안

입력은 `observation_snapshot`, `measured_motion`, `requested_motion`, `active_profile`, `command_owner`다. 관측을 무조건 같은 stamp로 강제하지는 않지만 센서별 age와 허용 skew를 검사하고, 비교 시각까지 이동 보정하거나 불확실성을 확대한다. 최신 값만 묶었다고 동시 관측이라고 부르지 않는다.

출력은 `safe_v`, `safe_omega`, `action`(allow/limit/stop), `reason`, `limiting_direction`, `evaluated_sweep`, `profile_revision`, `observation_generation`, `valid_until`로 구성한다. scalar 속도 상한은 설명용이며, 안전하다고 평가한 v와 omega의 조합을 독립적으로 다시 섞지 않는다. 메시지/클래스의 구체적 형식은 구현 단계에서 기존 계약과 맞춘다.

현재 기기는 어디로 움직이고 있는지까지 입력해야 한다. 후진 요청으로 바뀌어도 아직 전진 중이면 전진 관성부터 검사한다. 요청이 zero여도 실제 정지가 확인될 때까지 기존 운동의 잔여 sweep을 유지한다. 정지 명령 발행과 정지 완료는 별도 상태다.

### 14.2 공간을 세 층으로 관리

- `body/support geometry`: 조립 상태에 따른 고정 기준.
- `stopping sweep`: 현재 속도·예상 출력·실측 제동 모델에 따른 정지까지의 차체 영역. obstacle와 support 검사는 구분.
- `observation uncertainty margin`: 데이터 age·장착/거리 오차·미끄러짐 등에 따른 여유. unknown을 작은 margin만 더한 free 공간으로 바꾸지 않음.

주변 공간이 좁으면 허용 속도/경로를 줄인다. 충분히 감속한 뒤 실제 운동 상태가 바뀌어야 dynamic sweep을 줄일 수 있다. 목표 속도만 낮아졌다고 그 주기에 즉시 안전영역을 축소하지 않는다. 고정 geometry와 최소 여유는 유지한다.

### 14.3 실시간 계산과 실패 동작

현재 safety timer는 0.05초다. 이것을 실제 반응시간 보장으로 간주하지 않고 ARM64에서 관측→판단→최종 출력의 worst observed latency와 여유를 계측한다. 무거운 scan fitting/보정 계산은 최종 gate의 경량 tick을 막지 않게 분리한다. 먼저 같은 프로세스의 bounded pure 계산으로 구현하고 측정 후 별도 실행 구조 필요성을 판단한다.

bounded 후보 집합에 대해 geometry/관측/제동 조건을 검사한다. 계산 budget 초과·snapshot 만료·profile 불일치일 때 이전 allow 결과를 계속 재사용하지 않는다. 검증된 정지 정책을 적용하고 원인을 표시한다. 입력이 같은 동안 UI와 제어가 다른 threshold를 계산하지 않게 SafetyDecision을 증거로 공유한다.

### 14.4 단계적인 도입

1. 관측 snapshot과 effective profile 공개부터 추가하여 기존 판단을 설명 가능하게 만든다.
2. 기존 gate를 유지한 채 새 정책을 shadow로 계산한다. 판단 차이와 근거를 저장하고 검증되지 않은 판단은 출력에 적용하지 않는다.
3. 독립 재현·보호된 실기 시험을 통과한 범위에서 새 정책을 활성화한다. 초기에는 기존 검증 제약보다 느슨한 허가가 되지 않도록 한다. invalid legacy 값을 정상 근거로 승계하지 않는다.
4. 공통 정책 결과를 wander/map follower가 받아 불가능한 동작을 반복 요청하지 않도록 연결한다. 정책은 최종 gate에서도 재검사한다.
5. 일관성이 입증된 뒤 중복 threshold 계산을 compatibility adapter로 축소한다. 새로운 최대 속도나 더 좁은 통과 허가는 별도 검증 후 확대한다.

추가 acceptance: 전진 관성 중 후진 요청, zero 명령 후 잔여 회전, v/omega 수정으로 궤적 변경, 서로 다른 시각의 센서 조합, 계산 지연, shadow/active 전환, 최종 출력과 UI의 revision 일치.
