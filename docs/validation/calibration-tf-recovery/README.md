# 보정 중 TF 지연 복구

작업 기준: main 8e9cb78. 수정 범위는 이동 보정 중 일시적인 map TF 지연 처리와 진단이다.

## 원인과 수정

기존 read_tf는 조회 예외를 모두 valid=False로 바꾸어 상세 원인을 버렸다. 보정은 유효하지 않은 TF 샘플 하나에도 전체 실패했다. 기존 실행은 첫 왕복 복귀 중 20.761초에 map_tf 오류로 중단됐다. 진단 로그를 추가한 재실행에서는 초기 TF 연결 전 ExtrapolationException과, 이후 정상 quaternion(norm=1)인 변환의 source age가 약 1.7초인 경우를 확인했다. 원래 실행의 상세 예외는 기록되지 않았으므로 동일한 원인이라고 단정하지 않는다. 진단 실행 자체는 안전 게이트가 명령을 수정하여 종료됐다.

- TF 원본 시각, 지연시간, quaternion norm, 예외 종류와 상세를 보고한다.
- 지연/조회 불가 상태에서 이동 보정은 즉시 0 명령을 발행하고 최대 1초만 재수신을 기다린다.
- 기존 원점, 벽 추적, 원시 거리, 후방 여유, 안전 신호, 보정 전체 35초 제한을 유지한다. 이전 TF를 새 유효 데이터로 재사용하지 않는다.
- 복구한 지도 이동량이 odometry와 일치해야만 재개한다. 위치 차이 1.5cm 또는 각도 차이 0.1rad 초과는 실패한다.
- 잘못된 좌표/quaternion, 미래 시각, 다른 센서 이상, 실제 위험, 대기시간 초과는 실패한다.
- 새로운 기본 파라미터를 추가하거나 저장된 보정값을 덮어쓰지 않았다.

## 검증

- 수정 전 ROS 재현: 정상 입력 중 map_tf 지연 시 즉시 실패. 새 회귀 테스트 2개 실패를 확인했다.
- 수정 후 순수 로직 전체: 538 passed, 1 skipped.
- 수정 후 ROS 보정 테스트 전체: 32 passed. 실제 TF Buffer의 오래된 source timestamp, 새 변환 복구, 조회 예외 보고, 정지 후 재개, 지도 위치 튐, 대기시간 초과, 원시 거리/낭떠러지 위험 동시 발생을 검사했다.
- 코드 리뷰: 차단 사유 없음. 리뷰에서 요청한 동시 위험 검사를 추가했다.
- Gazebo: 원본 map_260905.world 16개 벽, 바퀴 물리, 실제 렌더링 카메라. 두 차례 왕복 및 8단계 양방향 회전 보정 후 ready, route_explore 시작을 관측했다.
- Gazebo 재검증의 목표 실시간 배율은 0.5(기존 실패 실행 1.0)다. 처리 지연을 줄이기 위한 실행 조건 변경이며, 시뮬레이션 성공만으로 코드 수정의 효과를 독립적으로 증명하지 않는다. 같은 입력을 사용하는 ROS 회귀 테스트에서 복구 동작을 검증했다.
- precision_pause_total_s는 LiDAR와 TF 대기의 합계이므로 이 값을 모두 TF 지연으로 해석하지 않는다.

original/, diagnostic/, fixed/에 실행별 소스 해시와 결과를 분리했다. fixed/tf-status-events.json은 관찰 도중 추가한 기록이며 전체 실행 기록이 아니다. fixed/track_samples.json이 전체 모니터 구간을 나타낸다. 전체 맵핑 완료와 물리 기기 적용은 이 수정의 완료 증거가 아니다.

TF API 참고: https://docs.ros.org/en/jazzy/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Listener-Py.html
## 최종 실행 결과

- 실행 ID: ebdd7c7c894c440d91adb7e4a96c039d
- 모니터 관측 95.196 simulation seconds. 목표 100초, wall timeout 600초로 제한한 실행이다.
- 최종 ready=true, rotation_verified=true, settings_applied=true, 왕복 이동 다리 4개.
- 최종 map TF valid=true, source age 0.057초. 실행 소스 해시와 수정된 두 production 파일의 현재 해시 일치를 확인했다.
- 전체 경로 0.248915m, 최종 속도 발행자는 safety_node 한 개.
- mapping_complete=false. 실행 스크립트는 전체 맵 완료 조건에서 exit 1이며, TF 또는 보정 실패가 아니다. 전체 맵핑 검증은 여전히 HOLD다.