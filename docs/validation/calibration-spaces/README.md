# 공간 확보 보정 검증 기록 — 2026-09-08

이 폴더는 실제 Gazebo 바퀴 동역학 실행 기록이다. 결과 그림은 `track_samples.json`의 ground-truth 궤적과 `track_map.npz`의 실제 SLAM 래스터를 각각 그린 것이다. 그림의 입력 해시는 `.inputs.json`에 기록한다.

## 확인한 동작

최종 v4 재배치/복귀 행렬은 **3/3 통과**했다. [실행 행렬](return-v4-matrix.json), [전체 실행 화면](return-v4-all.png), [통로 실행 화면](corridor-return-v4/recorded-run.png).

| 시작 공간 | 공간 확보 이동 | 총 sim 시간 | 결과 |
|---|---:|---:|---|
| 앞벽 근처 | 후진 2.595cm | 73.505초 | 보정·원점 복귀·정지 |
| 뒤벽 근처 | 전진 2.763cm | 68.045초 | 보정·원점 복귀·정지 |
| 좁은 통로 출구 | 전진 14.959cm | 109.010초 | 보정·원점 복귀·정지 |

- `front-return-v4`: 짧은 방향 확인 이동 후 공간을 확보하여 전체 보정을 수행하고 복귀했다. 실행 ID `83c503edddad4898a61f95dc74ee1831`.

- `rear-return-v4`: 직선 보정 후 앞쪽 2.763cm 공간 이동, 회전 보정, 원점 복귀 완료. 68.045초(sim), 원점 오차 약 4.3mm, 최종 속도 0. 실행 ID `dbb8bc924c944bd6b041f1e1becf0220`. 실행 소스 해시와 현재 구현 파일의 일치를 확인했다.
- `corridor-return-v4`: 통로 출구 쪽으로 14.959cm 이동한 뒤 회전 보정, 약 14.46cm 복귀 완료. 109.01초(sim), 최종 속도 0. 실행 ID `1bee4a6573d34f3d8b54f7f0df975b25`.

- `front-return-v3`: 앞벽 16cm 위치에서 2.10cm 후진하여 새 기준값 수집, 직선/양방향 회전 보정, 원점 복귀를 완료했다. 69.62초(sim), 원점 오차 4.4mm, 최종 속도 0. 단독 `/cmd_vel` 발행자는 safety였다. 실행 ID `cf57c0bc5e714b15ab12b01161da27a3`.
- `isolated-stay-v3/open`: 개방 공간 보정 완료, 56.545초(sim), 최종 속도 0. 실행 ID `185eaf9cefe94de4bbff9136b681679e`.
- `load-run-v3/trapped`: 양방향 짧은 이동의 차체 여유가 부족하여 `waiting_space`로 정지했다. 실제 이동은 수치 오차 수준이며 보정 완료를 발행하지 않았다. 실행 ID `111af7845bbb4106bc946cd000ddc382`.

v3 결과는 후속 v4 공간 선택 여유 강화 전 기록이다. v4는 회전 시험의 허용 중심 이동 8mm까지 미리 확보한다. v4 구현 커밋은 `df5609b`이며 시험 도구 파일 일치는 `v4-source-verification.json`에 기록했다.

## 실패와 관측 한계

- `front-return-v1`: 회전 공간만 확보한 뒤 직선 보정 공간이 부족했다. 시험 중단 후 정지 확인. 공간 선택에 직선 보정 조건을 추가했다.
- `front-return-v2`: 선택한 직선 거리의 여유가 약 0.1mm 부족해 즉시 안전 정지했다. 목표 거리 선택에 3mm 여유를 추가했다. 이 실행의 최종 속도 표본은 정지 명령 전달 전 값이라 정지 완료 증거로 사용하지 않는다. 후속 monitor는 정지 확인을 기다린다.
- `load-run-v3`: 웹/브라우저 동시 실행 중 open/rear/corridor가 센서 무효 또는 지연으로 중단됐다. 부하가 원인이라는 단정은 하지 않는다.
- `isolated-stay-v3/rear_wall`: 최초 보정 상태 뒤 heartbeat가 진행하지 않아 운영자가 해당 시험 프로세스 그룹만 중단했다(exit 130). 결과 파일이 없으므로 미완료다. Python 스택은 executor 대기였으며 교착이나 원인은 확정하지 않았다.
- `isolated-stay-v3/corridor_exit`: 13.1cm 앞으로 이동했지만 첫 회전 중 약 1mm 중심 이동으로 공간 조건을 잃어 정지했다. v4의 중심 이동 여유 강화 근거다.
- 시험 중 WSL 서비스 `0x8007274c` 연결 오류도 발생했다. 재부팅 없이 이후 연결이 복구됐다. 다른 WSL 작업을 종료하지 않았다.
- `live-browser`는 실제 웹 실행 중 UI 캡처이며 JavaScript 오류는 없었다. 여러 월드 전환 중 같은 웹 프로세스가 유지되어 이전 지도 캐시가 섞일 수 있으므로 월드 일치/보정 성공 증거로 사용하지 않는다.
- `baseline-open`은 더 이른 구현의 이력이다. 최종 코드 검증을 대신하지 않는다.

## 코드 검사

- `python -m pytest test/ -q`: **494 passed, 1 skipped**.
- ROS adapter/통합 보정/safety/web HTTP/startup 묶음: **67 passed**.
- dashboard Node 테스트: **25 passed**(11 calibration, 8 actions, 6 map view).
- 독립 코드 리뷰에서 보고된 sensor source freshness, 전체 스캔, 보정 단계 자격 및 원위치 복귀 문제를 수정하고 재검사했다.

## 재현과 범위

ROS 2 Jazzy/Gazebo 환경에서 다음을 실행한다. 테스트는 ROS domain 227 및 `pinky_calmap227` partition 전용이며 기본 realtime factor는 0.2다.

```bash
python3 tools/gz/run_calibration_spaces.py --after stay --output /tmp/calibration-stay
python3 tools/gz/run_calibration_spaces.py front_wall rear_wall corridor_exit --after return_origin --output /tmp/calibration-return
python3 tools/gz/plot_calibration_spaces.py /tmp/calibration-return/front_wall --out /tmp/front-return.png
```

완전히 관측된 전후 직선 경로, 20cm 이동 예산 내의 공간 탐색만 지원한다. 모든 형상의 공간에서 탈출을 보장하지 않는다. generated calibration world의 전체 지도 완성은 이 시험의 판정 대상이 아니며, 원본 미로의 전체 지도 완성도 별도 미완료다.

카메라/IR은 합성, IMU는 ground-truth 파생, US는 라이다 파생이다. **실물 센서 융합과 실물 주행은 미검증**이며 공간 이동 기능은 실물 기본 비활성이다.
