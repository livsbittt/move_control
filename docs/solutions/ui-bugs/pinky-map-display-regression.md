---
title: "Pinky 지도 위치 표시 오류와 동시 배포에 의한 재발"
date: "2026-09-07"
module: "move_control web map"
problem_type: ui_bug
component: frontend
severity: high
symptoms:
  - "로봇이 지도 벽 위 또는 미관측 영역에 표시됨"
  - "정상 화면을 확인한 뒤 다른 실행본으로 교체되면 오류가 재발함"
root_cause: missing_validation
resolution_type: code_fix
tags: [pinky, map, tf, deployment, raster]
---

# Pinky 지도 위치 표시 오류와 동시 배포에 의한 재발

## 최종 상태: 코드 수정 완료, 실행본 유지 HOLD

수정과 아래 성공 검증 기록은 로컬 main의 `055ed2b`에 커밋했다.
그러나 최종 확인에서 실행 경로가 `pinky-calibration-1c40085`, PID 57886으로
다시 바뀌었고, 제공 중인 HTML에 각도 정렬 함수와 연속 좌표 보정이 없었다.
따라서 아래 성공 검증은 `0db34f6` 실행본에 대한 시점 기록이며,
현재 또는 다음 실행본까지 수정됐다는 증거로 사용할 수 없다.

[최종 재덮임 증거](../../evidence/2026-09-07-map-display/final-drift.json)에
실행 명령과 HTML 해시를 저장했다. 진행 중인 배포가 수정 커밋을 포함한 뒤
같은 읽기 전용 검증을 다시 통과해야 실행본 상태를 GO로 바꿀 수 있다.

## 문제와 원인

ROS 지도에서 로봇 중심은 빈 공간인데 웹에서는 벽 위나 미관측 영역에 표시됐다.
원인은 두 가지 표시 좌표 오류와 수정이 빠진 실행본의 재배포였다.

1. ROS OccupancyGrid의 첫 행은 남쪽이다. PNG 첫 행은 화면 위쪽이다.
   웹 이미지에 행 반전을 적용하지 않으면, 위아래를 바꿔 계산한 로봇 위치와
   지도 이미지가 서로 맞지 않는다.
2. 연속 세계 좌표를 이미지 좌표로 바꿀 때 `height - 1 - y/res`를 사용했다.
   이는 이산 행 인덱스 계산이며, 연속 좌표에는 한 셀 오차를 만든다.
   현재 지도 해상도 0.02m에서는 로봇과 목표 표시가 2cm 위로 어긋난다.
3. 다른 작업의 배포가 진행 중이었다는 사용자 확인을 받았다. 실행 경로는
   `pinky-calibration-b92f9f8`, `pinky-calibration-95774ac`,
   `pinky-calibration-0db34f6`로 바뀌었고, 새 실행본의 렌더러에 행 반전이 없었다.
   한 실행 폴더를 수정하고 캡처한 것만으로 다음 배포의 수정을 보장할 수 없었다.
   이 이름들은 관측한 실행 디렉터리 식별자다. 접미사에 해당하는 커밋이 현재
   main에 포함됐다는 의미로 인용한 것이 아니다.

## 수정

- `move_control/sensing/map_raster.py`의 `occupancy_bgr()`가 ROS 없이 이미지
  행 방향을 처리한다. `move_control/web_node.py`는 이 함수를 호출한다.
- `test/test_map_raster.py`를 일반 테스트 경로에 추가했다. 이전 재현 테스트는
  `tools/test_web_map_raster.py`에만 있어 `pytest test/`에 포함되지 않았다.
- `web/dashboard.html`의 연속 좌표 변환과 역변환을 함께 수정했다.
  셀 중심 `(column + 0.5, row + 0.5)`가 이미지 픽셀 중심에 놓이는 테스트를 추가했다.
- 지도 각도 정렬은 화면 변환으로 유지한다. 측정된 TF와 지도 자체를 임의의
  30도로 회전시켜 표시 문제를 감추지 않는다.
- `tools/audit_web_map_ros.py`를 추가했다. 주행 명령을 보내지 않고 ROS 지도,
  TF, 라이다, 웹 PNG, 웹 위치를 비교하고 실행 소스 경로·SHA-256을 저장한다.

핵심 변환은 다음과 같다.

```python
# ROS south-to-north rows -> PNG top-to-bottom rows
image = np.ascontiguousarray(image[::-1])
```

```javascript
// Continuous world position -> image position, before view rotation
u = (x - originX) / resolution;
v = height - (y - originY) / resolution;
```

## 실제 Pinky 검증

수정 대상은 현재 프로세스가 실제로 불러온
`/home/pinky/pinky-calibration-0db34f6/src/move_control`이었다.
원래 안내된 `/home/pinky/dev_ws/wj/src/move_control`은 다른 오래된 체크아웃이었다.
원본 폴더 이름만 보고 수정 대상을 선택하지 않았다.

| 항목 | 수정 전 | 수정 후 |
|---|---:|---:|
| 웹 PNG와 ROS 지도 행 방향 일치 | 실패 | 통과 |
| ROS 로봇 셀 | (11, 27), 빈 공간 | (11, 27), 빈 공간 |
| 웹 위치와 TF 위치 차이 | 약 0.39mm | 약 0.39mm |
| 라이다 점과 지도 벽 거리 중앙값 | 약 1.15cm | 약 1.15cm |
| 라이다 점의 벽 4cm 이내 비율 | 약 99.85% | 약 99.85% |
| 비상정지 / 속도 | 유지 / 0 | 유지 / 0 |

- Python 테스트 189개, 지도 JavaScript 테스트 10개 통과.
- 실제 주소를 Playwright로 열어 로봇이 통로 안에 표시되는 것을 확인했다.
  브라우저 오류는 없었고 표시 정렬 각도는 약 -22.9도였다.
- 재시작 관리자가 이미 실행 중이어서 임시 웹 프로세스를 종료하고,
  기존 관리자가 띄운 PID 53760으로 최종 검증했다. 독립 웹 프로세스를
  추가로 유지하면 포트 충돌과 재시작 반복이 발생하므로 피한다.

기록:

- [수정 전 화면](../../evidence/2026-09-07-map-display/before.png)
- [수정 후 화면](../../evidence/2026-09-07-map-display/after.png)
- [수정 전 수치와 실행 파일 해시](../../evidence/2026-09-07-map-display/before.json)
- [수정 후 수치와 실행 파일 해시](../../evidence/2026-09-07-map-display/after.json)
- [약 2분 뒤 동일 실행본 재검증](../../evidence/2026-09-07-map-display/repeat.json)
- [실행 파일 변경 기록](../../evidence/2026-09-07-map-display/runtime-patch.json)

실행 파일 변경 기록의 `pid_after`는 잠시 실행한 임시 PID다. 최종 관리 프로세스
PID와 반복 검증은 별도 기록으로 남긴다. Pinky 원본 백업과 로그는
`/home/pinky/map-display-records/20260907T134327Z`에 있다.
재검증에서도 표시 검사는 통과했고, 라이다 벽 거리 중앙값은 약 1.06cm,
4cm 이내 비율은 약 99.40%였다. 수치는 채집 시점마다 달라지는 표본값이다.

## 재발 방지와 판정 범위

다음 배포에는 이 문서와 테스트를 포함한 수정 커밋을 반영한다. 수정 커밋이 없는
과거 실행본을 배포하면 다시 발생할 수 있다. 동시 작업자의 배포는 이 작업에서
중단하지 않았으므로, 미래의 모든 실행본까지 수정됐다고 판단하지 않는다.

배포 뒤 **실제로 실행한 설치 환경**을 불러온 터미널에서 아래 읽기 전용 검사를
실행한다. 출력 폴더는 매번 새 경로여야 한다.

```bash
python3 tools/audit_web_map_ros.py --out /tmp/pinky-map-check-UNIQUE
```

종료 코드 0과 `display_pass: true`, 실행 경로와 해시를 함께 확인한다.
지도 세대가 바뀌어 표본이 일치하지 않으면 성공으로 간주하지 말고 다시 채집한다.
`display_pass`는 PNG 방향·웹/TF 좌표·빈 셀의 표시 검사다. 주행 중 오차,
들어 옮긴 뒤 재위치 추정, 물리적 거리 실측을 통과했다는 뜻은 아니다.

교훈: **지도 표시가 이상할 때 TF를 먼저 바꾸지 말고, 같은 시점의 원본 지도 셀,
PNG 픽셀, 화면 좌표와 실제 실행 소스를 대조한다. 실행본 교체 후에도 다시 검증한다.**
