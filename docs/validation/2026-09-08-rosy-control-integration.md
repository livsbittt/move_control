# Rosy Control 워크트리 통합 기록

## 범위

- Rosy OS와 Rosy Fleet은 독립 개발 공간으로 유지한다.
- 이 프로젝트의 이름은 Rosy Control이며 ROS/Python 패키지명 `move_control`은 유지한다.
- 로봇의 `/home/pinky/dev_ws/wj/src/move_control` 배포 경로와 ROS 명령은 변경하지 않았다.
- 원격 push, 로봇 배포, 모터 구동은 이번 작업 범위가 아니다.

## 통합

- `0af4c01`: `codex/finish-world-mapping`의 미병합 이력을 통합하고 기존 워크트리의 문서·검증 자료를 수집했다.
- `d649746`: `369ac73` TF 교정 복구 커밋을 통합했다. 두 병합 커밋 모두 main에 반영했다.
- 목표 노드 import와 실험 결과 초기화 목록의 충돌은 양쪽 기능을 유지하도록 해결했다.
- 동적 장애물 대기가 새 직선 탈출 분기에서 누락되는 상호작용을 발견했다. 장애물 감지 시 탈출을 영속도 중단하고 자동 재실행하지 않도록 수정했다.
- 별도 코드 검토에서 주요 미해결 병합 오류가 없음을 확인했다.

## 검증

- main 기준선, Ubuntu ROS Jazzy: `python3 -m pytest test/ -q` → 535 passed.
- 매핑 통합, Windows: `python -m pytest test/ -q` → 591 passed, 1 skipped, 13 subtests passed.
- 대시보드: `node --test tools/test_dashboard_calibration.cjs tools/test_dashboard_mapping.cjs tools/test_dashboard_rotation.cjs` → 26 passed.
- 최종 TF 포함, Ubuntu ROS Jazzy, 격리 domain 231/LOCALHOST:
  `python3 -m pytest test/ tools/test_goal_route_ros.py tools/test_recovery_ros.py tools/test_safety_gate_ros.py tools/test_web_teleop_ros.py tools/test_startup_calibration_ros.py -q` → 677 passed.
- `colcon build --packages-select move_control` → 1 package finished. 교정 후속 변경은 최종 ROS 시험에서 import·실행 검증했다.
- `git diff --check` 확인.
- 첫 Linux 임시 복사본에서는 docs의 지도 fixture 누락으로 2개 시험이 실패했다. 파일 보완 후 관련 14개와 최종 677개 시험이 통과했다. 제품 코드 실패로 분류하지 않는다.
- Linux 마운트 IO 지연과 임시 폴더 소실 후 `/root/rosy-control-integration-20260908` 검증 복사본을 사용했다. WSL 종료 명령은 실행하지 않았다.

## 보존과 복구

ROS 상위의 `archive/rosy-control-transition-20260908/`에 출처 목록과 SHA-256을 기록했다.

- `worktrees-initial.json`, `worktree-changes/`: 초기 변경·미추적 파일 49개의 복사본.
- `ignored-evidence-manifest.json`, `ignored-evidence/`: Git 무시 규칙 아래 있던 실험 자료 151개.
- `root-files/`: 루트의 압축파일·실험 스크립트를 원래 이름으로 보관.
- `backups/`: 비등록 백업 폴더 두 개와 파일 해시.
- `removed-worktrees.json`: 실제 제거된 워크트리의 경로와 HEAD.
- main에 이미 수집한 기존 수정 자료의 원형은 `Rosy Control transition: preserved source evidence`라는 Git stash에도 남겼다. 이는 병합 누락 코드가 아닌 복구용 원본이다.
- 작업 브랜치는 삭제하지 않는다. 워크트리 제거 전에 main 포함 여부, 새 변경, 링크, 무시 자료의 보존 해시를 확인한다.

## 최종 폴더 상태

- 실제 Git 저장소와 소스는 `ROS/Rosy Control`로 이동했다. Git worktree는 이 main 한 개만 등록되어 있다.
- 보조 워크트리 16개와 이번 통합용 워크트리 1개를 제거했다. `git branch --no-merged main` 결과는 비어 있다.
- 루트 파일 46개와 비등록 백업 폴더 2개를 archive로 보관했다. 이전 작업 원본은 삭제하지 않았다.
- 새 경로에서 교정·장애물 탈출 시험 7개가 통과했다. `git fsck --connectivity-only`는 오류 없이 종료했으며, 과거 작업의 dangling 객체는 보존했다.
- Windows의 사용 중 폴더 잠금 때문에 부모 디렉터리의 직접 rename은 실패했다. 내부 항목 이동으로 실제 저장소 이전을 완료했다.
- 기존 `ROS/move_control`에는 데이터가 없는 빈 `.git` 디렉터리만 남아 있다. 빈 이전 폴더 제거 명령은 자동 승인 검토에서 구체적인 사유 없이 차단되어 재시도하지 않았다. 실제 Git 데이터는 새 경로에 있으며 연결성 검증을 마쳤다.
- 이후 작업은 `ROS/Rosy Control`을 열어 진행한다.

이 기록은 로컬 통합·검증 증거다. 이전 시뮬레이션 자료를 이번 결합 코드의 실물 주행 성공 증거로 재해석하지 않는다.
