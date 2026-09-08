# Rosy Fleet 폴더 전환 초안

> 후속 결정: 아래 통합·legacy 배치 제안은 채택하지 않는다. `Rosy`는 OS,
> `Rosy Fleet`은 독립 관제 개발 공간으로 유지한다. 현재 저장소는 보조
> 워크트리 통합 후 `Rosy Control`로 폴더명을 변경하며 ROS 패키지명은
> `move_control`을 유지한다. 아래 내용은 당시 조사 기록이다.

작성일: 2026-09-08. 상태: 현황 조사 및 구조 제안. 실제 이동·삭제·런타임 변경은 미실행.

## 방향

제품 이름은 Rosy Fleet. 기존 ROS/Rosy 저장소를 플랫폼 기반으로 사용한다.
현재 Rosy/src/rosy_fleet 패키지가 실제로 존재하므로 별도 관제 구현을 중복 생성하지 않는다.
README와 AGENTS의 구현 예정 설명은 현재 파일 상태와 차이가 있어 구현 범위는 후속 코드 검토로 확정한다.
Rosy 저장소의 이름 변경은 경로 의존성을 조사한 뒤 별도 단계에서 수행한다.

## 목표 구조

```text
ROS/
  Rosy/                          # 기존 플랫폼 저장소; 제품명 Rosy Fleet
    src/rosy_fleet/               # 기존 중앙 관제 패키지
    src/rosy_core/                # 로봇 외부 API와 명령 관리
    src/rosy_navigation/          # 기존 내비게이션
    src/rosy_*/                   # 하드웨어·센서 등 기존 패키지
    maps/                        # 향후 검증된 운영 지도와 버전 명세
    docs/                        # 기존 요구사항·API·설계
    deploy/                      # 기존 배포 체계
  legacy/
    move_control/                # 이관 기간 독립 Git 저장소와 기존 패키지 보존
  .worktrees/
    move_control/<task>/          # 보존할 작업만 Git으로 이동
  archive/
    move-control/2026-09-08/      # 백업·검증 증거·임시 전송 파일과 출처 목록
```

maps, legacy, .worktrees, archive는 제안 경로이며 아직 생성하지 않았다.
대안은 Rosy를 RosyFleet로 즉시 개명하거나 두 저장소를 즉시 합치는 것이다.
둘 다 기존 실행 경로와 독립 모터 명령 소유권을 함께 변경하므로 단계적 전환을 우선한다.

## 확인한 정리 대상

- 본체를 포함해 등록된 move_control 워크트리 16개.
- 본체: 수정된 docs/validation/calibration-spaces/return-v4-all.png 1개.
- dynamic_obstacles: main에 미병합, 수정·미추적 항목 7개.
- mapping_finish: main에 미병합, 작업 트리 깨끗함.
- navigation_fix: HEAD는 main에 포함되지만 미추적 검증 디렉터리 5개.
- release: main에 포함되지만 변경 항목 1개.
- rotation: main에 포함되지만 변경 항목 2개.
- 나머지 10개 보조 워크트리: HEAD가 main에 포함되고 status가 깨끗함. 프로세스·잠금·무시 파일 검사는 아직 하지 않았으므로 삭제 가능 판정은 아님.
- move_control-pre-main-7e3d416: docs를 포함한 비등록 폴더.
- pinky-premerge-untracked-20260907: move_control/test/tools를 포함한 비등록 폴더.
- ROS 루트의 실험용 sh/py와 tar: 내용·해시·사용 경로 검사 후 보관/도구 승격을 결정.

## 실행 순서와 검증

1. 모든 대상의 절대 경로, Git 공통 디렉터리, HEAD, 상태, ignored 파일, 링크 대상과 사용 프로세스를 manifest로 기록한다. 실제 이동 직전에 재검사한다.
2. 미병합 커밋과 미추적 파일은 별도 보존한다. main 포함 여부만으로 데이터 중복이라고 판단하지 않는다.
3. 백업·전송 자료는 archive로 이동하고 원본/목적지와 SHA-256을 기록한다. 스크립트 경로 의존성을 먼저 조사한다.
4. 남길 보조 워크트리는 git worktree move로 .worktrees 아래에 배치한다. 미사용·병합·자료 보존이 모두 확인된 워크트리만 git worktree remove로 제거한다. 강제 삭제하지 않는다.
5. 현재 본체는 열린 작업 세션과 실행 경로를 정리한 뒤 legacy/move_control로 이동한다. .git/worktrees 역참조를 점검하고 필요한 Git repair 후 모든 워크트리 상태를 재검증한다.
6. 지도 후보를 비교하고 실제 운영할 지도만 Rosy/maps로 승격한다. 이미지뿐 아니라 YAML 원점·해상도·파일 해시·출처·실물 검증 상태를 함께 기록한다.
7. move_control 기능은 ROSY 계약에 맞게 선별 이관한다. 두 런타임을 그대로 동시 구동하지 않는다. move_control safety_node와 rosy_core bridge 모두 모터 토픽을 소유하므로 최종 출력 소유자 하나를 정하는 설계·테스트가 선행되어야 한다.
8. 최종 git status/worktree list, import·빌드·실행 경로, 지도 해시를 확인한다. 코드 이동이 발생하면 각 저장소의 관련 테스트를 실행한다. 실물 배포와 두 로봇 주행 검증은 별도 결과로 기록한다.

## 완료 조건

ROS 루트에는 제품 저장소와 용도가 명확한 작업/보관 폴더만 남는다.
기존 Git 이력, 미병합 작업, 검증 증거와 지도를 복구할 수 있어야 한다.
move_control이라는 문자열은 호환 패키지·이력·출처에는 남을 수 있으며 일괄 치환 대상이 아니다.
