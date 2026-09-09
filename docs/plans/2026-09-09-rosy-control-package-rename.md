# ROS 패키지명 전환: move_control → rosy_control

작성일: 2026-09-09. 상태: 저장소 전환 완료, 로봇 마이그레이션 미실행.

## 결정

`docs/plans/2026-09-08-rosy-fleet-folder-transition.md`의 "폴더명만 `Rosy Control`로
바꾸고 ROS 패키지명은 `move_control`을 유지한다"는 결정을 뒤집는다. 패키지 식별자까지
`rosy_control`로 전환한다. 제품명과 실행 이름이 갈라져 있으면 배포 경로·문서·토픽 설명이
계속 두 이름을 오가야 하고, 그 비용이 전환 비용을 넘어선다고 판단했다.

전환에 따르는 대가는 명시적으로 수용한다: **전환 이전에 배포된 릴리스는 새 업데이터의
롤백 대상이 아니다.** 이전 번들은 내부 경로가 `move_control/`이고 package.xml이
`move_control`을 선언하므로, 이름이 바뀐 워크스페이스에 그대로 스왑해 넣을 수 없다.
첫 `rosy_control` 릴리스가 새로운 롤백 바닥이 된다.

## 저장소에서 바꾼 것

| 영역 | 내용 |
|---|---|
| 패키지 메타 | `package.xml` `<name>`, `setup.py` `package_name` + 콘솔 스크립트 11개, `setup.cfg` script_dir/install_scripts, `resource/move_control` → `resource/rosy_control` |
| Python 패키지 | `move_control/` → `rosy_control/` (git mv, 이력 보존) 및 전 모듈 import |
| 테스트 | `test/` 60개 파일의 import 및 경로 문자열 |
| launch | 10개 launch 파일의 `package=` 와 `get_package_share_directory` |
| CI/배포 | `.github/workflows/release.yml`, `tools/ci/make_bundle.sh` (tar prefix, 매니페스트 name), `tools/ci/run_tests.sh`, `tools/deploy/*` |
| 하드코딩 경로 | `calib_node.py`의 `/home/pinky/dev_ws/wj/src/...`, `startup_calibration_node.py`의 `~/.local/state/...`, `tools/pinky_adc_lock.py`의 락 경로 |
| 문서 | `README.md`, `AGENTS.md`, `CLAUDE.md`, `STEPS.txt`, 디렉터리별 `AGENTS.md` |

## 일부러 바꾸지 않은 것

`docs/validation/`, `docs/evidence/`, `docs/archive/`, 그리고 기존 `docs/plans/`·
`docs/research/`·`docs/solutions/` 문서의 `move_control` 문자열 약 3,500건은 그대로 둔다.
이것들은 특정 시점에 그 이름으로 실행된 결과의 기록이므로, 치환하면 증거가 아니라
사후에 손댄 문서가 된다. 2026-09-08 문서의 "`move_control`이라는 문자열은 호환
패키지·이력·출처에는 남을 수 있으며 일괄 치환 대상이 아니다"라는 판단을 그대로 따른다.

## 로봇 전환 절차

로봇 위 상태는 저장소 밖에 있으므로 별도로 옮겨야 한다. 순서를 지킬 것.

1. 로봇을 정지시킨다.
2. 업데이터 스크립트를 새 버전으로 동기화한다 — `./tools/deploy/deploy.sh` 의
   `sync_scripts` 가 `migrate_to_rosy_control.sh` 를 함께 올린다.
3. 로봇에서 `~/pinky-deploy/migrate_to_rosy_control.sh --dry-run` 으로 계획을 확인한 뒤
   인자 없이 다시 실행한다. 이 스크립트가 옮기는 것:
   - `~/releases/move_control/` → `~/releases/rosy_control/`
   - `~/.local/state/move_control/` → `~/.local/state/rosy_control/` — **캘리브레이션
     결과(`calibration.json`)가 여기 있다.** 이 단계를 건너뛰면 로봇이 캘리브를 잃는다.
   - `~/dev_ws/wj/src/move_control` 심링크 제거, `current`/`previous` 포인터 정리
   - `build/move_control`, `install/move_control` 잔여 산출물 제거
4. 첫 `rosy_control` 태그를 릴리스하고 설치한다:
   `~/pinky-deploy/pinky_update.sh --from-github --tag <tag>`
5. `ros2 pkg executables rosy_control` 로 11개 실행 파일을 확인한 뒤
   `ros2 launch rosy_control robot.launch.py` 로 기동한다.

## 저장소 외 남은 작업

- GitHub 저장소 `livsbittt/move_control` → `rosy_control` rename, 이후
  `git remote set-url origin`. GitHub가 옛 URL을 리다이렉트하므로 순서는 무관하다.
- 위 로봇 마이그레이션은 실제 장비에서 아직 실행하지 않았다.

## 검증 결과

- `./tools/ci/run_tests.sh` — 590 passed, 1 skipped. ROS 의존 2개 파일
  (`test_localization_gate.py`, `test_scale.py`)은 PC에 ROS가 없어 건너뛴 것으로,
  이번 전환과 무관한 기존 동작이다. 로봇에서 `--system` 으로 전체가 돈다.
- `./tools/ci/make_bundle.sh` — `dist/rosy_control-<version>.tar.gz` 생성, tar 최상위
  prefix `rosy_control/`, 매니페스트 `"name": "rosy_control"`. release.yml 의
  prefix 게이트 통과.
- `bash -n` — `tools/deploy/*.sh`, `tools/ci/*.sh` 전부 문법 정상.
- 저장소에서 `docs/` 를 제외한 `move_control` 잔여 참조 0건.
