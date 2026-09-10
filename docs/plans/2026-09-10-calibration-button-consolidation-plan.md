# 보정 버튼 정리 구현 계획

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 대시보드 보정 버튼 6개를 3개(전체 보정 / 부분 보정 / 검증 중단)로 줄이고, 실패 센서 판단을 운전자가 아니라 노드가 하게 한다.

**Architecture:** 센서 등급표를 `control/sensor_tiers.py` 한 곳에 두고, 노드가 실패 센서를 보고 부분 보정 가능 여부와 경로를 결정해 `partial_option`으로 발행한다. 화면은 그 판정을 렌더링만 한다. 기존 세 모드(`sensing_only` / `use_existing_settings` / `use_limited_sensors`)의 백엔드 커맨드는 유지하고 화면에서만 제거한다.

**Tech Stack:** Python 3 / rclpy (ROS 2 Jazzy), 의존성 없는 단일 HTML + IIFE, `unittest` + `pytest`.

설계 근거: `docs/plans/2026-09-09-calibration-button-consolidation.md`

---

### Task 1: 센서 등급표

**Files:**
- Create: `rosy_control/control/sensor_tiers.py`
- Test: `test/test_sensor_tiers.py`

- [ ] **Step 1: 실패 테스트 작성**

`test/test_sensor_tiers.py`:

```python
"""Exclusion tiers decide what a partial calibration may drop; nothing else may."""
import unittest

from rosy_control.control.calibration import SENSORS
from rosy_control.control.sensor_tiers import EXCLUDABLE, REQUIRED_FOR_MOTION, partial_plan


class SensorTierTests(unittest.TestCase):
    def healthy(self):
        return {name: {'eligible': True} for name in SENSORS}

    def failing(self, *names):
        sensors = self.healthy()
        for name in names:
            sensors[name]['eligible'] = False
        return sensors

    def test_tiers_do_not_overlap_and_stay_inside_the_sensor_list(self):
        self.assertEqual(set(EXCLUDABLE) & set(REQUIRED_FOR_MOTION), set())
        for name in EXCLUDABLE + REQUIRED_FOR_MOTION:
            self.assertIn(name, SENSORS)

    def test_no_failure_leaves_nothing_to_exclude(self):
        plan = partial_plan(self.healthy())
        self.assertFalse(plan['available'])
        self.assertEqual(plan['reason'], 'no_failed_sensor')

    def test_camera_failure_keeps_rotation_available(self):
        plan = partial_plan(self.failing('camera'))
        self.assertTrue(plan['available'])
        self.assertEqual(plan['excluded_sensors'], ('camera',))
        self.assertTrue(plan['rotation_available'])

    def test_imu_failure_withdraws_rotation(self):
        plan = partial_plan(self.failing('imu'))
        self.assertTrue(plan['available'])
        self.assertEqual(plan['excluded_sensors'], ('imu',))
        self.assertFalse(plan['rotation_available'])

    def test_every_motion_sensor_blocks_partial_calibration(self):
        for name in REQUIRED_FOR_MOTION:
            plan = partial_plan(self.failing(name))
            self.assertFalse(plan['available'], name)
            self.assertEqual(plan['reason'], 'required_sensor_failed')
            self.assertIn(name, plan['blocking_sensors'])

    def test_map_blocks_only_when_localization_is_required(self):
        self.assertTrue(partial_plan(self.failing('map', 'imu'))['available'])
        blocked = partial_plan(self.failing('map', 'imu'), localization_required=True)
        self.assertFalse(blocked['available'])
        self.assertIn('map', blocked['blocking_sensors'])

    def test_ok_is_accepted_when_eligible_is_absent(self):
        self.assertEqual(partial_plan({name: {'ok': True} for name in SENSORS})['reason'], 'no_failed_sensor')


if __name__ == '__main__':
    unittest.main()
```

- [ ] **Step 2: 실패 확인**

Run: `python3 -m pytest test/test_sensor_tiers.py -q`
Expected: FAIL — `ModuleNotFoundError: No module named 'rosy_control.control.sensor_tiers'`

- [ ] **Step 3: 구현**

`rosy_control/control/sensor_tiers.py`:

```python
"""Exclusion tiers; the only place that decides which sensors a partial calibration may drop."""
from .calibration import SENSORS

# Motion permission is impossible without these. They are never excludable.
REQUIRED_FOR_MOTION = ('lidar', 'odom', 'ir', 'us', 'tf')
# The map pair is required only while localization is in use.
REQUIRED_FOR_LOCALIZATION = ('map', 'map_tf')
# Dropping imu costs rotation verification; dropping camera costs nothing, as
# camera is advisory and never gates motion.
EXCLUDABLE = ('imu', 'camera')


def failed_sensors(sensors):
    return tuple(name for name in SENSORS
                 if not sensors.get(name, {}).get('eligible', sensors.get(name, {}).get('ok', False)))


def partial_plan(sensors, localization_required=False):
    """Report what a partial calibration can do about the current sensor failures.

    Availability is never a motion permission; the runtime gates still apply.
    """
    failed = failed_sensors(sensors)
    required = REQUIRED_FOR_MOTION + (REQUIRED_FOR_LOCALIZATION if localization_required else ())
    blocking = tuple(name for name in failed if name in required)
    if blocking:
        return {'available': False, 'reason': 'required_sensor_failed',
                'blocking_sensors': blocking, 'excluded_sensors': (), 'rotation_available': False}
    excluded = tuple(name for name in failed if name in EXCLUDABLE)
    if not excluded:
        return {'available': False, 'reason': 'no_failed_sensor',
                'blocking_sensors': (), 'excluded_sensors': (), 'rotation_available': True}
    rotation = 'imu' not in excluded
    return {'available': True, 'reason': 'calibration_continues' if rotation else 'rotation_unavailable',
            'blocking_sensors': (), 'excluded_sensors': excluded, 'rotation_available': rotation}
```

- [ ] **Step 4: 통과 확인**

Run: `python3 -m pytest test/test_sensor_tiers.py -q`
Expected: PASS, 7 passed

- [ ] **Step 5: 커밋**

```bash
git add rosy_control/control/sensor_tiers.py test/test_sensor_tiers.py
git commit -m "Add the sensor exclusion tier table for partial calibration"
```

---

### Task 2: `partial_sensing_report` 제외 대상 일반화

기존 호출부가 IMU를 계속 쓰므로 기본값을 `('imu',)`로 두어 동작을 바꾸지 않는다.

**Files:**
- Modify: `rosy_control/control/sensing_only.py`
- Test: `test/test_sensing_only.py`

- [ ] **Step 1: 실패 테스트 추가**

`test/test_sensing_only.py`의 `PartialSensingTests` 클래스 안에 추가한다:

```python
    def test_exclusion_set_is_a_parameter_and_defaults_to_imu(self):
        result = partial_sensing_report(self.healthy(), excluded=('camera',))
        self.assertEqual(result['excluded_sensors'], ['camera'])
        self.assertFalse(result['sensors']['camera']['eligible'])
        self.assertTrue(result['sensors']['imu'].get('eligible', result['sensors']['imu'].get('ok')))
        self.assertEqual(partial_sensing_report(self.healthy())['excluded_sensors'], ['imu'])

    def test_multiple_exclusions_still_never_grant_motion(self):
        result = partial_sensing_report(self.healthy(), excluded=('imu', 'camera'))
        self.assertEqual(result['excluded_sensors'], ['imu', 'camera'])
        self.assertFalse(result['motion_allowed'])
        self.assertFalse(result['ready'])
```

- [ ] **Step 2: 실패 확인**

Run: `python3 -m pytest test/test_sensing_only.py -q`
Expected: FAIL — `TypeError: partial_sensing_report() got an unexpected keyword argument 'excluded'`

- [ ] **Step 3: 구현**

`rosy_control/control/sensing_only.py` 전체를 이것으로 바꾼다:

```python
"""Stationary diagnostic reporting; exclusion never authorizes calibration or motion."""
from .calibration import SENSORS


def partial_sensing_report(sensors, excluded=('imu',)):
    report = {name: dict(item) for name, item in sensors.items()}
    excluded = tuple(excluded)
    reason = 'operator_requested_' + '_'.join(excluded) + '_exclusion'
    for name in excluded:
        report[name] = {**report.get(name, {}), 'ok': False, 'eligible': False,
                        'excluded': True, 'status': 'excluded', 'reason': reason,
                        'detail': name.upper() + ' excluded for stationary diagnostics; motion prohibited'}
    required = [name for name in SENSORS if name not in excluded]
    qualified = all(report.get(name, {}).get('eligible', report.get(name, {}).get('ok', False))
                    for name in required)
    return {'mode': 'sensing_only', 'partial': True, 'excluded_sensors': list(excluded),
            'exclusion_reason': reason, 'partial_baseline_ready': qualified,
            'ready': False, 'calibration_verified': False, 'motion_allowed': False,
            'rotation_verified': False, 'settings_applied': False, 'sensors': report}
```

- [ ] **Step 4: 통과 확인**

Run: `python3 -m pytest test/test_sensing_only.py -q`
Expected: PASS — 기존 테스트 전부 포함해 green

- [ ] **Step 5: 커밋**

```bash
git add rosy_control/control/sensing_only.py test/test_sensing_only.py
git commit -m "Take the stationary exclusion set as a parameter"
```

---

### Task 3: `configured_operation` 제외 대상 일반화

**Files:**
- Modify: `rosy_control/control/configured_operation.py`
- Modify: `rosy_control/startup_calibration_node.py:625`
- Test: `test/test_configured_operation.py`

- [ ] **Step 1: 테스트 갱신**

`test/test_configured_operation.py`의 `test_limited_mode_excludes_only_imu_from_live_requirements`를 이것으로 교체한다:

```python
    def test_limited_mode_excludes_only_the_named_sensors(self):
        sensors = self.health()
        sensors['imu']['eligible'] = False
        self.assertEqual(self.reasons(sensors, excluded=('imu',)), [])
        for name in ('lidar', 'odom', 'ir', 'us', 'tf'):
            missing = {key: dict(value) for key, value in sensors.items()}
            missing[name]['eligible'] = False
            self.assertIn(name, self.reasons(missing, excluded=('imu',)))

    def test_status_reports_the_exclusion_set_it_was_given(self):
        result = configured_status(False, [], limited_sensors=True, excluded_sensors=('imu', 'camera'))
        self.assertEqual(result['excluded_sensors'], ['imu', 'camera'])
        self.assertEqual(configured_status(False, [], limited_sensors=True)['excluded_sensors'], ['imu'])
        self.assertEqual(configured_status(False, [])['excluded_sensors'], [])
```

- [ ] **Step 2: 실패 확인**

Run: `python3 -m pytest test/test_configured_operation.py -q`
Expected: FAIL — `TypeError: configured_waiting_reasons() got an unexpected keyword argument 'excluded'`

- [ ] **Step 3: 구현**

`rosy_control/control/configured_operation.py`에서 `configured_waiting_reasons`의 첫 다섯 줄을 이것으로 교체한다:

```python
def configured_waiting_reasons(sensors, localization_required, geometry_fresh, estop, hazards, now, excluded=()):
    required = ('lidar', 'odom', 'ir', 'imu', 'us', 'tf')
    required = tuple(name for name in required if name not in excluded)
    if localization_required:
        required += ('map', 'map_tf')
```

`configured_status` 전체를 이것으로 교체한다:

```python
def configured_status(ready, waiting_reasons, limited_sensors=False, excluded_sensors=None):
    if excluded_sensors is None:
        excluded_sensors = ('imu',) if limited_sensors else ()
    return {'mode': 'limited_sensors' if limited_sensors else 'existing_settings', 'existing_settings': True,
            'limited_sensors': limited_sensors, 'degraded': limited_sensors,
            'excluded_sensors': list(excluded_sensors),
            **({'speed_limits': {'linear_mps': .005, 'angular_rad_s': .05}} if limited_sensors else {}),
            'settings_source': 'configured_parameters', 'calibration_skipped': True,
            'calibration_complete': True, 'completion_source': 'operator_override',
            'calibration_verified': False, 'rotation_verified': False,
            'ready': bool(ready), 'operating_ready': bool(ready), 'motion_allowed': bool(ready),
            'waiting_reasons': list(waiting_reasons)}
```

- [ ] **Step 4: 호출부 갱신**

`rosy_control/startup_calibration_node.py:625`의 `exclude_imu=limited`로 끝나는 줄을 다음으로 바꾼다:

```python
                {key.rsplit('/', 1)[-1]: value for key, value in self.hazards.items()}, now,
                excluded=tuple(getattr(self, 'excluded_sensors', ()) or (('imu',) if limited else ())))
```

- [ ] **Step 5: 통과 확인**

Run: `python3 -m pytest test/test_configured_operation.py test/test_limited_sensor_lease.py test/test_limited_safety_adapter.py -q`
Expected: PASS

- [ ] **Step 6: 커밋**

```bash
git add rosy_control/control/configured_operation.py rosy_control/startup_calibration_node.py test/test_configured_operation.py
git commit -m "Take the configured-operation exclusion set as a parameter"
```

---

### Task 4: `partial_calibration` 커맨드와 보정 범위

**Files:**
- Modify: `rosy_control/startup_calibration_node.py` (import, `reset`, `on_command`, `publish`)
- Test: `test/test_partial_calibration.py`

- [ ] **Step 1: 실패 테스트 작성**

`test/test_partial_calibration.py`:

```python
"""The node decides the partial-calibration route; the screen only renders the verdict."""
import ast
import unittest
from pathlib import Path
from types import SimpleNamespace

from rosy_control.control.configured_operation import configured_status
from rosy_control.control.sensor_tiers import partial_plan


def node_method(name, **bindings):
    path = Path(__file__).parents[1] / 'rosy_control/startup_calibration_node.py'
    tree = ast.parse(path.read_text(encoding='utf-8'))
    cls = next(n for n in tree.body if isinstance(n, ast.ClassDef))
    method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == name)
    namespace = dict(bindings)
    exec(compile(ast.Module(body=[method], type_ignores=[]), str(path), 'exec'), namespace)
    return namespace[name]


class PartialCalibrationCommandTests(unittest.TestCase):
    def node(self, sensors):
        calls = []
        return SimpleNamespace(
            calls=calls, message='',
            stationary_report=lambda now: sensors,
            get_parameter=lambda name: SimpleNamespace(value=False),
            reset=lambda **kwargs: calls.append(kwargs),
            publish=lambda: None,
            after_relocation='stay', calibration_scope='full')

    def run_command(self, node, command):
        node_method('on_command', partial_plan=partial_plan,
                    time=SimpleNamespace(monotonic=lambda: 0.))(node, SimpleNamespace(data=command))

    def healthy(self):
        return {name: {'eligible': True} for name in
                ('lidar', 'odom', 'ir', 'imu', 'us', 'camera', 'tf', 'map', 'map_tf')}

    def test_imu_failure_routes_to_limited_sensor_operation(self):
        sensors = self.healthy()
        sensors['imu']['eligible'] = False
        node = self.node(sensors)
        self.run_command(node, 'partial_calibration')
        self.assertEqual(node.calls, [{'existing_settings': True, 'limited_sensors': True,
                                       'excluded_sensors': ('imu',)}])

    def test_camera_failure_continues_a_real_calibration(self):
        sensors = self.healthy()
        sensors['camera']['eligible'] = False
        node = self.node(sensors)
        self.run_command(node, 'partial_calibration')
        self.assertEqual(node.calls, [{'excluded_sensors': ('camera',)}])

    def test_required_sensor_failure_resets_nothing_and_names_the_sensor(self):
        sensors = self.healthy()
        sensors['lidar']['eligible'] = False
        node = self.node(sensors)
        self.run_command(node, 'partial_calibration')
        self.assertEqual(node.calls, [])
        self.assertIn('lidar', node.message)

    def test_no_failure_resets_nothing(self):
        node = self.node(self.healthy())
        self.run_command(node, 'partial_calibration')
        self.assertEqual(node.calls, [])
        self.assertIn('full calibration', node.message)

    def test_retry_carries_position_and_scope(self):
        node = self.node(self.healthy())
        self.run_command(node, 'retry:return_origin:skip_motion')
        self.assertEqual(node.after_relocation, 'return_origin')
        self.assertEqual(node.calibration_scope, 'skip_motion')

    def test_legacy_two_part_retry_still_means_full_scope(self):
        node = self.node(self.healthy())
        self.run_command(node, 'retry:stay')
        self.assertEqual(node.after_relocation, 'stay')
        self.assertEqual(node.calibration_scope, 'full')

    def test_skipped_motion_scope_routes_to_existing_settings(self):
        node = self.node(self.healthy())
        self.run_command(node, 'retry:stay:skip_motion')
        self.assertEqual(node.calls, [{'invalidate_certificate': True, 'existing_settings': True}])

    def test_full_scope_runs_a_real_calibration(self):
        node = self.node(self.healthy())
        self.run_command(node, 'retry:stay:full')
        self.assertEqual(node.calls, [{'invalidate_certificate': True}])

    def test_partial_calibration_never_claims_a_verified_calibration(self):
        status = configured_status(True, [], limited_sensors=True, excluded_sensors=('imu',))
        self.assertFalse(status['calibration_verified'])
        self.assertFalse(status['rotation_verified'])
        self.assertTrue(status['calibration_skipped'])


if __name__ == '__main__':
    unittest.main()
```

- [ ] **Step 2: 실패 확인**

Run: `python3 -m pytest test/test_partial_calibration.py -q`
Expected: FAIL — `partial_calibration`을 모르는 `on_command`가 `reset`을 부르지 않아 `node.calls == []`

- [ ] **Step 3: import와 `reset` 확장**

`rosy_control/startup_calibration_node.py`의 import 블록(38행 `from .control.sensing_only import partial_sensing_report` 아래)에 추가한다:

```python
from .control.sensor_tiers import partial_plan
```

`reset` 시그니처(111행)를 다음으로 바꾼다:

```python
    def reset(self, invalidate_certificate=False, sensing_only=False, existing_settings=False,
              limited_sensors=False, excluded_sensors=()):
```

`reset` 본문의 `self.limited_sensors = limited_sensors` 바로 다음 줄에 추가한다:

```python
        self.excluded_sensors = tuple(excluded_sensors)
        self.calibration_scope = getattr(self, 'calibration_scope', 'full')
```

- [ ] **Step 4: `on_command` 확장**

`on_command`에서 `if command == 'use_existing_settings':` 바로 위에 추가한다:

```python
        if command == 'partial_calibration':
            plan = partial_plan(self.stationary_report(time.monotonic()),
                                bool(self.get_parameter('localization_required').value))
            if not plan['available']:
                self.message = ('Required sensor failed and cannot be excluded: '
                                + ', '.join(plan['blocking_sensors'])
                                if plan['blocking_sensors']
                                else 'No failed sensor to exclude; use full calibration')
                self.publish()
                return
            if plan['rotation_available']:
                self.reset(excluded_sensors=plan['excluded_sensors'])
            else:
                self.reset(existing_settings=True, limited_sensors=True,
                           excluded_sensors=plan['excluded_sensors'])
            return
```

기존 `if command in ('retry:stay', 'retry:return_origin'):` 분기 두 줄을 다음으로 교체한다:

```python
        if command.startswith('retry:'):
            parts = command.split(':')
            self.after_relocation = parts[1] if len(parts) > 1 and parts[1] else 'stay'
            self.calibration_scope = parts[2] if len(parts) > 2 and parts[2] else 'full'
            command = 'retry'
```

같은 메서드의 `elif command == 'retry':` 분기를 다음으로 교체한다. 범위를 저장만 하고
쓰지 않으면 select가 아무것도 바꾸지 못하므로, `skip_motion`이 실제로 측정을 건너뛰게 한다:

```python
        elif command == 'retry':
            if getattr(self, 'calibration_scope', 'full') == 'skip_motion':
                self.reset(invalidate_certificate=True, existing_settings=True)
            else:
                self.reset(invalidate_certificate=True)
```

- [ ] **Step 5: 통과 확인**

Run: `python3 -m pytest test/test_partial_calibration.py -q`
Expected: PASS, 9 passed

- [ ] **Step 6: `publish`에 판정 노출**

`publish`가 만드는 상태 dict에서 `'settings_applied':`로 시작하는 항목(841행 근처) 바로 다음 줄에 추가한다:

```python
                'partial_option': {key: (list(value) if isinstance(value, tuple) else value)
                                   for key, value in partial_plan(
                                       self.sensors,
                                       bool(self.get_parameter('localization_required').value)).items()},
                'calibration_scope': getattr(self, 'calibration_scope', 'full'),
```

- [ ] **Step 7: 전체 테스트**

Run: `python3 -m pytest test/ -q`
Expected: PASS — 회귀 없음

- [ ] **Step 8: 커밋**

```bash
git add rosy_control/startup_calibration_node.py test/test_partial_calibration.py
git commit -m "Let the node route partial calibration from the failed sensor set"
```

---

### Task 5: HTTP 커맨드 허용 목록

**Files:**
- Modify: `rosy_control/web_node.py:883`, `rosy_control/web_node.py:895`
- Test: `test/test_calibration_receiver.py`

- [ ] **Step 1: 실패 테스트 추가**

`test/test_calibration_receiver.py`의 테스트 클래스 안에 추가한다. 파일 상단에 `from pathlib import Path`가 없으면 함께 추가한다:

```python
    def test_partial_calibration_and_scoped_retry_are_accepted(self):
        source = Path(__file__).parents[1].joinpath('rosy_control/web_node.py').read_text(encoding='utf-8')
        for command in ('partial_calibration', 'retry:stay:full', 'retry:return_origin:skip_motion'):
            self.assertIn(command, source, command)
```

- [ ] **Step 2: 실패 확인**

Run: `python3 -m pytest test/test_calibration_receiver.py -q`
Expected: FAIL — `'partial_calibration' not found`

- [ ] **Step 3: 허용 목록 교체**

883행(`if body not in (...)`)을 다음으로 바꾼다:

```python
                if body not in ('retry', 'retry:stay', 'retry:return_origin', 'retry:stay:full',
                                'retry:stay:skip_motion', 'retry:return_origin:full',
                                'retry:return_origin:skip_motion', 'validate_motion', 'abort',
                                'partial_calibration', 'sensing_only', 'use_existing_settings',
                                'use_limited_sensors'):
```

895행(`if body in (...)`)을 다음으로 바꾼다:

```python
                if body in ('retry', 'retry:stay', 'retry:return_origin', 'retry:stay:full',
                            'retry:stay:skip_motion', 'retry:return_origin:full',
                            'retry:return_origin:skip_motion', 'abort', 'partial_calibration',
                            'sensing_only', 'use_existing_settings', 'use_limited_sensors'):
```

- [ ] **Step 4: 통과 확인**

Run: `python3 -m pytest test/test_calibration_receiver.py -q`
Expected: PASS

- [ ] **Step 5: 커밋**

```bash
git add rosy_control/web_node.py test/test_calibration_receiver.py
git commit -m "Accept partial calibration and scoped retry over HTTP"
```

---

### Task 6: 대시보드 버튼 정리

**Files:**
- Modify: `web/dashboard.html:451-461` (select와 버튼), `web/dashboard.html:797-805` (렌더 로직), `calibrationAction`
- Test: `test/test_calibration_buttons.py`

- [ ] **Step 1: 실패 테스트 작성**

`test/test_calibration_buttons.py`:

```python
"""The operating row offers three choices; removed modes must not linger in the markup."""
import re
import unittest
from pathlib import Path


class CalibrationButtonTests(unittest.TestCase):
    def markup(self):
        return Path(__file__).parents[1].joinpath('web/dashboard.html').read_text(encoding='utf-8')

    def commands(self):
        return re.findall(r'data-calibration="([^"]+)"', self.markup())

    def test_only_the_three_mode_choices_and_the_motion_step_remain(self):
        self.assertEqual(sorted(self.commands()),
                         ['abort', 'partial_calibration', 'retry', 'validate_motion'])

    def test_removed_modes_are_gone_from_the_markup(self):
        for command in ('sensing_only', 'use_existing_settings', 'use_limited_sensors'):
            self.assertNotIn('data-calibration="' + command + '"', self.markup(), command)

    def test_scope_select_offers_full_and_skipped_motion(self):
        markup = self.markup()
        self.assertIn('id="calibrationscope"', markup)
        self.assertIn('value="full"', markup)
        self.assertIn('value="skip_motion"', markup)

    def test_partial_button_and_its_reason_line_exist(self):
        markup = self.markup()
        self.assertIn('id="calibrationpartialaction"', markup)
        self.assertIn('id="calibrationpartialreason"', markup)


if __name__ == '__main__':
    unittest.main()
```

- [ ] **Step 2: 실패 확인**

Run: `python3 -m pytest test/test_calibration_buttons.py -q`
Expected: FAIL — 명령이 6개라 첫 테스트부터 실패

- [ ] **Step 3: 마크업 교체**

`web/dashboard.html`에서 `<label for="calibrationafter">`로 시작하는 줄부터 버튼을 감싼 `</div>` 직전까지를 다음으로 교체한다:

```html
      <label for="calibrationscope">보정 범위</label>
      <select id="calibrationscope"><option value="full">전체 측정</option><option value="skip_motion">이동·회전 생략 · 기존 설정값</option></select>
      <label for="calibrationafter">공간 이동 후 보정이 끝나면</label>
      <select id="calibrationafter"><option value="stay">확보한 위치에 머물기</option><option value="return_origin">원래 위치로 복귀</option></select>
      <div class="row">
        <button type="button" id="calibrationretry" data-calibration="retry">전체 보정</button>
        <button type="button" id="calibrationpartialaction" data-calibration="partial_calibration">부분 보정 · 실패 센서 제외</button>
        <button type="button" id="calibrationmotion" data-calibration="validate_motion" hidden disabled>4초 저속 이동 검증</button>
        <button type="button" data-calibration="abort">검증 중단</button>
      </div>
      <p class="command-status" id="calibrationpartialreason" role="status" aria-live="polite" hidden></p>
```

- [ ] **Step 4: 렌더 로직 교체**

`$('calibrationretry').disabled`로 시작하는 줄부터 `$('calibrationlimited').disabled = limited;`까지를 다음으로 교체한다:

```javascript
  $('calibrationretry').disabled = phase === 'validating_motion' || phase === 'validating_rotation';
  setTxt('calibrationretry', '전체 보정');
  const option = calibration && calibration.partial_option || {available: false, reason: 'unknown'};
  const blocking = Array.isArray(option.blocking_sensors) ? option.blocking_sensors : [];
  const dropped = Array.isArray(option.excluded_sensors) ? option.excluded_sensors : [];
  const names = list => list.map(name => String(name).toUpperCase()).join(', ');
  $('calibrationpartialaction').disabled = !option.available ||
    phase === 'validating_motion' || phase === 'validating_rotation';
  const partialReasons = {
    required_sensor_failed: names(blocking) + ' 실패 — 제외 불가(주행 필수). 전체 보정 재시도 또는 중단.',
    no_failed_sensor: '제외할 실패 센서 없음 — 전체 보정을 사용하세요',
    rotation_unavailable: names(dropped) + ' 제외 가능 — 회전 검증 없이 기존 설정값 · 저속 제한 주행',
    calibration_continues: names(dropped) + ' 제외 가능 — 나머지 센서로 보정을 완주합니다',
  };
  $('calibrationpartialreason').hidden = !partialReasons[option.reason];
  setTxt('calibrationpartialreason', partialReasons[option.reason] || '');
```

- [ ] **Step 5: `calibrationAction` 갱신**

`calibrationAction` 안의 `if (action === 'retry')`로 시작하는 줄을 다음으로 바꾼다:

```javascript
  if (action === 'retry') action += ':' + ($('calibrationafter').value || 'stay') + ':' + ($('calibrationscope').value || 'full');
```

- [ ] **Step 6: 통과 확인**

Run: `python3 -m pytest test/test_calibration_buttons.py -q`
Expected: PASS, 4 passed

- [ ] **Step 7: 전체 테스트**

Run: `python3 -m pytest test/ -q`
Expected: PASS — 회귀 없음

- [ ] **Step 8: 커밋**

```bash
git add web/dashboard.html test/test_calibration_buttons.py
git commit -m "Reduce the calibration row to full, partial and abort"
```

---

## 마무리 확인

- [ ] `python3 -m pytest test/ -q` 전체 green
- [ ] `grep -c 'data-calibration' web/dashboard.html` 결과가 4
- [ ] 삭제한 세 커맨드가 백엔드에는 남아 있다: `grep -n "use_limited_sensors" rosy_control/startup_calibration_node.py`
- [ ] `docs/plans/2026-09-09-calibration-button-consolidation.md`의 15개 항목을 훑어 빠진 것이 없는지 대조
