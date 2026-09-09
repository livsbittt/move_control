import ast
import math
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock

from rosy_control.control.calibration_runtime import calibration_runtime, precision_scan_required


def health():
    return {name: {'eligible': True, 'ok': True} for name in ('lidar','odom','ir','imu','us','tf','camera','map','map_tf')}


def policy(sensors=None, **kwargs):
    values = dict(localization_required=False, geometry_fresh=True,
                  established_revision='verified', current_revision='verified', current_geometry_fresh=True)
    values.update(kwargs)
    return calibration_runtime(health() if sensors is None else sensors, **values)


def test_missing_map_only_holds_localization_required_operation():
    sensors=health()
    sensors['map']['eligible']=sensors['map_tf']['eligible']=False
    report=policy(sensors)
    assert report['waiting_reasons']==[]
    assert not report['sensors']['map']['required_for_operation']
    assert policy(sensors,localization_required=True)['waiting_reasons']==['map','map_tf']


def test_transient_data_or_geometry_hold_never_requests_recalibration():
    sensors=health();sensors['imu']['eligible']=False
    report=policy(sensors,geometry_fresh=False,current_geometry_fresh=False,current_revision=None)
    assert report['waiting_reasons']==['imu','safety_geometry']
    assert report['recalibration_required'] is False
    assert policy()['waiting_reasons']==[]


def test_only_fresh_established_identity_mismatch_requires_recalibration():
    assert policy(geometry_fresh=False,current_revision='changed')['recalibration_required']
    for updates in ({'current_geometry_fresh':False}, {'established_revision':None}, {'current_revision':None}):
        values=dict(geometry_fresh=False,current_revision='changed');values.update(updates)
        assert not policy(**values)['recalibration_required']


def test_heavy_scan_work_is_only_required_for_calibration_phases():
    for phase in ('ready','existing_settings','limited_sensors','sensing_only','failed','aborted'):
        assert not precision_scan_required(phase)
    for phase in ('collecting','waiting_motion','validating_motion','validating_rotation','relocating_calibration','returning_calibration'):
        assert precision_scan_required(phase)


def node_method(name, **bindings):
    path=Path(__file__).parents[1]/'rosy_control/startup_calibration_node.py'
    cls=next(n for n in ast.parse(path.read_text(encoding='utf-8')).body if isinstance(n,ast.ClassDef))
    method=next(n for n in cls.body if isinstance(n,ast.FunctionDef) and n.name==name)
    scope=dict(bindings)
    exec(compile(ast.Module(body=[method],type_ignores=[]),str(path),'exec'),scope)
    return scope[name]


def test_ready_runtime_recovers_without_reset_or_repeating_calibration():
    clock=SimpleNamespace(value=10.)
    sensors=health();sensors['map']['eligible']=False
    calls=[]
    node=SimpleNamespace(phase='ready',existing_settings=False,sensing_only=False,
        runtime_ready=True,runtime_healthy_since=None,read_tf=lambda:None,runtime_health=lambda now:sensors,
        get_parameter=lambda name:SimpleNamespace(value=False),geometry_fresh=lambda now:True,
        geometry_received=10.,geometry_revision='same',trial_geometry_revision='same',
        zero=lambda:calls.append('zero'),wander_pub=SimpleNamespace(publish=lambda msg:calls.append(msg.data)),
        last_report=0.,publish=lambda:calls.append('publish'),reset=Mock())
    tick=node_method('tick',time=SimpleNamespace(monotonic=lambda:clock.value),
                     calibration_runtime=calibration_runtime,String=SimpleNamespace)
    tick(node)
    assert node.runtime_ready and not node.recalibration_required
    sensors['lidar']['eligible']=False
    tick(node)
    assert not node.runtime_ready and node.phase=='ready'
    assert calls.count('stop')==1
    sensors['lidar']['eligible']=True
    clock.value=11.;node.geometry_received=11.;tick(node)
    clock.value=12.1;node.geometry_received=12.1;tick(node)
    assert node.runtime_ready and not node.recalibration_required
    node.reset.assert_not_called()


def test_ready_scan_keeps_live_health_without_wall_fit_or_rotation_registration():
    baseline=SimpleNamespace(latest=lambda name:(0.,0.,0.),samples={'odom':[(10.,(0.,0.,0.),True)]})
    transform=SimpleNamespace(transform=SimpleNamespace(rotation=SimpleNamespace(x=0.,y=0.,z=0.,w=1.),
                                                       translation=SimpleNamespace(x=-.017,y=0.)))
    node=SimpleNamespace(phase='ready',stamped=lambda *args:True,
        tf=SimpleNamespace(lookup_transform=lambda *args:transform),baseline=baseline,
        rotation_scan_sample=Mock(),wall_tracker=SimpleNamespace(update=Mock()),add_range=Mock(),raw_ranges={})
    scan=node_method('on_scan',is_robot_scan=lambda msg:True,math=math,
        rclpy=SimpleNamespace(time=SimpleNamespace(Time=lambda:None)),
        nose_from_quaternion=lambda *args:math.pi,sector_range=lambda *args,**kwargs:.4,
        precision_scan_required=precision_scan_required,time=SimpleNamespace(monotonic=lambda:10.))
    msg=SimpleNamespace(header=SimpleNamespace(frame_id='lidar'))
    for phase in ('ready','existing_settings','limited_sensors','sensing_only','failed'):
        node.phase=phase
        scan(node,msg)
    node.rotation_scan_sample.assert_not_called()
    node.wall_tracker.update.assert_not_called()
    assert node.add_range.call_args.args==('lidar',.4,True)
    node.phase='validating_rotation'
    scan(node,msg)
    node.rotation_scan_sample.assert_called_once()


def test_live_health_accepts_motion_without_stationary_variance_rechecks():
    samples={name:[(10.,(0.,0.,0.,.5),True)] for name in health()}
    node=SimpleNamespace(baseline=SimpleNamespace(samples=samples),us_source_valid=True,
        get_parameter=lambda name:SimpleNamespace(value=False),map_tf_diagnostic={})
    report=node_method('runtime_health')(node,10.)
    assert all(item['eligible'] for item in report.values())
    assert 'stationary limits do not apply' in report['odom']['detail']
