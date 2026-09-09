"""Retain odometry accounting without doing display TF work at sensor rate."""
import ast
import threading
import time
from pathlib import Path
from types import SimpleNamespace
from unittest.mock import Mock
from rosy_control.sensing.map_pose import record_odom


def test_odometry_burst_records_all_motion_without_reprojecting_display():
    path = Path(__file__).parents[1]/'rosy_control/web_node.py'
    cls = next(n for n in ast.parse(path.read_text(encoding='utf-8')).body if isinstance(n, ast.ClassDef) and n.name == 'WebNode')
    method = next(n for n in cls.body if isinstance(n, ast.FunctionDef) and n.name == 'on_odom')
    state = {}
    scope = dict(LOCK=threading.Lock(), STATE=state, time=time, record_odom=record_odom, TRAIL_MAX=3000)
    exec(compile(ast.Module(body=[method], type_ignores=[]), str(path), 'exec'), scope)
    node = SimpleNamespace(refresh_pose=Mock())
    for i in range(50):
        msg = SimpleNamespace(header=SimpleNamespace(stamp=SimpleNamespace(sec=100,nanosec=i*20_000_000),frame_id='odom'),
            pose=SimpleNamespace(pose=SimpleNamespace(position=SimpleNamespace(x=i*.001,y=0.))))
        scope['on_odom'](node,msg)
    assert abs(state['path_exact_m']-.049)<1e-9
    assert state['pose_prev'] == (.049,0.)
    assert node.odom_stamp == 100.98
    node.refresh_pose.assert_not_called()


def test_display_refresh_retains_steady_five_hz_timer():
    path = Path(__file__).parents[1]/'rosy_control/web_node.py'
    tree = ast.parse(path.read_text(encoding='utf-8'))
    timers = [n for n in ast.walk(tree) if isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute)
              and n.func.attr=='create_timer' and len(n.args)>1 and isinstance(n.args[1],ast.Attribute)
              and n.args[1].attr=='refresh_pose']
    assert len(timers)==1
    assert ast.literal_eval(timers[0].args[0])==.2
    assert any(k.arg=='clock' for k in timers[0].keywords)
