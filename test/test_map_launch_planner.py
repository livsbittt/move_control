"""Inspect constructed launch actions without starting ROS processes."""
import ast
import os
from pathlib import Path
from types import SimpleNamespace
import unittest


def actions(filename):
    path = Path(__file__).parents[1] / 'launch' / filename
    method = next(n for n in ast.parse(path.read_text(encoding='utf-8')).body
                  if isinstance(n, ast.FunctionDef) and n.name == 'generate_launch_description')
    def record(kind):
        return lambda *args, **kwargs: SimpleNamespace(kind=kind, args=args, **kwargs)
    scope = {'os':os, 'get_package_share_directory':lambda package:'/share/'+package,
             'LaunchDescription':lambda entries:entries}
    for name in ('DeclareLaunchArgument','IncludeLaunchDescription','PythonLaunchDescriptionSource',
                 'LaunchConfiguration','IfCondition','TimerAction','Node'):
        scope[name] = record(name)
    exec(compile(ast.Module(body=[method],type_ignores=[]),str(path),'exec'),scope)
    return scope['generate_launch_description']()


class MapPlannerLaunchTest(unittest.TestCase):
    def test_map_starts_idle_goal_capability_with_explicit_opt_out(self):
        entries = actions('map.launch.py')
        option = next(a for a in entries if a.kind == 'DeclareLaunchArgument')
        self.assertEqual(option.args, ('start_goal',))
        self.assertEqual(option.default_value, 'true')
        includes = [a for a in entries if a.kind == 'IncludeLaunchDescription']
        self.assertEqual(len(includes), 2)
        planner = next(a for a in includes if os.path.basename(a.args[0].args[0]) == 'goal.launch.py')
        self.assertEqual(planner.condition.args[0].args, ('start_goal',))

    def test_dashboard_launch_keeps_single_delayed_planner(self):
        entries = actions('dashboard_control.launch.py')
        include = next(a for a in entries if a.kind == 'IncludeLaunchDescription')
        self.assertEqual(dict(include.launch_arguments), {'start_goal':'false'})
        timers = [a for a in entries if a.kind == 'TimerAction']
        goals = [(timer.period,node) for timer in timers for node in timer.actions if node.executable == 'goal_node']
        self.assertEqual(len(goals), 1)
        self.assertEqual(goals[0][0], 3.5)
        self.assertEqual(goals[0][1].parameters[-1], {'mode':'stop'})
