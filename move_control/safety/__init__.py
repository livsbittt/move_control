"""Lazy ROS compatibility exports; pure safety subjects import without ROS."""
from importlib import import_module

__all__ = ['parse_us_range', 'roll_pitch', 'SafetyNode', 'main']


def __getattr__(name):
    owners = {'parse_us_range': 'bumper', 'roll_pitch': 'hazard',
              'SafetyNode': 'node', 'main': 'node'}
    if name not in owners:
        raise AttributeError(name)
    value = getattr(import_module('.' + owners[name], __name__), name)
    globals()[name] = value
    return value
