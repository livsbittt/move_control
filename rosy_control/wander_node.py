#!/usr/bin/env python3
"""Wander entry. Subjects live in rosy_control.wander."""
from .wander.node import WanderNode, main

__all__ = ['WanderNode', 'main']


if __name__ == '__main__':
    main()
