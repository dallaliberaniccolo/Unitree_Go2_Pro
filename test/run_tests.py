#!/usr/bin/env python3
"""Explicit offline suite: transport stubs, real mission/detector code, no robot."""
from pathlib import Path
import sys
import unittest

from ros_shim import install

root = Path(__file__).resolve().parents[1]
for package in ('go2_animal_detector', 'go2_mission_manager', 'go2_vision_test'):
    sys.path.insert(0, str(root / 'src' / package))
install()
suite = unittest.TestSuite()
for directory in (root / 'src/go2_mission_manager/test', root / 'test'):
    suite.addTests(unittest.TestLoader().discover(str(directory), top_level_dir=str(directory)))
result = unittest.TextTestRunner(verbosity=2).run(suite)
sys.exit(0 if result.wasSuccessful() else 1)
