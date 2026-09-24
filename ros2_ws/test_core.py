"""Run hardware-free tests from Windows or Linux: python ros2_ws/test_core.py."""
from pathlib import Path
import os
import sys

ROOT = Path(__file__).resolve().parent
for package in ('urc_kinematics', 'urc_can', 'urc_perception', 'urc_description', 'urc_autonomy', 'urc_simulation'):
    sys.path.insert(0, str(ROOT / 'src' / package))

if __name__ == '__main__':
    # The ROS launch-testing auto-plugin imports duplicate test basenames itself,
    # bypassing pytest's importlib mode. These tests do not need third-party plugins.
    os.environ['PYTEST_DISABLE_PLUGIN_AUTOLOAD'] = '1'
    os.environ['ROS_DOMAIN_ID'] = '178'
    os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE'] = 'LOCALHOST'
    import pytest
    raise SystemExit(pytest.main([
        str(ROOT / 'src' / 'urc_kinematics' / 'test'),
        str(ROOT / 'src' / 'urc_can' / 'test'),
        str(ROOT / 'src' / 'urc_perception' / 'test'),
        str(ROOT / 'src' / 'urc_simulation' / 'test'),
        str(ROOT / 'src' / 'urc_autonomy' / 'test'),
        '-q', '--import-mode=importlib', *sys.argv[1:],
    ]))
