"""Check ROS source wiring without claiming that launches ran."""
from pathlib import Path
import ast
import xml.etree.ElementTree as ET
import yaml

ROOT=Path(__file__).resolve().parents[2]


def test_unique_ros_packages_and_matching_ament_resources():
    seen=set()
    for manifest in ROOT.glob('arc_*/package.xml'):
        name=ET.parse(manifest).getroot().findtext('name')
        assert name not in seen and name==manifest.parent.name
        seen.add(name)
        if (manifest.parent/'setup.py').exists():
            assert (manifest.parent/'resource'/name).is_file()
    assert {'arc_course','arc_sensors','arc_mapping','arc_localization','arc_recovery'}<=seen


def test_launches_and_description_templates_parse():
    for path in ROOT.glob('arc_*/launch/*.py'):ast.parse(path.read_text())
    for path in (ROOT/'arc_description/urdf').glob('*.xacro'):ET.parse(path)
    for path in ROOT.glob('arc_*/config/*.yaml'):
        assert yaml.safe_load(path.read_text()) is not None


def test_physical_contract_in_controller_config():
    config=yaml.safe_load((ROOT/'arc_description/config/arc_bot_controllers.yaml').read_text())
    p=config['diff_drive_controller']['ros__parameters']
    from arc_rl.nav_core import ROBOT
    assert p['wheel_radius']==ROBOT.wheel_radius
    assert p['wheel_separation']==ROBOT.wheel_separation
    assert p['linear.x.max_velocity']==ROBOT.max_linear_velocity
    assert p['angular.z.max_velocity']==ROBOT.max_angular_velocity
    assert p['cmd_vel_timeout']==.5
