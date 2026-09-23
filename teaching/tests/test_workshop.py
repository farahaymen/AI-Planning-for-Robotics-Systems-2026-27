"""Exercise student maths and API boundaries; ROS launches need the course VM."""
import ast
import importlib.util
import math
from pathlib import Path
import re
import xml.etree.ElementTree as ET

import numpy as np
import pytest

ROOT = Path(__file__).resolve().parents[2]
PACKAGE = ROOT / 'teaching/building/robot_workshop'
MODULES = PACKAGE / 'robot_workshop'


def pure_function(module, name):
    """Run the exact pure function without importing unavailable ROS libraries."""
    tree = ast.parse((MODULES / f'{module}.py').read_text())
    node, = [item for item in tree.body if isinstance(item, ast.FunctionDef) and item.name == name]
    namespace = {'math': math}
    exec(compile(ast.Module(body=[node], type_ignores=[]), str(MODULES / module), 'exec'), namespace)
    return namespace[name]


def load_module(name):
    spec = importlib.util.spec_from_file_location(name, MODULES / f'{name}.py')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def test_manual_listings_are_the_shipped_reference():
    count = 0
    for manual in (ROOT / 'docs/labs').glob('lab*/manual.md'):
        for name, content in re.findall(r'<!-- reference: ([^\n]+) -->\n```\w+\n(.*?)\n```',
                                        manual.read_text(), re.S):
            assert content == (ROOT / name).read_text().rstrip(), (manual, name)
            count += 1
    assert count >= 2  # Full reference listings retained in unchanged Lab 1.


def test_front_sector_uses_angles_not_array_position():
    minimum = pure_function('front_range', 'front_min')
    scan = [4.0] * 360
    scan[0] = 0.2  # Behind the robot with an angle_min of -pi.
    scan[180] = 1.2
    assert minimum(scan, -math.pi, math.pi / 180, 0.12, 12) == 1.2
    # Equivalent 0..2pi ordering must select its first/last beams as forward.
    scan = [4.0] * 360
    scan[359] = 0.7
    assert minimum(scan, 0, math.pi / 180, 0.12, 12) == 0.7


def test_no_valid_front_rays_remains_invalid():
    minimum = pure_function('front_range', 'front_min')
    assert math.isnan(minimum([math.nan, math.inf, 0.01], -0.1, 0.1, 0.12, 12))
    assert minimum([0.12, 12, -1], -0.1, 0.1, 0.12, 12) == 0.12


def test_small_map_preserves_unknown_cells_and_separate_hit():
    ray = pure_function('small_map', 'horizontal_ray')
    grid = ray(20, 10, 4, 2, 8)
    assert len(grid) == 200
    assert grid[82:88] == [0] * 6
    assert grid[88] == 100
    assert grid[:82] + grid[89:] == [-1] * 193
    with pytest.raises(ValueError):
        ray(20, 10, 4, 2, 20)


def test_retry_exhaustion_and_reset():
    module = load_module('progress')
    assert module.displacement((0, 0), (0.3, 0.4)) == pytest.approx(0.5)
    budget = module.RetryBudget(2)
    assert [budget.decide(False) for _ in range(4)] == ['retry', 'retry', 'abort', 'abort']
    assert budget.decide(True) == 'continue'
    assert budget.decide(False) == 'retry'


def test_approach_reset_success_and_timeout():
    env = load_module('approach_env').ApproachEnv()
    first, _ = env.reset(seed=7)
    repeated, _ = env.reset(seed=7)
    np.testing.assert_array_equal(first, repeated)
    for _ in range(50):
        obs, reward, terminated, truncated, info = env.step(np.array([1], np.float32))
        assert env.observation_space.contains(obs)
        if terminated or truncated:
            break
    assert terminated and not truncated and info['x_m'] >= 0.98
    env.reset(seed=7)
    for _ in range(50):
        _, _, terminated, truncated, _ = env.step(np.array([0], np.float32))
    assert truncated and not terminated
    env.close()


def test_approach_gymnasium_contract():
    from gymnasium.utils.env_checker import check_env
    env = load_module('approach_env').ApproachEnv()
    check_env(env, skip_render_check=True)
    env.close()


def test_watchdog_limits_and_exact_freshness_boundary():
    limit = pure_function('command_gate', 'limited_command')
    assert limit(0.8, 2.5, 0.1) == (0.5, 1.8)
    assert limit(-0.4, -3, 0.1) == (-0.125, -1.8)
    assert limit(0.2, 0.1, 0.299) == (0.2, 0.1)
    for age in (0.3, 1, -0.1, math.inf):
        assert limit(0.2, 0.1, age) == (0, 0)
    assert limit(math.nan, 0, 0.1) == (0, 0)


def test_student_urdf_is_a_connected_fixed_tree():
    robot = ET.parse(PACKAGE / 'urdf/student_robot.urdf').getroot()
    links = {link.attrib['name'] for link in robot.findall('link')}
    joint, = robot.findall('joint')
    assert joint.attrib['type'] == 'fixed'
    assert joint.find('parent').attrib['link'] in links
    assert joint.find('child').attrib['link'] in links
    assert joint.find('origin').attrib['xyz'] == '0.10 0 0.12'
