"""Check the Lab 6 handoff without a ROS or Gazebo installation."""

import math
from pathlib import Path
import unittest
from xml.etree import ElementTree

import numpy as np

from teaching.tabular import GridRobot, learn
from teaching.tabular_motion import GridRun


class TabularDemoTests(unittest.TestCase):
    def test_view_observes_the_update_without_changing_learning(self):
        env = GridRobot()
        expected_q, expected_returns = learn(env, episodes=20, seed=5)
        seen = []

        def observe(q, step):
            if not seen:
                seen.append(step)
                expected = step.old_value + .2 * (step.target - step.old_value)
                self.assertAlmostEqual(q[step.state, step.action], expected)
                self.assertEqual(step.episode, 1)
                self.assertEqual(step.step, 1)

        q, returns = learn(env, episodes=20, seed=5, observe=observe)
        self.assertTrue(seen)
        np.testing.assert_array_equal(q, expected_q)
        np.testing.assert_array_equal(returns, expected_returns)

    def test_learned_policy_drives_to_goal_in_an_ideal_motion_model(self):
        env = GridRobot()
        q, _ = learn(env, episodes=3000, seed=0)
        run = GridRun(q)
        x = y = yaw = 0.
        dt = .05
        for _ in range(5000):
            forward, turn = run.command(x, y, yaw)
            self.assertLessEqual(forward, .25)
            self.assertLessEqual(abs(turn), .9)
            x += dt * forward * math.cos(yaw)
            y += dt * forward * math.sin(yaw)
            yaw += dt * turn
            if run.done:
                break
        self.assertTrue(run.done)
        self.assertLessEqual(math.hypot(x - 5., y - 5.), .05)
        self.assertEqual(run.command(x, y, yaw), (0., 0.))

    def test_gazebo_divider_matches_the_training_wall(self):
        root = Path(__file__).resolve().parents[2]
        world = ElementTree.parse(root / 'arc_gazebo/worlds/lab06_grid.sdf')
        wall = world.find(".//collision[@name='divider']")
        x, y, *_ = map(float, wall.findtext('pose').split())
        width, length, _ = map(float, wall.findtext('geometry/box/size').split())
        wall_cells = np.asarray([
            [abs(1 + c - x) < width / 2 and abs(1 + r - y) < length / 2 for c in range(6)]
            for r in range(6)
        ])
        np.testing.assert_array_equal(wall_cells, GridRobot().grid)

    def test_refuses_a_policy_that_drives_into_a_wall(self):
        q = np.zeros((36, 4))
        q[0, 2] = 1.
        with self.assertRaisesRegex(RuntimeError, 'blocked move'):
            GridRun(q).command(0., 0., 0.)
