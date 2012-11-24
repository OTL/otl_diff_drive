#! /usr/bin/env python

from odometry import Robot
import math
import unittest


class TestRobot(unittest.TestCase):
    def test_init(self):
        r = Robot()
        self.assertAlmostEqual(0.0, r.x)
        self.assertAlmostEqual(0.0, r.y)
        self.assertAlmostEqual(0.0, r.theta)

    def test_init_with_arg(self):
        r = Robot(1.0, 2.0, 3.0)
        self.assertAlmostEqual(1.0, r.x)
        self.assertAlmostEqual(2.0, r.y)
        self.assertAlmostEqual(3.0, r.theta)

    def test_update(self):
        r = Robot()
        r.update_by_velocity(0.1, math.pi / 2, 1.0)
        self.assertAlmostEqual(0.1, r.x)
        self.assertAlmostEqual(0.0, r.y)
        self.assertAlmostEqual(math.pi / 2, r.theta)

        r.update_by_velocity(1.0, 0.0, 0.1)
        self.assertAlmostEqual(0.1, r.x)
        self.assertAlmostEqual(0.1, r.y)
        self.assertAlmostEqual(math.pi / 2, r.theta)

if __name__ == '__main__':
    unittest.main()
