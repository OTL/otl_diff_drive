#! /usr/bin/env python

import twist_velocities
from twist_velocities import calc_filtered_velocity
from twist_velocities import limit_abs
import unittest

class TestTwistVelocityConverter(unittest.TestCase):
    def test_convert_straight(self):
        converter = twist_velocities.Converter(0.1, 0.5)
        l, r = converter.convert(0.2, 0.0)
        self.assertAlmostEqual(l, 2.0)
        self.assertAlmostEqual(r, 2.0)

    def test_convert_rotate(self):
        converter = twist_velocities.Converter(0.1, 0.5)
        l, r = converter.convert(0.0, 0.8)
        self.assertAlmostEqual(l, -4.0)
        self.assertAlmostEqual(r,  4.0)

    def test_direction_convert_rotate(self):
        converter = twist_velocities.Converter(0.1, 0.5, right_direction=-1)
        l, r = converter.convert(0.0, 0.8)
        self.assertAlmostEqual(l, -4.0)
        self.assertAlmostEqual(r, -4.0)

    def test_direction_convert_straight(self):
        converter = twist_velocities.Converter(0.1, 0.5, left_direction=-1)
        l, r = converter.convert(0.0, 0.8)
        self.assertAlmostEqual(l,  4.0)
        self.assertAlmostEqual(r,  4.0)

    def test_convert_arc(self):
        converter = twist_velocities.Converter(0.1, 0.5)
        l, r = converter.convert(0.5, 0.8)
        self.assertAlmostEqual(l, 1.0)
        self.assertAlmostEqual(r, 9.0)

class TestAccelFilter(unittest.TestCase):
    def setUp(self):
        self.filter = twist_velocities.AccelFilter(0.1, 0.2)

    def test_calc_filtered_velocity(self):
        self.assertAlmostEqual(calc_filtered_velocity(1.0, 0.0, 0.1, 1.0), 0.1)
        self.assertAlmostEqual(calc_filtered_velocity(2.0, 1.0, 0.2, 1.0), 1.2)
        self.assertAlmostEqual(calc_filtered_velocity(0.0, 2.0, 0.5, 0.1), 1.95)
        self.assertAlmostEqual(calc_filtered_velocity(-1.0, 0.0, 2.0, 0.1), -0.2)
    def test_filter(self):
        x, theta = self.filter.filter(0.1, 0.5, 1.0)
        self.assertAlmostEqual(x, 0.1)
        self.assertAlmostEqual(theta, 0.2)
        x, theta = self.filter.filter(0.1, 0.5, 0.1)
        self.assertAlmostEqual(x, 0.1)
        self.assertAlmostEqual(theta, 0.22)
        x, theta = self.filter.filter(0.0, 0.5, 0.1)
        self.assertAlmostEqual(x, 0.09)
        self.assertAlmostEqual(theta, 0.24)
        x, theta = self.filter.filter(0.0, 0.25, 0.1)
        self.assertAlmostEqual(x, 0.08)
        self.assertAlmostEqual(theta, 0.25)

class TestVelocityFilter(unittest.TestCase):
    def test_limit_abs(self):
        self.assertAlmostEqual(0.1, limit_abs(10, 0.1))
        self.assertAlmostEqual(-0.1, limit_abs(-10, 0.1))
        self.assertAlmostEqual(-1.0, limit_abs(-1.0, 10))
        self.assertAlmostEqual(1, limit_abs(1.0, 10))

    def test_filter(self):
        f = twist_velocities.VelocityFilter(0.1, 0.2)
        x, theta = f.filter(1.0, -2.0)
        self.assertAlmostEqual(0.1, x)
        self.assertAlmostEqual(-0.2, theta)
        x, theta = f.filter(0.05, -0.1)
        self.assertAlmostEqual(0.05, x)
        self.assertAlmostEqual(-0.1, theta)
        x, theta = f.filter(-0.05, 0.1)
        self.assertAlmostEqual(-0.05, x)
        self.assertAlmostEqual(0.1, theta)

if __name__ == '__main__':
    unittest.main()
