#! /usr/bin/env python

import rospy
import roslib
roslib.load_manifest("otl_diff_drive")

from otl_diff_drive import dynamixel
import base_node
import unittest
import time

from std_msgs.msg import Float64
from geometry_msgs.msg import Twist

class TestBaseNode(unittest.TestCase):
    def check_left(self, msg):
        self.left_data = msg.data
    def check_right(self, msg):
        self.right_data = msg.data

    def setUp(self):
        self.left_data = None
        self.right_data = None

    def test_convert_straight(self):
        rospy.set_param('base_node/wheel_radius', 0.1)
        rospy.set_param('base_node/wheel_offset', 0.5)
        rospy.set_param('base_node/left_wheel_controller', 'left')
        rospy.set_param('base_node/right_wheel_controller', 'right')
        rospy.set_param('base_node/max_translational_acceleration', 10.0)
        rospy.set_param('base_node/max_rotational_acceleration', 30.0)
        node = base_node.BaseNode(dynamixel.Publisher())
        pub = rospy.Publisher('/cmd_vel', Twist)
        left_sub = rospy.Subscriber('/left/command', Float64, self.check_left)
        right_sub = rospy.Subscriber('/right/command', Float64, self.check_right)
        time.sleep(1)
        vel = Twist()
        vel.linear.x = 0.5
        vel.angular.z = 0.8
        pub.publish(vel)
        while not self.left_data or not self.right_data:
            node.publish()
            time.sleep(0.1)
        self.assertAlmostEqual(self.left_data, 1.0)
        self.assertAlmostEqual(self.right_data, 9.0)

if __name__ == '__main__':
    rospy.init_node('base_node')
    unittest.main()
