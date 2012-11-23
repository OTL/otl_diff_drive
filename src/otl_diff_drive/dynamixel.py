#! /usr/bin/env python

import rospy
import roslib
roslib.load_manifest("otl_diff_drive")

from std_msgs.msg import Float64

class Publisher:
    def __init__(self):
        left_name = rospy.get_param("~left_wheel_controller")
        right_name = rospy.get_param("~right_wheel_controller")
        self._left_controller_name = left_name
        self._right_controller_name = right_name

        self._left_pub = rospy.Publisher(self._left_controller_name + '/command', Float64)
        self._right_pub = rospy.Publisher(self._right_controller_name + '/command', Float64)

    def publish(self, left_rotational_velocity, right_rotational_velocity):
        self._left_pub.publish(Float64(data=left_rotational_velocity))
        self._right_pub.publish(Float64(data=right_rotational_velocity))
