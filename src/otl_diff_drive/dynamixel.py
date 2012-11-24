#! /usr/bin/env python

import rospy
import roslib
roslib.load_manifest("otl_diff_drive")

from std_msgs.msg import Float64
from dynamixel_msgs.msg import JointState

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


class Subscriber:
    """ Subscribe dynamixel joint state topic and store the velocity

    use ~left_wheel_controller and ~right_wheel_controller param to determin the topic names of dynamixel_controllers.
    currently this uses state.velocity, but it is more better to use current_pos and culcurate velocities."""
    def __init__(self):
        if not rospy.has_param("~left_wheel_controller") or not rospy.has_param("~right_wheel_controller"):
            raise Exception("~left_wheel_controller and ~right_wheel_controller param is required")

        left_name = rospy.get_param("~left_wheel_controller")
        right_name = rospy.get_param("~right_wheel_controller")
        self._left_controller_name = left_name
        self._right_controller_name = right_name

        self._left_velocity = 0.0
        self._right_velocity = 0.0

        self._left_sub = rospy.Subscriber(self._left_controller_name + '/state', JointState, self._on_left_state)
        self._right_sub = rospy.Subscriber(self._right_controller_name + '/state', JointState, self._on_right_state)
        self._left_stamp = rospy.Time.now()
        self._right_stamp = rospy.Time.now()

    def _on_left_state(self, state):
        # try velocity value, is this correct?
        self._left_velocity = state.velocity
        self._left_stamp = state.header.stamp

    def _on_right_state(self, state):
        # try velocity value, is this correct?
        self._right_velocity = state.velocity
        self._right_stamp = state.header.stamp

    def get_stamped_velocity(self):
        # use old time stamp
        if self._right_stamp > self._left_stamp:
            stamp = self._left_stamp
        else:
            stamp = self._right_stamp
        return (stamp, self._left_velocity, self._right_velocity)
