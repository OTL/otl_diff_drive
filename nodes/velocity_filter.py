#! /usr/bin/env python

import rospy
import roslib
roslib.load_manifest("otl_diff_drive")

from otl_diff_drive import twist_velocities

from geometry_msgs.msg import Twist

def isStopVelocity(twist):
    VERY_SMALL = 0.0001
    return abs(twist.linear.x) < VERY_SMALL and abs(twist.angular.z) < VERY_SMALL

class VelocityFilter:
    def __init__(self):
        self._max_linear_velocity = rospy.get_param("~max_translational_velocity", 1.0)
        self._max_angular_velocity = rospy.get_param("~max_rotational_velocity", 3.0)
        self._velocity_filter = twist_velocities.VelocityFilter(self._max_linear_velocity, self._max_angular_velocity)

        self._max_linear_accel = rospy.get_param("~max_translational_acceleration", 1.0)
        self._max_angular_accel = rospy.get_param("~max_rotational_acceleration", 3.0)
        self._accel_filter = twist_velocities.AccelFilter(self._max_linear_accel, self._max_angular_accel)

        self._output_pub = rospy.Publisher('/output_vel', Twist)
        self._command_sub = rospy.Subscriber("/cmd_vel", Twist, self.on_twist_command)
        self._current_velocity = None
        self._output_velocity = Twist()
        self._last_command_stamp = None

    def on_twist_command(self, command):
        self._current_velocity = command
        self.publish()
        self._last_command_stamp = rospy.Time.now()

    def get_elapsed_sec(self):
        return (rospy.Time.now() - self._last_command_stamp).to_sec() 

    def publish(self):
        if self._current_velocity:
            limited_x, limited_theta = self._velocity_filter.filter(self._current_velocity.linear.x, self._current_velocity.angular.z)
            if self._last_command_stamp:
                duration = self.get_elapsed_sec()
            else:
                duration = 0.1
            if duration > 0.1:
                duration = 0.1
            x, theta = self._accel_filter.filter(limited_x, limited_theta, duration)
            self._output_velocity.linear.x = x
            self._output_velocity.angular.z = theta
            self._output_pub.publish(self._output_velocity)

    def main(self):
        r = rospy.Rate(10)
        TIME_FOR_STOP = 5.0
        while not rospy.is_shutdown():
            if self._last_command_stamp:
                # recently updated
                if self.get_elapsed_sec() < TIME_FOR_STOP:
                    # target is stop
                    if isStopVelocity(self._current_velocity):
                        # output is not stop velocity
                        if not isStopVelocity(self._output_velocity):
                            # then repeat publish
                            self.publish()
                r.sleep()


if __name__ == '__main__':
    rospy.init_node('velocity_filter')
    node = VelocityFilter()
    node.main()
