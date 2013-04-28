#! /usr/bin/env python

import rospy

from otl_diff_drive import twist_velocities

from geometry_msgs.msg import Twist

class BaseNode:
    def __init__(self, wheel_publisher):
        self._rate = rospy.get_param("~rate", 10.0)
        assert self._rate > 0.0
        self._time_out = rospy.get_param("~time_out", 1.0)
        offset = rospy.get_param("~wheel_offset")
        assert offset, "you have to set ~wheel_offset param"
        radius = rospy.get_param("~wheel_radius")
        assert radius, "you have to set ~wheel_radius param"

        self._max_linear_velocity = rospy.get_param("~max_translational_velocity", 1.0)
        self._max_angular_velocity = rospy.get_param("~max_rotational_velocity", 3.0)
        self._velocity_filter = twist_velocities.VelocityFilter(self._max_linear_velocity, self._max_angular_velocity)

        self._max_linear_accel = rospy.get_param("~max_translational_acceleration", 1.0)
        self._max_angular_accel = rospy.get_param("~max_rotational_acceleration", 3.0)
        self._accel_filter = twist_velocities.AccelFilter(self._max_linear_accel, self._max_angular_accel)

        # stop until velocity is zero
        self._stop_time_out = max(self._max_linear_velocity / self._max_linear_accel, self._max_angular_velocity / self._max_angular_accel) + (1.0 / self._rate) * 2.0

        self._wheel_pub = wheel_publisher

        self._twist_converter = twist_velocities.Converter(radius, offset)

        self._command_sub = rospy.Subscriber("/cmd_vel", Twist, self.on_twist_command)
        self._current_velocity = None
        self._last_command_stamp = None

    def on_twist_command(self, command):
        self._current_velocity = command
        self._last_command_stamp = rospy.Time.now()

    def publish(self):
        if self._current_velocity:
            limited_x, limited_theta = self._velocity_filter.filter(self._current_velocity.linear.x, self._current_velocity.angular.z)
            x, theta = self._accel_filter.filter(limited_x, limited_theta, 1.0 / self._rate)
            l, r = self._twist_converter.twist_to_wheels(x, theta)
            self._wheel_pub.publish(l, r)

    def stop(self):
        self._current_velocity = Twist()

    def main(self):
        r = rospy.Rate(self._rate)
        while not rospy.is_shutdown():
            if self._last_command_stamp:
                elapsed_sec = (rospy.Time.now() - self._last_command_stamp).to_sec()
                if elapsed_sec < self._time_out:
                    self.publish()
                elif elapsed_sec < self._time_out + self._stop_time_out:
                    self.stop()
                    self.publish()
                else:
                    pass
            r.sleep()

if __name__ == '__main__':
    rospy.init_node('base_node')
    from otl_diff_drive import dynamixel
    node = BaseNode(dynamixel.Publisher())
    node.main()
