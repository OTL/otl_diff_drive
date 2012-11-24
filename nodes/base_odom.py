#! /usr/bin/env python

import rospy
import roslib
roslib.load_manifest("otl_diff_drive")

import tf

from otl_diff_drive import twist_velocities
from otl_diff_drive import odometry

from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler

class Pose:
    def __init__(self, x, y, theta):
        self.x = x
        self.y = y
        self.theta = theta

class BaseOdom:
    def __init__(self, wheel_subscriber):
        self._rate = rospy.get_param("~rate", 10.0)
        assert self._rate > 0.0
        self._base_frame_id = rospy.get_param("~base_frame_id", 'base_link')
        self._odom_frame_id = rospy.get_param("~odom_frame_id", 'odom')
        self._br = tf.TransformBroadcaster()
        self._odom_pub = rospy.Publisher('odom', Odometry)
        self._robot = odometry.Robot()

        if not rospy.has_param("~wheel_offset"):
            raise Exception("~wheel_offset param is required")
        offset = rospy.get_param("~wheel_offset")
        if not rospy.has_param("~wheel_radius"):
            raise Exception("~wheel_radius param is required")
        radius = rospy.get_param("~wheel_radius")

        self._wheel_sub = wheel_subscriber

        self._twist_converter = twist_velocities.Converter(radius, offset)

    def create_odometry(self, stamp, robot, velocity_x, velocity_theta):
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self._odom_frame_id
        q = quaternion_from_euler(0, 0, robot.theta)
        #set the position
        odom.pose.pose.position.x = robot.x
        odom.pose.pose.position.y = robot.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]

        #set the velocity
        odom.child_frame_id = self._base_frame_id
        odom.twist.twist.linear.x = velocity_x
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.linear.z = 0.0
        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = velocity_theta
        return odom

    def update(self):
        stamp, l_vel, r_vel = self._wheel_sub.get_stamped_velocity()
        vel_x, vel_theta = self._twist_converter.wheels_to_twist(l_vel, r_vel)
        self._robot.update_by_velocity(vel_x, vel_theta, 1.0 / self._rate)

        q = quaternion_from_euler(0, 0, self._robot.theta)
        self._br.sendTransform((self._robot.x, self._robot.y, 0.0), q,
                               stamp,
                               self._base_frame_id,
                               self._odom_frame_id)

        odom = self.create_odometry(stamp, self._robot, vel_x, vel_theta)
        self._odom_pub.publish(odom)

    def main(self):
        r = rospy.Rate(self._rate)
        while not rospy.is_shutdown():
            self.update()
            r.sleep()

if __name__ == '__main__':
    from otl_diff_drive import dynamixel
    rospy.init_node('base_odom')
    node = BaseOdom(dynamixel.Subscriber())
    node.main()
