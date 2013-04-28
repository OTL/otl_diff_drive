cmake_minimum_required(VERSION 2.8.3)
project(otl_diff_drive)
find_package(catkin REQUIRED)

catkin_python_setup()

catkin_package(
    CATKIN_DEPENDS dynamixel_msgs std_msgs rospy geometry_msgs nav_msgs tf
)

install(PROGRAMS
  nodes/base_node.py
  nodes/base_odom.py
  nodes/velocity_filter.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  )
