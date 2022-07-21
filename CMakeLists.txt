cmake_minimum_required(VERSION 3.0.2)
project(localization_simulation)


find_package(catkin REQUIRED COMPONENTS
  rospy
  tf
  tf2
  tf2_ros
  nav_msgs 
  geometry_msgs 
  sensor_msgs 
  std_msgs
)

catkin_python_setup()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES practice
  CATKIN_DEPENDS rospy tf tf2 tf2_ros nav_msgs geometry_msgs sensor_msgs std_msgs
#  DEPENDS system_lib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


catkin_install_python(PROGRAMS
  nodes/noisy_odom.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_install_python(PROGRAMS
  nodes/AGV_teleop_key.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
