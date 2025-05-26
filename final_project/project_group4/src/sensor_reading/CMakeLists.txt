cmake_minimum_required(VERSION 3.0.2)
project(sensor_reading)

# Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  std_msgs
)
# add_message_files(
#   FILES
# )
# Declare a catkin package
generate_messages(
  DEPENDENCIES
  sensor_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime sensor_msgs std_msgs 
)

# Include directories
include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

# Install Python scripts
catkin_install_python(PROGRAMS
  src/display_node.py
  src/battery_pub_node.py
  src/tof_pub_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)