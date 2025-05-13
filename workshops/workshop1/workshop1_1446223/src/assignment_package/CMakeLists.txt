cmake_minimum_required(VERSION 3.0.2)
project(assignment_package)

# Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

# Declare a catkin package
catkin_package()

# Include directories
include_directories(
  ${catkin_INCLUDE_DIRS}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

# Install Python scripts
catkin_install_python(PROGRAMS
  src/subscriber_node.py
  src/publisher_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)