cmake_minimum_required(VERSION 3.0.2)
project(test_pkgs)

# Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  cv_bridge
  rospy
  sensor_msgs
  std_msgs
  message_generation
  geometry_msgs
  nav_msgs
)
add_message_files(
  FILES
  # our custom messages
)

add_service_files(
  FILES
  # our custom services
  Stop.srv
  MoveStraight.srv
  Rotate.srv
)

add_action_files(
  DIRECTORY action
  FILES CountUntil.action
)

# Declare a catkin package
generate_messages(
  DEPENDENCIES
  sensor_msgs
  std_msgs
  actionlib_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime sensor_msgs std_msgs geometry_msgs nav_msgs actionlib actionlib_msgs
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
  src/count_until_server.py
  src/count_until_client.py
  src/robot_move_API.py
  src/robot_control_script.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)