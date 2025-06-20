cmake_minimum_required(VERSION 3.0.2)
project(motor_control)

# Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
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
  twovids.msg
  motor_cmd.msg
  encoder.msg
)
# Declare a catkin package
generate_messages(
  DEPENDENCIES
  sensor_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime sensor_msgs std_msgs geometry_msgs nav_msgs
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
  src/martijn_control_node.py
  src/temp_control_node.py
  src/motor_pub_node.py
  src/motor_control_node.py
  src/odometry_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)