cmake_minimum_required(VERSION 3.0.2)
project(motor_control)

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

catkin_python_setup()

add_message_files(
  FILES
  # our custom messages
  twovids.msg
  motor_cmd.msg
  encoder.msg
)

add_service_files(
  FILES
  # our custom services
  Stop.srv
  MoveStraight.srv
  Rotate.srv
  LeftWheelDir.srv
  RightWheelDir.srv
  ConstRotate.srv
  ConstStraight.srv
  DriveLeftwheel.srv
  DriveRightwheel.srv
  SetMaxSpeed.srv
)

# add_action_files(
#   DIRECTORY action
#   FILES CountUntil.action
# )

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
  src/motor_control/motor_control_node_det.py
  src/motor_control/odometry_node.py
  src/motor_control/motor_control_node.py
  src/motor_control/robot_control_script.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)