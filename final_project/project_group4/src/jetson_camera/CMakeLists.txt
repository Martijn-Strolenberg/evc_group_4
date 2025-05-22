cmake_minimum_required(VERSION 3.0.2)
project(jetson_camera)

# Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  rospy
  sensor_msgs
  std_msgs
  message_generation
)
add_message_files(
  FILES
  twovids.msg
  test.msg
  motor_cmd.msg
)
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
  src/camera_processor_node.py
  src/camera_publisher_node.py
  src/camera_viewer_node.py
  src/marti_object_tracking.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)