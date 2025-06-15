cmake_minimum_required(VERSION 3.0.2)
project(camera)

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
  ObjectDetection.msg
)
add_service_files(
  FILES
  # our custom services
  ObjectDetected.srv
  ButtonPressed.srv
  CollisionDetection.srv
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
  src/object_tracking.py
  src/marti_linefollower.py
  src/processor_node.py
  src/camera_pub_node.py
  src/camera_viewer_node.py
  src/line_follower.py
  src/line_followerV2.py
  src/Motor_OCR_node.py
  src/line_follower_Luka.py
  src/speed_change_OCR.py
  src/line_followerV4.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)