cmake_minimum_required(VERSION 3.0.2)
project(assignment_3_ocr)

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
  vids.msg
  twovids.msg
  motor_cmd.msg
)
# Declare a catkin package 
generate_messages(
  DEPENDENCIES
  sensor_msgs
  std_msgs
)
# Declare a catkin package
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
  src/movie_publisher.py
  src/movie_processor_subscriber.py
  src/movie_processor.py
  src/OCR_node.py
  src/Motor_OCR_node.py
  src/R_Motor_OCR.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)