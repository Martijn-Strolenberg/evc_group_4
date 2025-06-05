cmake_minimum_required(VERSION 3.0.2)
project(mapping)

# Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  sensor_msgs
  std_msgs
  message_generation
)

catkin_python_setup()

# add_message_files(
#   FILES
# )
# Declare a catkin package

add_service_files(
  FILES
  MapTrack.srv
)

generate_messages(
  DEPENDENCIES
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
  src/mapping_node/mapping_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)