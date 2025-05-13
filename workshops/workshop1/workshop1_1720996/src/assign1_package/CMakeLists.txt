cmake_minimum_required(VERSION 3.0.2)
project(assign1_package)

# Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
)
add_message_files(
  FILES

  test1.msg
)
# Declare a catkin package
generate_messages(
  DEPENDENCIES
  std_msgs
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
  src/assign1_pub.py
  src/assign1_sub.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)