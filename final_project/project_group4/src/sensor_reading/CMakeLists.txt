cmake_minimum_required(VERSION 3.0.2)
project(sensor_reading)

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
  # our custom services
  BatteryInfo.srv
  CollisionDetection.srv
  ButtonPressed.srv
)

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
  src/sensor_reading/battery_test.py
  src/sensor_reading/battery_monitor_service.py
  src/sensor_reading/display_node.py
  src/sensor_reading/battery_pub_node.py
  src/sensor_reading/tof_pub_node.py
  src/sensor_reading/imu_pub_node.py
  src/sensor_reading/collision_detection_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)