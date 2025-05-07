#!/bin/bash

# Setup ROS environment
source "devel/setup.bash"

# Ensure scripts are in Unix format
dos2unix "src/assignment_2/src/movie_publisher_node.py"
dos2unix "src/assignment_2/src/camera_subscriber_node.py"

roscore &
export ROS_MASTER_URI=http://172.17.0.2:11311
export ROS_IP=172.17.0.2
export DISPLAY=172.17.0.2:0

# Give it time to initialize (optional)
sleep 2

# Launch movie publisher in the background
roslaunch assignment_2 movie_publisher.launch &



# Give it time to initialize (optional)
sleep 2

# Launch camera subscriber
roslaunch assignment_2 camera_subscriber.launch
