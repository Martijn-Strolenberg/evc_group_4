#!/bin/bash

# setup ros environment
source "devel/setup.bash"
dos2unix "src/laptop_camera/src/camera_subscriber_node.py"
roslaunch laptop_camera camera_subscriber.launch