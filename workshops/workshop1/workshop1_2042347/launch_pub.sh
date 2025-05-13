#!/bin/bash

# setup ros environment
source "devel/setup.bash"
dos2unix "src/workshop1_pkg/src/pub_node.py"
roslaunch workshop1_pkg pub_launch.launch