#!/bin/bash

# setup ros environment
source "devel/setup.bash"
dos2unix "src/workshop1_pkg/src/sub_node.py"
roslaunch workshop1_pkg sub_launch.launch