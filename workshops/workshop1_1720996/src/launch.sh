#!/bin/bash

# setup ros environment
source "devel/setup.bash"
dos2unix src/assign1_package/src/assign1_pub.py
dos2unix src/assign1_package/src/assign1_sub.py 
roslaunch assign1_package Assign1_node.launch