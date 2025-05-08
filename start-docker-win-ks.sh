# 0. Open Docker Desktop

# 0.1 Run the Xfile that is on your desktop

# 1. Build docker
docker build -t ros-custom:latest .

# 2. Run docker
docker run --rm -it -p 11311:11311 -p 45100-45101:45100-45101 -v "C:/Users/Karsten/Desktop/evc_group_4:/home/ubuntu/ros_ws" ros-custom:latest

# 3. Build after editing Ros
catkin_make

# 4. source installation files
source devel/setup.bash 

# 5. Run the Python file again
python ./src/jetson_camera/src/camera_subscriber_node.py

# Output yes
export ROS_LOG_DIR=~/output_log.txt 

# Output no
export ROS_LOG_DIR=~/ros_ws/workshops/workshop2_2042347/log

# For mounting to an already live container
docker exec -it admiring_ride /bin/bash

rostopic list

curl https://bootstrap.pypa.io/pip/2.7/get-pip.py -o get-pip.py
python2 get-pip.py
python2 -m pip install pyzbar


# For running the pybar import


