# 0. Open Docker Desktop

# 0.1 Run the Xfile that is on your desktop

# 1. Build docker
docker build -t ros-custom:latest .

# 2. Run docker
docker run --rm -it -p 11311:11311 -p 45100-45101:45100-45101 -v "C:/Users/Karsten/Desktop/evc_group_4:/home/ubuntu/ros_ws" ros-custom:latest

# 3. Run the Python file again
python src/jetson_camera/src/camera_subscriber_node.py

# Output yes
export ROS_LOG_DIR=~/output_log.txt 

# Output no
export ROS_LOG_DIR=~/ros_ws/workshops/workshop2_2042347/log





