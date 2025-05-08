 
curl https://bootstrap.pypa.io/pip/2.7/get-pip.py -o get-pip.py
python2 get-pip.py
python2 -m pip install pyzbar

# Output yes
export ROS_LOG_DIR=~/output_log.txt 


catkin_make

# 4. source installation files
source devel/setup.bash

# 5. Run the Python file again
# python ./src/jetson_camera/src/qr_detection_node.py






# For running the pybar import


