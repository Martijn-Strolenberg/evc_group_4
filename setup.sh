#!/bin/bash
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list' 
sudo apt install curl # if you haven't already installed curl 
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add - 
sudo apt update 
sudo apt install ros-melodic-desktop-full 
echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc 
source ~/.bashrc 
sudo apt install python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential 
sudo rosdep init 
rosdep update 
sudo apt install python-pip 
pip install --upgrade pip
pip install opencv-python==4.2.0.32
sudo apt install -y i2c-tools python-smbus 
sudo apt install -y build-essential cmake git pkg-config libgtk2.0-dev libavcodec-dev libavformat-dev libswscale-dev python-dev python-numpy python2-pip 
sudo apt-get install gstreamer1.0-tools  
sudo apt-get install gstreamer1.0-plugins-base  
sudo apt-get install gstreamer1.0-plugins-good  
sudo apt-get install gstreamer1.0-plugins-bad
sudo apt-get install libcanberra-gtk-module 
sudo apt-get install libcanberra-gtk3-module
python -m pip install -r testScripts/dependencies.txt
