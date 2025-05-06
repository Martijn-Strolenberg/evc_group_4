#!/bin/bash



# Add ROS repository
echo "Adding ROS repository..."
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'

# Ensure curl is installed
echo "Installing curl..."
sudo apt update
sudo apt install -y curl

# Add ROS GPG key
echo "Adding ROS GPG key..."
curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | sudo apt-key add -

# Update package index and install ROS
echo "Installing ROS Melodic..."
sudo apt update
sudo apt install -y ros-melodic-desktop-full

# Source ROS setup
echo "Sourcing ROS..."
if ! grep -Fxq "source /opt/ros/melodic/setup.bash" ~/.bashrc; then
    echo "source /opt/ros/melodic/setup.bash" >> ~/.bashrc
fi
source ~/.bashrc

# Install ROS dependencies
echo "Installing ROS tools..."
sudo apt install -y python-rosdep python-rosinstall python-rosinstall-generator python-wstool build-essential

# Initialize rosdep
echo "Initializing rosdep..."
sudo rosdep init || echo "rosdep already initialized"
rosdep update

# Install Python packages
echo "Installing pip and OpenCV..."
sudo apt install -y python-pip
pip install --upgrade pip
pip install opencv-python==4.2.0.32

# Install I2C and OpenCV dependencies
echo "Installing I2C and OpenCV dependencies..."
sudo apt install -y i2c-tools python-smbus 
sudo apt install -y build-essential cmake git pkg-config \
                    libgtk2.0-dev libavcodec-dev libavformat-dev \
                    libswscale-dev python-dev python-numpy python2-pip

# Install GStreamer components
echo "Installing GStreamer components..."
sudo apt install -y gstreamer1.0-tools \
                    gstreamer1.0-plugins-base \
                    gstreamer1.0-plugins-good \
                    gstreamer1.0-plugins-bad \
                    libcanberra-gtk-module \
                    libcanberra-gtk3-module

# Install dependencies from requirements file
echo "Installing Python dependencies from testScripts/dependencies.txt..."
python -m pip install -r testScripts/dependencies.txt

echo "✅ ROS setup and dependencies installed successfully!"
