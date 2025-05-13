FROM ros:melodic

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    curl gnupg2 lsb-release \
    python-pip \
    python3-pip \
    dos2unix \
    python-rosdep python-rosinstall python-rosinstall-generator python-wstool \
    build-essential i2c-tools python-smbus \
    cmake git pkg-config libgtk2.0-dev libavcodec-dev libavformat-dev libswscale-dev \
    gstreamer1.0-tools gstreamer1.0-plugins-base gstreamer1.0-plugins-good \
    gstreamer1.0-plugins-bad libcanberra-gtk-module libcanberra-gtk3-module

# Fix pip version and install OpenCV
RUN pip install --upgrade pip && \
    pip install opencv-python==4.2.0.32

RUN echo "source /opt/ros/melodic/setup.bash" >> /root/.bashrc && \
    [ -f /etc/ros/rosdep/sources.list.d/20-default.list ] || rosdep init && \
    rosdep update


WORKDIR /ros_ws
