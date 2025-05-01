
FROM osrf/ros: melodic-desktop
ARG USERNAME-ubuntu
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN if id -u $USER_UID >/dev/null 2>&1; then \
fi
groupadd --gid $USER GID $USERNAME && \
useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME; \
Add sudo support for the non-root user
RUN apt-get update && \
apt-get install -y sudo && \
echo "$USERNAME ALL= (root) NOPASSWD: ALL" > /etc/sudoers.d/$USERNAME && \ chmod 0440 /etc/sudoers.d/$USERNAME
Switch from root to user
USER $USERNAME
# Add user to video group to allow access to webcam
RUN sudo usermod --append --groups video $USERNAME
Update all packages
RUN sudo apt update && sudo apt upgrade -y
Install Git and X11 applications (for xeyes) and ping utility
RUN sudo apt install -y git iputils-ping x11-apps sshfs sshpass net-tools \
netcat openssh-server avahi-daemon libnss-mdns iproute2 tmux vim nano curl Rosdep update
RUN rosdep update
#Source the ROS setup file
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
## ADD ANY CUSTOM SETUP BELOW ##
Setup environment variables
Home directory
ARG HOME_DIR=/home/$USERNAME
# Based on DNS configuration: jetbot, jetbot.local, or jetbot.lan ARG ROS_MASTER_URI=http://jetbot:11311
The local device's wlan0 IP. ipconfig (Windows) or ifconfig (Linux) ARG CLIENT_IP=192.168.8.232
Move copied directory from the jetson to the container
COPY EVC $HOME_DIR/EVC
COPY talker.py $HOME_DIR/
COPY listener.py $HOME DIR/
RUN sudo chown -R $USERNAME: $USERNAME $HOME_DIR/EVC
Setup ROS_MASTER_URI
RUN echo "export ROS_MASTER_URI=$ROS_MASTER_URI" >> ~/.bashrc Setup ROS_IP
RUN echo "export ROS_IP=$ (hostname -I | awk '{print $1}')">> ~/.bashrc RUN echo "export ROS_IP=$CLIENT_IP" >> ~/.bashrc
Setup DISPLAY
RUN echo "export DISPLAY=$CLIENT_IP:0" >> ~/.bashrc