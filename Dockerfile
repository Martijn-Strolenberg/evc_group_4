FROM osrf/ros:melodic-desktop

# Set user info
ARG USERNAME=ubuntu
ARG USER_UID=1000
ARG USER_GID=1000

# Create user and add to sudo
RUN groupadd --gid $USER_GID $USERNAME && \
    useradd -m -s /bin/bash --uid $USER_UID --gid $USER_GID $USERNAME && \
    apt-get update && \
    apt-get install -y sudo && \
    echo "$USERNAME ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME

# Switch to new user
USER $USERNAME

# Add user to video group
RUN sudo usermod -aG video $USERNAME

# Update system and install tools
RUN sudo apt update && sudo apt upgrade -y && \
    sudo apt install -y git iputils-ping x11-apps sshfs sshpass net-tools \
    netcat openssh-server avahi-daemon libnss-mdns iproute2 tmux vim nano curl

# Setup ROS
RUN sudo rosdep update && \
    echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc

# Copy in project files
ARG HOME_DIR=/home/$USERNAME
# COPY EVC $HOME_DIR/EVC
# COPY talker.py $HOME_DIR/
# COPY listener.py $HOME_DIR/
# RUN sudo chown -R $USERNAME:$USERNAME $HOME_DIR/EVC $HOME_DIR/talker.py $HOME_DIR/listener.py

# Setup environment variables (use host IP if needed)
ARG ROS_MASTER_URI=http://192.168.8.179:11311
ARG CLIENT_IP=192.168.8.249

RUN echo "export ROS_MASTER_URI=$ROS_MASTER_URI" >> ~/.bashrc && \
    echo "export ROS_IP=$CLIENT_IP" >> ~/.bashrc && \
    echo "export DISPLAY=$CLIENT_IP:0" >> ~/.bashrc

WORKDIR $HOME_DIR
CMD ["bash"]
