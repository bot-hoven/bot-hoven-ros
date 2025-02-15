FROM arm64v8/ros:jazzy-ros-base-noble

# Add ubuntu user with same UID and GID as your host system, if it doesn't already exist
# Since Ubuntu 24.04, a non-root user is created by default with the name vscode and UID=1000
ARG USERNAME=ubuntu
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN if ! id -u $USER_UID >/dev/null 2>&1; then \
        groupadd --gid $USER_GID $USERNAME && \
        useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME; \
    fi && \
    groupadd --gid 993 gpio && \
    groupadd --gid 994 i2c && \
    groupadd --gid 1001 realtime

# Add sudo support for the non-root user
RUN apt-get update && \
    apt-get install -y \ 
    sudo \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-hardware-interface \
    ros-jazzy-xacro \
    libgpiod-dev \
    libi2c-dev && \
    rm -rf /var/lib/apt/lists/* && \
    echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME && \
    chmod 0440 /etc/sudoers.d/$USERNAME

# Switch from root to user
USER $USERNAME

# Add user to video group to allow access to webcam
RUN sudo usermod --append --groups video $USERNAME

# Update all packages
RUN sudo apt update && sudo apt upgrade -y

# Install Git
RUN sudo apt install -y git

# Rosdep update
RUN rosdep update

# Source the ROS setup file
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc

################################
## ADD ANY CUSTOM SETUP BELOW ##
################################

# Permissions for I2C AND GPIO
RUN sudo usermod -aG i2c $USERNAME && \
    sudo usermod -aG gpio $USERNAME && \
    sudo usermod -aG realtime $USERNAME

# Persist Bash history across sessions
RUN SNIPPET="export PROMPT_COMMAND='history -a' && export HISTFILE=/commandhistory/.bash_history" \
    && sudo mkdir /commandhistory \
    && sudo touch /commandhistory/.bash_history \
    && sudo chown -R $USERNAME /commandhistory \
    && echo "$SNIPPET" | sudo tee -a "/home/$USERNAME/.bashrc"

# Add real-time limits to /etc/security/limits.conf
RUN echo "@realtime soft rtprio 99" | sudo tee -a /etc/security/limits.conf && \
    echo "@realtime soft priority 99" | sudo tee -a /etc/security/limits.conf && \
    echo "@realtime soft memlock 102400" | sudo tee -a /etc/security/limits.conf && \
    echo "@realtime hard rtprio 99" | sudo tee -a /etc/security/limits.conf && \
    echo "@realtime hard priority 99" | sudo tee -a /etc/security/limits.conf && \
    echo "@realtime hard memlock 102400" | sudo tee -a /etc/security/limits.conf