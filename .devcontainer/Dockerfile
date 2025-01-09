# For development, it uses "osrf/ros:jazzy-desktop-full".
# For production (Raspberry Pi 5), it uses "arm64v8/ros:jazzy-ros-base-noble".
ARG BOTHOVEN_BASE_IMAGE=arm64v8/ros:jazzy-ros-base-noble
# Specify the environment: "production" or "development"
ARG BOTHOVEN_ENVIRONMENT=production
FROM ${BOTHOVEN_BASE_IMAGE}

# Add ubuntu user with same UID and GID as your host system, if it doesn't already exist
# Since Ubuntu 24.04, a non-root user is created by default with the name vscode and UID=1000
ARG USERNAME=ubuntu
ARG USER_UID=1000
ARG USER_GID=$USER_UID
RUN if ! id -u $USER_UID >/dev/null 2>&1; then \
        groupadd --gid $USER_GID $USERNAME && \
        useradd -s /bin/bash --uid $USER_UID --gid $USER_GID -m $USERNAME; \
    fi
# Add sudo support for the non-root user
RUN apt-get update && \
    apt-get install -y \ 
    sudo \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-hardware-interface \
    ros-jazzy-gz-ros2-control \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-xacro && \
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

# Persist Bash history across sessions
RUN SNIPPET="export PROMPT_COMMAND='history -a' && export HISTFILE=/commandhistory/.bash_history" \
    && sudo mkdir /commandhistory \
    && sudo touch /commandhistory/.bash_history \
    && sudo chown -R $USERNAME /commandhistory \
    && echo "$SNIPPET" | sudo tee -a "/home/$USERNAME/.bashrc"