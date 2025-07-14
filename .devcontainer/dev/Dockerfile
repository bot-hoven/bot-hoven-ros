FROM osrf/ros:humble-desktop-full

RUN apt-get update && \
    apt-get install -y \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-hardware-interface \
    ros-humble-ros-gz-sim \
    ros-humble-ros-gz-bridge \
    ros-humble-ros-gz-image \
    ros-humble-gz-ros2-control \
    ros-humble-joint-state-publisher-gui \
    ros-humble-xacro \
    libgpiod-dev \
    libi2c-dev && \
    rm -rf /var/lib/apt/lists/*

RUN rosdep update

ENV ROS_DOMAIN_ID=17

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc

CMD ["/bin/bash"]