FROM osrf/ros:jazzy-desktop-full

RUN apt-get update && \
    apt-get install -y \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-hardware-interface \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-ros-gz-image \
    ros-jazzy-gz-ros2-control \
    ros-jazzy-xacro \
    ros-jazzy-joint-state-publisher-gui \
    ros-jazzy-plotjuggler-ros && \
    libgpiod-dev \
    libi2c-dev && \
    clang-format && \
    rm -rf /var/lib/apt/lists/*

ENV ROS_DOMAIN_ID=42

RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc

CMD ["/bin/bash"]