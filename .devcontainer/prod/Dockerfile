FROM arm64v8/ros:jazzy-ros-base-noble

RUN apt-get update && \
    apt-get install -y \
    ros-jazzy-ros2-control \
    ros-jazzy-ros2-controllers \
    ros-jazzy-hardware-interface \
    ros-jazzy-xacro \
    libgpiod-dev \
    libi2c-dev && \
    rm -rf /var/lib/apt/lists/*

RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc

ENV ROS_DOMAIN_ID=42
# ENV CYCLONE_DDS_URI='<CycloneDDS><Domain><Id>${ROS_DOMAIN_ID}</Id><General><NetworkInterfaceAddress>AUTO</NetworkInterfaceAddress></General></Domain></CycloneDDS>'

# Add real-time limits to /etc/security/limits.conf
# RUN echo "@realtime soft rtprio 99" | sudo tee -a /etc/security/limits.conf && \
#     echo "@realtime soft priority 99" | sudo tee -a /etc/security/limits.conf && \
#     echo "@realtime soft memlock 102400" | sudo tee -a /etc/security/limits.conf && \
#     echo "@realtime hard rtprio 99" | sudo tee -a /etc/security/limits.conf && \
#     echo "@realtime hard priority 99" | sudo tee -a /etc/security/limits.conf && \
#     echo "@realtime hard memlock 102400" | sudo tee -a /etc/security/limits.conf

CMD ["/bin/bash"]