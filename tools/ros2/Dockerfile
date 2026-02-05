FROM ros:humble-ros-base

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y \
    ros-humble-rviz2 \
    ros-humble-slam-toolbox \
    ros-humble-nav2-map-server \
    ros-humble-rosbag2 \
    fluxbox \
    novnc \
    python3-colcon-common-extensions \
    python3-pip \
    websockify \
    x11vnc \
    xauth \
    xvfb \
    git \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ws

COPY tools/ros2/scripts/ros2_entrypoint.sh /usr/local/bin/ros2_entrypoint.sh
RUN chmod +x /usr/local/bin/ros2_entrypoint.sh

ENTRYPOINT ["/usr/local/bin/ros2_entrypoint.sh"]
CMD ["bash"]
