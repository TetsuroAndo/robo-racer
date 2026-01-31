FROM osrf/ros:humble-desktop

SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y \
    ros-humble-slam-toolbox \
    ros-humble-nav2-map-server \
    ros-humble-rosbag2 \
    python3-colcon-common-extensions \
    python3-pip \
    git \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ws

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
