FROM osrf/ros:jazzy-desktop-full

RUN apt-get update && apt-get install -y \
    ros-jazzy-nav2-bringup \
    ros-jazzy-slam-toolbox \
    ros-jazzy-navigation2 \
    ros-jazzy-gz-ros2-control \
    ros-jazzy-ros-gz \
    python3-colcon-common-extensions \
    python3-pip \
    libgtsam-dev \
    libomp-dev \
    && rm -rf /var/lib/apt/lists/*

RUN mkdir -p /ros2_ws/src
WORKDIR /ros2_ws

RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
RUN echo "source /ros2_ws/install/setup.bash 2>/dev/null || true" >> ~/.bashrc

COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
