FROM osrf/ros:jazzy-desktop-full

# Install ROS2 and base dependencies
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
    curl \
    && rm -rf /var/lib/apt/lists/*

# Install CUDA runtime
RUN curl -fsSL https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2404/x86_64/cuda-keyring_1.1-1_all.deb -o cuda-keyring.deb \
    && dpkg -i cuda-keyring.deb \
    && apt-get update \
    && apt-get install -y cuda-cudart-12-6 \
    && rm -rf /var/lib/apt/lists/*

# Setup CUDA library path
RUN echo "/usr/local/cuda-12.6/lib64" > /etc/ld.so.conf.d/cuda.conf \
    && echo "/usr/local/lib" > /etc/ld.so.conf.d/gtsam_points.conf \
    && ldconfig

# Install GLIM via PPA
RUN curl -s https://koide3.github.io/ppa/setup_ppa.sh | bash \
    && apt-get install -y \
    libiridescence-dev \
    libboost-all-dev \
    libglfw3-dev \
    libmetis-dev \
    libgtsam-points-cuda12.6-dev \
    ros-jazzy-glim-cuda12.6 \
    ros-jazzy-glim-ros-cuda12.6 \
    && ldconfig \
    && rm -rf /var/lib/apt/lists/*

RUN mkdir -p /ros2_ws/src
WORKDIR /ros2_ws
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
RUN echo "source /ros2_ws/install/setup.bash 2>/dev/null || true" >> ~/.bashrc
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]

RUN echo '<?xml version="1.0" encoding="UTF-8" ?>\n<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">\n  <transport_descriptors>\n    <transport_descriptor>\n      <transport_id>udp_transport</transport_id>\n      <type>UDPv4</type>\n    </transport_descriptor>\n  </transport_descriptors>\n  <participant profile_name="disable_shm" is_default_profile="true">\n    <rtps>\n      <userTransports>\n        <transport_id>udp_transport</transport_id>\n      </userTransports>\n      <useBuiltinTransports>false</useBuiltinTransports>\n    </rtps>\n  </participant>\n</profiles>' > /ros2_ws/fastdds_no_shm.xml
