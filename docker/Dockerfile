FROM osrf/ros:humble-desktop

ENV DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    curl \
    git \
    gnupg \
    lsb-release \
    python3-colcon-common-extensions \
    python3-vcstool \
    && rm -rf /var/lib/apt/lists/*

# Gazebo Fortress repository (OSRF)
RUN curl -sSL https://packages.osrfoundation.org/gazebo.gpg \
      -o /usr/share/keyrings/gazebo-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/gazebo-archive-keyring.gpg] \
http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
      > /etc/apt/sources.list.d/gazebo-stable.list

RUN apt-get update && apt-get install -y \
    gz-fortress \
    libgz-sim7-dev \
    ros-humble-ros-gz \
    ros-humble-as2-gazebo-assets \
    python3-matplotlib \
    python3-numpy \
    python3-tk \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ros2_ws
COPY ros2_ws/src /ros2_ws/src
COPY dashboard /dashboard
COPY docker/entrypoint.sh /ros2_ws/entrypoint.sh
RUN chmod +x /ros2_ws/entrypoint.sh

RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build --symlink-install"

ENTRYPOINT ["/ros2_ws/entrypoint.sh"]
CMD ["bash"]
