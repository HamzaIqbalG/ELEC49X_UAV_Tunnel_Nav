FROM osrf/ros:humble-desktop

ENV DEBIAN_FRONTEND=noninteractive

# General dev / ROS workflow tooling (no ArduPilot/PX4 stack — see below)
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    curl \
    git \
    gnupg \
    lsb-release \
    python3-colcon-common-extensions \
    python3-pip \
    python3-rosdep \
    python3-vcstool \
    socat \
    sudo \
    unzip \
    && rm -rf /var/lib/apt/lists/*

RUN if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then rosdep init; fi

# Gazebo Fortress repository (OSRF)
RUN curl -sSL https://packages.osrfoundation.org/gazebo.gpg \
      -o /usr/share/keyrings/gazebo-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/gazebo-archive-keyring.gpg] \
http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
      > /etc/apt/sources.list.d/gazebo-stable.list

# Core simulation packages (Gazebo Fortress + AS2; no ArduPilot SITL / PX4)
RUN apt-get update && apt-get install -y \
    gz-fortress \
    libgz-sim7-dev \
    ros-humble-ros-gz \
    ros-humble-as2-gazebo-assets \
    python3-matplotlib \
    python3-numpy \
    python3-tk \
    && rm -rf /var/lib/apt/lists/*

# Extra ROS packages for the tunnel demo (change here without rebuilding Gazebo layers above)
RUN apt-get update && apt-get install -y \
    ros-humble-robot-localization \
    ros-humble-slam-toolbox \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ros2_ws
COPY ros2_ws/src /ros2_ws/src
COPY dashboard /dashboard
COPY docker/entrypoint.sh /ros2_ws/entrypoint.sh
RUN chmod +x /ros2_ws/entrypoint.sh

RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build"

ENTRYPOINT ["/ros2_ws/entrypoint.sh"]
CMD ["bash"]
