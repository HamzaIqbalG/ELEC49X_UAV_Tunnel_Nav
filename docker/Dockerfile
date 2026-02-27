FROM osrf/ros:humble-desktop

ENV DEBIAN_FRONTEND=noninteractive
ENV ARDU_WS=/ardu_ws
ENV ARDUPILOT_HOME=${ARDU_WS}/src/ardupilot
ENV PATH=${ARDUPILOT_HOME}/Tools/autotest:/home/ardupilot/.local/bin:${PATH}

RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    curl \
    git \
    gnupg \
    openjdk-17-jdk-headless \
    lsb-release \
    python3-colcon-common-extensions \
    python3-pip \
    python3-rosdep \
    python3-vcstool \
    socat \
    sudo \
    unzip \
    && rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install --no-cache-dir -U pip setuptools pymavlink

# Gazebo repository (OSRF) – provides both Fortress and Garden packages
RUN curl -sSL https://packages.osrfoundation.org/gazebo.gpg \
      -o /usr/share/keyrings/gazebo-archive-keyring.gpg && \
    echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/gazebo-archive-keyring.gpg] \
http://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
      > /etc/apt/sources.list.d/gazebo-stable.list

# Core simulation packages.
# gz-fortress + libgz-sim7-dev gives us both Fortress runtime and Garden
# dev headers. ros-humble-ros-gz (Fortress) satisfies rosdep; we overlay
# it later with a Garden source build.
RUN apt-get update && apt-get install -y \
    gz-fortress \
    libgz-sim7-dev \
    ros-humble-ros-gz \
    ros-humble-as2-gazebo-assets \
    python3-matplotlib \
    python3-numpy \
    python3-tk \
    && rm -rf /var/lib/apt/lists/*

RUN if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then rosdep init; fi

RUN useradd -m -s /bin/bash ardupilot && \
    echo "ardupilot ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/ardupilot && \
    chmod 0440 /etc/sudoers.d/ardupilot && \
    mkdir -p ${ARDU_WS}/src && \
    chown -R ardupilot:ardupilot ${ARDU_WS}

USER ardupilot
ENV USER=ardupilot
ENV HOME=/home/ardupilot
WORKDIR ${ARDU_WS}/src
RUN curl -sSL https://raw.githubusercontent.com/ArduPilot/ardupilot/master/Tools/ros2/ros2.repos \
      -o ros2.repos && \
    vcs import --recursive < ros2.repos
RUN git clone https://github.com/ArduPilot/Micro-XRCE-DDS-Gen.git ${ARDU_WS}/src/Micro-XRCE-DDS-Gen && \
    /bin/bash -c "cd ${ARDU_WS}/src/Micro-XRCE-DDS-Gen && \
    ./gradlew jar && \
    ./gradlew install --install_path /home/ardupilot/.local && \
    if [ ! -f /home/ardupilot/.local/share/microxrceddsgen/java/microxrceddsgen.jar ]; then \
      mkdir -p /home/ardupilot/.local/share/microxrceddsgen/java && \
      cp -f share/microxrcedds/microxrceddsgen.jar /home/ardupilot/.local/share/microxrceddsgen/java/microxrceddsgen.jar; \
    fi"
RUN /bin/bash -c "cd ${ARDUPILOT_HOME} && DO_AP_STM_ENV=0 Tools/environment_install/install-prereqs-ubuntu.sh -y"
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    sudo apt-get update && \
    cd ${ARDU_WS} && \
    rosdep update && \
    rosdep install --rosdistro ${ROS_DISTRO} --from-paths src --ignore-src --skip-keys \"micro_ros_agent ardupilot_msgs\" -y && \
    sudo rm -rf /var/lib/apt/lists/*"
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    cd ${ARDU_WS} && \
    colcon build --cmake-args -DBUILD_TESTING=ON"

USER root

# Optional ROS packages and ArduPilot-Gazebo plugin build deps.
# Changing this layer does NOT rebuild ArduPilot (cache preserved).
RUN apt-get update && apt-get install -y \
    ros-humble-robot-localization \
    ros-humble-slam-toolbox \
    ros-humble-topic-tools \
    rapidjson-dev \
    libopencv-dev \
    libgstreamer1.0-dev \
    libgstreamer-plugins-base1.0-dev \
    gstreamer1.0-plugins-bad \
    gstreamer1.0-libav \
    gstreamer1.0-gl \
    && rm -rf /var/lib/apt/lists/*

# ── Gazebo Garden workspace ─────────────────────────────────────────
# Builds ros_gz bridge + ardupilot_gazebo against Garden (gz-sim7).
# This overlay takes precedence over the apt-installed Fortress ros_gz.
ENV GZ_WS=/gz_ws
ENV GZ_VERSION=garden
RUN mkdir -p ${GZ_WS}/src
WORKDIR ${GZ_WS}/src

RUN git clone -b humble https://github.com/gazebosim/ros_gz.git

# Clone ardupilot_gazebo and replace CMakeLists to build only
# ArduPilotPlugin (the only plugin the iris_tunnel model needs).
RUN git clone https://github.com/ArduPilot/ardupilot_gazebo.git && \
    cat > ardupilot_gazebo/CMakeLists.txt << 'CMAKEOF'
cmake_minimum_required(VERSION 3.10.2 FATAL_ERROR)
project(ardupilot_gazebo)

find_package(ament_cmake QUIET)
if(${ament_cmake_FOUND})
  message("Building ${PROJECT_NAME} as an ament_cmake project.")
endif()

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(gz-cmake3 REQUIRED)
set(GZ_CMAKE_VER ${gz-cmake3_VERSION_MAJOR})
gz_find_package(gz-common5 REQUIRED)
set(GZ_COMMON_VER ${gz-common5_VERSION_MAJOR})
gz_find_package(gz-rendering7 REQUIRED)
set(GZ_RENDERING_VER ${gz-rendering7_VERSION_MAJOR})
gz_find_package(gz-sim7 REQUIRED)
set(GZ_SIM_VER ${gz-sim7_VERSION_MAJOR})
message(STATUS "Compiling against Gazebo Garden")

find_package(RapidJSON REQUIRED)

add_library(ArduPilotPlugin SHARED
  src/ArduPilotPlugin.cc src/SocketUDP.cc src/Util.cc)
target_include_directories(ArduPilotPlugin PRIVATE include)
target_link_libraries(ArduPilotPlugin PRIVATE
  gz-sim${GZ_SIM_VER}::gz-sim${GZ_SIM_VER})

install(TARGETS ArduPilotPlugin DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY models/ DESTINATION share/${PROJECT_NAME}/models)
install(DIRECTORY worlds/ DESTINATION share/${PROJECT_NAME}/worlds)

if(${ament_cmake_FOUND})
  ament_environment_hooks(
    "${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.dsv.in")
  ament_environment_hooks(
    "${CMAKE_CURRENT_SOURCE_DIR}/hooks/${PROJECT_NAME}.sh.in")
  ament_package()
endif()
CMAKEOF

# Build with constrained parallelism to prevent OOM on resource-limited
# hosts (e.g. WSL2 with default memory limits).
RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    source ${ARDU_WS}/install/setup.bash && \
    export MAKEFLAGS='-j2' && \
    cd ${GZ_WS} && \
    colcon build \
      --packages-up-to ros_gz_bridge ros_gz_sim ros_gz_interfaces ardupilot_gazebo \
      --parallel-workers 1 \
      --cmake-args -DBUILD_TESTING=OFF"

# ── Project workspace ───────────────────────────────────────────────
WORKDIR /ros2_ws
COPY ros2_ws/src /ros2_ws/src
COPY dashboard /dashboard

RUN /bin/bash -c "source /opt/ros/humble/setup.bash && \
    source ${ARDU_WS}/install/setup.bash && \
    source ${GZ_WS}/install/setup.bash && \
    colcon build"

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
