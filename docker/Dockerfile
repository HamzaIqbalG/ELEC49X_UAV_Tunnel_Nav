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

RUN python3 -m pip install --no-cache-dir -U pip setuptools

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
    ros-humble-robot-localization \
    ros-humble-as2-gazebo-assets \
    ros-humble-slam-toolbox \
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

WORKDIR /ros2_ws
COPY ros2_ws/src /ros2_ws/src
COPY dashboard /dashboard
COPY docker/entrypoint.sh /ros2_ws/entrypoint.sh
RUN chmod +x /ros2_ws/entrypoint.sh

RUN /bin/bash -c "source /opt/ros/humble/setup.bash && colcon build"

ENTRYPOINT ["/ros2_ws/entrypoint.sh"]
CMD ["bash"]
