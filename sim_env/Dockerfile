FROM osrf/ros:humble-desktop-full

ENV DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-gazebo-ros-pkgs ros-humble-gazebo-ros2-control \
    ros-humble-navigation2 ros-humble-nav2-bringup \
    ros-humble-slam-toolbox ros-humble-cartographer ros-humble-cartographer-ros \
    ros-humble-robot-localization ros-humble-ros2-control ros-humble-ros2-controllers \
    ros-humble-joint-state-publisher ros-humble-joint-state-publisher-gui \
    ros-humble-xacro ros-humble-teleop-twist-keyboard ros-humble-twist-mux \
    ros-humble-tf2-tools ros-humble-rmw-cyclonedds-cpp \
    python3-colcon-common-extensions python3-rosdep python3-pip git nano \
    mesa-utils x11-apps \
    && rm -rf /var/lib/apt/lists/*

# DOGZILLA 底层库（串口控制）
COPY DOGZILLALib /tmp/DOGZILLALib
RUN pip3 install --no-cache-dir /tmp/DOGZILLALib || true

RUN echo 'source /opt/ros/humble/setup.bash' >> /root/.bashrc \
 && echo '[ -f /ws/install/setup.bash ] && source /ws/install/setup.bash' >> /root/.bashrc \
 && echo 'export GAZEBO_MODEL_PATH=/root/.gazebo/models:$GAZEBO_MODEL_PATH' >> /root/.bashrc
WORKDIR /ws
