FROM ros:humble-perception

ENV WS_DIR="/ros2_ws"
WORKDIR ${WS_DIR}

SHELL ["/bin/bash", "-c"]

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update \
 && apt-get install -y \
    build-essential \
    cmake \
    git-all \
    software-properties-common \
 && rm -rf /var/lib/apt/lists/*


RUN apt-get update \
 && apt-get install -y \
    ros-${ROS_DISTRO}-rviz2 \
 && rm -rf /var/lib/apt/lists/*

ARG DEBIAN_FRONTEND=dialog

RUN apt-get update

# Install packages
RUN apt-get install -y \
    sudo \
    build-essential \
    cmake \
    git \
    net-tools \
    iputils-ping \
    cmake-curses-gui \
    subversion \
    wget \

    # Other optional 3rd parties
    python3-pip
# Build libealsense

RUN apt-get install -y \
    udev \
    libssl-dev \
    libusb-1.0-0-dev \
    pkg-config \
    libgtk-3-dev \
    libglfw3-dev \
    libgl1-mesa-dev \
    libglu1-mesa-dev \
    at \
    libudev-dev \
    libgtk-3-dev \
    libpoco-dev 


#Building libfranka
RUN cd ${HOME} \
   && git clone --recursive https://github.com/tingelst/libfranka \
   && cd ${HOME}/libfranka \
   && mkdir build \
   && cd build \
   && cmake -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=OFF ..  \
   && cmake --build . -j4 \
   && cpack -G DEB \
   && sudo dpkg -i libfranka-0.9.3-x86_64.deb

# Building franka_ros

RUN apt install -y \
   ros-humble-hardware-interface \
   ros-humble-generate-parameter-library \
   ros-humble-ros2-control-test-assets \
   ros-humble-controller-manager \
   ros-humble-control-msgs \
   ros-humble-xacro \
   ros-humble-angles \
   ros-humble-ros2-control \
   ros-humble-realtime-tools \
   ros-humble-control-toolbox \
   ros-humble-moveit \
   ros-humble-ros2-controllers \
   ros-humble-joint-state-publisher \
   ros-humble-joint-state-publisher-gui \
   ros-humble-ament-cmake \
   ros-humble-ament-cmake-clang-format \
   python3-colcon-common-extensions

# RUN cd /ros2_ws/\
#    && mkdir src \
#    && cd src \
#    && git clone https://github.com/tingelst/franka_ros2 -b fer \
#    && source /opt/ros/humble/setup.bash \
#    && cd .. \
#    && colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release


