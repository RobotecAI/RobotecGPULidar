FROM nvidia/cuda:11.7.1-devel-ubuntu22.04

ARG DEBIAN_FRONTEND=noninteractive

RUN apt update
RUN apt install -y \
    cmake \
    git

WORKDIR /code
RUN git config --system --add safe.directory /code

##### PCL extension #####

# Install vcpkg dependencies
RUN apt install -y \
    curl \
    zip \
    unzip \
    tar \
    pkg-config \
    freeglut3-dev \
    libglew-dev \
    libglfw3-dev \
    python3

# Install RGL PCL dependencies via vcpkg
COPY setup.py /
RUN /setup.py --install-pcl-deps
RUN rm /setup.py

##### ROS2 extension #####

# Setup timezone
RUN echo 'Etc/UTC' > /etc/timezone && \
    ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
    apt-get update && \
    apt-get install -q -y --no-install-recommends tzdata && \
    rm -rf /var/lib/apt/lists/*

# Install packages
RUN apt-get update && apt-get install -q -y --no-install-recommends \
    dirmngr \
    gnupg2 \
    && rm -rf /var/lib/apt/lists/*

# Setup sources.list
RUN echo "deb http://packages.ros.org/ros2/ubuntu jammy main" > /etc/apt/sources.list.d/ros2-latest.list

# Setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# Setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8
ENV ROS_DISTRO humble

# Install ros2 packages
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-humble-ros-core=0.10.0-1* \
    && rm -rf /var/lib/apt/lists/*

# Install RGL ROS2 dependencies (e.g. radar_msgs)
COPY setup.py /
RUN /setup.py --install-ros2-deps
RUN rm /setup.py

# Install required packages for RGL ROS2 standalone build (may be moved to setup.py in the future)
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-cyclonedds ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    ros-${ROS_DISTRO}-fastrtps ros-${ROS_DISTRO}-rmw-fastrtps-cpp \
    patchelf

# Install required packages for UDP-ROS2 integration test
RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-velodyne-driver ros-${ROS_DISTRO}-velodyne-pointcloud \
    psmisc  # for `killall` command

CMD [ "/bin/bash" ]
