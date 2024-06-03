ARG BASE_IMAGE=base
ARG WITH_PCL=0
ARG WITH_ROS2=0
# Stage from full image tag name for dependabot detection
FROM nvidia/cuda:11.7.1-devel-ubuntu22.04 as base

################################################################################
# MARK: prepper - prep rgl dependencies
################################################################################
FROM $BASE_IMAGE as prepper-core
ARG DEBIAN_FRONTEND=noninteractive

# Edit apt config for caching and update once
RUN mv /etc/apt/apt.conf.d/docker-clean /etc/apt/ && \
    echo 'Binary::apt::APT::Keep-Downloaded-Packages "true";' \
      > /etc/apt/apt.conf.d/keep-cache && \
    apt-get update

# Install bootstrap tools for install scripts
RUN --mount=type=cache,sharing=locked,target=/var/cache/apt \
    apt-get install -y --no-install-recommends \
        cmake \
        git \
        python3 \
        sudo

# Set working directory using standard opt path
WORKDIR /opt/rgl

# Copy only dependencies definition files
COPY ./install_deps.py .

# Install dependencies while caching apt downloads
RUN --mount=type=cache,sharing=locked,target=/var/cache/apt \
    ./install_deps.py

# Handle PCL extension
FROM prepper-core AS prepper-pcl-0

FROM prepper-core AS prepper-pcl-1
# Copy only dependencies definition files
COPY ./extensions/pcl/install_deps.py .

# Install dependencies while caching apt downloads
RUN --mount=type=cache,sharing=locked,target=/var/cache/apt \
    ./install_deps.py

# Handle ROS2 extension
FROM prepper-pcl-${WITH_PCL} AS prepper-ros2-0

FROM prepper-pcl-${WITH_PCL} AS prepper-ros2-1
# Install ROS2
# Setup timezone
RUN --mount=type=cache,sharing=locked,target=/var/cache/apt \
    echo 'Etc/UTC' > /etc/timezone && \
    ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
    apt-get install -q -y --no-install-recommends tzdata

# Setup sources.list
RUN echo "deb http://packages.ros.org/ros2/ubuntu jammy main" > /etc/apt/sources.list.d/ros2-latest.list

# Setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# Setup environment
ENV LANG C.UTF-8
ENV LC_ALL C.UTF-8

# Install packages
RUN --mount=type=cache,sharing=locked,target=/var/cache/apt \
    apt-get update && \
    apt-get install -y --no-install-recommends \
    # ROS2
    dirmngr \
    gnupg2 \
    ros-humble-ros-core=0.10.0-1* \
    # Packages for RGL ROS2 standalone build
    ros-humble-cyclonedds \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-fastrtps \
    ros-humble-rmw-fastrtps-cpp \
    patchelf \
    # Packages for UDP-ROS2 integration test
    ros-humble-velodyne-driver \
    ros-humble-velodyne-pointcloud \
    psmisc # `killall` command

# Copy only dependencies definition files
COPY ./extensions/ros2/install_deps.py .

# Install dependencies while caching apt downloads
RUN --mount=type=cache,sharing=locked,target=/var/cache/apt \
    /bin/bash -c "source /opt/ros/humble/setup.bash; ./install_deps.py"

# Final prepper stage with selected extensions
FROM prepper-ros2-${WITH_ROS2} AS prepper

################################################################################
# MARK: builder - build rgl binaries
################################################################################
FROM prepper AS builder
ARG OptiX_INSTALL_DIR=/optix

# Disable DNS lookups
RUN cat /etc/nsswitch.conf && \
    sed -e 's#hosts:\(.*\)dns\(.*\)#hosts:\1\2#g' -i.bak /etc/nsswitch.conf && \
    cat /etc/nsswitch.conf

# Copy rest of source tree
COPY . .

ARG BUILD_CMD="./setup.py"
RUN --mount=type=bind,from=optix,target=${OptiX_INSTALL_DIR} \
    /bin/bash -c "$BUILD_CMD"

# Restore DNS lookups
RUN mv /etc/nsswitch.conf.bak /etc/nsswitch.conf && \
    cat /etc/nsswitch.conf

################################################################################
# MARK: exporter - export rgl binaries
################################################################################
FROM scratch AS exporter
# Note: Using glob patterns (`?`, `*`, `[]`) to handle conditional COPY
# Docker will not fail if it won't find any valid source

# rgl binary
COPY --from=builder /opt/rgl/build/libRobotecGPULidar.so /

# ros2 standalone libs
COPY --from=builder /opt/rgl/build/ros2_standalon[e]/*.so* /ros2_standalone/

# tests
COPY --from=builder /opt/rgl/build/tes[t]/RobotecGPULidar_test /test/
COPY --from=builder /opt/rgl/build/tes[t]/taped_test/RobotecGPULidar_taped_test /test/taped_test

# tools
COPY --from=builder /opt/rgl/build/tool[s]/inspectLibRGL /tools/
COPY --from=builder /opt/rgl/build/tool[s]/tapePlayer /tools/
COPY --from=builder /opt/rgl/build/tool[s]/tapeVisualize[r] /tools/
