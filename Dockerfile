ARG BASE_IMAGE=base
ARG WITH_PCL=0
ARG WITH_ROS2=0
# Stage from full image tag name for dependabot detection
FROM nvidia/cuda:11.7.1-devel-ubuntu22.04 as base

################################################################################
# MARK: prepper - prep rgl dependencies
################################################################################
### Core dependencies stage
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

### PCL extension dependencies stage (added on top of core depenencies based on `WITH_PCL` argument)
FROM prepper-core AS prepper-pcl-0
# Do nothing, PCL extension is not enabled
FROM prepper-core AS prepper-pcl-1
# Copy only dependencies definition files for PCL extension
COPY ./extensions/pcl/install_deps.py .

# Install PCL extension dependencies while caching apt downloads
RUN --mount=type=cache,sharing=locked,target=/var/cache/apt \
    ./install_deps.py

### ROS2 extension dependencies stage (added on top of PCL depenencies based on `WITH_ROS2` argument)
FROM prepper-pcl-${WITH_PCL} AS prepper-ros2-0
# Do nothing, ROS2 extension is not enabled
FROM prepper-pcl-${WITH_PCL} AS prepper-ros2-1

# Install ROS2: Setup sources.list and keys
RUN . /etc/os-release && \
    echo "deb http://packages.ros.org/ros2/ubuntu $UBUNTU_CODENAME main" > /etc/apt/sources.list.d/ros2-latest.list && \
    apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654 && \
    apt-get update

ARG ROS_DISTRO=humble
ENV ROS_DISTRO=$ROS_DISTRO
# Install ROS2: Install packages
RUN --mount=type=cache,sharing=locked,target=/var/cache/apt \
    apt-get install -y --no-install-recommends \
        # Packages for RGL ROS2 standalone build
        patchelf \
        pkg-config \
        ros-$ROS_DISTRO-cyclonedds \
        ros-$ROS_DISTRO-fastrtps \
        ros-$ROS_DISTRO-rmw-cyclonedds-cpp \
        ros-$ROS_DISTRO-rmw-fastrtps-cpp \
        ros-$ROS_DISTRO-ros-core \
        # Packages for UDP-ROS2 integration test
        # psmisc for`killall` command
        psmisc \
        ros-$ROS_DISTRO-velodyne-driver \
        ros-$ROS_DISTRO-velodyne-pointcloud

# Copy only dependencies definition files for ROS2 extension
COPY ./extensions/ros2/install_deps.py .

# Install ROS2 extension dependencies while caching apt downloads
RUN --mount=type=cache,sharing=locked,target=/var/cache/apt \
    . /opt/ros/$ROS_DISTRO/setup.sh && \
    ./install_deps.py

### Final prepper stage with selected extensions
FROM prepper-ros2-${WITH_ROS2} AS prepper

################################################################################
# MARK: builder - build rgl binaries
################################################################################
FROM prepper AS builder
ARG OptiX_INSTALL_DIR=/optix

# Copy rest of source tree
COPY . .

ARG BUILD_CMD="./setup.py"
RUN --mount=type=bind,from=optix,target=${OptiX_INSTALL_DIR} \
    sh -c "$BUILD_CMD"

################################################################################
# MARK: dancer - multi-stage for cache dancing
################################################################################
FROM builder AS dancer

# Copy entire build directory
# RUN mkdir /dancer && \
#     cp -rT build /dancer

# Copy only the lib and bin directories
RUN mkdir /dancer && \
    cp -r build/bin /dancer/ && \
    cp -r build/lib /dancer/

################################################################################
# MARK: exporter - export rgl binaries and executables
################################################################################
FROM scratch AS exporter

COPY --from=dancer /dancer /
