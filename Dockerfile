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
# TODO - install ROS2 here

# Copy only dependencies definition files
COPY ./extensions/ros2/install_deps.py .

# Install dependencies while caching apt downloads
RUN --mount=type=cache,sharing=locked,target=/var/cache/apt \
    ./install_deps.py

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
RUN --mount=type=bind,from=optix,target=${OptiX_INSTALL_DIR} \
    ./setup.py

# Restore DNS lookups
RUN mv /etc/nsswitch.conf.bak /etc/nsswitch.conf && \
    cat /etc/nsswitch.conf

################################################################################
# MARK: exporter - export rgl binaries
################################################################################
FROM scratch AS exporter
COPY --from=builder /opt/rgl/build/libRobotecGPULidar.so /
