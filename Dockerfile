ARG BASE_IMAGE=base
# Stage from full image tag name for dependabot detection
FROM nvidia/cuda:11.7.1-devel-ubuntu22.04 as base

################################################################################
# MARK: prepper - prep rgl dependencies
################################################################################
FROM $BASE_IMAGE as prepper
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
COPY ./setup.py .

# install dependencies while caching apt downloads
# RUN --mount=type=cache,sharing=locked,target=/var/cache/apt \
#    ./setup.py --install-deps-only

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
COPY --from=builder /code/build/libRobotecGPULidar.so /
