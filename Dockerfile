ARG BASE_IMAGE=base
# Stage from full image tag name for dependabot detection
FROM nvidia/cuda:11.7.1-devel-ubuntu22.04 as base

################################################################################
# MARK: prepper - prep rgl dependencies
################################################################################
FROM $BASE_IMAGE as prepper
ARG DEBIAN_FRONTEND=noninteractive
RUN apt update
RUN apt install -y \
    git \
    cmake \
    python3

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
