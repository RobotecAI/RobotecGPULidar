ARG BASE_IMAGE=base
# Stage from full image tag name for dependabot detection
FROM nvidia/cuda:11.7.1-devel-ubuntu22.04 as base
ARG DEBIAN_FRONTEND=noninteractive

################################################################################
# MARK: prepper - prep rgl dependencies
################################################################################
FROM $BASE_IMAGE as prepper
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

WORKDIR /code
COPY . .

RUN --mount=type=bind,from=optix,target=${OptiX_INSTALL_DIR} \
    ./setup.py

################################################################################
# MARK: exporter - export rgl binaries
################################################################################
FROM scratch AS exporter
COPY --from=builder /code/build/libRobotecGPULidar.so /
