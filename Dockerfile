ARG BASE_IMAGE=nvidia/cuda:11.7.1-devel-ubuntu22.04
ARG DEBIAN_FRONTEND=noninteractive

FROM ${BASE_IMAGE} as rgl-core
RUN apt update
RUN apt install -y \
    git \
    cmake \
    python3

FROM rgl-core AS build
ARG OptiX_INSTALL_DIR=/optix

WORKDIR /code
COPY . .

RUN --mount=type=bind,from=optix,target=${OptiX_INSTALL_DIR} \
    ./setup.py

FROM scratch AS export-binaries
COPY --from=build /code/build/libRobotecGPULidar.so /
