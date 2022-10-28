FROM nvidia/cuda:11.7.1-devel-ubuntu22.04

RUN export DEBIAN_FRONTEND=noninteractive

RUN apt update

# Install cmake from PIP since 3.17 is needed
RUN apt install -y \
    git \
    python3-pip
RUN pip3 install cmake --upgrade

# Install vcpkg dependencies
RUN apt install -y \
    git \
    curl \
    zip \
    unzip \
    tar \
    pkg-config \
    freeglut3-dev \
    python3

# Install RGL dependencies via vcpkg
COPY setup.bash /
RUN /setup.bash
RUN rm /setup.bash

WORKDIR /code
