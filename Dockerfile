FROM nvidia/cuda:11.7.1-devel-ubuntu22.04

ARG DEBIAN_FRONTEND=noninteractive

RUN apt update

RUN apt install -y cmake

# Install vcpkg dependencies
RUN apt install -y \
    git \
    curl \
    zip \
    unzip \
    tar \
    pkg-config \
    freeglut3-dev \
    libglew-dev \
    libglfw3-dev \
    python3

# Install RGL dependencies via vcpkg
COPY setup.py /
RUN /setup.py --install-pcl-deps
RUN rm /setup.py

WORKDIR /code

RUN git config --system --add safe.directory /code
