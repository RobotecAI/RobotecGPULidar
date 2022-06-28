FROM nvidia/cuda:11.2.0-devel-ubuntu20.04
WORKDIR /code
RUN apt update -y && DEBIAN_FRONTEND=noninteractive apt install -y git python3-pip
RUN pip3 install cmake --upgrade  # Install from PIP since 3.17 is needed


