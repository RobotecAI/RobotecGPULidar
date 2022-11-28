#!/usr/bin/env bash

set -e
set -o pipefail

CUDA_MIN_VER_MAJOR="11"
CUDA_MIN_VER_MINOR="2"
VCPKG_INSTALL_DIR="external/vcpkg"
VCPKG_TAG="2022.08.15"
INSIDE_DOCKER=false

if [ -f /.dockerenv ]; then
    VCPKG_INSTALL_DIR="/rgldep/vcpkg"
    INSIDE_DOCKER=true
    echo "Setup inside docker"
fi

CMAKE_ARGS=()
MAKE_ARGS=()
COLLECT_CMAKE_ARGS=false
COLLECT_MAKE_ARGS=false
DO_MAKE=false
DO_CMAKE=false
for arg in "$@"
do
    if [ "$arg" = "--cmake" ]; then
        COLLECT_CMAKE_ARGS=true && COLLECT_MAKE_ARGS=false && DO_CMAKE=true && continue;
    fi
    if [ "$arg" = "--make" ]; then
        COLLECT_MAKE_ARGS=true && COLLECT_CMAKE_ARGS=false && DO_MAKE=true && continue;
    fi
    if [ "$COLLECT_CMAKE_ARGS" = true ]; then CMAKE_ARGS+=("$arg") && continue; fi
    if [ "$COLLECT_MAKE_ARGS" = true ]; then MAKE_ARGS+=("$arg") && continue; fi
done

cd "$(dirname "$0")"

# Check CUDA
CUDA_VERSION=$(nvcc --version 2>/dev/null | grep -E -o "V[0-9]+.[0-9]+.[0-9]+" | cut -c2-)
if [ -z "$CUDA_VERSION" ]; then echo "CUDA not found!" && exit 1; fi
CUDA_MAJOR=$(echo "$CUDA_VERSION" | cut -d. -f1)
CUDA_MINOR=$(echo "$CUDA_VERSION" | cut -d. -f2)
if [ "$CUDA_MAJOR" -lt $CUDA_MIN_VER_MAJOR ] ||
    { [ "$CUDA_MAJOR" -eq $CUDA_MIN_VER_MAJOR ] && [ "$CUDA_MINOR" -lt $CUDA_MIN_VER_MINOR ]; }; then
    echo "CUDA missing or CUDA version not supported! Get CUDA $CUDA_MIN_VER_MAJOR.$CUDA_MIN_VER_MINOR+";
    echo "Remember to extend environment variables:
    - PATH with CUDA bin directory
    - LD_LIBRARY_PATH with CUDA lib directory";
    exit 1;
fi

# Check OptiX_INSTALL_DIR if building RGL
if [ -z "$OptiX_INSTALL_DIR" ] && { [ "$DO_CMAKE" = true ] || [ "$DO_MAKE" = true ]; }; then
    echo "OptiX not found! Make sure you have installed OptiX (non-empty output from command: 'locate libnvoptix'),
and you have exported environment variable OptiX_INSTALL_DIR";
    exit 1;
fi

# Install PCL using vcpkg
if [ ! -d "$VCPKG_INSTALL_DIR" ]; then
    if [ "$INSIDE_DOCKER" = false ]; then  # Inside docker already provided
        echo "Installing dependencies for vcpkg..."
        sudo apt update
        sudo apt install git curl zip unzip tar freeglut3-dev libglew-dev libglfw3-dev
    fi
    git clone -b $VCPKG_TAG --single-branch --depth 1 https://github.com/microsoft/vcpkg $VCPKG_INSTALL_DIR
fi
if [ ! -f $VCPKG_INSTALL_DIR"/vcpkg" ]; then $VCPKG_INSTALL_DIR"/bootstrap-vcpkg.sh"; fi

$VCPKG_INSTALL_DIR"/vcpkg" "install" "--clean-after-build" "pcl[core,visualization]"

# Build
if [ ! -d "build" ]; then mkdir build; fi
cd build
if [ "$DO_CMAKE" = true ]; then
    cmake .. -DCMAKE_TOOLCHAIN_FILE=$VCPKG_INSTALL_DIR"/scripts/buildsystems/vcpkg.cmake" "${CMAKE_ARGS[@]}";
else
    echo "--- IMPORTANT ---
Remember to pass vcpkg's CMAKE_TOOLCHAIN_FILE to cmake. See:
cmake .. -DCMAKE_TOOLCHAIN_FILE=$VCPKG_INSTALL_DIR/scripts/buildsystems/vcpkg.cmake
"
fi
if [ "$DO_MAKE" = true ]; then
    make "${MAKE_ARGS[@]}";
fi
