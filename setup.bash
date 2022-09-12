#!/usr/bin/env bash

CUDA_MIN_VER_MAJOR="11"
CUDA_MIN_VER_MINOR="2"
VCPKG_INSTALL_DIR="external/vcpkg"
VCPKG_TAG="2022.08.15"

CMAKE_ARGS=
MAKE_ARGS=
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
    if [ "$COLLECT_CMAKE_ARGS" = true ]; then CMAKE_ARGS=$CMAKE_ARGS" "$arg && continue; fi
    if [ "$COLLECT_MAKE_ARGS" = true ]; then MAKE_ARGS=$MAKE_ARGS" "$arg && continue; fi
done

cd "$(dirname "$0")"

# Check CUDA
CUDA_VERSION=$(nvcc --version 2>/dev/null | egrep -o "V[0-9]+.[0-9]+.[0-9]+" | cut -c2-)
if [ -z "$CUDA_VERSION" ]; then echo "CUDA not found!" && exit 1; fi
CUDA_MAJOR=$(echo $CUDA_VERSION | cut -d. -f1)
CUDA_MINOR=$(echo $CUDA_VERSION | cut -d. -f2)
if [ "$CUDA_MAJOR" -lt $CUDA_MIN_VER_MAJOR ] ||
    ([ "$CUDA_MAJOR" -eq $CUDA_MIN_VER_MAJOR ] && [ "$CUDA_MINOR" -lt $CUDA_MIN_VER_MINOR ]); then
    echo "CUDA version not supported! Get $CUDA_MIN_VER_MAJOR.$CUDA_MIN_VER_MINOR+" && exit 1;
fi

# Check OptiX_INSTALL_DIR
if [ -z "$OptiX_INSTALL_DIR" ]; then
    echo "OptiX not found! Make sure you export environment variable OptiX_INSTALL_DIR" && exit 1;
fi

# Install PCL using vcpkg
if [ ! -d "$VCPKG_INSTALL_DIR" ]; then
    git clone -b $VCPKG_TAG --single-branch https://github.com/microsoft/vcpkg $VCPKG_INSTALL_DIR &&
    $VCPKG_INSTALL_DIR"/bootstrap-vcpkg.sh"
fi
$VCPKG_INSTALL_DIR"/vcpkg" "install" "pcl[core,visualization]"

# Build
if [ ! -d "build" ]; then mkdir build; fi
cd build
if [ "$DO_CMAKE" = true ]; then
    cmake .. -DCMAKE_TOOLCHAIN_FILE=$VCPKG_INSTALL_DIR"/scripts/buildsystems/vcpkg.cmake" $CMAKE_ARGS;
else
    echo "--- IMPORTANT ---
Remember to set vcpkg CMAKE_TOOLCHAIN_FILE while doing cmake. See:
cmake .. -DCMAKE_TOOLCHAIN_FILE=$VCPKG_INSTALL_DIR/scripts/buildsystems/vcpkg.cmake"
fi
if [ "$DO_MAKE" = true ]; then
    make $MAKE_ARGS
fi
