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

COLLECT_CMAKE_ARGS=false
COLLECT_MAKE_ARGS=false
COLLECT_RPATH=false

INSTALL_DEPS=false
PRINT_HELP=false
CLEAN_BUILD=false
DO_INSTALL=false

RGL_LIB_RPATH="\$ORIGIN/"
BUILD_DIR=build
CMAKE_ARGS=("-DRGL_BUILD_ROS2_EXTENSION=OFF" "-DRGL_BUILD_ROS2_EXTENSION_STANDALONE=OFF")
MAKE_ARGS=()

reset_args_collectors() { COLLECT_MAKE_ARGS=false && COLLECT_CMAKE_ARGS=false && COLLECT_RPATH=false; }

for arg in "$@"
do
    if [ "$arg" = "--cmake" ]; then
        reset_args_collectors && COLLECT_CMAKE_ARGS=true && continue;
    fi
    if [ "$arg" = "--make" ]; then
        reset_args_collectors && COLLECT_MAKE_ARGS=true && continue;
    fi
    if [ "$arg" = "--help" ]; then
        reset_args_collectors && PRINT_HELP=true && break;
    fi
    if [ "$arg" = "--rgl-clean-build" ]; then
        reset_args_collectors && CLEAN_BUILD=true && continue;
    fi
    if [ "$arg" = "--rgl-install-deps" ]; then
        reset_args_collectors && INSTALL_DEPS=true && continue;
    fi
    if [ "$arg" = "--rgl-lib-rpath" ]; then
        reset_args_collectors && COLLECT_RPATH=true && continue;
    fi
    if [ "$arg" = "--rgl-with-ros2" ]; then
        reset_args_collectors && CMAKE_ARGS+=("-DRGL_BUILD_ROS2_EXTENSION=ON") && continue;
    fi
    if [ "$arg" = "--rgl-with-ros2-standalone" ]; then
        reset_args_collectors && CMAKE_ARGS+=("-DRGL_BUILD_ROS2_EXTENSION_STANDALONE=ON") && DO_INSTALL=true && continue;
    fi
    if [ "$COLLECT_CMAKE_ARGS" = true ]; then CMAKE_ARGS+=("$arg") && continue; fi
    if [ "$COLLECT_MAKE_ARGS" = true ]; then MAKE_ARGS+=("$arg") && continue; fi
    if [ "$COLLECT_RPATH" = true ]; then RGL_LIB_RPATH="${arg}" && reset_args_collectors && continue; fi
    # if not caught by the rules above, it must be `build directory`
    BUILD_DIR="${arg}"
done

# Print help
if [ "$PRINT_HELP" = true ]; then
echo "Usage
    ./setup.bash [options]
    ./setup.bash <path-to-build-dir> [options]
Default <path-to-build-dir> is 'build'.

Options
  --cmake <args...>          = Pass arguments to cmake.
  --make <args...>           = Pass arguments to make.
  --help                     = Print usage information and exit.
  --rgl-install-deps         = Install RGL dependencies and exit.
  --rgl-clean-build          = Remove build directory before cmake.
  --rgl-with-ros2            = Build RGL with ROS2 extension.
  --rgl-with-ros2-standalone = Build RGL with ROS2 extension in standalone mode.
  --rgl-lib-rpath <path>     = Add run-time search path for RGL library. Default is '${RGL_LIB_RPATH}'.
"
exit 0;
fi

cd "$(dirname "$0")"

# Install RGL dependencies
if [ "$INSTALL_DEPS" = true ]; then
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
    exit 0;
fi

# Check CUDA
CUDA_VERSION=$(nvcc --version 2>/dev/null | grep -E -o "V[0-9]+.[0-9]+.[0-9]+" | cut -c2-)
if [ -z "$CUDA_VERSION" ]; then echo "CUDA not found!" && exit 1; fi
CUDA_MAJOR=$(echo "$CUDA_VERSION" | cut -d. -f1)
CUDA_MINOR=$(echo "$CUDA_VERSION" | cut -d. -f2)
if [ "$CUDA_MAJOR" -lt $CUDA_MIN_VER_MAJOR ] ||
    { [ "$CUDA_MAJOR" -eq $CUDA_MIN_VER_MAJOR ] && [ "$CUDA_MINOR" -lt $CUDA_MIN_VER_MINOR ]; }; then
    echo "CUDA missing or CUDA version not supported! Get CUDA $CUDA_MIN_VER_MAJOR.$CUDA_MIN_VER_MINOR+";
    exit 1;
fi

# Check OptiX_INSTALL_DIR if building RGL
if [ -z "$OptiX_INSTALL_DIR" ] && { [ "$DO_CMAKE" = true ] || [ "$DO_MAKE" = true ]; }; then
    echo "OptiX not found! Make sure you have exported environment variable OptiX_INSTALL_DIR";
    exit 1;
fi

# Build
if [ "${CLEAN_BUILD}" = true ]; then rm -rf "${BUILD_DIR}"; fi
if [ ! -d "${BUILD_DIR}" ]; then mkdir "${BUILD_DIR}"; fi
cd "${BUILD_DIR}"

CMAKE_ARGS+=("-DRGL_LIB_RPATH=${RGL_LIB_RPATH}");
cmake .. -DCMAKE_TOOLCHAIN_FILE=$VCPKG_INSTALL_DIR"/scripts/buildsystems/vcpkg.cmake" --install-prefix $PWD "${CMAKE_ARGS[@]}";
make "${MAKE_ARGS[@]}";
if [ "${DO_INSTALL}" = true ]; then make install; fi
