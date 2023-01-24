#!/usr/bin/env python3
import os
import platform
import sys
import re
import subprocess
import shutil
import argparse

# Global variables (default values for Linux)
CUDA_MIN_VER_MAJOR = 11
CUDA_MIN_VER_MINOR = 2
CMAKE_GENERATOR = "'Unix Makefiles'"
VCPKG_INSTALL_DIR = os.path.join("external", "vcpkgA")
VCPKG_TAG = "2022.08.15"
VCPKG_EXEC = "vcpkg"
VCPKG_BOOTSTRAP = "bootstrap-vcpkg.sh"
VCPKG_PLATFORM_SPEC = ""

# Helper functions

def on_linux():
    return platform.system() == "Linux"


def on_windows():
    return platform.system() == "Windows"


def inside_docker():
    path = "/proc/self/cgroup"
    return (
        os.path.exists("/.dockerenv") or
        os.path.isfile(path) and any("docker" in line for line in open(path))
    )


def wait_and_check(process: subprocess.Popen):
    process.wait()
    if process.returncode != 0:
        sys.exit(process.returncode)


# Platform-dependent configuration

if inside_docker():
    VCPKG_INSTALL_DIR = os.path.join("/rgldep", "vcpkg")

if on_windows():
    CUDA_MIN_VER_MINOR = 4
    CMAKE_GENERATOR = "Ninja"
    VCPKG_EXEC = "vcpkg.exe"
    VCPKG_BOOTSTRAP = "bootstrap-vcpkg.bat"
    VCPKG_PLATFORM_SPEC = ":x64-windows"

if __name__ == "__main__":
    # Parse arguments
    parser = argparse.ArgumentParser(description="Helper script to build RGL.")
    parser.add_argument("--build-dir", type=str, nargs=1, default="build",
                        help="Path to build directory. Default: 'build'")
    parser.add_argument("--install-deps", action='store_true',
                        help="Install RGL dependencies and exit")
    parser.add_argument("--clean-build", action='store_true',
                        help="Remove build directory before cmake")
    parser.add_argument("--with-ros2", action='store_true',
                        help="Build RGL with ROS2 extension")
    parser.add_argument("--with-ros2-standalone", action='store_true',
                        help="Build RGL with ROS2 extension in standalone mode")
    parser.add_argument("--cmake", type=str, nargs=1, default=None,
                        help="Pass arguments to cmake. Usage: --cmake=\"args...\"")
    if on_linux():
        parser.add_argument("--make", type=str, nargs=1, default=None, dest="build_args",
                            help="Pass arguments to make. Usage: --make=\"args...\"")
        parser.add_argument("--lib-rpath", type=str, nargs='*',
                            help="Add run-time search path(s) for RGL library")
    if on_windows():
        parser.add_argument("--ninja", type=str, nargs=1, default=None, dest="build_args",
                            help="Pass arguments to ninja. Usage: --ninja=\"args...\"")
    args = parser.parse_args()

    # Install RGL dependencies
    if args.install_deps:
        # Clone vcpkg
        if not os.path.isdir(VCPKG_INSTALL_DIR):
            if on_linux() and not inside_docker():  # Inside docker already installed
                print("Installing dependencies for vcpkg...")
                os.system("sudo apt update")
                os.system("sudo apt install git curl zip unzip tar freeglut3-dev libglew-dev libglfw3-dev")
            vcpkg_clone_process = subprocess.Popen(f"git clone -b {VCPKG_TAG} --single-branch --depth 1 https://github.com/microsoft/vcpkg {VCPKG_INSTALL_DIR}", shell=True, stderr=sys.stderr, stdout=sys.stdout)
            wait_and_check(vcpkg_clone_process)
        # Bootstrap vcpkg
        if not os.path.isfile(os.path.join(VCPKG_INSTALL_DIR, VCPKG_EXEC)):
            vcpkg_bootstrap_process = subprocess.Popen(f"{os.path.join(VCPKG_INSTALL_DIR, VCPKG_BOOTSTRAP)}", shell=True, stderr=sys.stderr, stdout=sys.stdout)
            wait_and_check(vcpkg_bootstrap_process)
        # Install dependencies via vcpkg
        vcpkg_install_process = subprocess.Popen(f"{os.path.join(VCPKG_INSTALL_DIR, VCPKG_EXEC)} install --clean-after-build pcl[core,visualization]{VCPKG_PLATFORM_SPEC}", shell=True, stderr=sys.stderr, stdout=sys.stdout)
        wait_and_check(vcpkg_install_process)
        sys.exit(0)

    # Check CUDA
    nvcc_process = subprocess.run("nvcc --version", shell=True, stdout=subprocess.PIPE)
    nvcc_ver_match = re.search("V[0-9]+.[0-9]+.[0-9]+", nvcc_process.stdout.decode("utf-8"))
    if not nvcc_ver_match:
        print("CUDA not found")
        sys.exit(1)
    cuda_major = int(nvcc_ver_match[0].split(".")[0][1:])  # [1:] to remove char 'v'
    cuda_minor = int(nvcc_ver_match[0].split(".")[1])
    print(f"Found CUDA {cuda_major}.{cuda_minor}")
    if cuda_major < CUDA_MIN_VER_MAJOR or (cuda_major == CUDA_MIN_VER_MAJOR and cuda_minor < CUDA_MIN_VER_MINOR):
        print(f"CUDA version not supported! Get CUDA {CUDA_MIN_VER_MAJOR}.{CUDA_MIN_VER_MINOR}+")
        sys.exit(1)

    # Check OptiX_INSTALL_DIR
    if os.environ["OptiX_INSTALL_DIR"] == "":
        print("OptiX not found! Make sure you have exported environment variable OptiX_INSTALL_DIR")
        sys.exit(1)

    # Go to script directory
    os.chdir(sys.path[0])

    # Prepare build directory
    if args.clean_build and os.path.isdir(args.build_dir):
        shutil.rmtree(args.build_dir, ignore_errors=True)
    if not os.path.isdir(args.build_dir):
        os.makedirs(args.build_dir)

    # Extend Path with libRobotecGPULidar location to link tests properly during the build on Windows
    if on_windows():
        os.environ["Path"] = os.environ["Path"] + ";" + os.path.join(os.getcwd(), args.build_dir)

    # Build
    cmake_args = f"-DCMAKE_TOOLCHAIN_FILE={os.path.join(VCPKG_INSTALL_DIR, 'scripts', 'buildsystems', 'vcpkg.cmake')} "
    cmake_args += "-DRGL_BUILD_ROS2_EXTENSION={} ".format("ON" if args.with_ros2 else "OFF")
    cmake_args += "-DRGL_BUILD_ROS2_EXTENSION_STANDALONE={} ".format("ON" if args.with_ros2_standalone else "OFF")

    if on_linux():
        # Set rpaths
        if args.lib_rpath is not None:
            linker_rpath_flags = ""
            for rpath in args.lib_rpath:
                rpath = rpath.replace("$ORIGIN", "\$ORIGIN")  # cmake should not treat this as variable
                linker_rpath_flags += f"-Wl,-rpath={rpath} "
            cmake_args += f"-DCMAKE_SHARED_LINKER_FLAGS=\"{linker_rpath_flags}\" "

    if on_windows():
        cmake_args += "-DRGL_BUILD_TOOLS=OFF "  # On Windows not available

    if args.cmake is not None:
        cmake_args += args.cmake[0]

    print(f"Building RGL with cmake args: '{cmake_args}'")
    cmake_process = subprocess.Popen(f"cmake -B {args.build_dir} -G {CMAKE_GENERATOR} --install-prefix {os.path.join(os.getcwd(), args.build_dir)} {cmake_args}", shell=True, stderr=sys.stderr, stdout=sys.stdout)
    wait_and_check(cmake_process)

    build_args = "" if args.build_args is None else args.build_args[0]
    print(f"Building RGL with build args: '{build_args}'")
    build_process = subprocess.Popen(f"cmake --build {args.build_dir} -- {build_args}", shell=True, stderr=sys.stderr, stdout=sys.stdout)
    wait_and_check(build_process)

    if args.with_ros2_standalone:
        print("Installing ros2 standalone...")
        install_process = subprocess.Popen(f"cmake --install {args.build_dir}", shell=True, stderr=sys.stderr, stdout=sys.stdout)
        wait_and_check(install_process)
