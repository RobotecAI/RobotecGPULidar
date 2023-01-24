#!/usr/bin/env python3
import os
import platform
import sys
import re
import subprocess
import shutil
import argparse


class Config:
    # Default values for Linux
    CUDA_MIN_VER_MAJOR = 11
    CUDA_MIN_VER_MINOR = 2
    CUDA_MIN_VER_PATCH = 0
    CMAKE_GENERATOR = "'Unix Makefiles'"
    VCPKG_INSTALL_DIR = os.path.join("external", "vcpkg")
    VCPKG_TAG = "2022.08.15"
    VCPKG_EXEC = "vcpkg"
    VCPKG_BOOTSTRAP = "bootstrap-vcpkg.sh"
    VCPKG_PLATFORM_SPEC = ""

    def __init__(self):
        # Platform-dependent configuration
        if inside_docker():
            self.VCPKG_INSTALL_DIR = os.path.join("/rgldep", "vcpkg")

        if on_windows():
            self.CUDA_MIN_VER_MINOR = 4
            self.CUDA_MIN_VER_PATCH = 4
            self.CMAKE_GENERATOR = "Ninja"
            self.VCPKG_EXEC = "vcpkg.exe"
            self.VCPKG_BOOTSTRAP = "bootstrap-vcpkg.bat"
            self.VCPKG_PLATFORM_SPEC = ":x64-windows"


def main():
    cfg = Config()
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
    parser.add_argument("--cmake", type=str, nargs=1, default="",
                        help="Pass arguments to cmake. Usage: --cmake=\"args...\"")
    if on_linux():
        parser.add_argument("--make", type=str, nargs=1, default=f"-j{os.cpu_count()}", dest="build_args",
                            help="Pass arguments to make. Usage: --make=\"args...\"")
        parser.add_argument("--lib-rpath", type=str, nargs='*',
                            help="Add run-time search path(s) for RGL library")
    if on_windows():
        parser.add_argument("--ninja", type=str, nargs=1, default=f"-j{os.cpu_count()}", dest="build_args",
                            help="Pass arguments to ninja. Usage: --ninja=\"args...\"")
    args = parser.parse_args()

    # Install RGL dependencies
    if args.install_deps:
        # Clone vcpkg
        if not os.path.isdir(cfg.VCPKG_INSTALL_DIR):
            if on_linux() and not inside_docker():  # Inside docker already installed
                print("Installing dependencies for vcpkg...")
                run_system_command("sudo apt update")
                run_system_command("sudo apt install git curl zip unzip tar freeglut3-dev libglew-dev libglfw3-dev")
            run_subprocess_command(f"git clone -b {cfg.VCPKG_TAG} --single-branch --depth 1 https://github.com/microsoft/vcpkg {cfg.VCPKG_INSTALL_DIR}")
        # Bootstrap vcpkg
        if not os.path.isfile(os.path.join(cfg.VCPKG_INSTALL_DIR, cfg.VCPKG_EXEC)):
            run_subprocess_command(f"{os.path.join(cfg.VCPKG_INSTALL_DIR, cfg.VCPKG_BOOTSTRAP)}")
        # Install dependencies via vcpkg
        run_subprocess_command(f"{os.path.join(cfg.VCPKG_INSTALL_DIR, cfg.VCPKG_EXEC)} install --clean-after-build pcl[core,visualization]{cfg.VCPKG_PLATFORM_SPEC}")
        return 0

    # Check CUDA
    def is_cuda_version_ok():
        nvcc_process = subprocess.run("nvcc --version", shell=True, stdout=subprocess.PIPE)
        nvcc_ver_match = re.search("V[0-9]+.[0-9]+.[0-9]+", nvcc_process.stdout.decode("utf-8"))
        if not nvcc_ver_match:
            raise RuntimeError("CUDA not found")
        major = int(nvcc_ver_match[0].split(".")[0][1:])  # [1:] to remove char 'v'
        minor = int(nvcc_ver_match[0].split(".")[1])
        patch = int(nvcc_ver_match[0].split(".")[2])
        print(f"Found CUDA {major}.{minor}.{patch}")
        for (actual, expected) in [(major, cfg.CUDA_MIN_VER_MAJOR), (minor, cfg.CUDA_MIN_VER_MINOR), (patch, cfg.CUDA_MIN_VER_PATCH)]:
            if actual > expected:
                return True
            if actual < expected:
                return False
        return True

    if not is_cuda_version_ok():
        raise RuntimeError(f"CUDA version not supported! Get CUDA {cfg.CUDA_MIN_VER_MAJOR}.{cfg.CUDA_MIN_VER_MINOR}.{cfg.CUDA_MIN_VER_PATCH}+")

    # Check OptiX_INSTALL_DIR
    if os.environ["OptiX_INSTALL_DIR"] == "":
        raise RuntimeError("OptiX not found! Make sure you have exported environment variable OptiX_INSTALL_DIR")

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
    cmake_args = [
        f"-DCMAKE_TOOLCHAIN_FILE={os.path.join(cfg.VCPKG_INSTALL_DIR, 'scripts', 'buildsystems', 'vcpkg.cmake')}",
        f"-DRGL_BUILD_ROS2_EXTENSION={'ON' if args.with_ros2 else 'OFF'}",
        f"-DRGL_BUILD_ROS2_EXTENSION_STANDALONE={'ON' if args.with_ros2_standalone else 'OFF'}"
    ]

    if on_linux():
        # Set rpaths
        if args.lib_rpath is not None:
            linker_rpath_flags = []
            for rpath in args.lib_rpath:
                rpath = rpath.replace("$ORIGIN", "\\$ORIGIN")  # cmake should not treat this as variable
                linker_rpath_flags.append(f"-Wl,-rpath={rpath}")
            cmake_args.append(f"-DCMAKE_SHARED_LINKER_FLAGS=\"{' '.join(linker_rpath_flags)}\"")

    if on_windows():
        cmake_args.append("-DRGL_BUILD_TOOLS=OFF")  # Tools are not available on Windows

    # Append user args, possibly overwriting
    cmake_args.append(args.cmake)

    cmake_args = " ".join(cmake_args)
    run_subprocess_command(f"cmake -B {args.build_dir} -G {cfg.CMAKE_GENERATOR} --install-prefix {os.path.join(os.getcwd(), args.build_dir)} {cmake_args}")
    run_subprocess_command(f"cmake --build {args.build_dir} -- {args.build_args}")

    if args.with_ros2_standalone:
        run_subprocess_command(f"cmake --install {args.build_dir}")


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


def run_subprocess_command(command: str, shell=True, stderr=sys.stderr, stdout=sys.stdout):
    print(f"Executing command: '{command}'")
    process = subprocess.Popen(command, shell=shell, stderr=stderr, stdout=stdout)
    process.wait()
    if process.returncode != 0:
        raise RuntimeError(f"Failed to execute command: '{command}'")


def run_system_command(command: str):
    print(f"Executing command: '{command}'")
    if os.system(command) != 0:
        raise RuntimeError(f"Failed to execute command: '{command}'")


if __name__ == "__main__":
    sys.exit(main())
