#!/usr/bin/env python3
import sys
sys.dont_write_bytecode = True
import os
import platform
import sys
import re
import subprocess
import shutil
import argparse

import install_deps as core_deps
from extensions.ros2 import install_deps as ros2_deps
from extensions.pcl import install_deps as pcl_deps


class Config:
    # Default values for Linux
    CUDA_MIN_VER_MAJOR = 11
    CUDA_MIN_VER_MINOR = 7
    CUDA_MIN_VER_PATCH = 0
    CMAKE_GENERATOR = "'Unix Makefiles'"

    RGL_BLOBS_DIR = os.path.join("external", "rgl_blobs")
    RGL_BLOBS_REPO = "git@github.com:RobotecAI/RGL-blobs.git"
    RGL_BLOBS_BRANCH = "main"

    def __init__(self):
        # Platform-dependent configuration
        if on_windows():
            self.CUDA_MIN_VER_MINOR = 4
            self.CUDA_MIN_VER_PATCH = 152  # patch for CUDA 11.4 Update 4
            self.CMAKE_GENERATOR = "Ninja"


def main():
    cfg = Config()
    # Parse arguments
    parser = argparse.ArgumentParser(description="Helper script to build RGL.")
    parser.add_argument("--build-dir", type=str, default="build",
                        help="Path to build directory. Default: 'build'")
    parser.add_argument("--install-deps", action='store_true',
                        help="Install dependencies for RGL and exit")
    parser.add_argument("--install-pcl-deps", action='store_true',
                        help="Install dependencies for PCL extension and exit")
    parser.add_argument("--install-ros2-deps", action='store_true',
                        help="Install dependencies for ROS2 extension and exit")
    parser.add_argument("--fetch-rgl-blobs", action='store_true',
                        help="Fetch RGL blobs and exit (repo used for storing closed-source testing data)")
    parser.add_argument("--clean-build", action='store_true',
                        help="Remove build directory before cmake")
    parser.add_argument("--with-pcl", action='store_true',
                        help="Build RGL with PCL extension")
    parser.add_argument("--with-ros2", action='store_true',
                        help="Build RGL with ROS2 extension")
    parser.add_argument("--with-ros2-standalone", action='store_true',
                        help="Build RGL with ROS2 extension and install all dependent ROS2 libraries additionally")
    parser.add_argument("--with-udp", action='store_true',
                        help="Build RGL with UDP extension (closed-source extension)")
    parser.add_argument("--with-snow", action='store_true',
                        help="Build RGL with snow simulation extension (closed-source extension)")
    parser.add_argument("--cmake", type=str, default="",
                        help="Pass arguments to cmake. Usage: --cmake=\"args...\"")
    if on_linux():
        parser.add_argument("--make", type=str, default=f"-j{os.cpu_count()}", dest="build_args",
                            help="Pass arguments to make. Usage: --make=\"args...\". Defaults to \"-j <cpu count>\"")
        parser.add_argument("--lib-rpath", type=str, nargs='*',
                            help="Add run-time search path(s) for RGL library. $ORIGIN (actual library path) is added by default.")
        parser.add_argument("--build-taped-test", action='store_true',
                            help="Build taped test (requires RGL blobs repo in runtime)")
    if on_windows():
        parser.add_argument("--ninja", type=str, default=f"-j{os.cpu_count()}", dest="build_args",
                            help="Pass arguments to ninja. Usage: --ninja=\"args...\". Defaults to \"-j <cpu count>\"")
    args = parser.parse_args()

    # ROS2 standalone obviously implies ROS2
    if args.with_ros2_standalone:
        args.with_ros2 = True

    # Go to script directory
    os.chdir(sys.path[0])

    # Install RGL dependencies
    if args.install_deps:
        core_deps.install_deps()
        return 0

    # Install dependencies for PCL extension
    if args.install_pcl_deps:
        pcl_deps.install_deps()
        return 0

    # Install dependencies for ROS2 extension
    if args.install_ros2_deps:
        ros2_deps.install_deps()
        return 0

    # Install dependencies for ROS2 extension
    if args.fetch_rgl_blobs:
        fetch_rgl_blobs_repo(cfg)
        return 0

    # Check CUDA
    if not is_cuda_version_ok(cfg):
        raise RuntimeError(
            f"CUDA version not supported! Get CUDA {cfg.CUDA_MIN_VER_MAJOR}.{cfg.CUDA_MIN_VER_MINOR}.{cfg.CUDA_MIN_VER_PATCH}+")

    # Check OptiX_INSTALL_DIR
    if os.environ["OptiX_INSTALL_DIR"] == "":
        raise RuntimeError("OptiX not found! Make sure you have exported environment variable OptiX_INSTALL_DIR")

    # Check if dependencies are installed
    if not core_deps.are_deps_installed():
        raise RuntimeError(
            "RGL requires dependencies to be installed: run this script with --install-deps flag")

    if args.with_pcl and not pcl_deps.are_deps_installed():
        raise RuntimeError(
            "PCL extension requires dependencies to be installed: run this script with --install-pcl-deps flag")

    if args.with_ros2 and not ros2_deps.are_deps_installed():
        raise RuntimeError(
            "ROS2 extension requires radar_msgs to be built: run this script with --install-ros2-deps flag")

    # Prepare build directory
    if args.clean_build and os.path.isdir(args.build_dir):
        shutil.rmtree(args.build_dir, ignore_errors=True)
    if not os.path.isdir(args.build_dir):
        os.makedirs(args.build_dir)

    # Extend Path with libRobotecGPULidar location to link tests properly during the build on Windows
    if on_windows():
        os.environ["Path"] = os.environ["Path"] + ";" + os.path.join(os.getcwd(), args.build_dir)

    if args.with_ros2:
        cfg_ros2 = ros2_deps.Config()
        # Source environment for additional packages
        # ROS2 itself must be sourced by the user, because its location is unknown to this script
        ros2_deps.check_ros2_version()
        setup = "setup.bat" if on_windows() else "setup.sh"
        source_environment(os.path.join(os.getcwd(), cfg_ros2.RADAR_MSGS_INSTALL_DIR, setup))

    # Build
    cmake_args = [
        f"-DCMAKE_TOOLCHAIN_FILE={os.path.join(pcl_deps.Config().VCPKG_DIR, 'scripts', 'buildsystems', 'vcpkg.cmake') if args.with_pcl else ''}",
        f"-DVCPKG_TARGET_TRIPLET={pcl_deps.Config().VCPKG_TRIPLET if args.with_pcl else ''}",
        f"-DRGL_BUILD_PCL_EXTENSION={'ON' if args.with_pcl else 'OFF'}",
        f"-DRGL_BUILD_ROS2_EXTENSION={'ON' if args.with_ros2 else 'OFF'}",
        f"-DRGL_BUILD_UDP_EXTENSION={'ON' if args.with_udp else 'OFF'}",
        f"-DRGL_BUILD_SNOW_EXTENSION={'ON' if args.with_snow else 'OFF'}"
    ]

    if on_linux():
        # Set rpaths
        linker_rpath_flags = ["-Wl,-rpath=\\$ORIGIN"]  # add directory in which an RGL library is located
        if args.lib_rpath is not None:
            for rpath in args.lib_rpath:
                rpath = rpath.replace("$ORIGIN", "\\$ORIGIN")  # cmake should not treat this as variable
                linker_rpath_flags.append(f"-Wl,-rpath={rpath}")
        cmake_args.append(f"-DCMAKE_SHARED_LINKER_FLAGS=\"{' '.join(linker_rpath_flags)}\"")
        # Taped test
        cmake_args.append(f"-DRGL_BUILD_TAPED_TESTS={'ON' if args.build_taped_test else 'OFF'}")

    # Append user args, possibly overwriting
    cmake_args.append(args.cmake)

    cmake_args = " ".join(cmake_args)
    run_subprocess_command(f"cmake -B {args.build_dir} -G {cfg.CMAKE_GENERATOR} {cmake_args}")
    run_subprocess_command(f"cmake --build {args.build_dir} -- {args.build_args}")

    if args.with_ros2_standalone:
        # Build RobotecGPULidar_ros2_standalone project to find and install all dependent ROS2 libraries and their
        # dependencies. It cannot be added as a subdirectory of RobotecGPULidar project because there is a conflict in
        # the same libraries required by RGL and ROS2 RGL takes them from vcpkg as statically linked objects while
        # ROS2 standalone required them as a shared objects
        ros2_standalone_cmake_args = f"-DCMAKE_INSTALL_PREFIX={os.path.join(os.getcwd(), args.build_dir)}"
        run_subprocess_command(
            f"cmake ros2_standalone -B {args.build_dir}/ros2_standalone -G {cfg.CMAKE_GENERATOR} {ros2_standalone_cmake_args}")
        run_subprocess_command(f"cmake --install {args.build_dir}/ros2_standalone")


def on_linux():
    return platform.system() == "Linux"


def on_windows():
    return platform.system() == "Windows"


def run_subprocess_command(command: str, shell=True, stderr=sys.stderr, stdout=sys.stdout):
    print(f"Executing command: '{command}'")
    process = subprocess.Popen(command, shell=shell, stderr=stderr, stdout=stdout)
    process.wait()
    if process.returncode != 0:
        raise RuntimeError(f"Failed to execute command: '{command}'")


def is_cuda_version_ok(cfg):
    nvcc_process = subprocess.run("nvcc --version", shell=True, stdout=subprocess.PIPE)
    nvcc_ver_match = re.search("V[0-9]+.[0-9]+.[0-9]+", nvcc_process.stdout.decode("utf-8"))
    if not nvcc_ver_match:
        raise RuntimeError("CUDA not found")
    major = int(nvcc_ver_match[0].split(".")[0][1:])  # [1:] to remove char 'v'
    minor = int(nvcc_ver_match[0].split(".")[1])
    patch = int(nvcc_ver_match[0].split(".")[2])
    print(f"Found CUDA {major}.{minor}.{patch}")
    for (actual, expected) in [(major, cfg.CUDA_MIN_VER_MAJOR), (minor, cfg.CUDA_MIN_VER_MINOR),
                               (patch, cfg.CUDA_MIN_VER_PATCH)]:
        if actual > expected:
            return True
        if actual < expected:
            return False
    return True


# Returns a dict with env variables visible for a command after running in a system shell
# Used to capture effects of sourcing file such as ros2 setup
def capture_environment(command="cd ."):
    env = "set" if on_windows() else "env"
    command = f"{command} && {env}"

    # Run the command and capture the output
    proc = subprocess.Popen(command, stdout=subprocess.PIPE, shell=True)
    output, _ = proc.communicate()

    return {key: value for key, _, value in (line.partition('=')
                                             for line in output.decode().splitlines())}


# Updates environment of this python process with env variables added/modified after sourcing given file
def source_environment(filepath):
    print(f"Sourcing {filepath}")
    # Store the original environment variables
    source = "call" if on_windows() else "."
    original_env = capture_environment()  # No-op working on both Windows and Linux
    new_env = capture_environment(f"{source} {filepath}")

    for new_key, new_value in new_env.items():
        if new_key not in original_env:
            print(f"Added environment variable: {new_key}={new_env[new_key]}")
        if new_key in original_env and original_env[new_key] != new_value:
            print(f"Modified environment variable: {new_key}={new_env[new_key]}")
        os.environ[new_key] = new_env[new_key]


def is_command_available(command):
    process = subprocess.Popen(f"{command}", shell=True, stderr=subprocess.DEVNULL, stdout=subprocess.DEVNULL)
    process.wait()
    return process.returncode == 0


def fetch_rgl_blobs_repo(cfg):
    def is_up_to_date(branch):
        result = subprocess.Popen("git fetch --dry-run --verbose", shell=True, stdout=subprocess.PIPE,
                                  stderr=subprocess.STDOUT)
        stdout, _ = result.communicate()
        return f"[up to date]      {branch}" in stdout.decode()

    def ensure_git_lfs_installed():

        if not is_command_available("git-lfs --help"):
            print("Installing git-lfs...")
            run_subprocess_command(
                "curl -s https://packagecloud.io/install/repositories/github/git-lfs/script.deb.sh | sudo bash")
            run_subprocess_command("sudo apt install git-lfs")

    ensure_git_lfs_installed()
    if not os.path.isdir(cfg.RGL_BLOBS_DIR):
        print("Cloning rgl blobs repository...")
        run_subprocess_command(
            f"git clone -b {cfg.RGL_BLOBS_BRANCH} --single-branch --depth 1 {cfg.RGL_BLOBS_REPO} {cfg.RGL_BLOBS_DIR}")
        os.chdir(cfg.RGL_BLOBS_DIR)
        # Set up git-lfs for this repository
        run_subprocess_command("git-lfs install && git-lfs pull")
        print("RGL blobs repo cloned successfully")
        return

    print("Checking for updates in rgl blobs repository...")
    os.chdir(cfg.RGL_BLOBS_DIR)
    if not is_up_to_date(cfg.RGL_BLOBS_BRANCH):
        print("Updating rgl blobs repository...")
        run_subprocess_command("git pull && git-lfs pull")
    print("RGL blobs repo fetched successfully")


if __name__ == "__main__":
    sys.exit(main())
