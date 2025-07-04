#!/usr/bin/env python3
import os
import sys
import subprocess
from pathlib import Path

from .install_deps import check_ros2_version

class Config:
    # Paths relative to project root
    AGNOCAST_DIR = os.path.join("external", "agnocast")
    AGNOCAST_INSTALL_DIR = os.path.join("external", "agnocast", "install")
    AGNOCAST_VERSION = "2.1.1"


def install_deps():
    cfg = Config()

    # ROS2 must be sourced
    check_ros2_version()

    # Install dependencies for Agnocast extension
    install_agnocast_deps(cfg)

    print("Agnocast deps installed successfully")


def are_deps_installed() -> bool:
    cfg = Config()
    return os.path.isdir(cfg.AGNOCAST_INSTALL_DIR)


def install_agnocast_deps(cfg):
    # Clone agnocast repository and install additional dependencies
    if not os.path.isdir(cfg.AGNOCAST_DIR):
        run_subprocess_command(
            f"git clone --depth 1 --branch {cfg.AGNOCAST_VERSION} https://github.com/tier4/agnocast.git {cfg.AGNOCAST_DIR}")
        run_subprocess_command("rosdep install -y --from-paths src --ignore-src --rosdistro $ROS_DISTRO")
    # Build agnocast project
    if not os.path.isdir(cfg.AGNOCAST_INSTALL_DIR):
        original_path = Path.cwd()
        os.chdir(cfg.AGNOCAST_DIR)
        run_subprocess_command(f"colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release")
        os.chdir(original_path)


def run_subprocess_command(command: str, shell=True, stderr=sys.stderr, stdout=sys.stdout):
    print(f"Executing command: '{command}'")
    process = subprocess.Popen(command, shell=shell, stderr=stderr, stdout=stdout)
    process.wait()
    if process.returncode != 0:
        raise RuntimeError(f"Failed to execute command: '{command}'")


if __name__ == "__main__":
    print("Important: this script should be executed from the root of the project (e.g. `./extensions/ros2/install_deps.py`)")

    sys.exit(install_deps())
