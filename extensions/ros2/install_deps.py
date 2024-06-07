#!/usr/bin/env python3
import os
import platform
import sys
import subprocess
from pathlib import Path


class Config:
    SUPPORTED_ROS_DISTROS = ["humble"]

    # Paths relative to project root
    RADAR_MSGS_DIR = os.path.join("external", "radar_msgs")
    RADAR_MSGS_INSTALL_DIR = os.path.join("external", "radar_msgs", "install")
    RADAR_MSGS_COMMIT = "47d2f26906ef38fa15ada352aea6b5aad547781d"


def install_deps():
    cfg = Config()

    # Install dependencies for ROS2 extension
    check_ros2_version()
    install_ros2_deps(cfg)

    print("ROS2 deps installed successfully")


def are_deps_installed() -> bool:
    cfg = Config()
    return os.path.isdir(cfg.RADAR_MSGS_INSTALL_DIR)


def install_ros2_deps(cfg):
    # Install colcon if needed
    if not is_command_available("colcon --help"):
        if on_windows():
            run_subprocess_command("pip install colcon-common-extensions")
        else:
            run_subprocess_command("sudo apt-get install -y python3-colcon-common-extensions")
    # Clone radar msgs
    if not os.path.isdir(cfg.RADAR_MSGS_DIR):
        run_subprocess_command("ls")
        run_subprocess_command(
            f"git clone --single-branch --depth 1 https://github.com/ros-perception/radar_msgs.git {cfg.RADAR_MSGS_DIR}")
        run_subprocess_command(f"cd {cfg.RADAR_MSGS_DIR} && git checkout {cfg.RADAR_MSGS_COMMIT} && cd ..")
    # Build radar msgs
    if not os.path.isdir(cfg.RADAR_MSGS_INSTALL_DIR):
        original_path = Path.cwd()
        os.chdir(cfg.RADAR_MSGS_DIR)
        run_subprocess_command(f"colcon build")
        os.chdir(original_path)
    # TODO: cyclonedds rmw may be installed here (instead of manually in readme)


def check_ros2_version():
    if "ROS_DISTRO" not in os.environ and "AMENT_PREFIX_PATH" not in os.environ:
        raise RuntimeError("ROS2 environment not found! Make sure you have sourced ROS2 setup file")
    if os.environ["ROS_DISTRO"] not in Config().SUPPORTED_ROS_DISTROS:
        raise RuntimeError(f"ROS distro '{os.environ['ROS_DISTRO']}' not supported. Choose one of {Config().SUPPORTED_ROS_DISTROS}")


def on_windows():
    return platform.system() == "Windows"


def run_subprocess_command(command: str, shell=True, stderr=sys.stderr, stdout=sys.stdout):
    print(f"Executing command: '{command}'")
    process = subprocess.Popen(command, shell=shell, stderr=stderr, stdout=stdout)
    process.wait()
    if process.returncode != 0:
        raise RuntimeError(f"Failed to execute command: '{command}'")


def is_command_available(command):
    process = subprocess.Popen(f"{command}", shell=True, stderr=subprocess.DEVNULL, stdout=subprocess.DEVNULL)
    process.wait()
    return process.returncode == 0


if __name__ == "__main__":
    print("Important: this script should be executed from the root of the project (e.g. `./extensions/ros2/install_deps.py`)")

    sys.exit(install_deps())
