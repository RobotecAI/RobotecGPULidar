#!/usr/bin/env python3
import os
import platform
import sys
import subprocess


class Config:
    # Default values for Linux
    VCPKG_TAG = "2023.08.09"
    # Paths relative to project root
    VCPKG_DIR = os.path.join("external", "vcpkg")
    VCPKG_EXEC = "vcpkg"
    VCPKG_BOOTSTRAP = "bootstrap-vcpkg.sh"
    VCPKG_TRIPLET = "x64-linux"

    def __init__(self):
        if on_windows():
            self.VCPKG_EXEC = "vcpkg.exe"
            self.VCPKG_BOOTSTRAP = "bootstrap-vcpkg.bat"
            self.VCPKG_TRIPLET = "x64-windows"


def install_deps():
    cfg = Config()

    # Go to script directory
    os.chdir(sys.path[0])

    # Clone vcpkg
    if not os.path.isdir(cfg.VCPKG_DIR):
        if on_linux():
            print("Installing dependencies for vcpkg...")
            run_subprocess_command("sudo apt-get install -y git curl zip unzip tar freeglut3-dev libglew-dev libglfw3-dev")
        run_subprocess_command(
            f"git clone -b {cfg.VCPKG_TAG} --single-branch --depth 1 https://github.com/microsoft/vcpkg {cfg.VCPKG_DIR}")
    # Bootstrap vcpkg
    if not os.path.isfile(os.path.join(cfg.VCPKG_DIR, cfg.VCPKG_EXEC)):
        run_subprocess_command(f"{os.path.join(cfg.VCPKG_DIR, cfg.VCPKG_BOOTSTRAP)}")

    # Install dependencies via vcpkg
    run_subprocess_command(
        f"{os.path.join(cfg.VCPKG_DIR, cfg.VCPKG_EXEC)} install --clean-after-build pcl[core,visualization]:{cfg.VCPKG_TRIPLET}")

    print('PCL deps installed successfully')


def are_deps_installed() -> bool:
    cfg = Config()
    return os.path.isdir(cfg.VCPKG_DIR)


def on_windows():
    return platform.system() == "Windows"


def on_linux():
    return platform.system() == "Linux"


def run_subprocess_command(command: str, shell=True, stderr=sys.stderr, stdout=sys.stdout):
    print(f"Executing command: '{command}'")
    process = subprocess.Popen(command, shell=shell, stderr=stderr, stdout=stdout)
    process.wait()
    if process.returncode != 0:
        raise RuntimeError(f"Failed to execute command: '{command}'")


if __name__ == "__main__":
    sys.exit(install_deps())
