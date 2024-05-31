#!/usr/bin/env python3
import os
import sys
import subprocess


class Config:
    # Paths relative to project root
    SPDLOG_DIR = os.path.join("external", "spdlog")
    SPDLOG_VERSION = "v1.9.2"

    CMAKE_GIT_VERSION_TRACKING_DIR = os.path.join("external", "cmake_git_version_tracking")
    CMAKE_GIT_VERSION_TRACKING_VERSION = "904dbda1336ba4b9a1415a68d5f203f576b696bb"

    YAML_CPP_DIR = os.path.join("external", "yaml-cpp")
    YAML_CPP_VERSION = "yaml-cpp-0.7.0"

    GOOGLETEST_DIR = os.path.join("external", "googletest")
    GOOGLETEST_VERSION = "release-1.11.0"


def install_deps():
    cfg = Config()

    # Go to script directory
    os.chdir(sys.path[0])

    if not os.path.isdir(cfg.SPDLOG_DIR):
        run_subprocess_command(
            f"git clone -b {cfg.SPDLOG_VERSION} --single-branch --depth 1 https://github.com/gabime/spdlog.git {cfg.SPDLOG_DIR}")
        
    if not os.path.isdir(cfg.CMAKE_GIT_VERSION_TRACKING_DIR):
        run_subprocess_command(
            f"git clone --single-branch https://github.com/andrew-hardin/cmake-git-version-tracking.git {cfg.CMAKE_GIT_VERSION_TRACKING_DIR}")
        run_subprocess_command(
            f"cd {cfg.CMAKE_GIT_VERSION_TRACKING_DIR} && git reset --hard {cfg.CMAKE_GIT_VERSION_TRACKING_VERSION}")
        os.chdir(sys.path[0]) # Back to script directory

    if not os.path.isdir(cfg.YAML_CPP_DIR):
        run_subprocess_command(
            f"git clone -b {cfg.YAML_CPP_VERSION} --single-branch --depth 1 https://github.com/jbeder/yaml-cpp.git {cfg.YAML_CPP_DIR}")
        
    if not os.path.isdir(cfg.GOOGLETEST_DIR):
        run_subprocess_command(
            f"git clone -b {cfg.GOOGLETEST_VERSION} --single-branch --depth 1 https://github.com/google/googletest {cfg.GOOGLETEST_DIR}")

    print('Installed deps installed successfully')


def are_deps_installed() -> bool:
    cfg = Config()
    return os.path.isdir(cfg.SPDLOG_DIR) \
           and os.path.isdir(cfg.CMAKE_GIT_VERSION_TRACKING_DIR) \
           and os.path.isdir(cfg.YAML_CPP_DIR) \
           and os.path.isdir(cfg.GOOGLETEST_DIR)


def run_subprocess_command(command: str, shell=True, stderr=sys.stderr, stdout=sys.stdout):
    print(f"Executing command: '{command}'")
    process = subprocess.Popen(command, shell=shell, stderr=stderr, stdout=stdout)
    process.wait()
    if process.returncode != 0:
        raise RuntimeError(f"Failed to execute command: '{command}'")


if __name__ == "__main__":
    sys.exit(install_deps())
