import os
import sys
import re
import subprocess
import shutil
import argparse

CUDA_MIN_VER_MAJOR = 11
CUDA_MIN_VER_MINOR = 4
VCPKG_INSTALL_DIR = "external\\vcpkg"
VCPKG_TAG = "2022.08.15"

parser = argparse.ArgumentParser(description="Helper script to build RGL on Windows")
parser.add_argument("--build-dir", type=str, nargs=1, default="build",
                    help="Path to build directory. Default: 'build'")
parser.add_argument("--rgl-install-deps", action='store_true',
                    help="Install RGL dependencies and exit")
parser.add_argument("--rgl-clean-build", action='store_true',
                    help="Remove build directory before cmake")
parser.add_argument("--rgl-with-ros2", action='store_true',
                    help="Build RGL with ROS2 extension")
parser.add_argument("--rgl-with-ros2-standalone", action='store_true',
                    help="Build RGL with ROS2 extension in standalone mode")
parser.add_argument("--cmake", type=str, nargs=argparse.REMAINDER, action='append',
                    help="Pass arguments to cmake. Takes all remaining arguments")
args = parser.parse_args()

# Install RGL dependencies
if args.rgl_install_deps:
    if not os.path.isdir(VCPKG_INSTALL_DIR):
        vcpkg_clone_process = subprocess.Popen(f"git clone -b {VCPKG_TAG} --single-branch --depth 1 https://github.com/microsoft/vcpkg {VCPKG_INSTALL_DIR}", shell=True, stderr=sys.stderr, stdout=sys.stdout)
        vcpkg_clone_process.wait()
    if not os.path.isfile(os.path.join(VCPKG_INSTALL_DIR, "vcpkg.exe")):
        vcpkg_bootstrap_process = subprocess.Popen(f"{os.path.join(VCPKG_INSTALL_DIR, 'bootstrap-vcpkg.bat')}", shell=True, stderr=sys.stderr, stdout=sys.stdout)
        vcpkg_bootstrap_process.wait()
    vcpkg_install_process = subprocess.Popen(f"{os.path.join(VCPKG_INSTALL_DIR, 'vcpkg.exe')} install --clean-after-build pcl[core,visualization]:x64-windows", shell=True, stderr=sys.stderr, stdout=sys.stdout)
    vcpkg_install_process.wait()
    sys.exit(0)

# Check CUDA
nvcc_process = subprocess.run("nvcc --version", shell=True, stdout=subprocess.PIPE)
nvcc_ver_match = re.search("V[0-9]+.[0-9]+.[0-9]+", nvcc_process.stdout.decode("utf-8"))
if not nvcc_ver_match:
    print("CUDA not found")
    sys.exit(1)
cuda_major = int(nvcc_ver_match[0].split(".")[0][1:])
cuda_minor = int(nvcc_ver_match[0].split(".")[1])
if cuda_major < CUDA_MIN_VER_MAJOR or (cuda_major == CUDA_MIN_VER_MAJOR and cuda_minor < CUDA_MIN_VER_MINOR):
    print(f"CUDA version not supported! Get CUDA {CUDA_MIN_VER_MAJOR}.{CUDA_MIN_VER_MINOR}+")
    sys.exit(1)

# Check OptiX_INSTALL_DIR
if os.environ["OptiX_INSTALL_DIR"] == "":
    print("OptiX not found! Make sure you have exported environment variable OptiX_INSTALL_DIR")
    sys.exit(1)

os.chdir(sys.path[0])
# Extend Path with libRobotecGPULidar location to link tests properly during the build
os.environ["Path"] = os.environ["Path"] + ";" + os.path.join(os.getcwd(), args.build_dir)

# Build
if args.rgl_clean_build and os.path.isdir(args.build_dir):
    os.system(f"rmdir /S /Q {args.build_dir}")
if not os.path.isdir(args.build_dir):
    os.makedirs(args.build_dir)

cmake_args = f"-DCMAKE_TOOLCHAIN_FILE={os.path.join(VCPKG_INSTALL_DIR, 'scripts', 'buildsystems', 'vcpkg.cmake')} "
cmake_args += "-DRGL_BUILD_TOOLS=OFF "  # On Windows not available
cmake_args += "-DRGL_BUILD_ROS2_EXTENSION={} ".format("ON" if args.rgl_with_ros2 else "OFF")
cmake_args += "-DRGL_BUILD_ROS2_EXTENSION_STANDALONE={} ".format("ON" if args.rgl_with_ros2_standalone else "OFF")
if args.cmake is not None:
    for arg in args.cmake[0]:
        cmake_args += f"{arg} "

print(f"Building RGL with cmake args: {cmake_args}")
cmake_process = subprocess.Popen(f"cmake -B{args.build_dir} -G Ninja --install-prefix {os.path.join(os.getcwd(), args.build_dir)} {cmake_args}", shell=True, stderr=sys.stderr, stdout=sys.stdout)
cmake_process.wait()

make_process = subprocess.Popen(f"cmake --build {args.build_dir}", shell=True, stderr=sys.stderr, stdout=sys.stdout)
make_process.wait()

if args.rgl_with_ros2_standalone:
    install_process = subprocess.Popen(f"cmake --install {args.build_dir}", shell=True, stderr=sys.stderr, stdout=sys.stdout)
    install_process.wait()
