# Robotec GPU Lidar

<h1 align="center">
  <img src="docs/image/rgl-logo.png" width="60%" alt="RobotecGPULidar">
</h1>

## About the project

Robotec GPU Lidar (RGL) is a cross-platform (Windows and Linux), C/C++ library developed by [Robotec.AI](https://robotec.ai/)
for simulating [LiDARs](https://en.wikipedia.org/wiki/Lidar) on CUDA-enabled GPUs, accelerated by RTX cores if available.

One of the use-cases of RGL is implementing Lidar sensors in simulation engines.
We are working on integrations with popular game / simulation engines:
- [Unity](https://unity.com/)
- [O3DE](https://www.o3de.org/)
- [Gazebo](https://gazebosim.org/home)


If you would like to have a custom integration, feel free to [contact us](https://robotec.ai/contact/).

## Features

|   |   |
|---|---|
| **Configurable LiDAR pattern and range** | **High performance** |
| ![](docs/gif/flexible_pattern.GIF) | ![](docs/gif/high_performance.GIF) |
| **GPU-accelerated pointcloud processing** | **Flexible pipeline creation** |
| ![](docs/gif/pointcloud_processing.GIF) | ![](docs/image/flexible_pipeline.png) |

And more:
- Asynchronous raytracing
- Removing non-hit points
- Converting to custom binary output
- Downsampling
- Writing to PCD file
- Optional Gaussian noise (see [documentation](docs/GaussianNoise.md))

## Usage

An introduction to the RGL API along with an example can be found [here](docs/Usage.md).

## Building on Ubuntu

1. Install [CUDA Toolkit](https://developer.nvidia.com/cuda-downloads) 11.2+.
2. Download [NVidia OptiX](https://developer.nvidia.com/designworks/optix/downloads/legacy) 7.2
    1. You may be asked to create Nvidia account to download
3. Export environment variable:
   1. `export OptiX_INSTALL_DIR=<your-OptiX-path>`.
4. Use `setup.bash --cmake --make` script.
   - It will install dependencies from `apt`.
   - It will run CMake and then make.
   - You can pass optional CMake and make parameters, e.g.
     - `./setup.bash --cmake -DCMAKE_BUILD_TYPE=Debug --make -j 16`

## Building on Windows

1. Install [CUDA Toolkit](https://developer.nvidia.com/cuda-downloads) 11.2+.
2. Download [NVidia OptiX](https://developer.nvidia.com/designworks/optix/downloads/legacy) 7.2
   - use the default location or set environment variable `OptiX_INSTALL_DIR`
3. Install [PCL](https://pointclouds.org/) 1.12 on Windows (on Linux this will be done automatically by using a prepared script).
    1. Get [vcpkg](https://vcpkg.io/en/index.html):\
       `git clone -b 2022.08.15 --single-branch --depth 1 https://github.com/microsoft/vcpkg`
    2. Bootstrap `vcpkg`:\
       `.\vcpkg\bootstrap-vcpkg.bat`
    3. Install PCL:\
       `.\vcpkg\vcpkg.exe install pcl[core,visualization]:x64-windows`
    4. In order to use vcpkg with Visual Studio, run the following command (may require administrator elevation):\
       `.\vcpkg\vcpkg.exe integrate install`
    5. In order to use vcpkg with CMake, you can use the toolchain file:\
       `cmake -B [build directory] -S . "-DCMAKE_TOOLCHAIN_FILE=[path to vcpkg]/scripts/buildsystems/vcpkg.cmake"`\
       `cmake --build [build directory]`
4. Build the project:
   - You can use [CLion IDE](https://www.jetbrains.com/clion/) (tested)
   - Alternatively - [cmake-gui](https://cmake.org/download/) and Microsoft Visual Studio

## Troubleshooting

### Linux

To verify that your drivers and libraries are correctly installed, do the following:
*  Run `nvidia-smi` in your command line and see if the output shows the correct version of driver and CUDA (455.28+, 11.2+)
*  Check the CUDA compiler: `nvcc -V` in your command line. It should show the correct version (11.2+). If it points to an old version, make sure you uninstall `nvidia-cuda-toolkit` package and add the following to your library (cuda-11.2 toolkit should be installed if you followed the steps before: `export PATH=/usr/local/cuda-11.2/bin${PATH:+:${PATH}}
   export LD_LIBRARY_PATH=/usr/local/cuda-11.2/lib64\${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}`)
*  Run `locate libnvoptix`. It should point to the 455.28+ version of the library. If you uninstalled the old drivers, run `ubdatedb` first.
*  Run `ls ${OptiX_INSTALL_DIR}` in your terminal. It should list directories of your OptiX SDK.
*  Build OptiX SDK and run `./optixHello` from the `${OptiX_INSTALL_DIR}/SDK/build/bin` directory. It should run without error and show green window.

If all of these work correctly, your environment is likely setup correctly. Some problems are solved by restarting your computer (especially after you change/install drivers).

## Acknowledgements

The development of this project was made possible thanks to cooperation with Tier IV - challenging needs
in terms of features and performance of Tier IV's project allowed to significantly enrich Robotec GPU Lidar
with features such as Gaussian noise and animated meshes as well as optimize it to provide real-time performance with many lidars.