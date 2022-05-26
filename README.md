# Robotec GPU Lidar

## About the project

Robotec GPU Lidar (RGL) is a cross-platform (Windows and Linux), C/C++ library developed by [Robotec.AI](https://robotec.ai/)
for simulating LiDARs (computing a point cloud) on CUDA-enabled GPUs, accelerated by RTX cores if available.

One of the use-cases of RGL is implementing Lidar sensor in simulation engines. We provide integrations to some of the popular engines:
- [Unity](TODO)
- O3DE (planned)
- Gazebo (planned)

If you would like to have a custom integration, feel free to [contact us](https://robotec.ai/contact/). 

## Usage

An introduction to the RGL API along with an example can be found [here](USAGE.md).

## Building

**Note: you can use a pre-built binary version available on GitHub.**

1. Install [CUDA Toolkit](https://developer.nvidia.com/cuda-downloads) 11.2+.
2. Download [NVidia OptiX](https://developer.nvidia.com/designworks/optix/download) 7.4
   1. You may be asked to create Nvidia account to download
3. If you are on Linux or you have chosen non-standard location on Windows when installing OptiX, you need to export environment variable `OptiX_INSTALL_DIR`.
4. Proceed with a standard CMake build procedure:
   - Linux:
      - `mkdir build && cd build && cmake ../ && make`
   - On Windows
     - You can use [CLion IDE](https://www.jetbrains.com/clion/) (recommended)
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
