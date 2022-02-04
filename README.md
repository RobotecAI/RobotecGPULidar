# Robotec GPU Lidar

GPU-based implementation of raycasting algorithm employing specialized NVIDIA RTX hardware through  NVIDIA OptiX.

This repository contains:
- Native source code (C++/CUDA) for the native Robotec GPU Lidar library
  - src/
  - include/
- C# wrapper
  - dotnet_module/
- Helper code from Ingo Wald on Apache License 2.0 (to be removed in the future)
  - common/

Current features include:
- [x] Adding meshes to a global map.
- [x] Removing meshes from a global map.
- [x] Computing raycast results for a single lidar at a time.

Future plans/ideas include:
- [ ] (Re)-add support for computing intensity from (intensity or RGB) textures.
- [ ] Add support for LiDARs in motion (simulating skewed pointcloud).
- [ ] Add support for bulk computations (multiple LiDARs in a single call).
- [ ] Offload auxiliary operations like generating ray directions and filtering raycast results to the GPU.
- [ ] Provide stable C API to allow usage from other languages.
- [ ] Develop automated tests and benchmarks.
- [ ] ROS2 integration (automatic PCL publishing).
- [ ] Optimize out remaining inefficiencies, i.e. copy meshes immediately to the GPU.


### Dependencies
Our target is Ubuntu 20.04.

**NOTE: Currently Windows support is suspended, but it will be resumed.**

* **CUDA 11.2 or later**
    * Downloads available from developer.nvidia.com
    * Add /usr/local/cuda/bin to your PATH
* **OptiX 7.2 SDK**
    * Download available here: [NVIDIA-Optix](http://developer.nvidia.com/optix)
    * Linux: 
        * set the environment variable `OptiX_INSTALL_DIR` to wherever you installed the SDK.  
        `export OptiX_INSTALL_DIR=<wherever you installed OptiX 7.2 SDK>`. Best add this line to your ~/.bashrc file.
        * Optix 7.2 requires NVidia drivers version 455.28 or later. Make sure you update to the version your GPU supports. You may need to uninstall older drivers. Rebooting your machine after such reinstall is necessary.
        * Install compiler, dkms, libxi, xinerama
        `sudo apt -y install build-essential dkms libglfw3-dev pkg-config libglvnd-dev cmake cmake-curses-gui libxinerama-dev libxcursor-dev`
    * Windows: the installer should automatically put it into the right directory
* **libfmt-dev**
  * Linux: `sudo apt -y install libfmt-dev`
* **.net5.0**
  * Needed for tests only,
  * Download available here: [.net](https://dotnet.microsoft.com/download/dotnet/)
* **git-lfs** - See below.

#### Build prerequisites: Git LFS

The project contains large files such as model textures and meshes. We use [Git LFS](https://git-lfs.github.com/) (Large File Storage) for the purpose of managing large files in a seamless way. 
First, you need to install the package:
*  Ubuntu: `sudo apt-get install git-lfs`
*  Windows: Download and install [Git LFS](https://git-lfs.github.com/).

Then, in both cases, run the following command in a terminal or Command Prompt: `$ git lfs install`

Git LFS will automatically bootstrap for LFS repositories and dowload all files through the normal `git clone` command. Note that the clone command can now take a while as it needs
to download all the files, but subsequent updates will be much faster.

### Checking the installation, troubleshooting

To verify that your drivers and libraries are correctly installed, do the following:
*  Run `nvidia-smi` in your command line and see if the output shows the correct version of driver and CUDA (455.28+, 11.2+)
*  Check the CUDA compiler: `nvcc -V` in your command line. It should show the correct version (11.2+). If it points to an old version, make sure you uninstall `nvidia-cuda-toolkit` package and add the following to your library (cuda-11.2 toolkit should be installed if you followed the steps before: `export PATH=/usr/local/cuda-11.2/bin${PATH:+:${PATH}}
export LD_LIBRARY_PATH=/usr/local/cuda-11.2/lib64\${LD_LIBRARY_PATH:+:${LD_LIBRARY_PATH}}`)
*  Run `locate libnvoptix`. It should point to the 455.28+ version of the library. If you uninstalled the old drivers, run `ubdatedb` first.
*  Run `ls ${OptiX_INSTALL_DIR}` in your terminal. It should list directories of your OptiX SDK.
*  Build OptiX SDK and run `./optixHello` from the `${OptiX_INSTALL_DIR}/SDK/build/bin` directory. It should run without error and show green window.

If all of these work correctly, your environment is likely setup correctly. Some problems are solved by restarting your computer (especially after you change/install drivers).

### Building (Linux)
* Clone the code  
    `git clone git@gitlab.com:robotec.ai/internal/simulation-platform/modules/lidar-raycasting-gpu.git`  
    `cd lidar-raycasting-gpu`  
* Download LFS files
    `git lfs pull`
* Build the project with cmake  
    `mkdir build`  
    `cd build`   
    `cmake ..` or `cmake .. -DBUILD_TESTS=true` to build with tests     
    `make`

[comment]: <> (## Running tests)

[comment]: <> (* Make sure you have tests built:   )

[comment]: <> (    `cmake .. -DBUILD_TESTS=true`   )

[comment]: <> (* To run all tests:   )

[comment]: <> (    `make test`    )

[comment]: <> (    or   )

[comment]: <> (    `ctest --output-on-failure` to see more verbose output on failure.)

[comment]: <> (* For a certain test:   )

[comment]: <> (    `ctest -R <TEST_NAME> --verbose`)

[comment]: <> (### Building &#40;Windows&#41;)

[comment]: <> (* **Using Visual Studio &#40;recommended&#41;**)

[comment]: <> (    * Install Required Packages)

[comment]: <> (        * see above: CUDA 11.2, OptiX 7.2 SDK, latest driver, and cmake)

[comment]: <> (    * download or clone the source repository)

[comment]: <> (    *  In Visual Studio choose 'File > Open > CMake' to open CMakeLists.txt file)

[comment]: <> (        *  More details: [Microsoft CMake]&#40;https://docs.microsoft.com/en-us/cpp/build/cmake-projects-in-visual-studio?view=vs-2019&#41;)

[comment]: <> (* **Using CMake GUI**)

[comment]: <> (    * Install Required Packages)

[comment]: <> (        * see above: CUDA 11.2, OptiX 7.2 SDK, latest driver, and cmake)

[comment]: <> (    * download or clone the source repository)

[comment]: <> (    * Open CMake GUI from your start menu)

[comment]: <> (     * point "source directory" to the downloaded source directory)

[comment]: <> (     * point "build directory" to /build &#40;agree to create this directory when prompted&#41;)

[comment]: <> (     * click 'configure', then specify the generator as Visual Studio 2017 or 2019, and the Optional platform as x64. If CUDA, SDK, and compiler are all properly installed this should enable the 'generate' button. If not, make sure all dependencies are properly installed, "clear cache", and re-configure.)

[comment]: <> (     * click 'generate' &#40;this creates a Visual Studio project and solutions&#41;)

[comment]: <> (     * click 'open project' &#40;this should open the project in Visual Studio&#41;)

