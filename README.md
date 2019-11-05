# Lidar simulator with the use of Nvidia Optix reycast

Application simulating Lidar rays on GPU with Nvidia Optix Technology. Our target is Ubuntu 18.04. It has also been tested with Windows and Visual Studio 2017, 

### Dependencies

* **CUDA 10.1**
    * Downloads available from developer.nvidia.com
    * Add /usr/local/cuda/bin to your PATH
* **OptiX 7 SDK**
    * Dowload available here: [NVIDIA-Optix](http://developer.nvidia.com/optix)"
    * Linux: set the environment variable `OptiX_INSTALL_DIR` to wherever you installed the SDK.  
      `export OptiX_INSTALL_DIR=<wherever you installed OptiX 7.0 SDK>`
    * Windows: the installer should automatically put it into the right directory

### Building (Linux)
* Install required packages  
    `sudo apt install libglfw3-dev cmake-curses-gui`  
* Clone the code  
    `git clone https://gitlab.com/robotec.ai/volvo-cpac/optixlidar.git`  
   `cd optixlidar`  
* Build the project with cmake  
    `mkdir build`  
    `cd build`  
    `cmake ..`
    `make`  

### Building (Windows)
* **Using Visual Studio (recommended)**
    * Install Required Packages
        * see above: CUDA 10.1, OptiX 7 SDK, latest driver, and cmake
    * download or clone the source repository
    *  In Visual Studio choose 'File > Open > CMake' to open CMakeLists.txt file
        *  More details: [Microsoft CMake](https://docs.microsoft.com/en-us/cpp/build/cmake-projects-in-visual-studio?view=vs-2019)
* **Using CMake GUI**
    * Install Required Packages
        * see above: CUDA 10.1, OptiX 7 SDK, latest driver, and cmake
    * download or clone the source repository
    * Open CMake GUI from your start menu
     * point "source directory" to the downloaded source directory
     * point "build directory" to /build (agree to create this directory when prompted)
     * click 'configure', then specify the generator as Visual Studio 2017 or 2019, and the Optional platform as x64. If CUDA, SDK, and compiler are all properly installed this should enable the 'generate' button. If not, make sure all dependencies are properly installed, "clear cache", and re-configure.
     * click 'generate' (this creates a Visual Studio project and solutions)
     * click 'open project' (this should open the project in Visual Studio)



