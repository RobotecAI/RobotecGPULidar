# Lidar simulator with the use of Nvidia Optix reycast

Application simulating Lidar rays on GPU with Nvidia Optix Technology. Our target is Ubuntu 20.04. It has also been tested with Windows and Visual Studio 2017, 

### Dependencies

* **CUDA 11.2 or later**
    * Downloads available from developer.nvidia.com
    * Add /usr/local/cuda/bin to your PATH
* **OptiX 7.2 SDK**
    * Dowload available here: [NVIDIA-Optix](http://developer.nvidia.com/optix)"
    * Linux: 
        * set the environment variable `OptiX_INSTALL_DIR` to wherever you installed the SDK.  
        `export OptiX_INSTALL_DIR=<wherever you installed OptiX 7.2 SDK>`. Best add this line to your ~/.bashrc file.
        * Optix 7.2 requires NVidia drivers version 455.28 or later. Make sure you update to the version your GPU supports. You may need to uninstall older drivers. Rebooting your machine after such reinstall is necessary.
        * Install compiler, dkms, libxi, xinerama
        `sudo apt -y install build-essential dkms libglfw3-dev pkg-config libglvnd-dev cmake cmake-curses-gui libxinerama-dev libxcursor-dev`
    * Windows: the installer should automatically put it into the right directory
* **git-lfs** - note that if you were using simulation before, you have it installed. Otherwise see below

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
*  Check the CUDA compiler: `nvcc -V` in your command line. It should show the correct version (11.2+).
*  Run `locate libnvoptix`. It should point to the 455.28+ version of the library. If you uninstalled the old drivers, run `ubdatedb` first.
*  Run `ls ${OptiX_INSTALL_DIR}` in your terminal. It should list directories of your OptiX SDK.
*  Build OptiX SDK and run `./optixHello` from the `${OptiX_INSTALL_DIR}/SDK/build/bin` directory. It should run without error and show green window.

If all of these work correctly, your environment is likely setup correctly. Some problems are solved by restarting your computer (especially after you change/install drivers).

### Building (Linux)
* Clone the code  
    `git clone https://gitlab.com/robotec.ai/volvo-cpac/optixlidarsimulator.git`  
   `cd optixlidar`  
* Download LFS files
    `git lfs pull`
* Build the project with cmake  
    `mkdir build`  
    `cd build`   
    `cmake ..`     
    `make`


### Building (Windows)
* **Using Visual Studio (recommended)**
    * Install Required Packages
        * see above: CUDA 11.2, OptiX 7.2 SDK, latest driver, and cmake
    * download or clone the source repository
    *  In Visual Studio choose 'File > Open > CMake' to open CMakeLists.txt file
        *  More details: [Microsoft CMake](https://docs.microsoft.com/en-us/cpp/build/cmake-projects-in-visual-studio?view=vs-2019)
* **Using CMake GUI**
    * Install Required Packages
        * see above: CUDA 11.2, OptiX 7.2 SDK, latest driver, and cmake
    * download or clone the source repository
    * Open CMake GUI from your start menu
     * point "source directory" to the downloaded source directory
     * point "build directory" to /build (agree to create this directory when prompted)
     * click 'configure', then specify the generator as Visual Studio 2017 or 2019, and the Optional platform as x64. If CUDA, SDK, and compiler are all properly installed this should enable the 'generate' button. If not, make sure all dependencies are properly installed, "clear cache", and re-configure.
     * click 'generate' (this creates a Visual Studio project and solutions)
     * click 'open project' (this should open the project in Visual Studio)



