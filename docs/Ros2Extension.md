# RGL ROS2 extension

The extension introduces the node to publish [PointCloud2](https://docs.ros2.org/foxy/api/sensor_msgs/msg/PointCloud2.html) messages to ROS2. RGL creates a ROS2 node named `RobotecGPULidar` and registers publishers based on constructed RGL nodes.

Supported ROS2 distributions:
- Humble

Supported DDS implementations for ROS2 standalone build:
- Eclipse Cyclone DDS
- eProsima Fast DDS

## Building

RGL ROS2 extension can be built in two flavors:

- **standalone** - ROS2 installation is not required on the target machine. RGL build will include all required ROS2 dependencies.
- **overlay** - Assumes the existence of supported ROS2 installation on the target machine. RGL will try to use the existing installation of ROS2 dependencies.


### Ubuntu 22

#### Prerequisites

- Requirements listed in the main [README](../README.md) (section `Building on Ubuntu`).
- ROS2 installed on the system and sourced.
- For standalone build:
  - ROS2 `cyclonedds` and `fastrtps` packages.
  - `patchelf` tool.
    ```bash
    apt install -y ros-${ROS_DISTRO}-cyclonedds ros-${ROS_DISTRO}-rmw-cyclonedds-cpp
    apt install -y ros-${ROS_DISTRO}-fastrtps ros-${ROS_DISTRO}-rmw-fastrtps-cpp
    apt install patchelf
    ```

#### Steps for overlay
1. Use script `setup.py` with option `--with-ros2`.

#### Steps for standalone
1. Use script `setup.py` with option `--with-ros2-standalone`
    - You can specify run-time search path(s) of RGL dependencies by adding the option `--lib-rpath <paths>`. This can be useful if you want RGL to search for ROS2 libraries in a different directory than RGL, e.g., if you work with a library providing its own ROS2 libraries, such as [ROS2ForUnity](https://github.com/RobotecAI/ros2-for-unity).
2. Copy all ROS2 libraries from `<build-dir>/ros2_standalone/` to `libRobotecGPULidar.so` location (or location defined with `--lib-rpath`).

#### Tips for integrating RGL + ROS2 standalone with Unity and [ROS2ForUnity](https://github.com/RobotecAI/ros2-for-unity) plugin.
1. Build RGL with the command:
    ```bash
    ./setup.py --with-ros2-standalone --lib-rpath \$ORIGIN/../../../../Ros2ForUnity/Plugins/Linux/x86_64/
    ```
    *$ORIGIN represents the directory in which an object (library) originated. It is resolved at run-time.*\
    *Your rpath may differ. It depends on your relative location between RGLUnityPlugin and Ros2ForUnity.*
2. Copy library `<build-dir>/libRobotecGPULidar.so` to the appropriate directory in your RGLUnityPlugin.
3. Copy all ROS2 libraries from `<build-dir>/ros2_standalone/` to `Ros2ForUnity\Plugins\Linux\x86_64` directory. Skip for duplicates.

RGL's ROS2 standalone build is dependent on ROS2ForUnity's ROS2 standalone build. RobotecGPULidar library will find ROS2 because we have set the appropriate rpath for the ROS2 libraries in ROS2ForUnity.

### Windows

#### Prerequisites

- Requirements listed in the main [README](../README.md) (section `Building on Windows`).
- ROS2 installed on the system and sourced.
- Fixed ROS2 logging macros in rclcpp package to make it compile with C++20. More about this bug: [github PR](https://github.com/ros2/rclcpp/pull/2063).
  - Use `fix_ros2_humble.py` script to apply those changes:
    ```bash
    python ros2_standalone\fix_ros2_humble.py <your-path-to-ros2>
    ```
- For standalone build:
  - If you have installed ROS2 from pre-built binaries, there is `rmw_cyclonedds_cpp` package missing. You need to build this one from the source:
    1. Make sure you have installed [ROS2 prerequisites](https://docs.ros.org/en/humble/Installation/Alternatives/Windows-Development-Setup.html#installing-prerequisites).
    2. A good practice is to disable Windows path limits, [see](https://learn.microsoft.com/en-us/windows/win32/fileio/maximum-file-path-limitation?tabs=registry).
    3. Run `x64 Native Tools Command Prompt for VS 2019`, setup development folder (e.g., `C:\rosrmw`), clone ROS2 repos, and build:
        ```bash
        md \rosrmw\src
        cd \rosrmw
        vcs import --input https://raw.githubusercontent.com/ros2/ros2/humble/ros2.repos src
        colcon build --merge-install --packages-up-to rmw_cyclonedds_cpp
        ``` 
    4. Before building RGL, source `rosrmw` workspace: `call C:\rosrmw\install\setup.bat`.


#### Steps for overlay
1. Run `x64 Native Tools Command Prompt for VS 2019` and navigate to RGL repository.
2. Run `python setup.py --with-ros2` command to build RGL with ROS2 extension.

#### Steps for standalone
1. Run `x64 Native Tools Command Prompt for VS 2019` and navigate to RGL repository.
2. Run `python setup.py --with-ros2-standalone` command to build RGL with ROS2 extension and install ROS2 libraries.
3. Copy all ROS2 libraries from `<build-dir>/ros2_standalone/` into `RobotecGPULidar.dll` location, or extend environment variable `Path` appropriately.

#### Tips for integrating RGL + ROS2 standalone with Unity and [ROS2ForUnity](https://github.com/RobotecAI/ros2-for-unity) plugin.
1. Build RGL with ROS2 standalone as described above.
2. Copy `<build-dir>/RobotecGPULidar.dll` and all depend libraries located in `<build-dir>` to the appropriate directory in your RGLUnityPlugin.
3. Copy all ROS2 libraries from `<build-dir>/ros2_standalone/` to `Ros2ForUnity\Plugins\Windows\x86_64` directory. Skip for duplicates.

RGL's ROS2 standalone build is dependent on ROS2ForUnity's ROS2 standalone build. RobotecGPULidar library will find ROS2 because ROS2ForUnity sets the environment variable `Path` for the ROS2 libraries.

## Usage

Each RGL node for ROS2 publishing must be connected to a format node that defines fields and their layout in the binary data. For example, to publish PointCloud2 message with fields XYZ and DISTANCE, the code should look as follow:
```c
...
rgl_node_t nodeFormat = nullptr, nodeRos2 = nullptr;
std::vector<rgl_field_t> fieldsToPublish = { RGL_FIELD_XYZ_F32, RGL_FIELD_DISTANCE_F32 };
rgl_node_points_format(&nodeFormat, fieldsToPublish.data(), fieldsToPublish.size())
rgl_node_points_ros2_publish(&nodeRos2, "example_topic", "example_frame");

rgl_graph_node_add_child(..., nodeFormat);
rgl_graph_node_add_child(nodeFormat, nodeRos2);
```

## API documentation

More details can be found [here](../include/rgl/api/extensions/ros2.h).
