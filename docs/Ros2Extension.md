# RGL ROS2 extension

The extension introduces nodes to publish [PointCloud2](https://docs.ros2.org/foxy/api/sensor_msgs/msg/PointCloud2.html) messages to ROS2. RGL creates a ROS2 node named `RobotecGPULidar` and registers publishers based on constructed RGL nodes.

Supported ROS2 distributions:
- Humble
- Jazzy

Supported DDS implementations for ROS2 standalone build:
- Eclipse Cyclone DDS
- eProsima Fast DDS

## Building

RGL ROS2 extension can be built in two flavors:

- **standalone** - ROS2 installation is not required on the target machine. RGL build will include all required ROS2 dependencies.
- **overlay** - Assumes the existence of supported ROS2 installation on the target machine. RGL will use ROS2 libraries already installed on the system.

Before building RGL ROS2 extension, it is necessary to get the required dependencies.
For some, the process has been automated - run `setup.py --install-ros2-deps` to get them.

### Ubuntu 22/24

#### Prerequisites

- Requirements listed in the main [README](../README.md#building-on-ubuntu-2224).
- ROS2 installed on the system and sourced:
  - ROS2 Humble for Ubuntu 22
  - ROS2 Jazzy for Ubuntu 24
- Radar messages installed:
  ```bash
  apt install -y ros-${ROS_DISTRO}-radar-msgs
  ```
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
    - You can specify run-time search path(s) of RGL dependencies by adding the option `--lib-rpath <paths>`. This can be useful if you want RGL to search for ROS2 libraries in a different directory than RGL, e.g., if you work with a library that also provides its ROS2 libraries, such as [ROS2ForUnity](https://github.com/RobotecAI/ros2-for-unity).
2. Copy all ROS2 libraries from `<build-dir>/lib/ros2_standalone/` to `libRobotecGPULidar.so` location (or location defined with `--lib-rpath`).

#### Tips for integrating RGL with [AWSIM](https://github.com/tier4/AWSIM/tree/main)

AWSIM project includes two ROS-based plugins: [ROS2ForUnity](https://github.com/RobotecAI/ros2-for-unity) and [RGLUnityPlugin](https://github.com/tier4/AWSIM/tree/main/Assets/RGLUnityPlugin). Since AWSIM is configured to run in a standalone mode (where ROS does not need to be installed on the host machine), all necessary ROS libraries are included within the project.
Ros2ForUnity and RGLUnityPlugin share a common set of ROS libraries (e.g., `librcl.so`).
To avoid duplication - since Unity does not support having multiple instances of the same libraries - one of the solutions is to store all ROS libraries in a single plugin.
In this case, the ROS libraries are stored in the ROS2ForUnity plugin, and RGL is configured with a runtime search path to access them.

1. Build RGL with the command:
    ```bash
    ./setup.py --with-ros2-standalone --lib-rpath \$ORIGIN/../../../../Ros2ForUnity/Plugins/Linux/x86_64/
    ```
    *$ORIGIN represents the directory in which an object (library) originated. It is resolved at run-time.*\
    *Your rpath may differ. It depends on your relative location between RGLUnityPlugin and Ros2ForUnity.*
2. Copy library `<build-dir>/lib/libRobotecGPULidar.so` to the appropriate directory in your RGLUnityPlugin.
3. Copy all ROS2 libraries from `<build-dir>/lib/ros2_standalone/` to `Ros2ForUnity\Plugins\Linux\x86_64` directory. **Skip for duplicates**.

### Windows

#### Prerequisites

- Requirements listed in the main [README](../README.md#building-on-windows).
- ROS2 installed on the system and sourced. We recommend installation from a pre-built binary package:
  - [ROS2 Humble](https://docs.ros.org/en/humble/Installation/Windows-Install-Binary.html)
  - [ROS2 Jazzy](https://docs.ros.org/en/jazzy/Installation/Windows-Install-Binary.html)
- [ROS2 Humble only] Fixed ROS2 logging macros in rclcpp package to make it compile with C++20. More about this bug: [github PR](https://github.com/ros2/rclcpp/pull/2063).
  - Use `fix_ros2_humble.py` script to apply those changes:
    ```bash
    python ros2_standalone\fix_ros2_humble.py <your-path-to-ros2>
    ```

#### Steps for overlay
1. Run `x64 Native Tools Command Prompt for VS 2019` and navigate to RGL repository.
2. Run `python setup.py --with-ros2` command to build RGL with ROS2 extension.

#### Steps for standalone
1. Run `x64 Native Tools Command Prompt for VS 2019` and navigate to RGL repository.
2. Run `python setup.py --with-ros2-standalone` command to build RGL with ROS2 extension and install ROS2 libraries.
3. Copy all ROS2 libraries from `<build-dir>/lib/ros2_standalone/` into `RobotecGPULidar.dll` location, or extend environment variable `Path` appropriately.

#### Tips for integrating RGL with [AWSIM](https://github.com/tier4/AWSIM/tree/main)

See description from [Ubuntu section](#tips-for-integrating-rgl-with-awsim) for more details.

1. Build RGL with ROS2 standalone as described above.
2. Copy `<build-dir>/RobotecGPULidar.dll` and all dependent libraries located in `<build-dir>` to the appropriate directory in your RGLUnityPlugin.
3. Copy all ROS2 libraries from `<build-dir>/lib/ros2_standalone/` to `Ros2ForUnity\Plugins\Windows\x86_64` directory. Skip for duplicates.

*Note:* On Windows there is no `-rpath` linker flag to modify an executable’s search path. Instead, the `Path` environment variable can be set. ROS2ForUnity handles this automatically before the simulation starts.

## Usage

Each RGL node for ROS2 publishing must be connected to a format node that defines fields and their layout in the binary data. For example, to publish PointCloud2 message with fields XYZ and DISTANCE, the code should look as follow:
```c
...
rgl_node_t nodeFormat = nullptr, nodeRos2 = nullptr;
std::vector<rgl_field_t> fieldsToPublish = { RGL_FIELD_XYZ_VEC3_F32, RGL_FIELD_DISTANCE_F32 };
rgl_node_points_format(&nodeFormat, fieldsToPublish.data(), fieldsToPublish.size())
rgl_node_points_ros2_publish(&nodeRos2, "example_topic", "example_frame");

rgl_graph_node_add_child(..., nodeFormat);
rgl_graph_node_add_child(nodeFormat, nodeRos2);
```

## API documentation

More details can be found [here](../include/rgl/api/extensions/ros2.h).
