# RGL Agnocast extension

The extension Agnocast enables ROS2 publishing RGL nodes to utilize Agnocast for enhanced performance and capabilities.

## About Agnocast

To learn more about Agnocast please visit:

- [Agnocast announcement discussion](https://github.com/orgs/autowarefoundation/discussions/5835)
- [Agnocast repository](https://github.com/tier4/agnocast)

## Prerequisites

The Agnocast extension is part of the ROS2 extension. You must build and use Agnocast together with the ROS2 extension.

Agnocast extension is supported on Ubuntu only.

## Installation

### Automated Dependencies

Install most dependencies automatically by running:
```bash
setup.py --install-agnocast-deps
```

### Manual Dependencies

Two components require manual installation:
- `agnocast-heaphook`
- `agnocast-kmod`

**Installation steps:**
1. Follow the setup script: [Agnocast setup script](https://github.com/tier4/agnocast/blob/2.1.1/scripts/setup)
2. **Version compatibility:** Ensure you use the same version specified in the [install_deps script](https://github.com/RobotecAI/RobotecGPULidar/blob/develop/extensions/ros2/install_agnocast_deps.py)

### Building the Extension

Build the extension using the setup script with the Agnocast flag:
```bash
setup.py --with-agnocast
```

**Note:** The ROS2 extension must also be enabled during the build process.

**Note:** Agnocast libraries will be also installed for ROS2 standalone build.

## Usage

After creating an RGL ROS2 publishing node, enable Agnocast using the `rgl_node_configure_agnocast` API:
```c
...
rgl_node_t nodeFormat = nullptr, nodeRos2 = nullptr;
std::vector<rgl_field_t> fieldsToPublish = { RGL_FIELD_XYZ_VEC3_F32, RGL_FIELD_DISTANCE_F32 };
rgl_node_points_format(&nodeFormat, fieldsToPublish.data(), fieldsToPublish.size())
rgl_node_points_ros2_publish(&nodeRos2, "example_topic", "example_frame");
rgl_node_configure_agnocast(nodeRos2, true);

rgl_graph_node_add_child(..., nodeFormat);
rgl_graph_node_add_child(nodeFormat, nodeRos2);
```

## API documentation

More details can be found [here](../include/rgl/api/extensions/ros2.h).
