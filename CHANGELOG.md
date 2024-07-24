# Change Log

## [0.18.0] 24 July 2024

### Added

- Added radar object tracking feature
  - Added API call to produce point cloud with radar objects
    - `rgl_node_points_radar_track_objects`
- Added mechanism to simulate fault injection
  - Added API call to mask groups of rays
    - `rgl_node_raytrace_configure_mask`
- Added multi-return simulation
  - Currently, this feature is limited to outputting a single return point cloud, but the user can select which return mode is simulated (first or last)
  - Added API call to configure beam divergence
    - `rgl_node_raytrace_configure_beam_divergence`
  - Added API call to switch between return modes:
    - `rgl_node_multi_return_switch`
- Added new fields: `RGL_FIELD_INTENSITY_U8` and `RGL_FIELD_TIME_STAMP_U32`
  - These fields are duplicates of existing fields but are stored in other data types
- Added default intensity configuration
  - Added API call to configure default intensity
    - `rgl_node_raytrace_configure_default_intensity`
  - Default intensity is considered when hitting entities with no intensity texture set

### Changed

- Renamed ROS2 field name for `RGL_FIELD_RING_ID`
  - `ring` -> `channel`
- Changed the content of time stamp fields (`RGL_FIELD_TIME_STAMP_F64` and `RGL_FIELD_TIME_STAMP_U32`)
  - Previously, it was time on the scene when the raytrace is triggered
  - Now, it holds the time have passed since the time of the raytrace trigger when given point was measured
    - If velocity distortion is disabled, the time stamp for all points will be zero
- Intensity value of the point is based on intensity texture of the entity and the incident angle
  - Ihe influence of the incident angle was added

### Removed

- Removed deprecated API call
  - `rgl_node_points_compact`
  - Please use `rgl_node_points_compact_by_field` instead.

## [0.17.0] 11 June 2024

### Added
- Added radar sensor simulation
  - Added API call to process point cloud to create radar-like output
    - `rgl_node_points_radar_postprocess`
  - Added API call to remove ground using RANSAC method to fit the plane model to the point cloud
    - `rgl_node_points_remove_ground`
  - Added API call to filter ground based on the incident angle of the ray hitting the mesh triangle
    - `rgl_node_points_filter_ground`
  - Added API call to publish [RadarScan](http://docs.ros.org/en/noetic/api/radar_msgs/html/msg/RadarScan.html) message into ROS2 topic
    - `rgl_node_publish_ros2_radarscan`
  - Added new fields (point attributes) calculation:
    - Velocity of the hit point on the entity. Depends on entity's linear and angular velocity, and mesh deformations (inferred from calls `rgl_entity_set_pose`, `rgl_scene_set_time` and `rgl_mesh_update_vertices`).
      - `RGL_FIELD_ABSOLUTE_VELOCITY_VEC3_F32`
      - `RGL_FIELD_RELATIVE_VELOCITY_VEC3_F32`
      - `RGL_FIELD_RADIAL_SPEED_F32`
    - Normal vector of the mesh triangle where the hit-point is located
      - `RGL_FIELD_NORMAL_VEC3_F32`
    - Incident angle of the ray hitting the mesh triangle
      - `RGL_FIELD_INCIDENT_ANGLE_F32`
    - Azimuth and elevation angle of the hit point
      - `RGL_FIELD_AZIMUTH_F32`
      - `RGL_FIELD_ELEVATION_F32`
- Added API call to compact point cloud by given field (`RGL_FIELD_IS_HIT_I32` or `RGL_FIELD_IS_GROUND_I32`)
  - `rgl_node_points_compact_by_field`
  - At the same time, `rgl_node_points_compact` became deprecated
- Added set of calls to check if API objects are still valid
  - `rgl_mesh_is_alive`
  - `rgl_entity_is_alive`
  - `rgl_texture_is_alive`
  - `rgl_node_is_alive`
- Added raytrace node configurations
  - Added API call to configure values for non-hit points
    - `rgl_node_raytrace_configure_non_hits`
  - Added API call to configure sensor velocity (necessary for velocity distortion or calculating fields `RGL_FIELD_RELATIVE_VELOCITY_VEC3_F32` and `RGL_FIELD_RADIAL_SPEED_F32`)
    - `rgl_node_raytrace_configure_velocity`
  - Added API call to configure (turn on/off) velocity distortion
    - `rgl_node_raytrace_configure_distortion`
- Added support for LiDAR reflective value (similar to intensity but set as a single floating-point value for the entire entity)
  - Added API call to set laser retro value to the entity
    - `rgl_entity_set_laser_retro`
  - Note: could be replaced with 1x1 float-type texture in the future
- Added CI to the project using GitHub actions
- Added static build option
  - Added variable to the CMakeLists (default OFF):
    - `RGL_BUILD_STATIC`

### Changed
- Optimized tape files format to be lighter and be loaded faster
  - Note: old tape files are not compatible
- Improved docker-based build pipeline
  - Introduced multistage dockerfile for the build process
  - Decoupled dependency installation from compilation scripts (better caching)

### Removed
- Removed API call for modifying RaytraceNode to apply velocity distortion
  - `rgl_node_raytrace_with_distortion`
  - Replaced by `rgl_node_raytrace_configure_distortion`

## [0.16.2] 19 November 2023

### Fixed

- Fixed Gaussian noise nodes to always perform in rays coordinates
- Fixed distance Gaussian noise to apply error based on distance traveled by the ray (not distance to rays coordinates origin)
- Fixed graph validation when rays definition is modified

## [0.16.1] 13 November 2023

### Fixed

- Fixed a bug introduced in v0.16.0 causing RGL to crash in Compact and Downsample Node when some field was removed from the graph.

## [0.16.0] 10 November 2023

### Added

- Added API calls to set relative node's priority, which determines their execution order within a graph (but not between graphs).
  - `rgl_graph_node_set_priority`
  - `rgl_graph_node_get_priority`

### Changed

- `rgl_graph_run(...)` is now fully asynchronous, i.e. it will initiate computations without waiting for them to finish
  - RGL API calls attempting to obtain results from a given node will block until the node has completed execution
  - RGL API calls modifying scene will block until all graphs have completed execution
  - Running a graph will block until the previous run has been completed
- Changed internal implementation of Entity ID feature
  - The Entity ID is now held in a 28-bit integer (OptiX limit) instead of a 32-bit integer
- Renamed XYZ field name to be more precise and compatible with future features
  - `RGL_FIELD_XYZ_F32` -> `RGL_FIELD_XYZ_VEC3_F32`

### Known Issues

- Modifying fields between graph's runs (consisted of Compact or Downsample nodes) may cause a segmentation fault. Fixed in v0.16.1.
- Gaussian noise is not performed in rays coordinates. Fixed in v0.16.2.
- Modifying rays definition between graph's runs may allow computations on invalid pipeline. Fixed in v0.16.2.

## [0.15.0] 23 September 2023

### Added

- Added feature to simulate velocity distortion
  - Added API call to specify the delay between firing each ray:
    - `rgl_node_rays_set_time_offsets`
  - Added API call to perform raytracing with velocity distortion:
    - `rgl_node_raytrace_with_distortion`
  - Note: it is a simplified version of that feature
    - The distortion takes into account only sensor velocity
    - The velocity of the objects being scanned by the sensor is not considered
- Improved ray range configuration
  - Added API call to define min and max range for rays:
    - `rgl_node_rays_set_range`
  - The range for each ray can be specified individually
  - Modified API call:
    - `rgl_node_raytrace`
    - No longer takes range as a parameter
- Improved tape tools
  - Multi-lidar tapes are now supported by `tapeVisualizer`

### Fixed

- Fixed `-march=native` imposed compilation option
  - Library builds are compatible with all 64-bit CPUs
  - `vcpkg` tag has been updated
  - `PCL` version has been upgraded (1.13.0 -> 1.13.1)
- Fixed missing dependencies for ROS2 standalone build
- Fixed tape for API call `rgl_entity_set_id`
- Fixed `rgl_node_points_visualize`
  - Fixed blocking spin loop for up to 1s on some machines
  - Fixed segmentation fault on window close
  - Fixed handling multiple windows at the same time
- Fixed linking the tests on Windows

### Known Issues
- Destroying `rgl_node_points_visualize` may deadlock on some machines if multiple windows are spawn
- Destroying `rgl_node_points_visualize` may not close window immediately - main thread exit needed

## [0.14.1] 5 July 2023

### Fixed

- Fixed building `PCL` on Windows
  - `vcpkg` tag has been updated
  - `PCL` version has been upgraded (1.12.0 -> 1.13.0)
- Fixed `rclcpp` (`ROS2` package) initialization
  - `RGL` checks whether `rclcpp` is already initialized
  - Resolved `spdlog` conflict between `RGL` and `rclcpp` by downgrading its version on the `RGL` site (1.10.0 -> 1.9.2)

## [0.14.0] 22 June 2023

### Added

- Added feature to query in runtime if specific extensions were compiled in the binary
  - Added API call:
    - `rgl_get_extension_info`
  - Created a tool that prints all the extensions in the given RGL binary:
    - `inspectLibRGL`
- Added instance/semantic segmentation feature
  - Added API call to set the ID of the entity:
    - `rgl_entity_set_id`
  - Added a new field that can be returned:
    - `RGL_FIELD_ENTITY_ID_I32`
- Added material information reading and converting to intensity based on intensity texture assigned to the entity:
  - Added API calls:
    - `rgl_mesh_set_texture_coords`
    - `rgl_entity_set_intensity_texture`
    - `rgl_texture_create`
    - `rgl_texture_destroy`
- Added publishing raw lidar packets via UDP
  - In the closed-source version only
- Added unity tests to field `RGL_FIELD_DISTANCE_F32`

### Changed

- Changed value of non-hits points from `CUDART_INF_F` to `FLT_MAX`
- Updated docker README information

### Known Issues
- `rclcpp` (`ROS2` package) is always initialized by `RGL`. It could cause a double initialization if the client's code also did it before `RGL`.
  - Fixed in v0.14.1

## [0.13.1] 19 April 2023

### Fixed
- Improved performance of temporal merge node.
  - The node doubles the array capacity if it's running out (instead of reallocating every iteration).

### Changed
- Point cloud formatting for `rgl_graph_write_pcd_file` is performed on the CPU now.
  - We prefer to avoid transferring huge point cloud to GPU (risk of cuda out of memory error).

### Known Issues
- `rgl_graph_write_pcd_file` causes SEH exception on Windows when trying to save point cloud with ~375 000 000 or more points.
  - The issue has been reported to PCL ([link](https://github.com/PointCloudLibrary/pcl/issues/5674)).
- `rclcpp` (`ROS2` package) is always initialized by `RGL`. It could cause a double initialization if the client's code also did it before `RGL`.
  - Fixed in v0.14.1

## [0.13.0] 29 March 2023

### Added
- Reimplemented Gaussian Noise from RGL v10
  - Added API calls:
    - `rgl_node_gaussian_noise_angular_ray`
    - `rgl_node_gaussian_noise_angular_hitpoint`
    - `rgl_node_gaussian_noise_distance`
- Added nodes for spatial and temporal point clouds merge
  - Added API calls:
    - `rgl_node_points_spatial_merge`
    - `rgl_node_points_temporal_merge`
- Added node providing a user-defined point cloud to the RGL pipeline (for testing purposes)
  - Added API call:
    - `rgl_node_points_from_array`
- Added parameterized tests for:
  - `rgl_node_points_from_array`
  - `rgl_node_points_transform`
- Updated API surface tests
- Added support for multi-raytrace graphs

### Changed
- Separated PCL nodes and created PCL extension
- Converted node `WritePCDFilePointsNode` into single API call
  - Removed API call `rgl_node_points_write_pcd_file`
  - Added API call `rgl_graph_write_pcd_file`

### Fixed
- Fixed `--build-dir` flag in `setup.py` script

### Removed
- Removed `rgl_graph_node_set_active` API call

### Known Issues
- `rclcpp` (`ROS2` package) is always initialized by `RGL`. It could cause a double initialization if the client's code also did it before `RGL`.
  - Fixed in v0.14.1
- Distance Gaussian noise:
  - Noise is added in wrong coordinate system.
  - Error is calculated based on distance to rays coordinates origin (not distance traveled by the ray)
  - All fixed in v0.16.2

## [0.12.0] 8 March 2023

### Added
- ROS2 extension providing a node to publish point cloud into ROS2 topic
  - Added API calls (`rgl/api/extensions/ros2.h`):
    - `rgl_node_points_ros2_publish`
    - `rgl_node_points_ros2_publish_with_qos`
  - Necessary scripting to build ROS2 integration in the standalone mode (not requiring ROS2 installation)
- API call for setting time on the scene
- TapeVisualizer tool (PoC)
  - Currently, it does not handle multiple graphs in the tape, to be improved

### Changed
- Rewritten build script from bash to python to enable easy building on Windows

### Fixed
- Minor fixes and refactor in the Tape code
  - Use safer .at() instead of operator[]
  - Using alias type APIObjectID instead of size_t

### Known Issues
- `rclcpp` (`ROS2` package) is always initialized by `RGL`. It could cause a double initialization if the client's code also did it before `RGL`.
  - Fixed in v0.14.1

## [0.11.3] 11 January 2023

### Added
- API call for tape recording activation status

### Fixed
- Fixed dependencies for tapePlayer tool
- Handle invalid logger configuration properly
- Handle empty path passed to tape

## [0.11.2] 1 December 2022

### Added
- RGL Auto Tape - feature to start Tape on the first API call writing to a path given in compile-time. 

### Changed
- Improved control flow around API initialization and error reporting

### Fixed
- Release build on Linux no longer use debug builds of dependencies (e.g., libPCL)
- Fixed a hidden bug in CacheManager - unsafe removing while iterating

## [0.11.1] 28 November 2022

### Fixed
- CUDA Runtime is now linked statically.

## [0.11.0] - 22 November 2022

The initial release.
