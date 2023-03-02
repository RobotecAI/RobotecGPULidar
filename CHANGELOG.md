# Change Log

## [0.12.0] 06 February 2023

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
