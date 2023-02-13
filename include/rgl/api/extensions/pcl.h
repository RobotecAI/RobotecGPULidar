// Copyright 2023 Robotec.AI
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <rgl/api/core.h>

/******************************** GRAPH ********************************/

/**
 * Obtains the point cloud of any node in the graph and saves it to given file.
 * Output file will be saved in PCD format (https://pointclouds.org/documentation/tutorials/pcd_file_format.html)
 * @param node Node to get point cloud from.
 * @param file_path Path to the output pcd file.
 */
RGL_API rgl_status_t
rgl_graph_write_pcd_file(rgl_node_t node, const char* file_path);

/******************************** NODES ********************************/

/**
 * Creates or modifies DownSampleNode.
 * The node uses VoxelGrid down-sampling filter from PCL library to reduce the number of points.
 * Graph input: point cloud
 * Graph output: point cloud (downsampled)
 * @param node If (*node) == nullptr, a new node will be created. Otherwise, (*node) will be modified.
 * @param leaf_size_* Dimensions of the leaf voxel passed to VoxelGrid filter.
 */
RGL_API rgl_status_t
rgl_node_points_downsample(rgl_node_t* node, float leaf_size_x, float leaf_size_y, float leaf_size_z);

/**
 * Creates or modifies VisualizePointsNode.
 * The node creates and manages PCLVisualizer to visualize output point cloud from previous node in the graph.
 * Graph input: point cloud
 * Graph output: none
 * @param node If (*node) == nullptr, a new node will be created. Otherwise, (*node) will be modified.
 * @param window_name The window name.
 * @param window_width The window width (default: 1280).
 * @param window_height The window height (default: 1024).
 * @param fullscreen true for window full screen mode, false otherwise (default: false).
 */
RGL_API rgl_status_t
rgl_node_points_visualize(rgl_node_t* node, const char* window_name, int window_width = 1280, int window_height = 1024, bool fullscreen = false);
