#pragma once

#include <rgl/api/experimental.h>

/******************************** NODES ********************************/

/**
 * Creates or modifies VisualizePointsNode.
 * The node creates and manages PCLVisualizer to visualize output point cloud from previous node in the graph.
 * @param node If (*node) == nullptr, a new node will be created. Otherwise, (*node) will be modified.
 * @param window_name The window name.
 * @param window_width The window width (default: 1280).
 * @param window_height The window height (default: 1024).
 * @param fullscreen true for window full screen mode, false otherwise (default: false).
 */
RGL_API rgl_status_t
rgl_node_visualize(rgl_node_t* node, const char* window_name, int window_width = 1280, int window_height = 1024, bool fullscreen = false);
