#pragma once

#include <rgl/api/core.h>

/******************************** NODES ********************************/


RGL_API rgl_status_t
rgl_node_points_ros2_publish(rgl_node_t* node, const char* topic_name, const char* frame_id);
