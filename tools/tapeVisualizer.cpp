// Copyright 2022 Robotec.AI
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

#include "rgl/api/extensions/tape.h"
#include "rgl/api/extensions/pcl.h"
#include "spdlog/fmt/fmt.h"
#include "Tape.hpp"
#include "macros/checkRGL.hpp"

int main(int argc, char** argv)
{
	if (argc != 2) {
		fmt::print(stderr, "USAGE: {} <path-to-tape-without-suffix>\n", argv[0]);
		return 1;
	}

	fmt::print("Reading tape '{}' ...\n", argv[1]);
	TapePlayer player{ argv[1] };

	// Note this will visualize only one (first) graph (lidar)
	// This will be fixed after SpatialMergeNode is implemented
	YAML::iterator compactIt = player.getFirstOf({ "rgl_node_points_compact" }).value();
	player.playUntil(compactIt);
	player.playNext();
	rgl_node_t compactNode = player.getNode((*compactIt)[0].as<TapeAPIObjectID>());

	rgl_node_t visualizeNode = nullptr;
	CHECK_RGL(rgl_node_points_visualize(&visualizeNode, "Tape Visualizer", 1920, 1080, false));
	CHECK_RGL(rgl_graph_node_add_child(compactNode, visualizeNode));

	YAML::iterator animationBegin = player.getFirstOf({ "rgl_entity_set_pose" }).value();
	YAML::iterator animationEnd = player.getFirstOf({ "rgl_entity_destroy", "rgl_mesh_destroy" }).value();

	while (true) {
		player.playUntil(animationEnd);
		player.rewindTo(animationBegin);
	}
	rgl_cleanup();
}
