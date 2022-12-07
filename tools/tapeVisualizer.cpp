#include "rgl/api/extensions/tape.h"
#include "rgl/api/extensions/visualize.h"
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
	TapePlayer player {argv[1]};

	// Note this will visualize only one (first) graph (lidar)
	// This will be fixed after SpatialMergeNode is implemented
	YAML::iterator compactIt = player.getFirstOf({"rgl_node_points_compact"}).value();
	player.playUntil(compactIt);
	player.playNext();
	rgl_node_t compactNode = player.getNode((*compactIt)[0].as<TapeAPIObjectID>());

	rgl_node_t visualizeNode = nullptr;
	CHECK_RGL(rgl_node_points_visualize(&visualizeNode, "Tape Visualizer", 1920, 1080, false));
	CHECK_RGL(rgl_graph_node_add_child(compactNode, visualizeNode));

	YAML::iterator animationBegin = player.getFirstOf({"rgl_entity_set_pose"}).value();
	YAML::iterator animationEnd = player.getFirstOf({"rgl_entity_destroy", "rgl_mesh_destroy"}).value();

	while (true) {
		player.playUntil(animationEnd);
		player.rewindTo(animationBegin);
	}
}
