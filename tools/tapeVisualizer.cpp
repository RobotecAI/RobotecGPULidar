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

constexpr std::string_view INTERCEPTED_NODE_CALL = "rgl_node_points_compact";

// Do not modify, code does not handle > 1 FIELD_COUNT (!)
constexpr size_t FIELDS_COUNT = 1;
constexpr rgl_field_t FIELDS[FIELDS_COUNT] = {RGL_FIELD_XYZ_F32};

struct InputNode
{
	InputNode()
	{
		points.resize(1);
		update(std::nullopt);
	}

	void update(std::optional<rgl_node_t> source)
	{
		if (source.has_value()) {
			int32_t count, sizeOf;
			CHECK_RGL(rgl_graph_get_result_size(source.value(), FIELDS[0], &count, &sizeOf));
			points.resize(count);
			CHECK_RGL(rgl_graph_get_result_data(source.value(), FIELDS[0], points.data()));
		}
		CHECK_RGL(rgl_node_points_from_array(&fromArrayNode, points.data(), points.size(), FIELDS, FIELDS_COUNT));
	}

	rgl_node_t fromArrayNode = nullptr;

private:
	std::vector<rgl_vec3f> points;
};

int main(int argc, char** argv)
try {
	if (argc != 2) {
		fmt::print(stderr, "USAGE: {} <path-to-tape-without-suffix>\n", argv[0]);
		std::exit(EXIT_FAILURE);
	}

	fmt::print("Reading tape '{}' ...\n", argv[1]);
	TapePlayer player{argv[1]};

	std::vector<TapePlayer::APICallIdx> interceptedNodeCalls = player.findAll({INTERCEPTED_NODE_CALL});
	if (interceptedNodeCalls.empty()) {
		throw std::runtime_error("no nodes to intercept");
	}

	// Uncomment to get only specific lidar
	// std::vector<TapePlayer::APICallIdx> fix;
	// fix.push_back(interceptedNodeCalls[2]);
	// interceptedNodeCalls = fix;

	// Collect handles to intercepted nodes
	player.playThrough(interceptedNodeCalls.back());
	std::vector<rgl_node_t> interceptedNodes;
	for (auto&& call : interceptedNodeCalls) {
		// Those are rgl_node_*, so the first argument will be node
		auto nodeId = player.getCallArg<TapeAPIObjectID>(call, 0);
		interceptedNodes.push_back(player.getNodeHandle(nodeId));
	}
	fmt::print("Found {} nodes to intercept\n", interceptedNodes.size());

	std::vector<TapePlayer::APICallIdx> graphRuns = player.findAll({"rgl_graph_run"});
	std::unordered_map<TapeAPIObjectID, bool> runnableNodeState;
	for (auto&& run : graphRuns) {
		runnableNodeState.insert({player.getCallArg<TapeAPIObjectID>(run, 0), false});
	}
	fmt::print("Tape executes rgl_graph_run() on {} unique nodes\n", runnableNodeState.size());

	// Create separate visualization graph: UsePoints* -> SpatialMerge -> Visualize
	// It might be tempting to just add spatial merge as a child of intercepted nodes
	// However, that would potentially join multiple original graphs into a single one
	// which is a bad idea here, because any rgl_graph_run would run all existing nodes.
	std::vector<InputNode> inputNodes{interceptedNodes.size()};
	rgl_node_t visualizeParent = inputNodes.back().fromArrayNode, visualizeNode = nullptr;
	if (inputNodes.size() > 1) {
		visualizeParent = nullptr;
		CHECK_RGL(rgl_node_points_spatial_merge(&visualizeParent, FIELDS, FIELDS_COUNT));
		for (auto&& in : inputNodes) {
			CHECK_RGL(rgl_graph_node_add_child(in.fromArrayNode, visualizeParent));
		}
	}
	CHECK_RGL(rgl_node_points_visualize(&visualizeNode, "Tape Visualizer", 1920, 1080, false));
	CHECK_RGL(rgl_graph_node_add_child(visualizeParent, visualizeNode));

	//	// Here we assume that rgl_entity_set_pose is AFTER NODE_CREATION_CALLS
	//	// TODO: fix repeating visualization
	//	auto animationBegin = player.findFirst({"rgl_entity_set_pose"}).value();
	//	player.playUntil(animationBegin);
	//	auto animationEnd = player.findLast({"rgl_graph_run"}).value() + 3; // get_size + get_data + 1

	for (auto&& [node, executed] : runnableNodeState) {
		executed = false;
	}

	// TODO: this will not work if LiDARS have different capture rate.
	for (auto&& run : graphRuns) {
		// Run till next run call
		auto runNodeArg = player.getCallArg<TapeAPIObjectID>(run, 0);
		player.playThrough(run);
		runnableNodeState[runNodeArg] = true;

		// Check if all runnable were run
		bool allExecuted = true;
		for (auto&& [node, executed] : runnableNodeState) {
			allExecuted &= executed;
		}

		// Update visualization once all runnable were executed
		if (allExecuted) {
			for (int i = 0; i < interceptedNodes.size(); i++) {
				inputNodes[i].update(interceptedNodes[i]);
			}
			CHECK_RGL(rgl_graph_run(visualizeNode));
			// Synchronize visualization graph
			int32_t count, sizeOf;
			CHECK_RGL(rgl_graph_get_result_size(visualizeNode, RGL_FIELD_XYZ_F32, &count, &sizeOf));
			fmt::print("Visualized {} points\n", count);

			// Reset runnable nodes status
			for (auto&& [node, executed] : runnableNodeState) {
				executed = false;
			}
		}
	}
	player.playUntil();
}
catch (std::exception& e) {
	fmt::print(stderr, "Exception was thrown: {}\n", e.what());
	fmt::print(stderr, "You may need to adjust tapeVisualizer source code to your specific tape\n");
	std::exit(1);
}
