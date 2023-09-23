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

#include <graph/NodesCore.hpp>
#include <graph/NodesPcl.hpp>

void VisualizePointsNode::setParameters(const char* windowName, int windowWidth, int windowHeight, bool fullscreen)
{
	if (!this->windowName.empty()) {
		throw std::invalid_argument("VisualizePointsNode parameters cannot be changed!");
	}
	// Fill data required to create PCLVisualizer
	this->windowName = windowName;
	this->windowWidth = windowWidth;
	this->windowHeight = windowHeight;
	this->fullscreen = fullscreen;

	if (!visualizeThread.has_value()) {
		visualizeThread.emplace();
	}
	std::lock_guard lock { visualizeThread.value().visualizeNodesMutex };
	visualizeThread->visualizeNodes.push_back(std::static_pointer_cast<VisualizePointsNode>(shared_from_this()));
}

void VisualizePointsNode::validate()
{
	input = getValidInput<IPointsNode>();
	if (!input->hasField(XYZ_F32)) {
		auto msg = fmt::format("{} requires XYZ to be present", getName());
		throw InvalidPipeline(msg);
	}
}

// All calls to the viewers must be executed from the same thread
void VisualizePointsNode::VisualizeThread::runVisualize() try
{
	// We cannot initialize iterator without having lock!
	static std::optional<decltype(visualizeNodes)::iterator> it = std::nullopt;
	VisualizePointsNode::Ptr node = nullptr;
	while (!shouldQuit) {
		// Get next node:
		{
			std::lock_guard lock { visualizeNodesMutex };
			if (visualizeNodes.empty()) {
				continue;
			}
			it = [&]() {
				if (!it.has_value()) { // First loop iteration
					return visualizeNodes.begin();
				}
				++(it.value());
				if (it.value() == visualizeNodes.end()) {
					return visualizeNodes.begin();
				}
				return it.value();
			}();
			node = *it.value();

			// Only two references are left - the local one and the one in visualizeNodes
			// Mark node for immediate destruction
			if (node.use_count() == 2) {
				node->eraseRequested = true;
			}
			// Nodes are always destroyed and removed from visualizeNodes list
			// in cooperation with the thread doing ~VisualizePointsNode.
			// Therefore, from visualizeThread, we should never witness VisualizePointsNode just disappearing.
			assert(node != nullptr);
		}

		// Perform lazy initialization of the viewer (pcl requires doing this in visualizeThread)
		if (node->viewer == nullptr) {
			node->viewer = std::make_shared<PCLVisualizerFix>();
			node->viewer->setWindowName(node->windowName);
			node->viewer->setSize(node->windowWidth, node->windowHeight);
			node->viewer->setFullScreen(node->fullscreen);
			node->viewer->setBackgroundColor(0, 0, 0);
			node->viewer->initCameraParameters();
			node->viewer->setShowFPS(false);

			node->viewer->addCoordinateSystem(0.5);
			node->viewer->setCameraPosition(-3, 3, 3, 0.1, -0.1, 1);
			node->viewer->addPointCloud(node->cloudPCL, pcl::visualization::PointCloudColorHandlerRGBField<PCLPointType>(node->cloudPCL));
			node->viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1);
		}

		// Remove node if requested, either by user (GUI) or client's thread
		if (node->eraseRequested || node->viewer->wasStopped()) {
			node->viewer->close();
			std::lock_guard lock { visualizeNodesMutex };
			visualizeNodes.erase(it.value());
			it = std::nullopt; // Restart iterator to avoid incrementing invalid one later.
			if (visualizeNodes.empty()) {
				// To close last viewer window need to call special close function. This is a workaround to bug in vtk.
				// More info in PCLVisualizerFix.hpp
				node->viewer->closeFinalViewer();
			}
			node->isClosed = true;
			node.reset(); // May trigger destructor
			continue;
		}

		// Handle input events and update point cloud if a new one has been delivered by the graph's thread.
		bool forceRedraw = false;
		if (node->hasNewPointCloud) {
			std::lock_guard lock { node->updateCloudMutex };
			node->viewer->updatePointCloud(node->cloudPCL);
			node->hasNewPointCloud = false;
			forceRedraw = true;
		}
		node->viewer->spinOnce(1000 / FRAME_RATE, forceRedraw);
	}
} catch (std::exception& e) {
	RGL_WARN("Visualize thread captured exception: {}", e.what());
}
catch (...) {
	RGL_WARN("Visualize thread captured unknown exception :((");
}


void VisualizePointsNode::schedule(cudaStream_t stream)
{
	if (isClosed) {
		return;  // No need to update point cloud because viewer was closed
	}

	if (input->getPointCount() == 0) {
		std::lock_guard lock { updateCloudMutex };
		cloudPCL->clear();
		hasNewPointCloud = true;
		return;
	}

	// Get formatted input data
	FormatPointsNode::formatAsync(inputFmtData, input, getRequiredFieldList(), stream);

	// Convert to PCL cloud
	const PCLPointType * data = reinterpret_cast<const PCLPointType*>(inputFmtData->getReadPtr(MemLoc::Host));

	std::lock_guard updateLock { updateCloudMutex };

	cloudPCL->resize(input->getWidth(), input->getHeight());
	cloudPCL->assign(data, data + cloudPCL->size(), input->getWidth());
	cloudPCL->is_dense = input->isDense();

	// Colorize
	const auto [minPt, maxPt] = std::minmax_element(cloudPCL->begin(), cloudPCL->end(),
	                                                [] (PCLPointType const &lhs, PCLPointType const &rhs) {
	                                                    return lhs.z < rhs.z;
	                                                });
	float min = (*minPt).z;
	float max = (*maxPt).z;
	float lutScale = 1.0;
	if (min != max) {
		lutScale = 255.0f / (max - min);
	}
	for (auto cloud_it = cloudPCL->begin(); cloud_it != cloudPCL->end(); ++cloud_it) {
		int value = std::lround((cloud_it->z - min) * lutScale);
		value = std::max(std::min(value, 255), 0);
		cloud_it->r = value > 128 ? (value - 128) * 2 : 0;
		cloud_it->g = value < 128 ? 2 * value : 255 - ((value - 128) * 2);
		cloud_it->b = value < 128 ? 255 - (2 * value) : 0;
	}

	hasNewPointCloud = true;
}

VisualizePointsNode::~VisualizePointsNode()
{
	// May be called from client's thread or visualize thread
	eraseRequested = true;
	// TODO fix deadlock here
	while (!isClosed)
		;
}

std::vector<rgl_field_t> VisualizePointsNode::getRequiredFieldList() const
{
	return {XYZ_F32, PADDING_32, PADDING_32, PADDING_32, PADDING_32, PADDING_32};
}
