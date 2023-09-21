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
	if (viewers.contains(windowName)) {
		throw InvalidAPIArgument("Visualizer with the same window name already exist. Parameters cannot be updated.");
	}

	this->windowName = windowName;
	std::lock_guard<std::mutex> vdLock(modifyViewersMutex);
	// Cannot create and copy/move ViewerData because of mutex (must be constructed in-place)
	viewers.emplace(std::piecewise_construct, std::make_tuple(windowName), std::make_tuple());

	// Fill data required to create PCLVisualizer
	viewers[windowName].windowWidth = windowWidth;
	viewers[windowName].windowHeight = windowHeight;
	viewers[windowName].fullscreen = fullscreen;

	if (!visThread.joinable()) {
		visThread = std::thread(VisualizePointsNode::runVisualize);
	}
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
void VisualizePointsNode::runVisualize()
{
	using namespace std::literals;
	std::optional<std::string> lastSpunViewerName = std::nullopt;
	while (!viewers.empty()) {
		// Need to sleep to allow locking modifyViewersMutex from other thread
		std::this_thread::sleep_for(1ms);
		std::lock_guard<std::mutex> vdLock(modifyViewersMutex);
		bool allViewerClosed = true;
		for (auto&& [windowName, vd] : viewers) {
			// Create a new viewer
			if (vd.viewer.get() == nullptr) {
				vd.viewer = std::make_shared<PCLVisualizerFix>();
				vd.viewer->setWindowName(windowName);
				vd.viewer->setSize(vd.windowWidth, vd.windowHeight);
				vd.viewer->setFullScreen(vd.fullscreen);
				vd.viewer->setBackgroundColor(0, 0, 0);
				vd.viewer->initCameraParameters();
				vd.viewer->setShowFPS(false);

				vd.viewer->addCoordinateSystem(0.5);
				vd.viewer->setCameraPosition(-3, 3, 3, 0.1, -0.1, 1);

				vd.viewer->addPointCloud(vd.cloudPCL,
				                         pcl::visualization::PointCloudColorHandlerRGBField<PCLPointType>(vd.cloudPCL));
				vd.viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1);
			}

			if (vd.isClosed.load()) {
				continue;
			}

			// Check if viewer was stopped
			if (vd.viewer->wasStopped() || vd.isViewerStoppedByRglNode) {
				vd.viewer->close();
				vd.isClosed.store(true);
				fmt::print("closed {}\n", windowName);
				continue;
			}

			allViewerClosed = false;

			// Update point cloud and render
			bool forceRedraw = false;
			{
				std::lock_guard<std::mutex> updateLock(vd.updateCloudMutex);
				if (vd.isNewCloud) {
					vd.viewer->updatePointCloud(vd.cloudPCL);
					vd.isNewCloud = false;
					forceRedraw = true;
				}
			}
			fmt::print("spinning {}\n", windowName);
			vd.viewer->spinOnce(1000 / FRAME_RATE, forceRedraw);
			lastSpunViewerName = windowName;
		}

		// To close last viewer window need to call special close function. This is a workaround to bug in vtk.
		// More info in PCLVisualizerFix.hpp
		if (allViewerClosed && lastSpunViewerName.has_value()) {
			viewers[lastSpunViewerName.value()].viewer->closeFinalViewer();
			lastSpunViewerName.reset();
		}
	}
}

void VisualizePointsNode::schedule(cudaStream_t stream)
{
	if (viewers[windowName].isClosed.load()) {
		return;  // No need to update point cloud because viewer was closed
	}

	if (input->getPointCount() == 0) {
		auto& viewerData = viewers[windowName];
		std::lock_guard<std::mutex> updateLock(viewerData.updateCloudMutex);
		viewerData.cloudPCL->clear();
		viewerData.isNewCloud = true;
		return;
	}

	// Get formatted input data
	FormatPointsNode::formatAsync(inputFmtData, input, getRequiredFieldList(), stream);

	// Convert to PCL cloud
	const PCLPointType * data = reinterpret_cast<const PCLPointType*>(inputFmtData->getReadPtr(MemLoc::Host));

	auto& viewerData = viewers[windowName];
	std::lock_guard<std::mutex> updateLock(viewerData.updateCloudMutex);

	viewerData.cloudPCL->resize(input->getWidth(), input->getHeight());
	viewerData.cloudPCL->assign(data, data + viewerData.cloudPCL->size(), input->getWidth());
	viewerData.cloudPCL->is_dense = input->isDense();

	// Colorize
	const auto [minPt, maxPt] = std::minmax_element(viewerData.cloudPCL->begin(), viewerData.cloudPCL->end(),
	                                                [] (PCLPointType const &lhs, PCLPointType const &rhs) {
	                                                    return lhs.z < rhs.z;
	                                                });
	float min = (*minPt).z;
	float max = (*maxPt).z;
	float lutScale = 1.0;
	if (min != max) {
		lutScale = 255.0f / (max - min);
	}
	for (auto cloud_it = viewerData.cloudPCL->begin(); cloud_it != viewerData.cloudPCL->end(); ++cloud_it) {
		int value = std::lround((cloud_it->z - min) * lutScale);
		value = std::max(std::min(value, 255), 0);
		cloud_it->r = value > 128 ? (value - 128) * 2 : 0;
		cloud_it->g = value < 128 ? 2 * value : 255 - ((value - 128) * 2);
		cloud_it->b = value < 128 ? 255 - (2 * value) : 0;
	}

	viewerData.isNewCloud = true;
}

VisualizePointsNode::~VisualizePointsNode()
{
	if (!viewers.contains(windowName)) {
		return;
	}

	if (!viewers[windowName].isClosed.load()) {
		fmt::print("closing {}\n", windowName);
		viewers[windowName].isViewerStoppedByRglNode = true;
	}

	// Wait for close
	while (!viewers[windowName].isClosed.load()) {;}

	fmt::print("want to erase {}\n", windowName);

	{
		std::lock_guard<std::mutex> vdLock(modifyViewersMutex);
		viewers.erase(windowName);
	}

	if (viewers.empty()) {
		visThread.join();
	}
}

std::vector<rgl_field_t> VisualizePointsNode::getRequiredFieldList() const
{
	return {XYZ_F32, PADDING_32, PADDING_32, PADDING_32, PADDING_32, PADDING_32};
}
