#include <graph/Nodes.hpp>

void VisualizePointsNode::setParameters(const char* windowName, int windowWidth, int windowHeight, bool fullscreen)
{
	if (viewer.get() != nullptr) {
		RGL_WARN("Could not update parameters for VisualizePointsNode.");
		return;
	}

	this->windowName = windowName;
	this->windowWidth = windowWidth;
	this->windowHeight = windowHeight;
	this->fullscreen = fullscreen;

	visThread = std::thread(&VisualizePointsNode::runVisualize, this);
}

void VisualizePointsNode::validate()
{
	input = getValidInput<IPointsNode>();
}

void VisualizePointsNode::runVisualize()
{
	viewer = std::make_shared<PCLVisualizerFix>();
	viewer->setWindowName(windowName);
	viewer->setSize(windowWidth, windowHeight);
	viewer->setFullScreen(fullscreen);
	viewer->setBackgroundColor(0, 0, 0);
	viewer->initCameraParameters();

	viewer->addCoordinateSystem(0.5);
	viewer->setCameraPosition(-3, 3, 3, 0.1, -0.1, 1);

	viewer->addPointCloud(cloudPCL, pcl::visualization::PointCloudColorHandlerRGBField<PCLPointType>(cloudPCL));
	viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1);

	while (!viewer->wasStopped ())
	{
		bool forceRedraw = false;
		{
			std::scoped_lock<std::mutex> updateLock(updateCloudMutex);
			if (isNewCloud) {
				viewer->updatePointCloud(cloudPCL);
				isNewCloud = false;
				forceRedraw = true;
			}
		}
		viewer->spinOnce(1000 / FRAME_RATE, forceRedraw);
	}
}

void VisualizePointsNode::schedule(cudaStream_t stream)
{
	// Get formatted input data
	VArray::Ptr fmtInputData = FormatPointsNode::formatAsync<char>(input, requiredFields, stream);

	// Convert to PCL cloud
	fmtInputData->hintLocation(VArray::CPU);
	const PCLPointType * data = reinterpret_cast<const PCLPointType*>(fmtInputData->getHostPtr());

	std::scoped_lock<std::mutex> updateLock(updateCloudMutex);

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
		lutScale = 255.0 / (max - min);
	}
	for (auto cloud_it = cloudPCL->begin(); cloud_it != cloudPCL->end(); ++cloud_it) {
		int value = std::lround((cloud_it->z - min) * lutScale);
		value = std::max(std::min(value, 255), 0);
		cloud_it->r = value > 128 ? (value - 128) * 2 : 0;
		cloud_it->g = value < 128 ? 2 * value : 255 - ((value - 128) * 2);
		cloud_it->b = value < 128 ? 255 - (2 * value) : 0;
	}

	isNewCloud = true;
}

VisualizePointsNode::~VisualizePointsNode()
{
	visThread.join();
}
