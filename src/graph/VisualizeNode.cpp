#include <graph/Nodes.hpp>

#include <chrono>

void VisualizeNode::setParameters(const char* windowName, int windowWidth, int windowHeight, bool fullscreen)
{
	this->windowName = windowName;
	this->windowWidth = windowWidth;
	this->windowHeight = windowHeight;
	this->fullscreen = fullscreen;

	visThread = std::thread(&VisualizeNode::runVisualize, this);

	// Wait for viewer initialization
	std::this_thread::sleep_for(std::chrono::seconds(1));
}

void VisualizeNode::validate()
{
	input = getValidInput<IPointCloudNode>();
}

void VisualizeNode::runVisualize()
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

	viewer->spin();
}

void VisualizeNode::schedule(cudaStream_t stream)
{
	// Get formatted input data
	VArray::Ptr fmtInputData = FormatNode::formatAsync<char>(input, requiredFields, stream);

	// Convert to PCL cloud
	fmtInputData->hintLocation(VArray::CPU);
	const PCLPointType * data = reinterpret_cast<const PCLPointType*>(fmtInputData->getHostPtr());
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

	// Update
	viewer->updatePointCloud(cloudPCL, pcl::visualization::PointCloudColorHandlerRGBField<PCLPointType>(cloudPCL));
}

VisualizeNode::~VisualizeNode()
{
	visThread.join();
}
