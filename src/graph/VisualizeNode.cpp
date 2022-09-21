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
	viewer->addPointCloud(cloudPCL);
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

	viewer->updatePointCloud(cloudPCL);
}

VisualizeNode::~VisualizeNode()
{
	visThread.join();
}
