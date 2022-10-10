#include <graph/Nodes.hpp>

#include <pcl/io/pcd_io.h>


void WritePCDFilePointsNode::validate()
{
	input = getValidInput<IPointsNode>();
}

void WritePCDFilePointsNode::schedule(cudaStream_t stream)
{
	if (input->getWidth() == 0) {
		return;
	}

	// Get formatted input data
	FormatPointsNode::formatAsync(inputFmtData, input, requiredFields, stream);

	// Convert to PCL cloud
	inputFmtData->hintLocation(VArray::CPU);
	const PCLPointType * data = reinterpret_cast<const PCLPointType*>(inputFmtData->getHostPtr());
	pcl::PointCloud<PCLPointType> cloud;
	cloud.resize(input->getWidth(), input->getHeight());
	cloud.assign(data, data + cloud.size(), input->getWidth());
	cloud.is_dense = input->isDense();
	cachedPCLs += cloud;
	inputFmtData->hintLocation(VArray::GPU);
}

WritePCDFilePointsNode::~WritePCDFilePointsNode()
{
	if (cachedPCLs.empty()) {
		RGL_WARN("{}: skipped saving PCD file {} - empty point cloud", getName(), filePath.string());
		return;
	}
	pcl::io::savePCDFileASCII(filePath.string(), cachedPCLs);
}
