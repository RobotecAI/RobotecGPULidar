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
	VArray::Ptr fmtInputData = FormatPointsNode::formatAsync<char>(input, requiredFields, stream);

	// Convert to PCL cloud
	fmtInputData->hintLocation(VArray::CPU);
	const PCLPointType * data = reinterpret_cast<const PCLPointType*>(fmtInputData->getHostPtr());
	pcl::PointCloud<PCLPointType> cloud;
	cloud.resize(input->getWidth(), input->getHeight());
	cloud.assign(data, data + cloud.size(), input->getWidth());
	cloud.is_dense = input->isDense();
	cachedPCLs += cloud;
}

WritePCDFilePointsNode::~WritePCDFilePointsNode()
{
	if (cachedPCLs.empty()) {
		RGL_WARN("{}: skipped saving PCD file {} - empty point cloud", getName(), filePath.string());
		return;
	}
	pcl::io::savePCDFileASCII(filePath.string(), cachedPCLs);
}
