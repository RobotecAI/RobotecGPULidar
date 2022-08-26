#include <pipeline/Nodes.hpp>

#include <pcl/io/pcd_io.h>


void WritePCDFileNode::validate()
{
	static const std::vector<rgl_field_t> fmtFields = {XYZ_F32, PADDING_32};  // Needed by PCL
	internalFmt = Node::create<FormatNode>();
	internalFmt->setParameters(fmtFields);
	prependNode(internalFmt);
	internalFmt->validate();
}

void WritePCDFileNode::schedule(cudaStream_t stream)
{
	internalFmt->schedule(stream);
	if (internalFmt->getWidth() == 0) {
		return;
	}
	internalFmt->getData()->hintLocation(VArray::CPU);
	const PCLPointType * data = reinterpret_cast<const PCLPointType*>(internalFmt->getData()->getHostPtr());
	pcl::PointCloud<PCLPointType> cloud;
	cloud.resize(internalFmt->getWidth(), internalFmt->getHeight());
	cloud.assign(data, data + cloud.size(), internalFmt->getWidth());
	cloud.is_dense = internalFmt->isDense();
	cachedPCLs += cloud;
}

WritePCDFileNode::~WritePCDFileNode()
{
	if (cachedPCLs.empty()) {
		RGL_WARN("{}: skipped saving PCD file {} - empty point cloud", getName(), filePath.string());
		return;
	}
	pcl::io::savePCDFileASCII(filePath.string(), cachedPCLs);
}
