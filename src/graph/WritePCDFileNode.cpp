#include <graph/Nodes.hpp>

#include <pcl/io/pcd_io.h>


void WritePCDFileNode::validate()
{
	input = getValidInput<IPointCloudNode>();
}

void WritePCDFileNode::schedule(cudaStream_t stream)
{
	if (input->getWidth() == 0) {
		return;
	}

	// Get formatted input data
	std::size_t pointSize = 0;
	for (auto&& field : requiredFields) {
		pointSize += getFieldSize(field);
	}
	std::size_t pointCount = input->getPointCount();
	VArray::Ptr fmtInputData = VArray::create<char>(pointCount * pointSize);
	auto gpuFields = VArrayProxy<GPUFieldDesc>::create(requiredFields.size());
	std::size_t offset = 0;
	std::size_t gpuFieldIdx = 0;
	for (size_t i = 0; i < requiredFields.size(); ++i) {
		if (!isDummy(requiredFields[i])) {
			(*gpuFields)[gpuFieldIdx] = GPUFieldDesc {
				.data = static_cast<char*>(input->getFieldData(requiredFields[i], stream)->getDevicePtr()),
				.size = getFieldSize(requiredFields[i]),
				.dstOffset = offset,
			};
			gpuFieldIdx += 1;
		}
		offset += getFieldSize(requiredFields[i]);
	}
	char* fmtInputDataPtr = static_cast<char*>(fmtInputData->getDevicePtr());
	gpuFormat(stream, pointCount, pointSize, requiredFields.size(), gpuFields->getDevicePtr(), fmtInputDataPtr);
	CHECK_CUDA(cudaStreamSynchronize(stream));

	// Convert to PCL cloud
	fmtInputData->hintLocation(VArray::CPU);
	const PCLPointType * data = reinterpret_cast<const PCLPointType*>(fmtInputData->getHostPtr());
	pcl::PointCloud<PCLPointType> cloud;
	cloud.resize(input->getWidth(), input->getHeight());
	cloud.assign(data, data + cloud.size(), input->getWidth());
	cloud.is_dense = input->isDense();
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
