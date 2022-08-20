#include <pipeline/Nodes.hpp>

void streamCallback(cudaStream_t stream, cudaError_t error, void* data)
{
	CHECK_CUDA(error);
	reinterpret_cast<WritePCDFileNode*>(data)->execute(stream);
}

void WritePCDFileNode::validate()
{
	// pclNode = getValidInput<IPointCloudNode>();
	// if (!pclNode->hasField(RGL_FIELD_XYZ_F32)) {
	// 	auto msg = fmt::format("{} requires XYZ format which was not provided by {}",
	// 	                       name(typeid(*this)), name(typeid(*pclNode)));
	// 	throw InvalidPipeline(msg);
	// }
	// if (internalFormatNode == nullptr) {
	// 	internalFormatNode = std::make_shared<FormatNode>();
	// 	internalFormatNode->setParameters({RGL_FIELD_XYZ_F32, RGL_FIELD_PADDING_32});
	// }
	// internalFormatNode->validate();
}

void WritePCDFileNode::schedule(cudaStream_t stream)
{
	// pclNode->getFieldData(RGL_PCD_FIELD, stream)->getTypedProxy<RGLPointType>()->hintLocation(VArray::CPU);
	// CHECK_CUDA(cudaStreamAddCallback(stream, streamCallback, this, 0));
}

void WritePCDFileNode::execute(cudaStream_t stream)
{
	// VArrayProxy<RGLPointType>::ConstPtr data = pclNode->getFieldData(RGL_PCD_FIELD, stream)->getTypedProxy<RGLPointType>();
	// const auto* dataPtr = reinterpret_cast<const PCLPointType*>(data->getHostPtr());
	// pcl::PointCloud<PCLPointType> cloud;
	// cloud.resize(pclNode->getWidth(), pclNode->getHeight());
	// cloud.assign(dataPtr, dataPtr + data->getCount(), pclNode->getWidth());
	// cloud.is_dense = pclNode->isDense();
	// cachedPCLs += cloud;
}

WritePCDFileNode::~WritePCDFileNode()
{
	// pcl::io::savePCDFileASCII(filePath.string(), cachedPCLs);
}
