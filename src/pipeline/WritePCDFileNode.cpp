#include <pipeline/Nodes.hpp>

#include <pcl/io/pcd_io.h>


void WritePCDFileNode::validate(cudaStream_t stream)
{
	input = getValidInput<IFormatNode>();
	if (input->getPointSize() != sizeof(PCLPointType)) {
		auto msg = fmt::format("{} requires points of size {}, but input has points of size {}",
		                       name(typeid(*this)), sizeof(PCLPointType), input->getPointSize());
		throw InvalidPipeline(msg);
	}
}

void WritePCDFileNode::schedule(cudaStream_t stream)
{
	input->getData()->hintLocation(VArray::CPU, stream);
	CHECK_CUDA(cudaStreamSynchronize(stream));
	const PCLPointType * data = reinterpret_cast<const PCLPointType*>(input->getData()->getHostPtr());
	pcl::PointCloud<PCLPointType> cloud;
	cloud.resize(input->getWidth(), input->getHeight());
	cloud.assign(data, data + cloud.size(), input->getWidth());
	cloud.is_dense = input->isDense();
	cachedPCLs += cloud;
}

WritePCDFileNode::~WritePCDFileNode()
{
	pcl::io::savePCDFileASCII(filePath.string(), cachedPCLs);
}
