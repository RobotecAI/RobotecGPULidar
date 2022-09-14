#include <graph/Nodes.hpp>
#include <gpu/nodeKernels.hpp>
#include <RGLFields.hpp>

void FormatNode::validate()
{
	input = getValidInput<IPointCloudNode>();
}

void FormatNode::schedule(cudaStream_t stream)
{
	std::size_t pointSize = getPointSize(fields);
	std::size_t pointCount = input->getPointCount();
	output->resize(pointCount * pointSize, false, false);
	auto gpuFields = getGPUFields(fields, input, stream);
	char* outputPtr = static_cast<char*>(output->getDevicePtr());
	gpuFormat(stream, pointCount, pointSize, fields.size(), gpuFields->getDevicePtr(), outputPtr);
}
