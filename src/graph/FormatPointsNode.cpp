#include <graph/Nodes.hpp>
#include <gpu/nodeKernels.hpp>
#include <RGLFields.hpp>

void FormatPointsNode::setParameters(const std::vector<rgl_field_t>& fields)
{
	if (std::find(fields.begin(), fields.end(), RGL_FIELD_DYNAMIC_FORMAT) != fields.end()) {
		throw InvalidAPIArgument("cannot format field 'RGL_FIELD_DYNAMIC_FORMAT'");
	}
	this->fields = fields;
}

void FormatPointsNode::validate()
{
	input = getValidInput<IPointsNode>();
}

void FormatPointsNode::schedule(cudaStream_t stream)
{
	formatAsync(output, input, fields, stream);
}

void FormatPointsNode::formatAsync(const VArray::Ptr& output, const IPointsNode::Ptr& input,
                                   const std::vector<rgl_field_t>& fields, cudaStream_t stream)
{
	std::size_t pointSize = getPointSize(fields);
	std::size_t pointCount = input->getPointCount();
	output->resize(pointCount * pointSize, false, false);
	auto gpuFields = input->getGPUFields(fields, stream);
	char* outputPtr = static_cast<char*>(output->getDevicePtr());
	gpuFormat(stream, pointCount, pointSize, fields.size(), gpuFields->getDevicePtr(), outputPtr);
}
