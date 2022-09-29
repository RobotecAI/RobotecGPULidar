#include <graph/Nodes.hpp>
#include <gpu/nodeKernels.hpp>
#include <RGLFields.hpp>

void FormatNode::setParameters(const std::vector<rgl_field_t>& fields)
{
	if (std::find(fields.begin(), fields.end(), RGL_FIELD_DYNAMIC_FORMAT) != fields.end()) {
		throw InvalidAPIArgument("cannot format field 'RGL_FIELD_DYNAMIC_FORMAT'");
	}
	this->fields = fields;
}

void FormatNode::validate()
{
	input = getValidInput<IPointCloudNode>();
}

void FormatNode::schedule(cudaStream_t stream)
{
	output = formatAsync<char>(input, fields, stream);
}

template<typename T>
VArray::Ptr FormatNode::formatAsync(IPointCloudNode::Ptr input, const std::vector<rgl_field_t>& fields, cudaStream_t stream)
{
	std::size_t pointSize = getPointSize(fields);
	std::size_t pointCount = input->getPointCount();
	VArray::Ptr out = VArray::create<T>(pointCount * pointSize);
	auto gpuFields = input->getGPUFields(fields, stream);
	T* outPtr = static_cast<T*>(out->getDevicePtr());
	gpuFormat(stream, pointCount, pointSize, fields.size(), gpuFields->getDevicePtr(), outPtr);
	return out;
}
