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
	auto gpuFields = getGPUFields(input, fields, stream);
	char* outputPtr = static_cast<char*>(output->getWritePtr(MemLoc::Device));
	gpuFormat(stream, pointCount, pointSize, fields.size(), gpuFields->getDevicePtr(), outputPtr);
}

VArrayProxy<GPUFieldDesc>::Ptr FormatPointsNode::getGPUFields(IPointsNode::Ptr input, const std::vector<rgl_field_t> &fields, cudaStream_t stream)
{
	auto gpuFields = VArrayProxy<GPUFieldDesc>::create(fields.size());
	std::size_t offset = 0;
	std::size_t gpuFieldIdx = 0;
	for (size_t i = 0; i < fields.size(); ++i) {
		if (!isDummy(fields[i])) {
			(*gpuFields)[gpuFieldIdx] = GPUFieldDesc {
			// TODO(prybicki): distinguish between read / write fields here
			.data = static_cast<const char*>(input->getFieldData(fields[i], stream)->getReadPtr(MemLoc::Device)),
			.size = getFieldSize(fields[i]),
			.dstOffset = offset,
			};
			gpuFieldIdx += 1;
		}
		offset += getFieldSize(fields[i]);
	}
	return gpuFields;
}

VArray::ConstPtr FormatPointsNode::getFieldData(rgl_field_t field, cudaStream_t stream) const
{
	if (field == RGL_FIELD_DYNAMIC_FORMAT) {
		CHECK_CUDA(cudaStreamSynchronize(stream));
		return output;
	}
	return input->getFieldData(field, stream);
}
