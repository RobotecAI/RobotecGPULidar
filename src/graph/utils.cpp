#include <graph/utils.hpp>

std::size_t getPointSize(const std::vector<rgl_field_t>& fields)
{
	std::size_t size = 0;
	for (auto&& field : fields) {
		size += getFieldSize(field);
	}
	return size;
}

VArrayProxy<GPUFieldDesc>::Ptr getGPUFields(const std::vector<rgl_field_t>& fields,
                                            IPointCloudNode::Ptr input, cudaStream_t stream)
{
    auto gpuFields = VArrayProxy<GPUFieldDesc>::create(fields.size());
	std::size_t offset = 0;
	std::size_t gpuFieldIdx = 0;
	for (size_t i = 0; i < fields.size(); ++i) {
		if (!isDummy(fields[i])) {
			(*gpuFields)[gpuFieldIdx] = GPUFieldDesc {
				.data = static_cast<char*>(input->getFieldData(fields[i], stream)->getDevicePtr()),
				.size = getFieldSize(fields[i]),
				.dstOffset = offset,
			};
			gpuFieldIdx += 1;
		}
		offset += getFieldSize(fields[i]);
	}
    return gpuFields;
}
