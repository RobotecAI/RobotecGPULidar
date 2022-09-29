#pragma once

#include <rgl/api/experimental.h>
#include <math/Mat3x4f.hpp>
#include <VArray.hpp>
#include <VArrayProxy.hpp>
#include <RGLFields.hpp>
#include <gpu/GPUFieldDesc.hpp>

struct IRaysNode
{
	using Ptr = std::shared_ptr<IRaysNode>;
	virtual VArrayProxy<Mat3x4f>::ConstPtr getRays() const = 0;
	virtual std::size_t getRayCount() const = 0;
	virtual std::optional<VArrayProxy<int>::ConstPtr> getRingIds() const = 0;
	virtual std::optional<std::size_t> getRingIdsCount() const = 0;
};

struct IPointcloudDescription
{
	virtual bool hasField(rgl_field_t field) const = 0;
	virtual bool isDense() const = 0;
	virtual std::size_t getWidth() const = 0;
	virtual std::size_t getHeight() const = 0;
	virtual std::size_t getPointCount() const { return getWidth() * getHeight(); }
};

// TODO(prybicki): getFieldData* may act lazily, so they take stream as a parameter to do the lazy evaluation.
// TODO(prybicki): This requires synchronizing with the potentially different stream provided by the schedule(...)
// TODO(prybicki): This situation is bug-prone, requiring greater mental effort when implementing nodes.
// TODO(prybicki): It might be better to remove stream as a parameter and assume that all pipeline nodes are using common stream.
struct IPointCloudNode : public IPointcloudDescription
{
	using Ptr = std::shared_ptr<IPointCloudNode>;

	virtual VArray::ConstPtr getFieldData(rgl_field_t field, cudaStream_t stream) const = 0;

	template<rgl_field_t field>
	typename VArrayProxy<typename Field<field>::type>::ConstPtr getFieldDataTyped(cudaStream_t stream)
	{ return getFieldData(field, stream)->template getTypedProxy<typename Field<field>::type>(); }

	VArrayProxy<GPUFieldDesc>::Ptr getGPUFields(const std::vector<rgl_field_t>& fields, cudaStream_t stream)
	{
	    auto gpuFields = VArrayProxy<GPUFieldDesc>::create(fields.size());
		std::size_t offset = 0;
		std::size_t gpuFieldIdx = 0;
		for (size_t i = 0; i < fields.size(); ++i) {
			if (!isDummy(fields[i])) {
				(*gpuFields)[gpuFieldIdx] = GPUFieldDesc {
					.data = static_cast<char*>(getFieldData(fields[i], stream)->getDevicePtr()),
					.size = getFieldSize(fields[i]),
					.dstOffset = offset,
				};
				gpuFieldIdx += 1;
			}
			offset += getFieldSize(fields[i]);
		}
		return gpuFields;
	}
};
