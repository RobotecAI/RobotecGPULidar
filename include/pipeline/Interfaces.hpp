#pragma once

#include <rgl/api/experimental.h>
#include <math/Mat3x4f.hpp>

struct IRaysNode
{
	using Ptr = std::shared_ptr<IRaysNode>;
	virtual std::shared_ptr<const VArrayProxy<Mat3x4f>> getRays() const = 0;
	virtual std::size_t getRayCount() const = 0;
};

struct IPointcloudDescription
{
	virtual bool hasField(rgl_field_t field) const = 0;
	virtual bool isDense() const = 0;
	virtual std::size_t getWidth() const = 0;
	virtual std::size_t getHeight() const = 0;
};

struct IPointCloudNode : public IPointcloudDescription
{
	using Ptr = std::shared_ptr<IPointCloudNode>;

	virtual std::shared_ptr<const VArray> getFieldData(rgl_field_t field, cudaStream_t stream) const = 0;
};

struct IFormatNode : public IPointcloudDescription
{
	using Ptr = std::shared_ptr<IFormatNode>;

	virtual std::shared_ptr<const VArray> getData() const = 0;
	virtual std::size_t getPointSize() const = 0;
};