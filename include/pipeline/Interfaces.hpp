#pragma once

#include <rgl/api/experimental.h>
#include <math/Mat3x4f.hpp>
#include <VArray.hpp>
#include <VArrayProxy.hpp>

struct IRaysNode
{
	using Ptr = std::shared_ptr<IRaysNode>;
	virtual VArrayProxy<Mat3x4f>::ConstPtr getRays() const = 0;
	virtual std::size_t getRayCount() const = 0;
};

struct IPointcloudDescription
{
	virtual bool hasField(rgl_field_t field) const = 0;
	virtual bool isDense() const = 0;
	virtual std::size_t getWidth() const = 0;
	virtual std::size_t getHeight() const = 0;
	virtual std::size_t getPointCount() const { return getWidth() * getHeight(); }
};

struct IPointCloudNode : public IPointcloudDescription
{
	using Ptr = std::shared_ptr<IPointCloudNode>;

	virtual VArray::ConstPtr getFieldData(rgl_field_t field, cudaStream_t stream) const = 0;
};

struct IFormatNode : public IPointcloudDescription
{
	using Ptr = std::shared_ptr<IFormatNode>;

	virtual VArray::ConstPtr getData() const = 0;
	virtual std::size_t getPointSize() const = 0;
};