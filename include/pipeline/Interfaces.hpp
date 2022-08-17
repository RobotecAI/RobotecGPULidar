#pragma once

struct IRaysNode
{
	using Ptr = std::shared_ptr<IRaysNode>;
	virtual std::shared_ptr<const VArrayProxy<Mat3x4f>> getRays() const = 0;
};

struct IPointcloudNode
{
	using Ptr = std::shared_ptr<IPointcloudNode>;
	virtual std::shared_ptr<const VArray> getFieldData(rgl_field_t field) const = 0;
	virtual bool hasField(rgl_field_t field) const = 0;
	virtual bool isDense() const = 0;
	virtual std::size_t getWidth() const = 0;
	virtual std::size_t getHeight() const = 0;
};
