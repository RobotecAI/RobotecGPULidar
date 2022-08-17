#pragma once

struct IRaysNode
{
	virtual std::shared_ptr<const VArrayProxy<Mat3x4f>> getRays() const = 0;
};

struct IPointcloudNode
{
	virtual std::shared_ptr<const VArray> getFieldData(rgl_field_t field) const = 0;
};
