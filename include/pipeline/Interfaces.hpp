#pragma once

struct IRaysNode
{
	virtual std::shared_ptr<const VArrayTyped<rgl_mat3x4f>> getRays() const = 0;
};

struct IPointcloudNode
{
	virtual std::shared_ptr<const VArray> getFieldData(rgl_field_t field) const = 0;
};
