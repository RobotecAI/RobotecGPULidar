#pragma once

struct IRaysProvider
{
	virtual std::shared_ptr<const VArrayTyped<rgl_mat3x4f>> getRays() const = 0;
};

struct IPointcloudProvider
{
	virtual std::shared_ptr<const VArray> getField(rgl_field_t field) const = 0;
};
