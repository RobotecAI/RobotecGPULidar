#pragma once

struct UseRaysMat3x4fNode : Node, IRaysNode
{
	using Node::Node;

	void setParameters(const rgl_mat3x4f* raysRaw, size_t rayCount)
	{
		rays = VArrayTyped<rgl_mat3x4f>::create();
		rays->copyFrom(raysRaw, rayCount);
	}

	std::shared_ptr<const VArrayTyped<rgl_mat3x4f>> getRays() const override
	{ return rays; }

	void validate() override {}
	void schedule(cudaStream_t stream) override {}

private:
	std::shared_ptr<VArrayTyped<rgl_mat3x4f>> rays;
};
