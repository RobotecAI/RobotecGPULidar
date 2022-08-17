#pragma once

struct UseRaysMat3x4fNode : Node, IRaysNode
{
	using Node::Node;

	void setParameters(const Mat3x4f* raysRaw, size_t rayCount)
	{
		rays = VArrayProxy<Mat3x4f>::create();
		rays->copyFrom(raysRaw, rayCount);
	}

	std::shared_ptr<const VArrayProxy<Mat3x4f>> getRays() const override
	{ return rays; }

	void validate() override {}
	void schedule(cudaStream_t stream) override {}

private:
	std::shared_ptr<VArrayProxy<Mat3x4f>> rays;
};
