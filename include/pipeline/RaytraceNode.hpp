#pragma once

struct RaytraceNode : Node
{
	using Node::Node;

	void setParameters(float range)
	{
		this->range = range;
	}

	void validate() override
	{
		auto parentTyped = validateParent<IRaysProvider>();
	}

	void execute() override
	{

	}



private:
	float range;
	std::shared_ptr<IRaysProvider> parentTyped = validateParent<IRaysProvider>();
};