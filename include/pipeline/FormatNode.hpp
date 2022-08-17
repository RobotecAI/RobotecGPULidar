#pragma once

#include <pipeline/Node.hpp>
#include <pipeline/Interfaces.hpp>

struct FormatNode : Node, IPointcloudNode
{
	std::shared_ptr<const VArray> getFieldData(rgl_field_t field) const override
	{
		return std::shared_ptr<const VArray>();
	}

	void setParameters(std::vector<rgl_field_t> fields)
	{
		this->fields = std::move(fields);
	}

	std::vector<rgl_field_t> getFieldList() const
	{
		return fields;
	}

	void validate() override
	{
		// compaction node optional
	}

	void schedule(cudaStream_t stream) override
	{

	}

	std::vector<rgl_field_t> fields;
	std::shared_ptr<IPointcloudNode> input;
	// std::shared_ptr<ICompactionNode> compaction;
};