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

	bool hasField(rgl_field_t field) const override
	{
		return false;
	}

	bool isDense() const override
	{
		return false;
	}

	size_t getWidth() const override
	{
		return 0;
	}

	size_t getHeight() const override
	{
		return 0;
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