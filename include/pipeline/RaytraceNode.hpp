#pragma once


struct RaytraceNode : Node, IPointcloudNode
{
	using Node::Node;
	using Ptr = std::shared_ptr<RaytraceNode>;

	void setParameters(float range)
	{
		this->range = range;
	}

	void setFields(std::set<rgl_field_t> fields)
	{
		this->fields = fields;
	}

	void validate() override
	{
		for (auto&& field : fields) {
			if (!fieldData.contains(field)) {
				fieldData.insert({field, VArray::create(field)});
			}
			fieldData[field]->ensureCapacity(raysNode->getRays()->size());
		}
	}

	void schedule(cudaStream_t stream) override
	{
		// dim3 launchDims = {static_cast<unsigned int>(currentJob->rayCount), 1, 1};
		// CHECK_OPTIX(optixLaunch(Optix::instance().pipeline, stream, pipelineArgsPtr, pipelineArgsSize, &sceneSBT, launchDims.x, launchDims.y, launchDims.y));
	}

	std::shared_ptr<const VArray> getFieldData(rgl_field_t field) const override
	{

	}


private:
	float range;
	std::shared_ptr<IRaysNode> raysNode;
	std::unordered_map<rgl_field_t, std::shared_ptr<VArray>> fieldData;
	std::set<rgl_field_t> fields;
};