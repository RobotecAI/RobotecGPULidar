#pragma once

#include <gpu/RaytraceRequestParams.hpp>


struct RaytraceNode : Node, IPointcloudNode
{
	using Node::Node;
	using Ptr = std::shared_ptr<RaytraceNode>;

	void setParameters(std::shared_ptr<Scene> scene, float range)
	{
		this->scene = scene;
		this->range = range;
	}

	void setFields(std::set<rgl_field_t> fields)
	{
		fields.insert(RGL_FIELD_XYZ_F32);
		this->fields = std::move(fields);
	}

	void validate() override
	{
		raysNode = getValidInput<IRaysNode>();
		for (auto&& field : fields) {
			if (!fieldData.contains(field)) {
				fieldData.insert({field, VArray::create(field)});
			}
			// TODO: optimization opportunity - do not preserve or zero-initialize
			fieldData[field]->resize(raysNode->getRays()->getCount());
		}
	}

	void schedule(cudaStream_t stream) override
	{
		auto rays = raysNode->getRays();
		auto sceneAS = scene->getAS();
		auto sceneSBT = scene->getSBT();
		dim3 launchDims = {static_cast<unsigned int>(rays->getCount()), 1, 1};

		(*requestCtx)[0] = {
			.rays = rays->getDevicePtr(stream),
			.rayCount = rays->getCount(),
			.rayOriginToWorld = Mat3x4f::identity(),
			.rayRange = range,
			.scene = sceneAS,
			.xyz = fields.contains(RGL_FIELD_XYZ_F32) ? fieldData.at(RGL_FIELD_XYZ_F32)->getTypedProxy<Vec3f>()->getDevicePtr(stream) : nullptr,
			.xyzp = fields.contains(RGL_FIELD_XYZP_F32) ? fieldData.at(RGL_FIELD_XYZP_F32)->getTypedProxy<Vec4f>()->getDevicePtr(stream) : nullptr
		};

		// TODO(prybicki): VArray may use CUDA managed memory, which hasn't been proven to work with OptiX.
		// TODO(prybicki): Based on my intuition, it should work fine, but beware.
		CUdeviceptr pipelineArgsPtr = requestCtx->getCUdeviceptr(stream);
		std::size_t pipelineArgsSize = requestCtx->getBytesInUse();
		CHECK_OPTIX(optixLaunch(Optix::instance().pipeline, stream, pipelineArgsPtr, pipelineArgsSize, &sceneSBT, launchDims.x, launchDims.y, launchDims.y));
	}

	std::shared_ptr<const VArray> getFieldData(rgl_field_t field) const override
	{
		return std::const_pointer_cast<const VArray>(fieldData.at(field));
	}

	bool hasField(rgl_field_t field) const override	{ return fields.contains(field); }

	bool isDense() const override { return false; }

	size_t getWidth() const override { return raysNode->getRays()->getCount(); }

	size_t getHeight() const override { return 1; }  // TODO: implement height in use_rays


private:
	float range;
	std::shared_ptr<Scene> scene;
	std::set<rgl_field_t> fields;
	std::shared_ptr<IRaysNode> raysNode;
	std::shared_ptr<VArrayProxy<RaytraceRequestContext>> requestCtx = VArrayProxy<RaytraceRequestContext>::create(1);
	std::unordered_map<rgl_field_t, std::shared_ptr<VArray>> fieldData;
};