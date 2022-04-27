#include <scene/SceneObject.hpp>

API_OBJECT_INSTANCE(SceneObject);

SceneObject::SceneObject(std::shared_ptr<Mesh> mesh, std::optional<std::string> name)
: mesh(std::move(mesh))
, transform(TransformMatrix::identity())
, humanReadableName(std::move(name)) { }

void SceneObject::setTransform(TransformMatrix newTransform)
{
	transform = newTransform;
	if (auto activeScene = scene.lock()) {
		activeScene->requestASRebuild();
	}
}

OptixInstance SceneObject::getIAS(int idx)
{
	logInfo("[RGL] Bulding IAS for object {}\n", humanReadableName.has_value() ? *humanReadableName : "<unnamed>");
	// NOTE: this assumes a single SBT record per GAS
	OptixInstance instance = {
		.instanceId = static_cast<unsigned int>(idx),
		.sbtOffset = static_cast<unsigned int>(idx * 1), // TODO: LIDAR_RAY_TYPE_COUNT was here, figure out implications
		.visibilityMask = 255,
		.flags = OPTIX_INSTANCE_FLAG_DISABLE_ANYHIT,
		.traversableHandle = mesh->getGAS(),
	};
	memcpy(instance.transform, transform.matrix_flat, sizeof(float) * 12);
	return instance;
}

SceneObject::~SceneObject()
{
	// TODO: remove this once mesh:object 1:1 relation is dropped
	Mesh::release(mesh);
}
