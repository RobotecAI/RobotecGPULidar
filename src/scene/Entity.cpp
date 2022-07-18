#include <scene/Entity.hpp>

API_OBJECT_INSTANCE(Entity);

Entity::Entity(std::shared_ptr<Mesh> mesh, std::optional<std::string> name)
: mesh(std::move(mesh))
, transform(TransformMatrix::identity())
, humanReadableName(std::move(name)) { }

void Entity::setTransform(TransformMatrix newTransform)
{
	transform = newTransform;
	if (auto activeScene = scene.lock()) {
		activeScene->requestASRebuild();
	}
}

OptixInstance Entity::getIAS(int idx)
{
	// NOTE: this assumes a single SBT record per GAS
	OptixInstance instance = {
		.instanceId = static_cast<unsigned int>(idx),
		.sbtOffset = static_cast<unsigned int>(idx),
		.visibilityMask = 255,
		.flags = OPTIX_INSTANCE_FLAG_DISABLE_ANYHIT,
		.traversableHandle = mesh->getGAS(),
	};
	memcpy(instance.transform, transform.matrix_flat, sizeof(float) * 12);
	return instance;
}

Entity::~Entity()
{
}
