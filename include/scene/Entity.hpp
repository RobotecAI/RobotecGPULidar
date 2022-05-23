#pragma once

#include <scene/Scene.hpp>
#include <scene/Mesh.hpp>
#include <APIObject.hpp>
#include <utility>
#include <TransformMatrix.h>


struct Entity : APIObject<Entity>
{
	Entity(std::shared_ptr<Mesh> mesh, std::optional<std::string> name=std::nullopt);
	~Entity();

	// TODO(prybicki): low-prio optimization: do not rebuild whole IAS if only transform changed
	void setTransform(TransformMatrix newTransform);
	OptixInstance getIAS(int idx);

	std::shared_ptr<Mesh> mesh;
	std::weak_ptr<Scene> scene;
private:
	TransformMatrix transform;
	std::optional<std::string> humanReadableName;
	friend struct APIObject<Entity>;
	friend struct Scene;
};

