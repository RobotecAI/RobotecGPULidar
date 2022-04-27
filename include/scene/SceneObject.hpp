#pragma once

#include <scene/Scene.hpp>
#include <scene/Mesh.hpp>
#include <APIObject.hpp>
#include <utility>
#include <TransformMatrix.h>


struct SceneObject : APIObject<SceneObject>
{
	SceneObject(std::shared_ptr<Mesh> mesh, std::optional<std::string> name=std::nullopt);
	~SceneObject();

	// TODO(prybicki): low-prio optimization: do not rebuild whole IAS if only transform changed
	void setTransform(TransformMatrix newTransform);
	OptixInstance getIAS(int idx);

	std::shared_ptr<Mesh> mesh;
private:
	TransformMatrix transform;
	std::weak_ptr<Scene> scene;
	std::optional<std::string> humanReadableName;
	friend struct APIObject<SceneObject>;
	friend struct Scene;
};

