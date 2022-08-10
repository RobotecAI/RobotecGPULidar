#pragma once

struct Node : APIObject<Node>, std::enable_shared_from_this<Node>
{
	Node() = default;

	void setParent(std::shared_ptr<Node> parent)
	{
		this->parent = parent;
		if (parent != nullptr) {
			parent->children.push_back(shared_from_this());
		}
	}

	virtual ~Node() = default;

	std::shared_ptr<Node> parent = nullptr;
	std::vector<std::shared_ptr<Node>> children;
};

API_OBJECT_INSTANCE(Node);

struct UseRaysMat3x4fNode : Node
{
	using Node::Node;

	void setParameters(const rgl_mat3x4f* rays, size_t ray_count)
	{
		dRays.copyFromHost(rays, ray_count);
	}

private:
	DeviceBuffer<rgl_mat3x4f> dRays;
};

struct RaytraceNode : Node
{
	using Node::Node;

	void setParameters(float range)
	{
		this->range = range;
	}

private:
	float range;
};

struct WritePCDFileNode : Node
{
	using Node::Node;

	void setParameters(const char* filePath)
	{
		this->filePath = filePath;
	}

private:
	std::filesystem::path filePath;
};