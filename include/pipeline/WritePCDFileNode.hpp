#pragma once

struct WritePCDFileNode : Node
{
	using Node::Node;

	void setParameters(const char* filePath)
	{
		this->filePath = filePath;
	}

	void validate() override {}
	void execute() override {}

private:
	std::filesystem::path filePath{};
};
