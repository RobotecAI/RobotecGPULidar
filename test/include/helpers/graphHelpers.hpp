#pragma once

#include "graph/Node.hpp"
#include "graph/Interfaces.hpp"
#include "graph/Node.hpp"

struct RecordTimeNode : IPointsNodeSingleInput
{
	void setParameters() {}
	double getMeasurement()
	{
		this->synchronize();
		return std::chrono::duration<double>(measurement.time_since_epoch()).count();
	}

protected:
	void enqueueExecImpl() override { measurement = std::chrono::steady_clock::now(); }

	void validateImpl() override { IPointsNodeSingleInput::validateImpl(); }

private:
	std::chrono::time_point<std::chrono::steady_clock> measurement;
};

struct SleepNode : IPointsNodeSingleInput
{
	void setParameters(double seconds) { this->seconds = seconds; }

protected:
	void enqueueExecImpl() override { std::this_thread::sleep_for(std::chrono::duration<double>(seconds)); }

	void validateImpl() override { IPointsNodeSingleInput::validateImpl(); }

private:
	double seconds;
	std::chrono::time_point<std::chrono::steady_clock> measurement;
};


struct EmptyNode : IPointsNode, INoInputNode
{
	void setParameters() {}
	bool isDense() const override { return false; }
	bool hasField(rgl_field_t field) const override { return true; }
	size_t getWidth() const override { return 0; }
	size_t getHeight() const override { return 0; }
	IAnyArray::ConstPtr getFieldData(rgl_field_t field) override { return createArray<DeviceAsyncArray>(field, arrayMgr); }

protected:
	void enqueueExecImpl() override {}
};