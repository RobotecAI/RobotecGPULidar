// Copyright 2022 Robotec.AI
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <rgl/api/core.h>
#include <math/Mat3x4f.hpp>
#include <RGLFields.hpp>
#include <gpu/GPUFieldDesc.hpp>

#include <memory/Array.hpp>

struct IRaysNode : virtual Node
{
	using Ptr = std::shared_ptr<IRaysNode>;

	// Transforms
	virtual std::size_t getRayCount() const = 0;
	virtual Array<Mat3x4f>::ConstPtr getRays() const = 0;

	// Ring Ids (client-specific data)
	virtual std::optional<std::size_t> getRingIdsCount() const = 0;
	virtual std::optional<Array<int>::ConstPtr> getRingIds() const = 0;

	// Ranges
	virtual std::optional<std::size_t> getRangesCount() const = 0;
	virtual std::optional<Array<Vec2f>::ConstPtr> getRanges() const = 0;

	// Firing time offsets
	virtual std::optional<std::size_t> getTimeOffsetsCount() const = 0;
	virtual std::optional<Array<float>::ConstPtr> getTimeOffsets() const = 0;

	virtual Mat3x4f getCumulativeRayTransfrom() const { return Mat3x4f::identity(); }
};

struct IRaysNodeSingleInput : IRaysNode
{
	using Ptr = std::shared_ptr<IRaysNodeSingleInput>;

	virtual void validateImpl() override { input = Node::getExactlyOneInputOfType<IRaysNode>(); }

	// Rays description
	virtual size_t getRayCount() const override { return input->getRayCount(); }
	virtual Array<Mat3x4f>::ConstPtr getRays() const override { return input->getRays(); };

	// Ring Ids (client-specific data)
	virtual std::optional<size_t> getRingIdsCount() const override { return input->getRingIdsCount(); }
	virtual std::optional<Array<int>::ConstPtr> getRingIds() const override { return input->getRingIds(); }

	// Ranges
	virtual std::optional<size_t> getRangesCount() const override { return input->getRangesCount(); }
	virtual std::optional<Array<Vec2f>::ConstPtr> getRanges() const override { return input->getRanges(); }

	// Firing time offsets
	virtual std::optional<size_t> getTimeOffsetsCount() const override { return input->getTimeOffsetsCount(); }
	virtual std::optional<Array<float>::ConstPtr> getTimeOffsets() const override { return input->getTimeOffsets(); }

	virtual Mat3x4f getCumulativeRayTransfrom() const override { return input->getCumulativeRayTransfrom(); }

protected:
	IRaysNode::Ptr input{0};
};

struct IPointsNode : virtual Node
{
	using Ptr = std::shared_ptr<IPointsNode>;

	// Node requirements
	virtual std::vector<rgl_field_t> getRequiredFieldList() const { return {}; };

	// Point cloud description
	virtual bool isDense() const = 0;
	virtual bool hasField(rgl_field_t field) const = 0;
	virtual std::size_t getWidth() const = 0;
	virtual std::size_t getHeight() const = 0;
	virtual std::size_t getPointCount() const { return getWidth() * getHeight(); }

	virtual Mat3x4f getLookAtOriginTransform() const { return Mat3x4f::identity(); }
	virtual Vec3f getLinearVelocity() const { return Vec3f{0.0f}; }
	virtual Vec3f getAngularVelocity() const { return Vec3f{0.0f}; }

	// Data getters
	virtual IAnyArray::ConstPtr getFieldData(rgl_field_t field) = 0;
	virtual std::size_t getFieldPointSize(rgl_field_t field) const { return getFieldSize(field); }

	template<rgl_field_t field>
	typename Array<typename Field<field>::type>::ConstPtr getFieldDataTyped()
	{
		return getFieldData(field)->template asTyped<typename Field<field>::type>();
	}
};

struct IPointsNodeSingleInput : IPointsNode
{
	using Ptr = std::shared_ptr<IPointsNodeSingleInput>;

	virtual void validateImpl() override
	{
		input = getExactlyOneInputOfType<IPointsNode>();
		for (auto&& field : getRequiredFieldList()) {
			if (!input->hasField(field) && !isDummy(field)) {
				auto msg = fmt::format("{} requires {} to be present", getName(), toString(field));
				throw InvalidPipeline(msg);
			}
		}
	}

	// Point cloud description
	virtual bool isDense() const override { return input->isDense(); }
	virtual size_t getWidth() const override { return input->getWidth(); }
	virtual size_t getHeight() const override { return input->getHeight(); }

	virtual Mat3x4f getLookAtOriginTransform() const override { return input->getLookAtOriginTransform(); }
	virtual Vec3f getLinearVelocity() const { return input->getLinearVelocity(); }
	virtual Vec3f getAngularVelocity() const { return input->getAngularVelocity(); }

	// Data getters
	virtual bool hasField(rgl_field_t field) const { return input->hasField(field); }
	virtual IAnyArray::ConstPtr getFieldData(rgl_field_t field) override { return input->getFieldData(field); }

protected:
	IPointsNode::Ptr input{0};
};

struct INoInputNode : virtual Node
{
	virtual void validateImpl() override
	{
		if (!inputs.empty()) {
			auto msg = fmt::format("inputs for node {} are not allowed", getName());
			throw InvalidPipeline(msg);
		}
	}
};
