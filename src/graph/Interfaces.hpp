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
#include <VArray.hpp>
#include <VArrayProxy.hpp>
#include <RGLFields.hpp>
#include <gpu/GPUFieldDesc.hpp>

struct IRaysNode
{
	using Ptr = std::shared_ptr<IRaysNode>;
	virtual VArrayProxy<Mat3x4f>::ConstPtr getRays() const = 0;
	virtual std::size_t getRayCount() const = 0;
	virtual std::optional<VArrayProxy<int>::ConstPtr> getRingIds() const = 0;
	virtual std::optional<std::size_t> getRingIdsCount() const = 0;
};

struct IPointCloudDescription
{
	virtual bool hasField(rgl_field_t field) const = 0;
	virtual bool isDense() const = 0;
	virtual std::size_t getWidth() const = 0;
	virtual std::size_t getHeight() const = 0;
	virtual std::size_t getPointCount() const { return getWidth() * getHeight(); }
};

// TODO(prybicki): getFieldData* may act lazily, so they take stream as a parameter to do the lazy evaluation.
// TODO(prybicki): This requires synchronizing with the potentially different stream provided by the schedule(...)
// TODO(prybicki): This situation is bug-prone, requiring greater mental effort when implementing nodes.
// TODO(prybicki): It might be better to remove stream as a parameter and assume that all pipeline nodes are using common stream.
struct IPointsNode : public IPointCloudDescription
{
	using Ptr = std::shared_ptr<IPointsNode>;

	virtual VArray::ConstPtr getFieldData(rgl_field_t field, cudaStream_t stream) const = 0;
	virtual std::size_t getFieldPointSize(rgl_field_t field) const
	{ return getFieldSize(field); }

	template<rgl_field_t field>
	typename VArrayProxy<typename Field<field>::type>::ConstPtr getFieldDataTyped(cudaStream_t stream)
	{ return getFieldData(field, stream)->template getTypedProxy<typename Field<field>::type>(); }
};
