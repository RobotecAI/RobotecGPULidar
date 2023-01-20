// Copyright 2023 Robotec.AI
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

#include <gpu/GPUFieldDesc.hpp>
#include <graph/Interfaces.hpp>
#include <graph/NodesCore.hpp>

// Builder for GPUFieldDesc. Separated struct to avoid polluting gpu-visible header (gpu/GPUFieldDesc.hpp).
struct GPUFieldDescBuilder
{
	static VArrayProxy<GPUFieldDesc>::Ptr buildReadable(IPointsNode::Ptr input, const std::vector<rgl_field_t> &fields, cudaStream_t stream)
	{
		auto gpuFields = VArrayProxy<GPUFieldDesc>::create(fields.size());
		std::size_t offset = 0;
		std::size_t gpuFieldIdx = 0;
		for (size_t i = 0; i < fields.size(); ++i) {
			if (!isDummy(fields[i])) {
				(*gpuFields)[gpuFieldIdx] = GPUFieldDesc {
					.readDataPtr = static_cast<const char*>(input->getFieldData(fields[i], stream)->getReadPtr(MemLoc::Device)),
					.size = getFieldSize(fields[i]),
					.dstOffset = offset,
				};
				gpuFieldIdx += 1;
			}
			offset += getFieldSize(fields[i]);
		}
		return gpuFields;
	}

	static VArrayProxy<GPUFieldDesc>::Ptr buildWritable(IPointsSourceNode::Ptr input, const std::vector<rgl_field_t> &fields, cudaStream_t stream)
	{
		auto gpuFields = VArrayProxy<GPUFieldDesc>::create(fields.size());
		std::size_t offset = 0;
		std::size_t gpuFieldIdx = 0;
		for (size_t i = 0; i < fields.size(); ++i) {
			if (!isDummy(fields[i])) {
				(*gpuFields)[gpuFieldIdx] = GPUFieldDesc {
					.writeDataPtr = static_cast<char*>(input->fieldData[fields[i]]->getWritePtr(MemLoc::Device)),
					.size = getFieldSize(fields[i]),
					.dstOffset = offset,
				};
				gpuFieldIdx += 1;
			}
			offset += getFieldSize(fields[i]);
		}
		return gpuFields;
	}
};
