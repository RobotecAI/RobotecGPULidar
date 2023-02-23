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

#include <RGLExceptions.hpp>
#include <gpu/GPUFieldDesc.hpp>
#include <graph/Interfaces.hpp>
#include <graph/NodesCore.hpp>

// Builder for GPUFieldDesc. Separated struct to avoid polluting gpu-visible header (gpu/GPUFieldDesc.hpp).
struct GPUFieldDescBuilder
{
	static VArrayProxy<GPUFieldDesc>::Ptr buildReadable(IPointsNode::Ptr input, const std::vector<rgl_field_t> &fields, cudaStream_t stream)
	{ return GPUFieldDescBuilder::build(input, fields, stream, true); }

	static VArrayProxy<GPUFieldDesc>::Ptr buildWritable(IPointsSourceNode::Ptr input, const std::vector<rgl_field_t> &fields, cudaStream_t stream)
	{ return GPUFieldDescBuilder::build(input, fields, stream, false); }

private:
	template<typename T>
	static VArrayProxy<GPUFieldDesc>::Ptr build(T input, const std::vector<rgl_field_t> &fields, cudaStream_t stream, bool makeDataConst)
	{
		auto gpuFields = VArrayProxy<GPUFieldDesc>::create(fields.size());
		std::size_t offset = 0;
		std::size_t gpuFieldIdx = 0;
		for (auto field : fields) {
			if (!isDummy(field)) {
				(*gpuFields)[gpuFieldIdx] = GPUFieldDesc {
					.readDataPtr = nullptr,
					.writeDataPtr = nullptr,
					.size = getFieldSize(field),
					.dstOffset = offset
				};
				if (makeDataConst) {
					(*gpuFields)[gpuFieldIdx].readDataPtr = static_cast<const char*>(input->getFieldData(field, stream)->getReadPtr(MemLoc::Device));
				} else {
					if constexpr (requires { input->fieldData[field]; }) {
						(*gpuFields)[gpuFieldIdx].writeDataPtr = static_cast<char*>(input->fieldData[field]->getWritePtr(MemLoc::Device));
					} else {
						throw InvalidAPIObject("attempted to get writable pointer to field that is not available.");
					}
				}
				++gpuFieldIdx;
			}
			offset += getFieldSize(field);
		}
		return gpuFields;
	}
};
