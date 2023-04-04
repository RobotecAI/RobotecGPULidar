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

#include <type_traits>
#include <typeinfo>

#include <RGLExceptions.hpp>
#include <gpu/GPUFieldDesc.hpp>
#include <graph/Interfaces.hpp>
#include <graph/NodesCore.hpp>
#include <memory/HostPageableArray.hpp>
#include <memory/DeviceSyncArray.hpp>

// Builder for GPUFieldDesc. Separated struct to avoid polluting gpu-visible header (gpu/GPUFieldDesc.hpp).
struct GPUFieldDescBuilder
{
	// TODO: This class has a hidden bug. If hostBuffer was not zeroed on each build,
	// TODO: fillSizeAndOffset would leave trash in some GPUFieldDesc causing cudaIllegalMemoryAccess
	// TODO: Also, this class is over-engineered; likely due to using GPUFieldDesc for both directions of formatting
	// TODO: This should be fixed by splitting GPUFieldDesc into two separate structs and merging fill* methods.

	DeviceSyncArray<GPUFieldDesc>::Ptr buildReadable(const std::vector<std::pair<rgl_field_t, const void*>>& fieldsData)
	{
		hostBuffer->clear(false);
		hostBuffer->resize(fieldsData.size(), true, false);
		fillSizeAndOffset(getFields(fieldsData));
		fillPointers(fieldsData);
		deviceBuffer->copyFrom(hostBuffer);
		return deviceBuffer;
	}

	DeviceSyncArray<GPUFieldDesc>::Ptr buildWritable(const std::vector<std::pair<rgl_field_t, void*>>& fieldsData)
	{
		hostBuffer->clear(false);
		hostBuffer->resize(fieldsData.size(), true, false);
		fillSizeAndOffset(getFields(fieldsData));
		fillPointers(fieldsData);
		deviceBuffer->copyFrom(hostBuffer);
		return deviceBuffer;
	}

private:
	void fillSizeAndOffset(const std::vector<rgl_field_t>& fields)
	{
		std::size_t offset = 0;
		std::size_t gpuFieldIdx = 0;
		for (auto field : fields) {
			if (!isDummy(field)) {
				(*hostBuffer)[gpuFieldIdx] = GPUFieldDesc {
					.readDataPtr = nullptr,
					.writeDataPtr = nullptr,
					.size = getFieldSize(field),
					.dstOffset = offset
				};
			}
			++gpuFieldIdx;
			offset += getFieldSize(field);
		}
	}

	template <typename T>
	std::vector<rgl_field_t> getFields(const std::vector<std::pair<rgl_field_t, T>>& fieldsData)
	{
		std::vector<rgl_field_t> fields;
		std::transform(fieldsData.begin(), fieldsData.end(), std::back_inserter(fields),
		               [&](const auto fieldData) { return fieldData.first; });
		return fields;
	}

	template <typename T>
	void fillPointers(const std::vector<std::pair<rgl_field_t, T>>& fieldsData)
	{
		static_assert(std::is_same_v<T, void*> || std::is_same_v<T, const void*>);
		for (size_t i = 0; i < hostBuffer->getCount(); ++i) {
			if (fieldsData[i].second == nullptr) {  // dummy field
				continue;
			}
			if constexpr (std::is_same_v<T, const void*>) {
				(*hostBuffer)[i].readDataPtr = static_cast<const char*>(fieldsData[i].second);
				continue;
			}
			if constexpr (std::is_same_v<T, void*>) {
				(*hostBuffer)[i].writeDataPtr = static_cast<char*>(fieldsData[i].second);
				continue;
			}
		}
	}

private:
	HostPageableArray<GPUFieldDesc>::Ptr hostBuffer = HostPageableArray<GPUFieldDesc>::create();
	DeviceSyncArray<GPUFieldDesc>::Ptr deviceBuffer = DeviceSyncArray<GPUFieldDesc>::create();
};
