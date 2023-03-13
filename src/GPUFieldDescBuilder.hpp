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

// Builder for GPUFieldDesc. Separated struct to avoid polluting gpu-visible header (gpu/GPUFieldDesc.hpp).
struct GPUFieldDescBuilder
{
	static VArrayProxy<GPUFieldDesc>::Ptr buildReadable(const std::vector<std::pair<rgl_field_t, const void*>>& fieldsData)
	{
		auto gpuFields = GPUFieldDescBuilder::initialize(GPUFieldDescBuilder::getFields(fieldsData));
		GPUFieldDescBuilder::fillWithData(gpuFields, fieldsData);
		return gpuFields;
	}

	static VArrayProxy<GPUFieldDesc>::Ptr buildWritable(const std::vector<std::pair<rgl_field_t, void*>>& fieldsData)
	{
		auto gpuFields = GPUFieldDescBuilder::initialize(GPUFieldDescBuilder::getFields(fieldsData));
		GPUFieldDescBuilder::fillWithData(gpuFields, fieldsData);
		return gpuFields;
	}

private:
	static VArrayProxy<GPUFieldDesc>::Ptr initialize(const std::vector<rgl_field_t>& fields)
	{
		auto gpuFields = VArrayProxy<GPUFieldDesc>::create(fields.size());
		std::size_t offset = 0;
		std::size_t gpuFieldIdx = 0;
		for (auto field : fields) {
			if (!isDummy(field)) {
				(*gpuFields)[gpuFieldIdx] = GPUFieldDesc{
					.readDataPtr = nullptr, .writeDataPtr = nullptr, .size = getFieldSize(field), .dstOffset = offset
				};
			}
			++gpuFieldIdx;
			offset += getFieldSize(field);
		}
		return gpuFields;
	}

	template<typename T>
	static std::vector<rgl_field_t> getFields(const std::vector<std::pair<rgl_field_t, T>>& fieldsData)
	{
		std::vector<rgl_field_t> fields;
		std::transform(fieldsData.begin(), fieldsData.end(), std::back_inserter(fields),
		               [&](const auto fieldData) { return fieldData.first; });
		return fields;
	}

	template<typename T>
	static void fillWithData(VArrayProxy<GPUFieldDesc>::Ptr& gpuFields,
	                         const std::vector<std::pair<rgl_field_t, T>>& fieldsData)
	{
		static_assert(std::is_same_v<T, void*> || std::is_same_v<T, const void*>);
		for (size_t i = 0; i < gpuFields->getCount(); ++i) {
			if (fieldsData[i].second == nullptr) { // dummy field
				continue;
			}
			if constexpr (std::is_same_v<T, const void*>) {
				(*gpuFields)[i].readDataPtr = static_cast<const char*>(fieldsData[i].second);
				continue;
			}
			if constexpr (std::is_same_v<T, void*>) {
				(*gpuFields)[i].writeDataPtr = static_cast<char*>(fieldsData[i].second);
				continue;
			}
		}
	}
};
