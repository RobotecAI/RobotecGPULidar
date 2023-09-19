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

#include <graph/NodesCore.hpp>
#include <gpu/nodeKernels.hpp>
#include <RGLFields.hpp>
#include <graph/GraphRunCtx.hpp>

void FormatPointsNode::setParameters(const std::vector<rgl_field_t>& fields)
{
	if (std::find(fields.begin(), fields.end(), RGL_FIELD_DYNAMIC_FORMAT) != fields.end()) {
		throw InvalidAPIArgument("cannot format field 'RGL_FIELD_DYNAMIC_FORMAT'");
	}
	this->fields = fields;
}

void FormatPointsNode::enqueueExecImpl()
{
	formatAsync(output, input, fields, gpuFieldDescBuilder);
}

void FormatPointsNode::formatAsync(DeviceAsyncArray<char>::Ptr output, const IPointsNode::Ptr& input,
                                   const std::vector<rgl_field_t>& fields, GPUFieldDescBuilder& gpuFieldDescBuilder)
{
	// Prepare output array
	std::size_t pointSize = getPointSize(fields);
	std::size_t pointCount = input->getPointCount();
	output->resize(pointCount * pointSize, false, false);

	// Kernel Call
	const GPUFieldDesc* gpuFieldsPtr = gpuFieldDescBuilder.buildReadableAsync(output->getStream(), getFieldToPointerMappings(input, fields)).getReadPtr();
	char* outputPtr = output->getWritePtr();
	gpuFormatSoaToAos(output->getStream()->getHandle(), pointCount, pointSize, fields.size(), gpuFieldsPtr, outputPtr);
}

IAnyArray::ConstPtr FormatPointsNode::getFieldData(rgl_field_t field)
{
	if (field == RGL_FIELD_DYNAMIC_FORMAT) {
		return output;
	}
	return input->getFieldData(field);
}

std::size_t FormatPointsNode::getFieldPointSize(rgl_field_t field) const
{
	if (field == RGL_FIELD_DYNAMIC_FORMAT) {
		return getPointSize(fields);
	}
	return getFieldSize(field);
}

std::vector<std::pair<rgl_field_t, const void*>> FormatPointsNode::getFieldToPointerMappings(const IPointsNode::Ptr& input,
                                                                                             const std::vector<rgl_field_t>& fields)
{
	std::vector<std::pair<rgl_field_t, const void*>> outFieldsData;
	for (auto&& field : fields) {
		// We want to ensure that memory kind is something accessible from GPU (i.e. not pageable),
		// however, we cannot go the regular route (do a type-cast to Array<T>),
		// and then find out real memory kind (further down-cast to concrete Array),
		// because we do not know the type in compile time - all we have is rgl_field_t.
		// Furthermore, all we need here is a void*, because gpuFormat* knows what to do.
		// Therefore, we need to use IAnyArray::getRawReadPtr().
		outFieldsData.push_back({field, nullptr});
		if (!isDummy(field)) {
			IAnyArray::ConstPtr fieldArray = input->getFieldData(field);
			if (!isDeviceAccessible(fieldArray->getMemoryKind())) {
				auto msg = fmt::format("FormatPointsNode: all input fields must be device-accessible, {} is not", toString(field));
				throw InvalidPipeline(msg);
			}
			outFieldsData.rbegin()->second = fieldArray->getRawReadPtr();
		}
	}
	return outFieldsData;
}
