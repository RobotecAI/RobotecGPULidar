// Copyright 2024 Robotec.AI
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

#include <cassert>

#include <macros/cuda.hpp>
#include <rgl/api/core.h>

inline HostDevFn size_t getReturnCount(rgl_return_mode_t returnMode)
{
	return static_cast<size_t>(returnMode >> RGL_RETURN_MODE_BIT_SHIFT);
}

inline HostDevFn rgl_return_type_t getReturnType(rgl_return_mode_t returnMode, size_t returnIndex)
{
	return static_cast<rgl_return_type_t>((returnMode >> (returnIndex * RGL_RETURN_TYPE_BIT_SHIFT)) & 0xff);
}

inline HostFn const std::string& getReturnTypeName(rgl_return_type_t returnType)
{
	// clang-format off
	static const std::unordered_map<rgl_return_type_t, std::string> typeNames = {
		{RGL_RETURN_TYPE_UNKNOWN, "Unknown"},
		{RGL_RETURN_TYPE_FIRST, "First"},
		{RGL_RETURN_TYPE_SECOND, "Second"},
		{RGL_RETURN_TYPE_LAST, "Last"},
	    {RGL_RETURN_TYPE_STRONGEST, "Strongest"},
	    {RGL_RETURN_TYPE_SECOND_STRONGEST, "SecondStrongest"},
	};
	// clang-format on
	assert(typeNames.find(returnType) != typeNames.cend());
	return typeNames.at(returnType);
}

inline HostFn std::string getReturnModeName(rgl_return_mode_t returnMode)
{
	std::string returnModeName = getReturnTypeName(getReturnType(returnMode, 0));
	for (size_t returnIdx = 1; returnIdx < getReturnCount(returnMode); ++returnIdx) {
		returnModeName += "-" + getReturnTypeName(getReturnType(returnMode, returnIdx));
	}
	return returnModeName;
}
