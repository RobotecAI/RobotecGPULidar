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

#include <memory/HostArray.hpp>
#include <memory/MemoryKind.hpp>

template <typename T>
struct HostPageableArray : public HostArray<MemoryKind::HostPageable, T>
{
	using Ptr = std::shared_ptr<HostPageableArray<T>>;
	using ConstPtr = std::shared_ptr<const HostPageableArray<T>>;

	static HostPageableArray<T>::Ptr create()
	{
		return HostPageableArray<T>::Ptr {
			new HostPageableArray(MemoryOperations::get<MemoryKind::HostPageable>())
		};
	}

protected:
	using HostArray<MemoryKind::HostPageable, T>::HostArray;
};
