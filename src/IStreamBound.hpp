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

#include <memory>

#include <CudaStream.hpp>

/**
 * Interface for objects that hold a stream and allow changing it in runtime.
 * It is a caller responsibility to make sure that stream can be changed.
 */
struct IStreamBound
{
	using Ptr = std::shared_ptr<IStreamBound>;
	virtual void setStream(CudaStream::Ptr) = 0;
	virtual CudaStream::Ptr getStream() const = 0;
};
