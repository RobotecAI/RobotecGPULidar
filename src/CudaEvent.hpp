// Copyright 2023s Robotec.AI
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
#include <macros/cuda.hpp>
#include <Logger.hpp>

// RAII object to (de)initialize cudaEvent_t
struct CudaEvent {
    using Ptr = std::shared_ptr<CudaEvent>;

    static CudaEvent::Ptr create(int flag = cudaEventDisableTiming)
    {
        return CudaEvent::Ptr(new CudaEvent(flag));
    }

    cudaEvent_t get() { return event; }

    ~CudaEvent()
    {
        if (event != nullptr) {
            try {
                CHECK_CUDA(cudaEventDestroy(event));
                event = nullptr;
            }
            catch(std::runtime_error& e) {
                RGL_ERROR("Error in ~CudaEvent: {}", e.what());
            }
        }
    }

private:
    explicit CudaEvent(int flag) { CHECK_CUDA(cudaEventCreateWithFlags(&event, flag)); }

private:
    cudaEvent_t event { nullptr };
};
