#pragma once

#include <memory/IAnyArray.hpp>

struct ArrayHub
{
    IAnyArray<MemoryKind::DeviceAsync>::Ptr async;
    IAnyArray<MemoryKind::HostPageable>::Ptr pageable;
    IAnyArray<MemoryKind::HostPinned>::Ptr pinned;
};