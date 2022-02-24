#pragma once

#include <data_types/PointTypes.h>
#include "data_types/PCLFormats.h"
#include "DeviceBuffer.hpp"

int formatPCLs(const DeviceBuffer<int>& dWasHit,
               DeviceBuffer<int>& dHitsBeforeIndex,
               const DeviceBuffer<PCL12>& dIn12,
               const DeviceBuffer<Point3f>& dInPoint3f,
               const DeviceBuffer<int>& dInRingIds,
               DeviceBuffer<Point3f>& dOutPoint3f,
               DeviceBuffer<PCL12>& dOut12,
               DeviceBuffer<PCL24>& dOut24,
               DeviceBuffer<PCL48>& dOut48
);
