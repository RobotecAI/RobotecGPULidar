#include <cuda_runtime_api.h>

#include "formatPCL.h"

#include "HostPinnedBuffer.hpp"
#include "DeviceBuffer.hpp"
#include <thrust/device_ptr.h>
#include <thrust/scan.h>
#include <cstdio>

__global__ void kFormatAll(int sparsePointCount,
                           const int* wasHit,
                           const int* threadWriteIndex,
                           const PCL12* in12,
                           const Point3f* inPoint3f,
                           const int* inRingIds,
                           const int inRingCount,
                           Point3f* outPoint3f,
                           PCL12* out12,
                           PCL24* out24,
                           PCL48* out48)
{
    int rIdx = threadIdx.x + blockIdx.x * blockDim.x;
    if (rIdx >= sparsePointCount) {
        return;
    }
    if (!wasHit[rIdx]) {
        return;
    }
    int wIdx = threadWriteIndex[rIdx] - 1;

    outPoint3f[wIdx] = inPoint3f[rIdx];
    out12[wIdx] = in12[rIdx];

    const float x = in12[rIdx].x;
    const float y = in12[rIdx].y;
    const float z = in12[rIdx].z;

    out48[wIdx].x = out24[wIdx].x = x;
    out48[wIdx].y = out24[wIdx].y = y;
    out48[wIdx].z = out24[wIdx].z = z;
    out48[wIdx].intensity = out24[wIdx].intensity = 100;
    out48[wIdx].ring = out24[wIdx].ring = inRingIds[rIdx % inRingCount];

    out48[wIdx].azimuth = 0; // TODO
    out48[wIdx].distance = sqrt(x*x + y*y + z*z);
    out48[wIdx].return_type = 0; // TODO
    out48[wIdx].time_stamp = 0; // TODO
}

void formatPCLsAsync(cudaStream_t stream,
                     const DeviceBuffer<int>& dWasHit,
                     DeviceBuffer<int>& dHitsBeforeIndex,
                     const DeviceBuffer<PCL12>& dIn12,
                     const DeviceBuffer<Point3f>& dInPoint3f,
                     const DeviceBuffer<int>& dInRingIds,
                     DeviceBuffer<Point3f>& dOutPoint3f,
                     DeviceBuffer<PCL12>& dOut12,
                     DeviceBuffer<PCL24>& dOut24,
                     DeviceBuffer<PCL48>& dOut48
)
{
    int sparsePointCount = dWasHit.getElemCount();

    // First step: perform stream compaction
    {
        // beg and end could be used as const pointers, however thrust does not support it
        auto beg = thrust::device_ptr<int>(const_cast<int*>(dWasHit.readDevice()));
        auto end = thrust::device_ptr<int>(const_cast<int*>(dWasHit.readDevice()) + sparsePointCount);
        auto dst = thrust::device_ptr<int>(dHitsBeforeIndex.writeDevice());

        // Note: this will compile only in a .cu file
        thrust::inclusive_scan(thrust::cuda::par.on(stream), beg, end, dst);
    }

    // Second step: format PCLs
    {
        kFormatAll<<<(sparsePointCount + 256) / 256, 256, 0, stream>>>(
                sparsePointCount,
                dWasHit.readDevice(),
                dHitsBeforeIndex.readDevice(),
                dIn12.readDevice(),
                dInPoint3f.readDevice(),
                dInRingIds.readDevice(),
                dInRingIds.getElemCount(),
                dOutPoint3f.writeDevice(),
                dOut12.writeDevice(),
                dOut24.writeDevice(),
                dOut48.writeDevice()
        );
    }


}