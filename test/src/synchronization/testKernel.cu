
using clock_value_t = long long;

#define LIMIT(count)                                         \
    const int tid = (blockIdx.x * blockDim.x + threadIdx.x); \
    do {                                                     \
        if (tid >= count) {                                  \
            return;                                          \
        }                                                    \
    } while (false)

__global__ void testKernel(size_t pointCount, float* infloats)
{
    // Waste some time
    clock_value_t start = clock64();
    clock_value_t cycles_elapsed;
    do {
        cycles_elapsed = clock64() - start;
    } while (cycles_elapsed < 100000);

    LIMIT(pointCount);
    infloats[tid] = infloats[tid] * infloats[tid];
}

void testKernelWrapper(size_t pointCount, float* inFloats, cudaStream_t stream)
{
    // Invoke kernel
    int threadsPerBlock = 256;
    int blocksPerGrid = (pointCount + threadsPerBlock - 1) / threadsPerBlock;
    testKernel<<<blocksPerGrid, threadsPerBlock, 0, stream>>>(pointCount, inFloats);
}