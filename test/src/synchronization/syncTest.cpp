#include "testKernel.hpp"
#include <RGLFields.hpp>
#include <cuda.h>
#include <gtest/gtest.h>
#include <utils.hpp>

using namespace testing;

class SyncTests : public RGLAutoCleanupTest {
protected:
};

TEST_F(SyncTests, GraphAndCopyStream)
{
    // Prepare two non-default streams.
    cudaStream_t graphStream;
    cudaStream_t copyStream;

    cudaFree(0);
    CHECK_CUDA(cudaStreamCreate(&graphStream));
    CHECK_CUDA(cudaStreamCreate(&copyStream));
    
    auto testDataCount = 20000;
    size_t size = testDataCount * sizeof(float);

    // Allocate input vector hostArray in host memory
    float* hostArray = (float*)malloc(size);

    //Create dummy arrays in order to perform parallel asynchronous copy operation on the copy stream.
    float* dummyHostArray = (float*)malloc(size);

    // Initialize input vectors
    for (int i = 0; i < testDataCount; ++i) {
        hostArray[i] = i;
        dummyHostArray[i] = i;
    }
    // Create event for sensitive job (kernel) end.
    cudaEvent_t jobDoneEvent;

    // Create event without timer (used for profiling).

    // According to CUDA documentation:
    // Events created with this flag specified and the cudaEventBlockingSync flag not specified
    // will provide the best performance when used with cudaStreamWaitEvent() and cudaEventQuery().
    // https://docs.nvidia.com/cuda/cuda-runtime-api/group__CUDART__EVENT.html#group__CUDART__EVENT_1g7b317e07ff385d85aa656204b971a042
    CHECK_CUDA(cudaEventCreateWithFlags(&jobDoneEvent, cudaEventDisableTiming));

    // Copy vector from host memory to device memory
    float* deviceArray;
    CHECK_CUDA(cudaMallocAsync(&deviceArray, size, graphStream));
    CHECK_CUDA(cudaMemcpyAsync(deviceArray, hostArray, size, cudaMemcpyHostToDevice, graphStream));

    // Run sensitive job that we forecast, it takes some time.
    testKernelWrapper(testDataCount, deviceArray, graphStream);

    // Record synchronization event to the stream after sensitive job.
    CHECK_CUDA(cudaEventRecord(jobDoneEvent, graphStream));

    // Perform copy operation on the copyStream parallel to graphStream kernel.
    float* dummyDeviceArray;
    CHECK_CUDA(cudaMallocAsync(&dummyDeviceArray, size, copyStream));
    CHECK_CUDA(cudaMemcpyAsync(dummyDeviceArray, dummyHostArray, size, cudaMemcpyHostToDevice, copyStream));
    CHECK_CUDA(cudaMemcpyAsync(dummyHostArray, dummyDeviceArray, size, cudaMemcpyDeviceToHost, copyStream));

    // Synchronize streams. This is pass or not for the test.
    //  All API Calls in given stream, will wait for the event  to be completed.
    CHECK_CUDA(cudaStreamWaitEvent(copyStream, jobDoneEvent));

    // Copy result from device memory to host memory by substream
    // cudaMemcpy(hostArray, deviceArray, size, cudaMemcpyDeviceToHost);
    CHECK_CUDA(cudaMemcpyAsync(hostArray, deviceArray, size, cudaMemcpyDeviceToHost, copyStream));

    for (int i = 0; i < testDataCount; ++i) {
        float temp = hostArray[i];
        EXPECT_EQ(temp, i * i);
    }

    CHECK_CUDA(cudaStreamDestroy(graphStream));
    CHECK_CUDA(cudaStreamDestroy(copyStream));

    CHECK_CUDA(cudaEventDestroy(jobDoneEvent));

    free(hostArray);
    CHECK_CUDA(cudaFreeAsync(deviceArray, 0));
}
