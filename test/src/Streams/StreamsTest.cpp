#include <RGLFields.hpp>
#include <gpu/testKernel.hpp>
#include <graph/Node.hpp>
#include <gtest/gtest.h>
#include <utils.hpp>

using namespace testing;

class StreamsTest : public RGLAutoCleanupTest {
protected:
};

TEST_F(StreamsTest, valid_arguments)
{
    // Prepare two non-default streams.
    cudaStream_t graphStream;
    cudaStream_t copyStream;

    cudaStreamCreate(&graphStream);
    cudaStreamCreate(&copyStream);

    int N = 20000;
    size_t size = N * sizeof(float);

    // Allocate input vector h_A in host memory
    float* h_A = (float*)malloc(size);

    // Initialize input vectors
    for (int i = 0; i < N; ++i) {
        h_A[i] = i;
    }
    // Create event for sensitive job (kernel) end.
    cudaEvent_t jobDoneEvent;
    // Create event without timer (used for profiling).
    // This way, we have got some speed, and also according to CUDA documentation, this flag is neccessary to make sure that multi-stream synchronization is bulltetproof.
    cudaEventCreateWithFlags(&jobDoneEvent, cudaEventDisableTiming);

    // Copy vector from host memory to device memory
    float* d_A;
    cudaMallocAsync(&d_A, size, graphStream);
    cudaMemcpyAsync(d_A, h_A, size, cudaMemcpyHostToDevice, graphStream);

    // Run sensitive job that we forecast, it takes some time.
    testKernelWrapper(N, d_A, graphStream);

    // Record synchronization event to the stream after sensitive job.
    cudaEventRecord(jobDoneEvent, graphStream);

    //Synchronize streams. This is pass or not for the test.
    // All API Calls in given stream, will wait for the event  to be completed.
    cudaStreamWaitEvent(copyStream, jobDoneEvent);

    // Copy result from device memory to host memory by substream
    // cudaMemcpy(h_A, d_A, size, cudaMemcpyDeviceToHost);
    cudaMemcpyAsync(h_A, d_A, size, cudaMemcpyDeviceToHost, copyStream);

    for (int i = 0; i < N; ++i) {
        float temp = h_A[i];
        EXPECT_EQ(temp, i * i);
    }

    cudaStreamDestroy(graphStream);
    cudaStreamDestroy(copyStream);

    cudaEventDestroy(jobDoneEvent);

    free(h_A);
    cudaFree(d_A);
}
