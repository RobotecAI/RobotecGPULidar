#include <gtest/gtest.h>
#include <gmock/gmock-matchers.h>

#include <memory/HostPinnedArray.hpp>
#include <StreamBoundObjectsManager.hpp>
#include <CudaEvent.hpp>


struct ArrayChangeStream : public ::testing::Test
{
	static constexpr int COUNT = 5;
	HostPinnedArray<int>::Ptr nonZeroIntPattern = HostPinnedArray<int>::create();
	HostPinnedArray<float>::Ptr nonZeroFloatPattern = HostPinnedArray<float>::create();
	HostPinnedArray<int>::Ptr hostIntTmp = HostPinnedArray<int>::create();
	HostPinnedArray<float>::Ptr hostFloatTmp = HostPinnedArray<float>::create();
	CudaStream::Ptr streamA = CudaStream::create();
	CudaStream::Ptr streamB = CudaStream::create();
	std::atomic<bool> shouldSleep = true;

protected:
	void SetUp() override
	{
		for (int i = 1; i <= COUNT; ++i) {
			nonZeroIntPattern->append(i);
			nonZeroFloatPattern->append(static_cast<float>(i));
		}
	}

	void TearDown() override
	{
		// Unblock streamA, otherwise the test would hang.
		shouldSleep = false;
	}

	template<typename T>
	void clearAndCheck(DeviceAsyncArray<T>::Ptr array, HostPinnedArray<T>::Ptr hostArray)
	{
		// Reset array contents to zeros. Should work because we changed stream.
		array->clear(false);
		array->resize(COUNT, true, false);

		// Synchronizes array's stream.
		// If setStream actually changed the stream, execution will continue.
		// If not, the test would hang.
		hostArray->copyFrom(array);

		for (int i = 0; i < COUNT; ++i) {
			EXPECT_EQ(hostArray->at(i), static_cast<T>(0));
		}
	}

};

static void waitCb(cudaStream_t _1,  cudaError_t _2, void*  userData)
{
	std::atomic<bool>& shouldSleep = *(reinterpret_cast<std::atomic<bool>*>(userData));
	while (shouldSleep)
		;
}

TEST_F(ArrayChangeStream, Standalone)
{
	DeviceAsyncArray<int>::Ptr array = DeviceAsyncArray<int>::createStandalone(streamA);
	array->copyFrom(nonZeroIntPattern); // Synchronizes array's stream.

	// Block streamA. This way, scheduled operations will be not executed.
	CHECK_CUDA(cudaStreamAddCallback(streamA->get(), waitCb, &shouldSleep, 0));

	// Change array's stream.
	array->setStream(streamB);

	clearAndCheck<int>(array, hostIntTmp);
}

TEST_F(ArrayChangeStream, WithManager)
{
	StreamBoundObjectsManager arrayMgr;

	arrayMgr.setStream(streamA);

	// Create arrays through managers
	auto arrayA = DeviceAsyncArray<int>::createWithManager(arrayMgr);
	auto arrayB = DeviceAsyncArray<float>::createWithManager(arrayMgr);
	auto arrayC = DeviceAsyncArray<float>::createWithManager(arrayMgr);

	// Calling setStream twice with the same stream should have no effect
	arrayMgr.setStream(streamA);

	// Do some operations in the stream
	arrayA->copyFrom(nonZeroIntPattern);
	arrayB->copyFrom(nonZeroFloatPattern);
	arrayC->copyFrom(nonZeroFloatPattern);

	// Remove one of the arrays, arrayMgr should not touch it afterwards
	arrayC.reset();

	// Block streamA. This way, scheduled operations will be not executed.
	CHECK_CUDA(cudaStreamAddCallback(streamA->get(), waitCb, &shouldSleep, 0));

	// Provide a new stream for arrays.
	arrayMgr.setStream(streamB);

	clearAndCheck<int>(arrayA, hostIntTmp);
	clearAndCheck<float>(arrayB, hostFloatTmp);
}
