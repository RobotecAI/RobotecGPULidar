#include <gtest/gtest.h>
#include <gmock/gmock.h>

#include <VArray.hpp>
#include <VArrayProxy.hpp>

using namespace ::testing;

TEST(VArray, Smoke)
{
	VArrayProxy<int>::Ptr array = VArrayProxy<int>::create(1);
	int value = 4;

	{
		// Goal:
		// CPU: ||
		// GPU: |4|
		int *devicePtr = array->getWritePtr(MemLoc::Device);
		EXPECT_TRUE(devicePtr != nullptr);
		CHECK_CUDA(cudaMemcpy(devicePtr, &value, sizeof(int), cudaMemcpyDefault));
	}
	{
		// CPU: |4|
		// GPU: |4|
		const int* hostPtr = reinterpret_cast<const int*>(array->getReadPtr(MemLoc::Host));
		EXPECT_TRUE(hostPtr != nullptr);
		EXPECT_EQ(hostPtr[0], value);
	}
	{
		// CPU: |4|3|0|
		// GPU: |4|
		array->resize(3);
		EXPECT_EQ(array->getCount(), 3);
		(*array)[1] = 3;
		// Last element should have been zero-initialized
	}
	{
		// CPU: |4|3|0|
		// GPU: |4|3|0|
		int values[3];
		const int* devicePtr = array->getReadPtr(MemLoc::Device);
		CHECK_CUDA(cudaMemcpy(values, devicePtr, 3 * sizeof(int), cudaMemcpyDefault));
		EXPECT_EQ(values[0], 4);
		EXPECT_EQ(values[1], 3);
		EXPECT_EQ(values[2], 0);
	}
}