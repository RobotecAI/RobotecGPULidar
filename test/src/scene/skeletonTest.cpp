// Copyright 2024 Robotec.AI
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

#include <helpers/commonHelpers.hpp>

#include <math/Mat3x4f.hpp>
#include <scene/Skeleton.hpp>

class SkeletonTest : public RGLTest
{};

TEST_F(SkeletonTest, CreationTest)
{
	rgl_skeleton_t skeletonHandle = nullptr;

	/*
	 * Skeleton definition of this test case:
	 *
	 *   B4 (0,0,1) #           # B2 (0,0,1)
	 *              |           |
	 *              |           |
	 *   B3 (0,1,1) #___________# B1 (0,1,1)
	 *                    |
	 *                    |
	 *                    # B0 (0,0,1)
	 *                    |
	 *                    * Model Origin
	 */

	std::vector<int32_t> parentIdxes = {
	    -1, // B0 (no parents - root bone)
	    0,  // B1
	    1,  // B2
	    0,  // B3
	    3,  // B4
	};
	std::vector<rgl_mat3x4f> restposes = {
	    Mat3x4f::TRS({0, 0, 1}).toRGL(), // B0
	    Mat3x4f::TRS({0, 1, 1}).toRGL(), // B1
	    Mat3x4f::TRS({0, 0, 1}).toRGL(), // B2
	    Mat3x4f::TRS({0, 1, 1}).toRGL(), // B3
	    Mat3x4f::TRS({0, 0, 1}).toRGL(), // B4
	};

	ASSERT_EQ(parentIdxes.size(), restposes.size());
	ASSERT_RGL_SUCCESS(rgl_skeleton_create(&skeletonHandle, parentIdxes.data(), restposes.data(), nullptr, restposes.size()));

	auto skeleton = Skeleton::validatePtr(skeletonHandle);
	ASSERT_TRUE(skeleton != nullptr);

	const auto& bones = skeleton->getBones();
	ASSERT_EQ(bones.size(), restposes.size());

	// Validate child indexes
	ASSERT_EQ(bones[0].childIdxes, (std::set<int32_t>{1, 3}));
	ASSERT_EQ(bones[1].childIdxes, (std::set<int32_t>{2}));
	ASSERT_TRUE(bones[2].childIdxes.empty());
	ASSERT_EQ(bones[3].childIdxes, (std::set<int32_t>{4}));
	ASSERT_TRUE(bones[4].childIdxes.empty());

	// Validate restpose to model transforms
	ASSERT_EQ(bones[0].restposeToModelTransform, Mat3x4f::TRS({0, 0, 1}));
	ASSERT_EQ(bones[1].restposeToModelTransform, Mat3x4f::TRS({0, 1, 2}));
	ASSERT_EQ(bones[2].restposeToModelTransform, Mat3x4f::TRS({0, 1, 3}));
	ASSERT_EQ(bones[3].restposeToModelTransform, Mat3x4f::TRS({0, 1, 2}));
	ASSERT_EQ(bones[4].restposeToModelTransform, Mat3x4f::TRS({0, 1, 3}));

	ASSERT_RGL_SUCCESS(rgl_skeleton_destroy(skeletonHandle));
	bool isAlive = true;
	ASSERT_RGL_SUCCESS(rgl_skeleton_is_alive(skeletonHandle, &isAlive));
	ASSERT_FALSE(isAlive);
}
