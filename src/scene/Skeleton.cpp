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

#include <functional>

#include <scene/Skeleton.hpp>

API_OBJECT_INSTANCE(Skeleton);

Skeleton::Skeleton(const int32_t* parentIdxes, const Mat3x4f* restposes, const char** names, std::size_t bonesCount)
{
	// TODO(msz-rai): Validate root and tree

	bones.resize(bonesCount);

	// Assign child to bones
	for (int i = 1; i < bonesCount; ++i) {
		bones[parentIdxes[i]].childIdxes.insert(i);
	}

	// Compute restpose to model transform
	std::function<void(int32_t, const Mat3x4f&)> computeRestposeToModel =
	    [this, &restposes, &computeRestposeToModel](int32_t boneIdx, const Mat3x4f& parentTransform) {
		    Bone& bone = bones[boneIdx];
		    bone.restposeToModelTransform = parentTransform * restposes[boneIdx];
		    for (auto&& childIdx : bone.childIdxes) {
			    computeRestposeToModel(childIdx, bone.restposeToModelTransform);
		    }
	    };
	computeRestposeToModel(0, Mat3x4f::identity());

	// Assign names
	if (names != nullptr) {
		for (int i = 0; i < bonesCount; ++i) {
			bones[i].name = names[i];
		}
	}
}
