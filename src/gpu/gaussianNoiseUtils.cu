// Copyright 2023 Robotec.AI
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

template<class T>
__device__ float length(const T& vector) {
	return sqrt(vector.x() * vector.x() + vector.y() * vector.y() + vector.z() * vector.z());
}

template<class T>
__device__ T scale(const T& point, float scale)
{
	return { point.x() * scale, point.y() * scale, point.z() * scale };
}

template<class T>
__device__ T normalize(const T& point)
{
	return scale(point, 1.0f / length(point));
}

template<class T>
__device__ T addDistanceNoise(const T& point, float distanceError)
{
	T pointNormalized = normalize(point);

	float distance = length(point);
	float newDistance = distance + distanceError;

	return scale(pointNormalized, newDistance);
}
