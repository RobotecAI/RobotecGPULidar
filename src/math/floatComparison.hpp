// Copyright 2022 Robotec.AI
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

#include <cmath>

// Relative comparisons of floating-point values after D. E. Knuth, The Art of Computer Programming, Vol. 2:
// Seminumerical Algorithms, section 4.2.2. The tolerance is epsilon scaled by the larger magnitude of the two
// values, or by the smaller one for essentiallyEqual, which is the stricter test.

template<typename T>
auto largerMagnitude(T a, T b)
{
	const auto absA = fabs(a);
	const auto absB = fabs(b);
	return absA < absB ? absB : absA;
}

template<typename T>
auto smallerMagnitude(T a, T b)
{
	const auto absA = fabs(a);
	const auto absB = fabs(b);
	return absB < absA ? absB : absA;
}

template<typename T>
bool approximatelyEqual(T a, T b, T epsilon)
{
	return fabs(a - b) <= largerMagnitude(a, b) * epsilon;
}

template<typename T>
bool essentiallyEqual(T a, T b, T epsilon)
{
	return fabs(a - b) <= smallerMagnitude(a, b) * epsilon;
}

template<typename T>
bool definitelyGreaterThan(T a, T b, T epsilon)
{
	return (a - b) > largerMagnitude(a, b) * epsilon;
}

template<typename T>
bool definitelyLessThan(T a, T b, T epsilon)
{
	return (b - a) > largerMagnitude(a, b) * epsilon;
}
