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

#pragma once

#define FORWARD_ITERATION(dst, ...)                                                                                            \
	__VA_ARGS__ auto begin() { return std::begin(dst); }                                                                       \
	__VA_ARGS__ auto rbegin() { return std::rbegin(dst); }                                                                     \
	__VA_ARGS__ auto end() { return std::end(dst); }                                                                           \
	__VA_ARGS__ auto rend() { return std::rend(dst); }                                                                         \
	__VA_ARGS__ auto begin() const { return std::cbegin(dst); }                                                                \
	__VA_ARGS__ auto rbegin() const { return std::crbegin(dst); }                                                              \
	__VA_ARGS__ auto end() const { return std::cend(dst); }                                                                    \
	__VA_ARGS__ auto rend() const { return std::crend(dst); }                                                                  \
	__VA_ARGS__ auto cbegin() const { return std::cbegin(dst); }                                                               \
	__VA_ARGS__ auto crbegin() const { return std::crbegin(dst); }                                                             \
	__VA_ARGS__ auto cend() const { return std::cend(dst); }                                                                   \
	__VA_ARGS__ auto crend() const { return std::crend(dst); }

#define FORWARD_INDEX_OPERATOR(dst, ...)                                                                                       \
	__VA_ARGS__ T& operator[](int i) { return dst[i]; }                                                                        \
	__VA_ARGS__ const T& operator[](int i) const { return dst[i]; }
