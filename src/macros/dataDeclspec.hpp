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

#pragma once

// NOTE: This macro is supposed to be used for static global data that needs to be accessible externally (e.g. in tests).

#if defined _MSC_VER && !defined RGL_BUILD
// If we are building client code (e.g. tests) on Windows, we need to explicitly import global data dymbols.
#define DATA_DECLSPEC __declspec(dllimport)
#else
// On Windows, when building library code and tests, all symbols will be exported thanks to CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS.
// On Linux symbols are exported by default.
#define DATA_DECLSPEC
#endif
