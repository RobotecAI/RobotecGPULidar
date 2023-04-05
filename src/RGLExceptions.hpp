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

struct InvalidAPIObject : public std::invalid_argument
{
	using std::invalid_argument::invalid_argument;
};

struct InvalidAPIArgument : public std::invalid_argument
{
	using std::invalid_argument::invalid_argument;
};

struct InvalidPipeline : public std::invalid_argument
{
	using std::invalid_argument::invalid_argument;
};

struct InvalidFilePath : public std::invalid_argument
{
    using std::invalid_argument::invalid_argument;
};

struct RecordError : public std::logic_error
{
    using std::logic_error::logic_error;
};

struct UdpError : public std::logic_error
{
	using std::logic_error::logic_error;
};
