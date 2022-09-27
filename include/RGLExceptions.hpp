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