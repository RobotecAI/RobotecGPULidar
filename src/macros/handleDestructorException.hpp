#pragma once

#include <stdexcept>

void handleDestructorException(std::exception_ptr e, const char* string);

#define STR(s) #s
#define HANDLE_DESTRUCTOR_EXCEPTION                                                                                            \
	catch (std::exception & e) { handleDestructorException(std::current_exception(), e.what()); }                              \
	catch (...)                                                                                                                \
	{                                                                                                                          \
		const char* msg = "unknown exception at " __FILE__ ":" STR(__LINE__);                                                  \
		handleDestructorException(std::current_exception(), msg);                                                              \
	}
