#pragma once

#ifdef __cplusplus
#define NO_MANGLING extern "C"
#else // NOT __cplusplus
#define NO_MANGLING
#endif

#if defined _WIN32 || defined __CYGWIN__
 #ifdef __GNUC__
  #define RGL_VISIBLE __attribute__ ((dllexport))
 #else
  #define RGL_VISIBLE __declspec(dllexport)
 #endif // __GNUC__
#else
 #define RGL_VISIBLE __attribute__ ((visibility("default")))
 #if __GNUC__ >= 4
  #define RGL_VISIBLE __attribute__ ((visibility("default")))
 #else
  #define RGL_VISIBLE
 #endif
#endif // _WIN32 || __CYGWIN__

#define RGL_API NO_MANGLING RGL_VISIBLE
