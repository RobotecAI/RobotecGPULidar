#pragma once

 #ifdef __cplusplus
 extern "C" {
 #endif

 #if defined _WIN32 || defined __CYGWIN__
   #ifdef __GNUC__
     #define RGL_API __attribute__ ((dllexport))
   #else
     #define RGL_API __declspec(dllexport)
   #endif // __GNUC__
   #define GPU_LIDAR_RAYCASTER_C_PUBLIC_TYPE GPU_LIDAR_RAYCASTER_C_PUBLIC
   #define GPU_LIDAR_RAYCASTER_C_LOCAL
 #else
   #define RGL_API extern "C" __attribute__ ((visibility("default")))
   #if __GNUC__ >= 4
     #define RGL_API extern "C" __attribute__ ((visibility("default")))
   #else
     #define RGL_API
   #endif
 #endif // _WIN32 || __CYGWIN__

 #ifdef __cplusplus
 }
 #endif
