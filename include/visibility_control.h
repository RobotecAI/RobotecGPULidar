#ifndef GPU_LIDAR_RAYCASTER_C__VISIBILITY_CONTROL_H_
 #define GPU_LIDAR_RAYCASTER_C__VISIBILITY_CONTROL_H_

 #ifdef __cplusplus
 extern "C"
 {
 #endif

 #if defined _WIN32 || defined __CYGWIN__
   #ifdef __GNUC__
     #define GPU_LIDAR_RAYCASTER_C_EXPORT __attribute__ ((dllexport))
     #define GPU_LIDAR_RAYCASTER_C_IMPORT __attribute__ ((dllimport))
   #else
     #define GPU_LIDAR_RAYCASTER_C_EXPORT __declspec(dllexport)
     #define GPU_LIDAR_RAYCASTER_C_IMPORT __declspec(dllimport)
   #endif
   #ifdef GPU_LIDAR_RAYCASTER_C_BUILDING_DLL
     #define GPU_LIDAR_RAYCASTER_C_PUBLIC GPU_LIDAR_RAYCASTER_C_EXPORT
   #else
     #define GPU_LIDAR_RAYCASTER_C_PUBLIC GPU_LIDAR_RAYCASTER_C_IMPORT
   #endif
   #define GPU_LIDAR_RAYCASTER_C_PUBLIC_TYPE GPU_LIDAR_RAYCASTER_C_PUBLIC
   #define GPU_LIDAR_RAYCASTER_C_LOCAL
 #else
   #define GPU_LIDAR_RAYCASTER_C_EXPORT __attribute__ ((visibility("default")))
   #define GPU_LIDAR_RAYCASTER_C_IMPORT
   #if __GNUC__ >= 4
     #define GPU_LIDAR_RAYCASTER_C_PUBLIC __attribute__ ((visibility("default")))
     #define GPU_LIDAR_RAYCASTER_C_LOCAL  __attribute__ ((visibility("hidden")))
   #else
     #define GPU_LIDAR_RAYCASTER_C_PUBLIC
     #define GPU_LIDAR_RAYCASTER_C_LOCAL
   #endif
   #define GPU_LIDAR_RAYCASTER_C_PUBLIC_TYPE
 #endif

 #ifdef __cplusplus
 }
 #endif

 #endif  // GPU_LIDAR_RAYCASTER_C__VISIBILITY_CONTROL_H_
