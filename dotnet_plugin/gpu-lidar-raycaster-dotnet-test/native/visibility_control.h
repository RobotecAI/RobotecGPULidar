#ifndef MODEL_LOADER_C__VISIBILITY_CONTROL_H_
 #define MODEL_LOADER_C__VISIBILITY_CONTROL_H_

 #ifdef __cplusplus
 extern "C"
 {
 #endif

 #if defined _WIN32 || defined __CYGWIN__
   #ifdef __GNUC__
     #define MODEL_LOADER_C_EXPORT __attribute__ ((dllexport))
     #define MODEL_LOADER_C_IMPORT __attribute__ ((dllimport))
   #else
     #define MODEL_LOADER_C_EXPORT __declspec(dllexport)
     #define MODEL_LOADER_C_IMPORT __declspec(dllimport)
   #endif
   #ifdef MODEL_LOADER_C_BUILDING_DLL
     #define MODEL_LOADER_C_PUBLIC MODEL_LOADER_C_EXPORT
   #else
     #define MODEL_LOADER_C_PUBLIC MODEL_LOADER_C_IMPORT
   #endif
   #define MODEL_LOADER_C_PUBLIC_TYPE MODEL_LOADER_C_PUBLIC
   #define MODEL_LOADER_C_LOCAL
 #else
   #define MODEL_LOADER_C_EXPORT __attribute__ ((visibility("default")))
   #define MODEL_LOADER_C_IMPORT
   #if __GNUC__ >= 4
     #define MODEL_LOADER_C_PUBLIC __attribute__ ((visibility("default")))
     #define MODEL_LOADER_C_LOCAL  __attribute__ ((visibility("hidden")))
   #else
     #define MODEL_LOADER_C_PUBLIC
     #define MODEL_LOADER_C_LOCAL
   #endif
   #define MODEL_LOADER_C_PUBLIC_TYPE
 #endif

 #ifdef __cplusplus
 }
 #endif

 #endif  // MODEL_LOADER_C__VISIBILITY_CONTROL_H_
