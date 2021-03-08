using System;
using System.Runtime.InteropServices;

namespace GPULidarRaycaster
{
    internal static class NativeMethods
    {
        private static readonly DllLoadUtils dllLoadUtils = DllLoadUtilsFactory.GetDllLoadUtils();
        private static readonly IntPtr native_raycaster_lib = dllLoadUtils.LoadLibraryNoSuffix("native_gpu_lidar_raycaster");

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        internal delegate IntPtr Internal_CreateNativeRaycasterType();
        internal static Internal_CreateNativeRaycasterType
            Internal_CreateNativeRaycaster =
            (Internal_CreateNativeRaycasterType)Marshal.GetDelegateForFunctionPointer(dllLoadUtils.GetProcAddress(
            native_raycaster_lib,
            "Internal_CreateNativeRaycaster"),
            typeof(Internal_CreateNativeRaycasterType));

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        internal delegate void Internal_DestroyNativeRaycasterType(IntPtr obj);
        internal static Internal_DestroyNativeRaycasterType
            Internal_DestroyNativeRaycaster =
            (Internal_DestroyNativeRaycasterType)Marshal.GetDelegateForFunctionPointer(dllLoadUtils.GetProcAddress(
            native_raycaster_lib,
            "Internal_DestroyNativeRaycaster"),
            typeof(Internal_DestroyNativeRaycasterType));

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        internal delegate IntPtr Internal_GetLastErrorType();
        internal static Internal_GetLastErrorType
            Internal_GetLastError =
            (Internal_GetLastErrorType)Marshal.GetDelegateForFunctionPointer(dllLoadUtils.GetProcAddress(
            native_raycaster_lib,
            "Internal_GetLastError"),
            typeof(Internal_GetLastErrorType));

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        internal delegate int Internal_AddOrUpdateMeshType(IntPtr obj, [In, MarshalAs(UnmanagedType.LPStr)] string id, [In] Vector3f[] vertices,
          [In] Vector3f[] normals, [In] Vector2f[] texture_coordinates, [In] Vector3i[] indices, int indices_size, int size);
        internal static Internal_AddOrUpdateMeshType
            Internal_AddOrUpdateMesh =
            (Internal_AddOrUpdateMeshType)Marshal.GetDelegateForFunctionPointer(dllLoadUtils.GetProcAddress(
            native_raycaster_lib,
            "Internal_AddOrUpdateMesh"),
            typeof(Internal_AddOrUpdateMeshType));

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        internal delegate int Internal_RaycastType(IntPtr obj, [In, MarshalAs(UnmanagedType.LPStr)] string source_id, [In] Point3f source_pos,
          [In] Point3f[] directions, [In] int directions_count, [In] float range);
        internal static Internal_RaycastType
            Internal_Raycast =
            (Internal_RaycastType)Marshal.GetDelegateForFunctionPointer(dllLoadUtils.GetProcAddress(
            native_raycaster_lib,
            "Internal_Raycast"),
            typeof(Internal_RaycastType));

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        internal delegate int Internal_GetPointsType(IntPtr obj, ref IntPtr results, ref int results_count);
        internal static Internal_GetPointsType
            Internal_GetPoints =
            (Internal_GetPointsType)Marshal.GetDelegateForFunctionPointer(dllLoadUtils.GetProcAddress(
            native_raycaster_lib,
            "Internal_GetPoints"),
            typeof(Internal_GetPointsType));
    }
}
