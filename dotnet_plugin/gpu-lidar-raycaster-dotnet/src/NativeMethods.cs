using System;
using System.Runtime.InteropServices;

namespace GPULidarRaycaster
{
    internal static class NativeMethods
    {
        private static readonly DllLoadUtils dllLoadUtils = DllLoadUtilsFactory.GetDllLoadUtils();
        private static readonly IntPtr native_raycaster_lib = dllLoadUtils.LoadLibraryNoSuffix("RobotecGPULidar");

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        internal delegate int Internal_CreateNativeRaycasterType(out IntPtr raycaster);
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
        internal delegate int Internal_AddMeshType(IntPtr obj, [In, MarshalAs(UnmanagedType.LPStr)] string id,
          [In] float[] transform, [In] bool is_global, [In] Vector3f[] vertices, [In] Vector3f[] normals,
          [In] Vector2f[] texture_coordinates, [In] Vector3i[] indices, int indices_size, int mesh_size, int transform_size);
        internal static Internal_AddMeshType
            Internal_AddMesh =
            (Internal_AddMeshType)Marshal.GetDelegateForFunctionPointer(dllLoadUtils.GetProcAddress(
            native_raycaster_lib,
            "Internal_AddMesh"),
            typeof(Internal_AddMeshType));

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        internal delegate int Internal_RemoveMeshType(IntPtr obj, [In, MarshalAs(UnmanagedType.LPStr)] string id);
        internal static Internal_RemoveMeshType
            Internal_RemoveMesh =
            (Internal_RemoveMeshType)Marshal.GetDelegateForFunctionPointer(dllLoadUtils.GetProcAddress(
            native_raycaster_lib,
            "Internal_RemoveMesh"),
            typeof(Internal_RemoveMeshType));

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        internal delegate int Internal_UpdateMeshTransformType(IntPtr obj, [In, MarshalAs(UnmanagedType.LPStr)] string id, [In] float[] transform, int transform_size);
        internal static Internal_UpdateMeshTransformType
            Internal_UpdateMeshTransform =
            (Internal_UpdateMeshTransformType)Marshal.GetDelegateForFunctionPointer(dllLoadUtils.GetProcAddress(
            native_raycaster_lib,
            "Internal_UpdateMeshTransform"),
            typeof(Internal_UpdateMeshTransformType));

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        internal delegate int Internal_CreateLidarContextType(IntPtr lidar, out IntPtr context, [In] float[] sourcePosesFloats, [In] int sourcePosesFloatsCount,  [In] int[] lidarArrayRingIds, [In] int lidarArrayRingCount);
        internal static Internal_CreateLidarContextType
            Internal_CreateLidarContext =
            (Internal_CreateLidarContextType)Marshal.GetDelegateForFunctionPointer(dllLoadUtils.GetProcAddress(
            native_raycaster_lib,
            "Internal_CreateLidarContext"),
            typeof(Internal_CreateLidarContextType));

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        internal delegate int Internal_DestroyLidarContextType(IntPtr lidar, IntPtr context);
        internal static Internal_DestroyLidarContextType
            Internal_DestroyLidarContext =
            (Internal_DestroyLidarContextType)Marshal.GetDelegateForFunctionPointer(dllLoadUtils.GetProcAddress(
            native_raycaster_lib,
            "Internal_DestroyLidarContext"),
            typeof(Internal_DestroyLidarContextType));

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        internal delegate int Internal_RaycastType(IntPtr obj, IntPtr lidarCtx, [In, MarshalAs(UnmanagedType.LPStr)] string source_id,
          [In] float [] lidarPose, [In] float [] postRaycastTransform, [In] float range);
        internal static Internal_RaycastType
            Internal_Raycast =
            (Internal_RaycastType)Marshal.GetDelegateForFunctionPointer(dllLoadUtils.GetProcAddress(
            native_raycaster_lib,
            "Internal_Raycast"),
            typeof(Internal_RaycastType));

        [UnmanagedFunctionPointer(CallingConvention.Cdecl)]
        internal delegate int Internal_GetPointsType(IntPtr obj, IntPtr lidarCtx, IntPtr xyz, IntPtr pcl12, IntPtr pcl24, IntPtr pcl48, ref int results_count);
        internal static Internal_GetPointsType
            Internal_GetPoints =
            (Internal_GetPointsType)Marshal.GetDelegateForFunctionPointer(dllLoadUtils.GetProcAddress(
            native_raycaster_lib,
            "Internal_GetPoints"),
            typeof(Internal_GetPointsType));
    }
}
