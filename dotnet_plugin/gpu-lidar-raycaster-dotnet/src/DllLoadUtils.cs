using System;
using System.Runtime;
using System.Runtime.InteropServices;

namespace GPULidarRaycaster {

    public class UnsatisfiedLinkError : System.Exception {
      public UnsatisfiedLinkError () : base () { }
      public UnsatisfiedLinkError (string message) : base (message) { }
      public UnsatisfiedLinkError (string message, System.Exception inner) : base (message, inner) { }
    }

    public class UnknownPlatformError : System.Exception {
      public UnknownPlatformError () : base () { }
      public UnknownPlatformError (string message) : base (message) { }
      public UnknownPlatformError (string message, System.Exception inner) : base (message, inner) { }
    }

    public enum Platform {
      Unix,
      Unknown
    }

    public class DllLoadUtilsFactory {
      [DllImport ("libdl.so", EntryPoint = "dlopen")]
      private static extern IntPtr dlopen_unix (String fileName, int flags);

      [DllImport ("libdl.so", EntryPoint = "dlclose")]
      private static extern int dlclose_unix (IntPtr handle);

      const int RTLD_NOW = 2;

      public static DllLoadUtils GetDllLoadUtils () {
        switch (CheckPlatform ()) {
          case Platform.Unix:
            return new DllLoadUtilsUnix ();
          default:
            throw new UnknownPlatformError ();
        }
      }

      private static bool IsUnix () {
        try {
          IntPtr ptr = dlopen_unix ("libdl.so", RTLD_NOW);
          dlclose_unix (ptr);
          return true;
        } catch (TypeLoadException) {
          return false;
        }
      }

      private static Platform CheckPlatform ()
      {
          if (IsUnix())
          {
              return Platform.Unix;
          }
          else
          {
              return Platform.Unknown;
          }
      }
    }

    public interface DllLoadUtils {
      IntPtr LoadLibrary (string fileName);
      IntPtr LoadLibraryNoSuffix (string fileName);
      void FreeLibrary (IntPtr handle);
      IntPtr GetProcAddress (IntPtr dllHandle, string name);
    }

    internal class DllLoadUtilsUnix : DllLoadUtils {

      [DllImport ("libdl.so", ExactSpelling = true)]
      private static extern IntPtr dlopen (String fileName, int flags);

      [DllImport ("libdl.so", ExactSpelling = true)]
      private static extern IntPtr dlsym (IntPtr handle, String symbol);

      [DllImport ("libdl.so", ExactSpelling = true)]
      private static extern int dlclose (IntPtr handle);

      [DllImport ("libdl.so", ExactSpelling = true)]
      private static extern IntPtr dlerror ();

      const int RTLD_NOW = 0x00002;
      const int RTLD_DEEPBIND = 0x00008;

      public void FreeLibrary (IntPtr handle) {
        dlclose (handle);
      }

      public IntPtr GetProcAddress (IntPtr dllHandle, string name) {
        // clear previous errors if any
        dlerror ();
        var res = dlsym (dllHandle, name);
        var errPtr = dlerror ();
        if (errPtr != IntPtr.Zero) {
          throw new Exception ("dlsym: " + Marshal.PtrToStringAnsi (errPtr));
        }
        return res;
      }

      public IntPtr LoadLibrary (string fileName) {
        string libraryName = "lib" + fileName + "_native.so";
        IntPtr ptr = dlopen (libraryName, RTLD_NOW);
        if (ptr == IntPtr.Zero) {
          throw new UnsatisfiedLinkError (libraryName);
        }
        return ptr;
      }

      public IntPtr LoadLibraryNoSuffix (string fileName) {
        string libraryName = "lib" + fileName + ".so";
        IntPtr ptr = dlopen (libraryName, RTLD_NOW);
        if (ptr == IntPtr.Zero) {
          throw new UnsatisfiedLinkError (libraryName);
        }
        return ptr;
      }
    }
}
