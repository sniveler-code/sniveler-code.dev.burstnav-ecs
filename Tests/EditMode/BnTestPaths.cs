using System.IO;
using UnityEditor.PackageManager;

namespace BurstNav.Tests
{
    public static class BnTestPaths
    {
        public static string PackageRoot =>
            PackageInfo.FindForAssembly(typeof(BnTierInfo).Assembly).resolvedPath;

        public static string Package(string relative) =>
            Path.Combine(PackageRoot, relative.Replace('/', Path.DirectorySeparatorChar));
    }
}
