using System;
using System.IO;
using System.Linq;
using NUnit.Framework;
using UnityEditor.PackageManager;
using UnityEngine;

namespace BurstNav.Tests
{
    public class BnScaffoldingTests
    {
        private const string RuntimeAssembly = "SnivelerCode.BurstNav.Runtime";
        private const string FreeRuntimeAsmdef = "Runtime/SnivelerCode.BurstNav.Runtime.asmdef";
        private const string FreeEditModeAsmdef =
            "Tests/EditMode/SnivelerCode.BurstNav.Tests.EditMode.asmdef";
        private const string FreePlayModeAsmdef =
            "Tests/PlayMode/SnivelerCode.BurstNav.Tests.PlayMode.asmdef";

        private static readonly string[] RequiredDirectories =
        {
            "Runtime/Core",
            "Runtime/Systems",
            "Runtime/Authoring",
            "Tests/EditMode",
            "Tests/PlayMode",
        };

        private static readonly string[] RuntimePackages =
        {
            "Unity.Entities",
            "Unity.Entities.Hybrid",
            "Unity.Transforms",
            "Unity.Burst",
            "Unity.Collections",
            "Unity.Mathematics",
            "Unity.Jobs",
        };

        [Serializable]
        private class AsmdefModel
        {
            public string name;
            public string rootNamespace;
            public string[] references;
            public string[] includePlatforms;
            public string[] defineConstraints;
            public string[] precompiledReferences;
        }

        [Test]
        public void PackageLayout_Exists()
        {
            foreach (var directory in RequiredDirectories)
            {
                Assert.IsTrue(Directory.Exists(PackagePath(directory)), "Missing directory: " + directory);
            }
        }

        [Test]
        public void EveryAssetAndFolder_HasAMetaFile()
        {
            var root = PackageRoot;

            foreach (var entry in Directory.GetFileSystemEntries(root, "*", SearchOption.AllDirectories))
            {
                var relative = Path.GetRelativePath(root, entry);
                if (relative.Split(Path.DirectorySeparatorChar, Path.AltDirectorySeparatorChar).Any(IsHidden))
                {
                    continue;
                }

                if (Path.GetFileName(entry).EndsWith(".meta", StringComparison.Ordinal))
                {
                    continue;
                }

                Assert.IsTrue(File.Exists(entry + ".meta"), "Missing .meta for " + relative);
            }
        }

        [Test]
        public void FreeAssembly_ReferencesDotsPackagesAndOnlyTheFreeRuntime()
        {
            var asmdef = Load(FreeRuntimeAsmdef);

            Assert.AreEqual(RuntimeAssembly, asmdef.name);
            Assert.AreEqual("BurstNav", asmdef.rootNamespace);
            CollectionAssert.IsSubsetOf(RuntimePackages, asmdef.references, "Missing DOTS package reference");
            AssertOnlyFreeRuntimeReferences(asmdef.references);
            Assert.IsTrue(asmdef.includePlatforms == null || asmdef.includePlatforms.Length == 0,
                "Runtime assembly must build for all platforms");
        }

        [Test]
        public void FreeTestAssemblies_ReferenceTheFreeRuntimeOnly()
        {
            foreach (var path in new[] { FreeEditModeAsmdef, FreePlayModeAsmdef })
            {
                var asmdef = Load(path);

                Assert.AreEqual(
                    path == FreeEditModeAsmdef
                        ? "SnivelerCode.BurstNav.Tests.EditMode"
                        : "SnivelerCode.BurstNav.Tests.PlayMode",
                    asmdef.name);
                CollectionAssert.Contains(asmdef.references, RuntimeAssembly, path);
                AssertOnlyFreeRuntimeReferences(asmdef.references);
                CollectionAssert.Contains(asmdef.references, "UnityEngine.TestRunner", path);
                CollectionAssert.Contains(asmdef.precompiledReferences, "nunit.framework.dll", path);
                CollectionAssert.Contains(asmdef.defineConstraints, "UNITY_INCLUDE_TESTS", path);
            }
        }

        [Test]
        public void FreeEditModeAssembly_IsEditorOnly()
        {
            var asmdef = Load(FreeEditModeAsmdef);
            CollectionAssert.AreEqual(new[] { "Editor" }, asmdef.includePlatforms);
            CollectionAssert.Contains(asmdef.references, "UnityEditor.TestRunner");
        }

        [Test]
        public void FreePlayModeAssembly_BuildsForAllPlatforms()
        {
            var asmdef = Load(FreePlayModeAsmdef);
            Assert.IsEmpty(asmdef.includePlatforms);
        }

        [Test]
        public void TierIdentity_LivesInTheFreeAssembly()
        {
            Assert.AreEqual("BurstNav", typeof(BnTierInfo).Namespace);
            Assert.AreEqual(RuntimeAssembly, typeof(BnTierInfo).Assembly.GetName().Name);
            Assert.IsFalse(BnTierInfo.IsPro);
            Assert.AreEqual(BnTier.Free, BnTierInfo.Current);
        }

        private static void AssertOnlyFreeRuntimeReferences(string[] references)
        {
            foreach (var reference in references)
            {
                if (reference.StartsWith("SnivelerCode.BurstNav", StringComparison.Ordinal))
                {
                    Assert.AreEqual(RuntimeAssembly, reference, "Free assemblies may reference only the Free runtime.");
                }
            }
        }

        private static string PackageRoot =>
            PackageInfo.FindForAssembly(typeof(BnTierInfo).Assembly).resolvedPath;

        private static bool IsHidden(string part) => part.Length > 0 && part[0] == '.';

        private static string PackagePath(string relative) =>
            Path.Combine(PackageRoot, relative.Replace('/', Path.DirectorySeparatorChar));

        private static AsmdefModel Load(string relative)
        {
            var path = PackagePath(relative);
            Assert.IsTrue(File.Exists(path), "Missing assembly definition: " + relative);
            var model = JsonUtility.FromJson<AsmdefModel>(File.ReadAllText(path));
            Assert.IsNotNull(model, "Unreadable assembly definition: " + relative);
            return model;
        }
    }
}
