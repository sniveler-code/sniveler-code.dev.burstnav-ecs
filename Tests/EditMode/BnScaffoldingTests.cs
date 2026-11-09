using System;
using System.IO;
using NUnit.Framework;
using UnityEngine;

namespace BurstNav.Tests
{
    public class BnScaffoldingTests
    {
        private const string FreeRuntimeAsmdef = "BurstNav/Free/Runtime/BurstNav.Free.asmdef";
        private const string FreeEditModeAsmdef =
            "BurstNav/Free/Tests/EditMode/BurstNav.Free.Tests.EditMode.asmdef";
        private const string FreePlayModeAsmdef =
            "BurstNav/Free/Tests/PlayMode/BurstNav.Free.Tests.PlayMode.asmdef";

        private static readonly string[] RequiredDirectories =
        {
            "BurstNav/Free/Runtime/Core",
            "BurstNav/Free/Runtime/Systems",
            "BurstNav/Free/Runtime/Authoring",
            "BurstNav/Free/Tests/EditMode",
            "BurstNav/Free/Tests/PlayMode",
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
                Assert.IsTrue(Directory.Exists(AssetPath(directory)), "Missing directory: " + directory);
            }
        }

        [Test]
        public void EveryAssetAndFolder_HasAMetaFile()
        {
            var root = AssetPath("BurstNav");
            Assert.IsTrue(File.Exists(root + ".meta"), "Missing BurstNav.meta");

            foreach (var entry in Directory.GetFileSystemEntries(root, "*", SearchOption.AllDirectories))
            {
                var name = Path.GetFileName(entry);
                if (name.Length == 0 || name[0] == '.' || name.EndsWith(".meta", StringComparison.Ordinal))
                {
                    continue;
                }

                Assert.IsTrue(File.Exists(entry + ".meta"), "Missing .meta for " + entry);
            }
        }

        [Test]
        public void FreeAssembly_ReferencesDotsPackagesAndOnlyTheFreeRuntime()
        {
            var asmdef = Load(FreeRuntimeAsmdef);

            Assert.AreEqual("BurstNav.Free", asmdef.name);
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
                        ? "BurstNav.Free.Tests.EditMode"
                        : "BurstNav.Free.Tests.PlayMode",
                    asmdef.name);
                CollectionAssert.Contains(asmdef.references, "BurstNav.Free", path);
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
            Assert.AreEqual("BurstNav.Free", typeof(BnTierInfo).Assembly.GetName().Name);
            Assert.IsFalse(BnTierInfo.IsPro);
            Assert.AreEqual(BnTier.Free, BnTierInfo.Current);
        }

        private static void AssertOnlyFreeRuntimeReferences(string[] references)
        {
            foreach (var reference in references)
            {
                if (reference.StartsWith("BurstNav.", StringComparison.Ordinal))
                {
                    Assert.AreEqual("BurstNav.Free", reference, "Free assemblies may reference only the Free runtime.");
                }
            }
        }

        private static string AssetPath(string relative) =>
            Path.Combine(Application.dataPath, relative.Replace('/', Path.DirectorySeparatorChar));

        private static AsmdefModel Load(string relative)
        {
            var path = AssetPath(relative);
            Assert.IsTrue(File.Exists(path), "Missing assembly definition: " + relative);
            var model = JsonUtility.FromJson<AsmdefModel>(File.ReadAllText(path));
            Assert.IsNotNull(model, "Unreadable assembly definition: " + relative);
            return model;
        }
    }
}
