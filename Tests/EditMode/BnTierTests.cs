using System;
using System.IO;
using System.Linq;
using System.Reflection;
using System.Text.RegularExpressions;
using BurstNav;
using NUnit.Framework;
using Unity.Collections.LowLevel.Unsafe;
using UnityEngine;

namespace BurstNav.Tests
{
    public class BnTierTests
    {
        private const string SourceRelativePath = "BurstNav/Free/Runtime/Core/BnTier.cs";

        [Test]
        public void TierInfo_ReportsTheFreeTier()
        {
            Assert.AreEqual(BnTier.Free, BnTierInfo.Current);
            Assert.AreEqual(false, BnTierInfo.IsPro);
            Assert.AreEqual("Free", BnTierInfo.Name);
        }

        [Test]
        public void TierInfo_ExposesExactlyTheContractMembers()
        {
            var fields = typeof(BnTierInfo)
                .GetFields(BindingFlags.Public | BindingFlags.Static)
                .Select(field => field.Name)
                .OrderBy(name => name, StringComparer.Ordinal)
                .ToArray();

            CollectionAssert.AreEqual(new[] { "Current", "IsPro", "Name" }, fields);
        }

        [Test]
        public void TierInfo_MembersAreCompileTimeConstants()
        {
            foreach (var field in typeof(BnTierInfo).GetFields(BindingFlags.Public | BindingFlags.Static))
            {
                Assert.IsTrue(field.IsLiteral, field.Name + " must stay a const.");
            }
        }

        [Test]
        public void TierInfo_CurrentMatchesTheEnumName()
        {
            Assert.AreEqual(BnTierInfo.IsPro ? BnTier.Pro : BnTier.Free, BnTierInfo.Current);
            Assert.AreEqual(BnTierInfo.IsPro ? "Pro" : "Free", BnTierInfo.Name);
        }

        [Test]
        public void TierEnum_IsBlittableByte()
        {
            Assert.AreEqual(0, (int)BnTier.Free);
            Assert.AreEqual(1, (int)BnTier.Pro);
            Assert.AreEqual(typeof(byte), Enum.GetUnderlyingType(typeof(BnTier)));
            Assert.AreEqual(1, UnsafeUtility.SizeOf<BnTier>());
        }

        [Test]
        public void Source_IsUnconditionalAndFreeOfTierDirectives()
        {
            var source = File.ReadAllText(AssetPath(SourceRelativePath));

            Assert.IsFalse(Regex.IsMatch(source, @"^\s*#\s*(?:if|elif|else|endif)\b", RegexOptions.Multiline));
            StringAssert.Contains("Current = BnTier.Free", source);
            StringAssert.Contains("IsPro = false", source);
            StringAssert.Contains("Name = \"Free\"", source);
        }

        private static string AssetPath(string relative) =>
            Path.Combine(Application.dataPath, relative.Replace('/', Path.DirectorySeparatorChar));
    }
}
