using BurstNav;
using NUnit.Framework;
using Unity.Collections;
using Unity.Mathematics;

namespace BurstNav.Tests
{
    public class BnRuntimeTests
    {
        [Test]
        [Category("Smoke")]
        public void TierInfo_ReportsTheFreeTierAtRuntime()
        {
            Assert.IsFalse(BnTierInfo.IsPro);
            Assert.AreEqual(BnTier.Free, BnTierInfo.Current);
            Assert.AreEqual("Free", BnTierInfo.Name);
        }

        [Test]
        [Category("Smoke")]
        public void NativeContainersAndMathematics_AreAvailable()
        {
            var samples = new NativeArray<float2>(8, Allocator.TempJob);
            try
            {
                for (var i = 0; i < samples.Length; i++)
                {
                    samples[i] = new float2(i, math.mad(i, 2f, 1f));
                }

                Assert.AreEqual(8, samples.Length);
                Assert.AreEqual(3f, samples[1].y, 1e-5f);
                Assert.AreEqual(16f, math.select(-1f, 16f, samples[7].x > 0f), 1e-5f);
            }
            finally
            {
                samples.Dispose();
            }
        }
    }
}
