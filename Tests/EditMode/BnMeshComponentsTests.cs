using System;
using System.IO;
using System.Linq;
using System.Reflection;
using System.Text.RegularExpressions;
using NUnit.Framework;
using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;

namespace BurstNav.Tests
{
    public class BnMeshComponentsTests
    {
        private const string SourceRelativePath = "BurstNav/Free/Runtime/Core/BnMeshComponents.cs";

        private static readonly Type[] GraphTypes =
        {
            typeof(BnLinkData),
            typeof(BnPolygon),
            typeof(BnMeshGraphBlob),
            typeof(BnBlobMeshGraph),
        };

        [Test]
        public void BnBlobMeshGraph_ExposesSingleBlobReferenceValue()
        {
            var fields = typeof(BnBlobMeshGraph).GetFields(BindingFlags.Public | BindingFlags.Instance);

            Assert.AreEqual(1, fields.Length, "BnBlobMeshGraph must hold a single field.");
            Assert.AreEqual("Value", fields[0].Name);
            Assert.AreEqual(typeof(BlobAssetReference<BnMeshGraphBlob>), fields[0].FieldType);
            Assert.IsTrue(typeof(IComponentData).IsAssignableFrom(typeof(BnBlobMeshGraph)));
            Assert.IsTrue(UnsafeUtility.IsBlittable<BnBlobMeshGraph>());
        }

        [Test]
        public void BnLinkData_HasOnlyTheSharedFieldSet()
        {
            var fields = typeof(BnLinkData).GetFields(BindingFlags.Public | BindingFlags.Instance)
                .Select(field => field.Name)
                .ToArray();

            CollectionAssert.AreEqual(new[] { "PortalLeft", "PortalRight", "Target", "Cost" }, fields);

            Assert.AreEqual(0, UnsafeUtility.GetFieldOffset(Field<BnLinkData>("PortalLeft")));
            Assert.AreEqual(8, UnsafeUtility.GetFieldOffset(Field<BnLinkData>("PortalRight")));
            Assert.AreEqual(16, UnsafeUtility.GetFieldOffset(Field<BnLinkData>("Target")));
            Assert.AreEqual(20, UnsafeUtility.GetFieldOffset(Field<BnLinkData>("Cost")));
            Assert.AreEqual(24, UnsafeUtility.SizeOf<BnLinkData>());
        }

        [Test]
        public void GraphStructs_AreBlittableWithoutManagedReferences()
        {
            foreach (var type in GraphTypes)
            {
                Assert.IsTrue(UnsafeUtility.IsBlittable(type), type.Name + " must be blittable.");
                Assert.IsTrue(type.IsValueType, type.Name + " must be a value type.");
                Assert.IsFalse(type.IsAutoLayout, type.Name + " must not use auto layout.");
                Assert.IsFalse(
                    type.GetFields(BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.Instance)
                        .Any(field => !field.FieldType.IsValueType),
                    type.Name + " must not contain managed references.");
            }
        }

        [Test]
        public void GraphStructs_OrderFieldsBySizeDescending()
        {
            foreach (var type in GraphTypes)
            {
                var fields = type.GetFields(BindingFlags.Public | BindingFlags.Instance);
                for (var index = 1; index < fields.Length; index++)
                {
                    var previous = FieldSize(fields[index - 1].FieldType);
                    var current = FieldSize(fields[index].FieldType);
                    Assert.GreaterOrEqual(
                        previous,
                        current,
                        $"{type.Name}.{fields[index].Name} ({current} B) is larger than " +
                        $"{fields[index - 1].Name} ({previous} B).");
                }
            }
        }

        [Test]
        public void BnMeshGraphBlob_CarriesOnlyTheSharedMembers()
        {
            var fields = typeof(BnMeshGraphBlob).GetFields(BindingFlags.Public | BindingFlags.Instance)
                .Select(field => field.Name)
                .ToArray();

            CollectionAssert.AreEqual(
                new[]
                {
                    "Vertices",
                    "Polygons",
                    "NeighborPolygons",
                    "Links",
                    "PolyLinkOffsets",
                    "PolyLinkCounts",
                },
                fields);

            foreach (var proField in new[]
                     {
                         "HpaConfig", "PolyClusterID", "ClusterLinkOffsetsAndCounts",
                         "ClusterLinks", "ClusterAABBs", "ClusterCount",
                     })
            {
                CollectionAssert.DoesNotContain(fields, proField);
            }
        }

        [Test]
        public void BlobGraph_ResolvesTopologyThroughOffsetsAndCounts()
        {
            using var graph = BuildTwoPolygonGraph();

            ref var blob = ref graph.Value;
            Assert.AreEqual(2, blob.PolygonCount);
            Assert.AreEqual(2, blob.LinkCount);
            Assert.AreEqual(6, blob.VertexCount);

            Assert.AreEqual(0, blob.GetPolyLinkOffset(0));
            Assert.AreEqual(1, blob.GetPolyLinkCount(0));
            Assert.AreEqual(1, blob.GetPolyLinkOffset(1));
            Assert.AreEqual(1, blob.GetPolyLinkCount(1));

            var forward = blob.GetPolyLink(0, 0);
            Assert.AreEqual(1, forward.Target);
            Assert.IsTrue(math.all(forward.PortalLeft == new float2(1f, 0f)));
            Assert.IsTrue(math.all(forward.PortalRight == new float2(1f, 1f)));
            Assert.AreEqual(1f, forward.Cost, 1e-5f);

            var backward = blob.GetPolyLink(1, 0);
            Assert.AreEqual(0, backward.Target);
            Assert.IsTrue(math.all(backward.PortalLeft == new float2(1f, 1f)));
            Assert.IsTrue(math.all(backward.PortalRight == new float2(1f, 0f)));

            Assert.AreEqual(1, blob.GetNeighborCount(0));
            Assert.AreEqual(1, blob.GetNeighbor(0, 0));
            Assert.AreEqual(0, blob.GetNeighbor(1, 0));

            Assert.AreEqual(4, blob.GetVertexCount(0));
            Assert.IsTrue(math.all(blob.GetVertex(0, 0) == float2.zero));
            Assert.IsTrue(math.all(blob.GetPolygonCenter(1) == new float2(1.5f, 0.5f)));
        }

        [Test]
        public void BlobGraph_ReadsTopologyFromBurstCompiledJob()
        {
            using var graph = BuildTwoPolygonGraph();
            using var result = new NativeArray<int>(4, Allocator.TempJob);

            var handle = new GraphProbeJob { Graph = graph, Result = result }.Schedule();
            handle.Complete();

            Assert.AreEqual(2, result[0], "link count");
            Assert.AreEqual(2, result[1], "neighbour count");
            Assert.AreEqual(1, result[2], "target of the first portal");
            Assert.AreEqual(1, result[3], "portal width, truncated");
        }

        [Test]
        public void Source_HasNoTierConditionalsAndNoProMembers()
        {
            var source = File.ReadAllText(AssetPath(SourceRelativePath));

            Assert.IsFalse(Regex.IsMatch(source, @"^[ \t]*#(?:if|elif|else|endif)", RegexOptions.Multiline));

            foreach (var shared in new[] { "BnLinkData", "PortalLeft", "PortalRight", "Target", "Cost", "BnBlobMeshGraph" })
            {
                StringAssert.Contains(shared, source);
            }

            foreach (var proField in new[]
                     {
                         "PolyClusterID", "ClusterCount", "ClusterLinks", "ClusterAABBs", "HpaConfig", "TagMask",
                     })
            {
                Assert.IsFalse(source.Contains(proField), "Free source must not declare " + proField + ".");
            }

            Assert.AreEqual(
                source.Count(c => c == '{'),
                source.Count(c => c == '}'),
                "Source must keep balanced braces.");
        }

        private static BlobAssetReference<BnMeshGraphBlob> BuildTwoPolygonGraph()
        {
            var builder = new BlobBuilder(Allocator.Temp);
            try
            {
                ref var root = ref builder.ConstructRoot<BnMeshGraphBlob>();

                var vertices = builder.Allocate(ref root.Vertices, 6);
                vertices[0] = new float2(0f, 0f);
                vertices[1] = new float2(1f, 0f);
                vertices[2] = new float2(1f, 1f);
                vertices[3] = new float2(0f, 1f);
                vertices[4] = new float2(2f, 0f);
                vertices[5] = new float2(2f, 1f);

                var polygons = builder.Allocate(ref root.Polygons, 2);
                polygons[0] = new BnPolygon
                {
                    Center = new float2(0.5f, 0.5f),
                    VertexOffset = 0,
                    VertexCount = 4,
                    NeighborOffset = 0,
                    NeighborCount = 1,
                };
                polygons[1] = new BnPolygon
                {
                    Center = new float2(1.5f, 0.5f),
                    VertexOffset = 1,
                    VertexCount = 3,
                    NeighborOffset = 1,
                    NeighborCount = 1,
                };

                var neighbors = builder.Allocate(ref root.NeighborPolygons, 2);
                neighbors[0] = 1;
                neighbors[1] = 0;

                var links = builder.Allocate(ref root.Links, 2);
                links[0] = new BnLinkData
                {
                    PortalLeft = new float2(1f, 0f),
                    PortalRight = new float2(1f, 1f),
                    Target = 1,
                    Cost = 1f,
                };
                links[1] = new BnLinkData
                {
                    PortalLeft = new float2(1f, 1f),
                    PortalRight = new float2(1f, 0f),
                    Target = 0,
                    Cost = 1f,
                };

                var linkOffsets = builder.Allocate(ref root.PolyLinkOffsets, 2);
                linkOffsets[0] = 0;
                linkOffsets[1] = 1;

                var linkCounts = builder.Allocate(ref root.PolyLinkCounts, 2);
                linkCounts[0] = 1;
                linkCounts[1] = 1;

                return builder.CreateBlobAssetReference<BnMeshGraphBlob>(Allocator.Persistent);
            }
            finally
            {
                builder.Dispose();
            }
        }

        private static FieldInfo Field<T>(string name)
        {
            var field = typeof(T).GetField(name, BindingFlags.Public | BindingFlags.Instance);
            Assert.IsNotNull(field, typeof(T).Name + " has no public field " + name + ".");
            return field;
        }

        private static int FieldSize(Type type) => UnsafeUtility.SizeOf(type);

        private static string AssetPath(string relative) =>
            Path.Combine(Application.dataPath, relative.Replace('/', Path.DirectorySeparatorChar));

        [BurstCompile(CompileSynchronously = true)]
        private struct GraphProbeJob : IJob
        {
            public BlobAssetReference<BnMeshGraphBlob> Graph;
            public NativeArray<int> Result;

            public void Execute()
            {
                ref var graph = ref Graph.Value;
                var links = 0;
                var neighbors = 0;
                for (var polygon = 0; polygon < graph.PolygonCount; polygon++)
                {
                    links += graph.GetPolyLinkCount(polygon);
                    neighbors += graph.GetNeighborCount(polygon);
                }

                var portal = graph.GetPolyLink(0, 0);
                Result[0] = links;
                Result[1] = neighbors;
                Result[2] = portal.Target;
                Result[3] = (int)math.distance(portal.PortalLeft, portal.PortalRight);
            }
        }
    }
}
