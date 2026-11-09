using NUnit.Framework;
using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;

namespace BurstNav.Tests
{
    [Category("Smoke")]
    public class BnAStarPlayModeTests
    {
        private const int RowCount = 4;

        [Test]
        public void FindPath_ReturnsOptimalPath_AtRuntime()
        {
            using var graph = BuildCorridorGraph();
            var scratch = BnPathScratchpad.Create(RowCount, Allocator.Persistent);
            try
            {
                ref var blob = ref graph.Value;
                Assert.IsTrue(BnAStar.FindPath(ref blob, 0, RowCount - 1, ref scratch, 32));
                Assert.AreEqual(BnPathSearchStatus.Found, scratch.Status);
                Assert.AreEqual(RowCount, scratch.PathCount, "The corridor has exactly one route.");
                Assert.AreEqual(RowCount - 1, scratch.PathCost, 1e-4f);
                Assert.AreEqual(0, scratch.GetPathPolygon(0));
                Assert.AreEqual(RowCount - 1, scratch.GetPathPolygon(scratch.PathCount - 1));

                var destination = new NativeArray<int>(RowCount, Allocator.Temp);
                try
                {
                    Assert.AreEqual(RowCount, scratch.CopyPathTo(destination));
                    Assert.AreEqual(RowCount - 1, destination[RowCount - 1]);
                }
                finally
                {
                    destination.Dispose();
                }
            }
            finally
            {
                scratch.Dispose();
            }
        }

        [Test]
        public void FindPath_SchedulesAndRunsInsideAWorld()
        {
            using var graph = BuildCorridorGraph();
            var scratch = BnPathScratchpad.Create(RowCount, Allocator.Persistent);
            var world = new World("BnAStarPlayMode");

            try
            {
                var result = new NativeArray<int>(3, Allocator.TempJob);
                try
                {
                    var entity = world.EntityManager.CreateEntity(typeof(BnBlobMeshGraph));
                    world.EntityManager.SetComponentData(entity, new BnBlobMeshGraph { Value = graph });

                    var owned = world.EntityManager.GetComponentData<BnBlobMeshGraph>(entity);
                    var handle = new CorridorSearchJob
                    {
                        Graph = owned.Value,
                        Scratch = scratch,
                        Target = RowCount - 1,
                        MaxIterations = 32,
                        Result = result,
                    }.Schedule();

                    handle.Complete();

                    Assert.AreEqual(1, result[0], "The scheduled search found no path.");
                    Assert.AreEqual(RowCount, result[1]);
                    Assert.AreEqual((RowCount - 1) * 1000, result[2]);
                }
                finally
                {
                    result.Dispose();
                }
            }
            finally
            {
                world.Dispose();
                scratch.Dispose();
            }
        }

        private static BlobAssetReference<BnMeshGraphBlob> BuildCorridorGraph()
        {
            var builder = new BlobBuilder(Allocator.Temp);
            try
            {
                ref var root = ref builder.ConstructRoot<BnMeshGraphBlob>();

                var vertices = builder.Allocate(ref root.Vertices, RowCount * 2);
                var polygons = builder.Allocate(ref root.Polygons, RowCount);
                var links = builder.Allocate(ref root.Links, RowCount - 1);
                var neighbors = builder.Allocate(ref root.NeighborPolygons, RowCount - 1);
                var linkOffsets = builder.Allocate(ref root.PolyLinkOffsets, RowCount);
                var linkCounts = builder.Allocate(ref root.PolyLinkCounts, RowCount);

                for (var row = 0; row < RowCount; row++)
                {
                    var center = new float2(row, 0f);
                    vertices[row * 2 + 0] = new float2(row, 0f);
                    vertices[row * 2 + 1] = new float2(row, 1f);
                    polygons[row] = new BnPolygon
                    {
                        Center = center,
                        VertexOffset = row * 2,
                        VertexCount = 2,
                        NeighborOffset = row < RowCount - 1 ? row : 0,
                        NeighborCount = row < RowCount - 1 ? 1 : 0,
                    };

                    linkOffsets[row] = row < RowCount - 1 ? row : 0;
                    linkCounts[row] = row < RowCount - 1 ? 1 : 0;

                    if (row < RowCount - 1)
                    {
                        links[row] = new BnLinkData
                        {
                            PortalLeft = new float2(row + 0.5f, 0f),
                            PortalRight = new float2(row + 0.5f, 1f),
                            Target = row + 1,
                            Cost = 1f,
                        };
                        neighbors[row] = row + 1;
                    }
                }

                return builder.CreateBlobAssetReference<BnMeshGraphBlob>(Allocator.Persistent);
            }
            finally
            {
                builder.Dispose();
            }
        }

        [BurstCompile(CompileSynchronously = true)]
        private struct CorridorSearchJob : IJob
        {
            public BlobAssetReference<BnMeshGraphBlob> Graph;
            public BnPathScratchpad Scratch;
            public int Target;
            public int MaxIterations;
            public NativeArray<int> Result;

            public void Execute()
            {
                ref var graph = ref Graph.Value;
                var scratch = Scratch;
                var found = BnAStar.FindPath(ref graph, 0, Target, ref scratch, MaxIterations);

                Result[0] = math.select(0, 1, found);
                Result[1] = scratch.PathCount;
                Result[2] = (int)math.round(scratch.PathCost * 1000f);
            }
        }
    }
}
