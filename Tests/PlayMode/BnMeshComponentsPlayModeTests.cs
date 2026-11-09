using NUnit.Framework;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;

namespace BurstNav.Tests
{
    [Category("Smoke")]
    public class BnMeshComponentsPlayModeTests
    {
        [Test]
        public void BlobGraph_AttachesToEntityAndStaysBlittable()
        {
            var graph = BuildSinglePolygonGraph();
            var world = new World("BnMeshComponentsPlayMode");
            try
            {
                var entityManager = world.EntityManager;
                var entity = entityManager.CreateEntity(typeof(BnBlobMeshGraph));
                entityManager.SetComponentData(entity, new BnBlobMeshGraph { Value = graph });

                var read = entityManager.GetComponentData<BnBlobMeshGraph>(entity);
                Assert.IsTrue(read.Value.IsCreated, "The entity lost its blob asset reference.");
                Assert.AreEqual(1, read.Value.Value.PolygonCount);
                Assert.AreEqual(4, read.Value.Value.VertexCount);
                Assert.AreEqual(1, read.Value.Value.GetPolyLinkCount(0));
                Assert.AreEqual(0, read.Value.Value.GetPolyLink(0, 0).Target);
                Assert.IsTrue(math.all(read.Value.Value.GetVertex(0, 2) == new float2(1f, 1f)));

                var chunk = entityManager.GetChunk(entity);
                Assert.AreEqual(1, chunk.Count, "The component must live in a dense chunk.");
            }
            finally
            {
                world.Dispose();
                if (graph.IsCreated)
                {
                    graph.Dispose();
                }
            }
        }

        private static BlobAssetReference<BnMeshGraphBlob> BuildSinglePolygonGraph()
        {
            var builder = new BlobBuilder(Allocator.Temp);
            try
            {
                ref var root = ref builder.ConstructRoot<BnMeshGraphBlob>();

                var vertices = builder.Allocate(ref root.Vertices, 4);
                vertices[0] = new float2(0f, 0f);
                vertices[1] = new float2(1f, 0f);
                vertices[2] = new float2(1f, 1f);
                vertices[3] = new float2(0f, 1f);

                var polygons = builder.Allocate(ref root.Polygons, 1);
                polygons[0] = new BnPolygon
                {
                    Center = new float2(0.5f, 0.5f),
                    VertexOffset = 0,
                    VertexCount = 4,
                    NeighborOffset = 0,
                    NeighborCount = 0,
                };

                builder.Allocate(ref root.NeighborPolygons, 0);

                var links = builder.Allocate(ref root.Links, 1);
                links[0] = new BnLinkData
                {
                    PortalLeft = new float2(0f, 0f),
                    PortalRight = new float2(1f, 0f),
                    Target = 0,
                    Cost = 0f,
                };

                var linkOffsets = builder.Allocate(ref root.PolyLinkOffsets, 1);
                linkOffsets[0] = 0;

                var linkCounts = builder.Allocate(ref root.PolyLinkCounts, 1);
                linkCounts[0] = 1;

                return builder.CreateBlobAssetReference<BnMeshGraphBlob>(Allocator.Persistent);
            }
            finally
            {
                builder.Dispose();
            }
        }
    }
}
