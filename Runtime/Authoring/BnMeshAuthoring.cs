using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;

namespace BurstNav
{
    [DisallowMultipleComponent]
    public sealed class BnMeshAuthoring : MonoBehaviour
    {
        [Min(1)]
        public int CellsX = 4;

        [Min(1)]
        public int CellsY = 4;

        [Min(0.01f)]
        public float CellSize = 1f;

        public Vector2 Origin = Vector2.zero;
    }

    public sealed class BnMeshAuthoringBaker : Baker<BnMeshAuthoring>
    {
        public override void Bake(BnMeshAuthoring authoring)
        {
            var entity = GetEntity(TransformUsageFlags.None);
            var origin = new float2(authoring.Origin.x, authoring.Origin.y);
            var graph = BuildGraph(authoring.CellsX, authoring.CellsY, authoring.CellSize, origin);

            AddBlobAsset(ref graph, out _);
            AddComponent(entity, new BnBlobMeshGraph { Value = graph });
        }

        public static BlobAssetReference<BnMeshGraphBlob> BuildGraph(
            int cellsX,
            int cellsY,
            float cellSize,
            float2 origin)
        {
            var xCount = math.max(cellsX, 1);
            var yCount = math.max(cellsY, 1);
            var size = math.max(cellSize, 1e-3f);

            var polygonCount = xCount * yCount;
            var vertexCount = polygonCount * 4;
            var linkCount = 2 * (math.max(xCount - 1, 0) * yCount + xCount * math.max(yCount - 1, 0));

            var builder = new BlobBuilder(Allocator.Temp);
            try
            {
                ref var root = ref builder.ConstructRoot<BnMeshGraphBlob>();

                var vertices = builder.Allocate(ref root.Vertices, vertexCount);
                var polygons = builder.Allocate(ref root.Polygons, polygonCount);
                var neighbors = builder.Allocate(ref root.NeighborPolygons, linkCount);
                var links = builder.Allocate(ref root.Links, linkCount);
                var linkOffsets = builder.Allocate(ref root.PolyLinkOffsets, polygonCount);
                var linkCounts = builder.Allocate(ref root.PolyLinkCounts, polygonCount);

                var linkIndex = 0;
                var neighborIndex = 0;

                for (var y = 0; y < yCount; y++)
                {
                    for (var x = 0; x < xCount; x++)
                    {
                        var polygon = y * xCount + x;
                        var vertex = polygon * 4;
                        var corner = origin + new float2(x, y) * size;

                        vertices[vertex + 0] = corner;
                        vertices[vertex + 1] = corner + new float2(size, 0f);
                        vertices[vertex + 2] = corner + new float2(size, size);
                        vertices[vertex + 3] = corner + new float2(0f, size);

                        var neighborStart = neighborIndex;
                        var linkStart = linkIndex;

                        if (x + 1 < xCount)
                        {
                            neighbors[neighborIndex++] = polygon + 1;
                            links[linkIndex++] = Portal(new float2(size, 0f), new float2(size, size), polygon + 1, corner, size);
                        }

                        if (x > 0)
                        {
                            neighbors[neighborIndex++] = polygon - 1;
                            links[linkIndex++] = Portal(new float2(0f, size), new float2(0f, 0f), polygon - 1, corner, size);
                        }

                        if (y + 1 < yCount)
                        {
                            neighbors[neighborIndex++] = polygon + xCount;
                            links[linkIndex++] = Portal(new float2(0f, size), new float2(size, size), polygon + xCount, corner, size);
                        }

                        if (y > 0)
                        {
                            neighbors[neighborIndex++] = polygon - xCount;
                            links[linkIndex++] = Portal(new float2(size, 0f), new float2(0f, 0f), polygon - xCount, corner, size);
                        }

                        polygons[polygon] = new BnPolygon
                        {
                            Center = corner + new float2(size * 0.5f, size * 0.5f),
                            VertexOffset = vertex,
                            VertexCount = 4,
                            NeighborOffset = neighborStart,
                            NeighborCount = neighborIndex - neighborStart,
                        };

                        linkOffsets[polygon] = linkStart;
                        linkCounts[polygon] = linkIndex - linkStart;
                    }
                }

                return builder.CreateBlobAssetReference<BnMeshGraphBlob>(Allocator.Persistent);
            }
            finally
            {
                builder.Dispose();
            }
        }

        private static BnLinkData Portal(float2 left, float2 right, int target, float2 corner, float size) => new BnLinkData
        {
            PortalLeft = corner + left,
            PortalRight = corner + right,
            Target = target,
            Cost = size,
        };
    }
}
