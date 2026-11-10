using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;

namespace BurstNav.Samples.StressTest
{
    public static class BnStressField
    {
        public const int WallPeriod = 7;

        public const int RowPeriod = 5;

        public const int SpawnWidth = 6;

        public const int GoalWidth = 6;

        public static bool IsBlocked(int x, int y, int cellsX, int cellsY)
        {
            if ((x < SpawnWidth + 1) | (x >= cellsX - GoalWidth - 1))
            {
                return false;
            }

            var shift = ((y / RowPeriod) & 1) * 2;
            var localX = (x + shift) % WallPeriod;
            var localY = y % RowPeriod;
            return (localX >= 2) & (localX <= 4) & (localY >= 1) & (localY <= 2);
        }

        public static BlobAssetReference<BnMeshGraphBlob> Build(int cellsX, int cellsY, float cellSize, float2 origin)
        {
            var xCount = math.max(cellsX, 1);
            var yCount = math.max(cellsY, 1);
            var size = math.max(cellSize, 1e-3f);
            var cellCount = xCount * yCount;

            var blocked = new NativeArray<byte>(cellCount, Allocator.Temp);
            var indexOf = new NativeArray<int>(cellCount, Allocator.Temp);
            var polygonCount = 0;
            var linkCount = 0;

            try
            {
                for (var y = 0; y < yCount; y++)
                {
                    for (var x = 0; x < xCount; x++)
                    {
                        var cell = y * xCount + x;
                        var isBlocked = IsBlocked(x, y, xCount, yCount);
                        blocked[cell] = (byte)math.select(0, 1, isBlocked);
                        indexOf[cell] = math.select(-1, polygonCount, !isBlocked);
                        polygonCount += math.select(0, 1, !isBlocked);
                    }
                }

                for (var y = 0; y < yCount; y++)
                {
                    for (var x = 0; x < xCount; x++)
                    {
                        if (blocked[y * xCount + x] != 0)
                        {
                            continue;
                        }

                        if ((x + 1 < xCount) && (blocked[y * xCount + x + 1] == 0))
                        {
                            linkCount++;
                        }

                        if ((x > 0) && (blocked[y * xCount + x - 1] == 0))
                        {
                            linkCount++;
                        }

                        if ((y + 1 < yCount) && (blocked[(y + 1) * xCount + x] == 0))
                        {
                            linkCount++;
                        }

                        if ((y > 0) && (blocked[(y - 1) * xCount + x] == 0))
                        {
                            linkCount++;
                        }
                    }
                }

                var builder = new BlobBuilder(Allocator.Temp);
                try
                {
                    ref var root = ref builder.ConstructRoot<BnMeshGraphBlob>();
                    var vertices = builder.Allocate(ref root.Vertices, polygonCount * 4);
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
                            var cell = y * xCount + x;
                            if (blocked[cell] != 0)
                            {
                                continue;
                            }

                            var polygon = indexOf[cell];
                            var vertex = polygon * 4;
                            var corner = origin + new float2(x, y) * size;

                            vertices[vertex + 0] = corner;
                            vertices[vertex + 1] = corner + new float2(size, 0f);
                            vertices[vertex + 2] = corner + new float2(size, size);
                            vertices[vertex + 3] = corner + new float2(0f, size);

                            var neighborStart = neighborIndex;
                            var linkStart = linkIndex;

                            if ((x + 1 < xCount) && (blocked[cell + 1] == 0))
                            {
                                neighbors[neighborIndex++] = indexOf[cell + 1];
                                links[linkIndex++] = Portal(new float2(size, 0f), new float2(size, size), indexOf[cell + 1], corner, size);
                            }

                            if ((x > 0) && (blocked[cell - 1] == 0))
                            {
                                neighbors[neighborIndex++] = indexOf[cell - 1];
                                links[linkIndex++] = Portal(new float2(0f, size), new float2(0f, 0f), indexOf[cell - 1], corner, size);
                            }

                            if ((y + 1 < yCount) && (blocked[cell + xCount] == 0))
                            {
                                neighbors[neighborIndex++] = indexOf[cell + xCount];
                                links[linkIndex++] = Portal(new float2(0f, size), new float2(size, size), indexOf[cell + xCount], corner, size);
                            }

                            if ((y > 0) && (blocked[cell - xCount] == 0))
                            {
                                neighbors[neighborIndex++] = indexOf[cell - xCount];
                                links[linkIndex++] = Portal(new float2(size, 0f), new float2(0f, 0f), indexOf[cell - xCount], corner, size);
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
            finally
            {
                blocked.Dispose();
                indexOf.Dispose();
            }
        }

        public static float3 Center(int x, int y, float cellSize, float2 origin) =>
            new float3(origin.x + (x + 0.5f) * cellSize, origin.y + (y + 0.5f) * cellSize, 0f);

        private static BnLinkData Portal(float2 left, float2 right, int target, float2 corner, float size) => new BnLinkData
        {
            PortalLeft = corner + left,
            PortalRight = corner + right,
            Target = target,
            Cost = size,
        };
    }
}
