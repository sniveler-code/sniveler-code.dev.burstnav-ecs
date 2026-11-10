using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;

namespace BurstNav.Samples.StressTest
{
    public static class BnStressField
    {
        public const int SpawnWidth = 8;

        public const int GoalWidth = 8;

        public const int ObstacleSeed = 20251110;

        public const int TargetBlockedPercent = 12;

        public const int MaxStraightRun = 10;

        public static void FillMask(NativeArray<byte> mask, int cellsX, int cellsY, int seed)
        {
            var xCount = math.max(cellsX, 1);
            var yCount = math.max(cellsY, 1);
            var count = math.min(mask.Length, xCount * yCount);

            for (var index = 0; index < count; index++)
            {
                mask[index] = 0;
            }

            if ((xCount <= SpawnWidth + GoalWidth + 8) | (yCount < 6))
            {
                return;
            }

            var seen = new NativeArray<byte>(count, Allocator.Temp);
            var queue = new NativeArray<int>(count, Allocator.Temp);

            try
            {
                var state = (uint)seed | 1u;
                var target = count * TargetBlockedPercent / 100;
                var blocked = 0;
                var attempts = 0;

                while ((blocked < target) & (attempts < 4000))
                {
                    attempts++;
                    var width = 2 + (int)(Next(ref state) % 2u);
                    var height = 1 + (int)(Next(ref state) % 2u);
                    var maxX = xCount - GoalWidth - width;

                    if (maxX <= SpawnWidth)
                    {
                        continue;
                    }

                    var originX = SpawnWidth + (int)(Next(ref state) % (uint)(maxX - SpawnWidth));
                    var originY = (int)(Next(ref state) % (uint)(yCount - height + 1));

                    var free = true;
                    for (var y = originY; y < originY + height; y++)
                    {
                        for (var x = originX; x < originX + width; x++)
                        {
                            free &= mask[y * xCount + x] == 0;
                        }
                    }

                    if (!free)
                    {
                        continue;
                    }

                    for (var y = originY; y < originY + height; y++)
                    {
                        for (var x = originX; x < originX + width; x++)
                        {
                            mask[y * xCount + x] = 1;
                        }
                    }

                    if (Connected(mask, seen, queue, xCount, yCount))
                    {
                        blocked += width * height;
                        continue;
                    }

                    for (var y = originY; y < originY + height; y++)
                    {
                        for (var x = originX; x < originX + width; x++)
                        {
                            mask[y * xCount + x] = 0;
                        }
                    }
                }

                var placements = 0;
                while (placements < 256)
                {
                    var bestRun = 0;
                    var bestRow = -1;
                    var bestColumn = -1;

                    for (var y = 0; y < yCount; y++)
                    {
                        var x = SpawnWidth;
                        while (x < xCount - GoalWidth)
                        {
                            if (mask[y * xCount + x] != 0)
                            {
                                x++;
                                continue;
                            }

                            var start = x;
                            while ((x < xCount - GoalWidth) && (mask[y * xCount + x] == 0))
                            {
                                x++;
                            }

                            var run = x - start;
                            if ((run > MaxStraightRun) & (run > bestRun))
                            {
                                bestRun = run;
                                bestRow = y;
                                bestColumn = (start + x) / 2;
                            }
                        }
                    }

                    if (bestRun <= MaxStraightRun)
                    {
                        break;
                    }

                    var placed = false;
                    for (var offset = 0; offset < 3; offset++)
                    {
                        var y = bestRow + math.select(0, math.select(-1, 1, offset == 2), offset > 0);
                        if ((y < 0) | (y >= yCount))
                        {
                            continue;
                        }

                        var cell = y * xCount + bestColumn;
                        if (mask[cell] != 0)
                        {
                            continue;
                        }

                        mask[cell] = 1;
                        if (Connected(mask, seen, queue, xCount, yCount))
                        {
                            blocked++;
                            placements++;
                            placed = true;
                            break;
                        }

                        mask[cell] = 0;
                    }

                    if (!placed)
                    {
                        break;
                    }
                }
            }
            finally
            {
                seen.Dispose();
                queue.Dispose();
            }
        }

        private static uint Next(ref uint state)
        {
            state = state * 1664525u + 1013904223u;
            return state >> 16;
        }

        private static bool Connected(NativeArray<byte> mask, NativeArray<byte> seen, NativeArray<int> queue, int xCount, int yCount)
        {
            var depth = yCount * xCount;
            for (var index = 0; index < depth; index++)
            {
                seen[index] = 0;
            }

            var head = 0;
            var tail = 0;

            for (var y = 0; y < yCount; y++)
            {
                var cell = y * xCount;
                if (mask[cell] == 0)
                {
                    seen[cell] = 1;
                    queue[tail++] = cell;
                }
            }

            while (head < tail)
            {
                var cell = queue[head++];
                var x = cell % xCount;

                if (x >= xCount - GoalWidth)
                {
                    return true;
                }

                if (((x + 1) < xCount) && (mask[cell + 1] == 0) && (seen[cell + 1] == 0))
                {
                    seen[cell + 1] = 1;
                    queue[tail++] = cell + 1;
                }

                if ((x > 0) && (mask[cell - 1] == 0) && (seen[cell - 1] == 0))
                {
                    seen[cell - 1] = 1;
                    queue[tail++] = cell - 1;
                }

                if (((cell + xCount) < depth) && (mask[cell + xCount] == 0) && (seen[cell + xCount] == 0))
                {
                    seen[cell + xCount] = 1;
                    queue[tail++] = cell + xCount;
                }

                if (((cell - xCount) >= 0) && (mask[cell - xCount] == 0) && (seen[cell - xCount] == 0))
                {
                    seen[cell - xCount] = 1;
                    queue[tail++] = cell - xCount;
                }
            }

            return false;
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
                FillMask(blocked, xCount, yCount, ObstacleSeed);

                for (var y = 0; y < yCount; y++)
                {
                    for (var x = 0; x < xCount; x++)
                    {
                        var cell = y * xCount + x;
                        indexOf[cell] = math.select(-1, polygonCount, blocked[cell] == 0);
                        polygonCount += math.select(0, 1, blocked[cell] == 0);
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
