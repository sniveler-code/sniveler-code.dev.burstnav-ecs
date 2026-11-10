using System;
using System.Runtime.InteropServices;
using Unity.Burst;
using Unity.Collections;
using Unity.Jobs;
using Unity.Mathematics;

namespace BurstNav
{
    public enum BnPathSearchStatus : byte
    {
        None = 0,

        Found = 1,

        Unreachable = 2,

        BudgetExhausted = 3,

        InvalidInput = 4,

        CapacityExceeded = 5,
    }

    public struct BnPathScratchpad : IDisposable
    {
        public NativeArray<float> GScore;

        public NativeArray<int> CameFrom;

        public NativeArray<int> HeapPolygons;

        public NativeArray<float> HeapKeys;

        public NativeArray<int> HeapSlots;

        public NativeArray<byte> Closed;

        public NativeArray<int> Path;

        public NativeArray<uint> VisitStamp;

        public int Capacity;

        public int HeapCount;

        public int PathCount;

        public int Iterations;

        public uint SearchId;

        public float PathCost;

        public float HeuristicScale;

        public BnPathSearchStatus Status;

        public const float DefaultHeuristicScale = 1f;

        public bool IsCreated => GScore.IsCreated;

        public int PathLength => PathCount;

        public static BnPathScratchpad Create(int polygonCapacity, Allocator allocator)
        {
            var scratch = default(BnPathScratchpad);
            scratch.Allocate(math.max(polygonCapacity, 1), allocator);
            return scratch;
        }

        public void EnsureCapacity(int polygonCount, Allocator allocator)
        {
            var required = math.max(polygonCount, 1);
            if (GScore.IsCreated && Capacity >= required)
            {
                return;
            }

            var heuristicScale = GScore.IsCreated ? HeuristicScale : DefaultHeuristicScale;
            Dispose();
            Allocate(required, allocator);
            HeuristicScale = heuristicScale;
        }

        public int GetPathPolygon(int index) => Path[index];

        public int CopyPathTo(NativeArray<int> destination)
        {
            var count = math.min(PathCount, destination.Length);
            for (var index = 0; index < count; index++)
            {
                destination[index] = Path[index];
            }

            return count;
        }

        public void Dispose(JobHandle handle)
        {
            if (GScore.IsCreated)
            {
                GScore.Dispose(handle);
            }

            if (CameFrom.IsCreated)
            {
                CameFrom.Dispose(handle);
            }

            if (HeapPolygons.IsCreated)
            {
                HeapPolygons.Dispose(handle);
            }

            if (HeapKeys.IsCreated)
            {
                HeapKeys.Dispose(handle);
            }

            if (HeapSlots.IsCreated)
            {
                HeapSlots.Dispose(handle);
            }

            if (Closed.IsCreated)
            {
                Closed.Dispose(handle);
            }

            if (Path.IsCreated)
            {
                Path.Dispose(handle);
            }

            if (VisitStamp.IsCreated)
            {
                VisitStamp.Dispose(handle);
            }

            Capacity = 0;
            HeapCount = 0;
            PathCount = 0;
            Iterations = 0;
            SearchId = 0;
            PathCost = 0f;
            Status = BnPathSearchStatus.None;
        }

        public void Dispose()
        {
            if (GScore.IsCreated)
            {
                GScore.Dispose();
            }

            if (CameFrom.IsCreated)
            {
                CameFrom.Dispose();
            }

            if (HeapPolygons.IsCreated)
            {
                HeapPolygons.Dispose();
            }

            if (HeapKeys.IsCreated)
            {
                HeapKeys.Dispose();
            }

            if (HeapSlots.IsCreated)
            {
                HeapSlots.Dispose();
            }

            if (Closed.IsCreated)
            {
                Closed.Dispose();
            }

            if (Path.IsCreated)
            {
                Path.Dispose();
            }

            if (VisitStamp.IsCreated)
            {
                VisitStamp.Dispose();
            }

            Capacity = 0;
            HeapCount = 0;
            PathCount = 0;
            Iterations = 0;
            SearchId = 0;
            PathCost = 0f;
            Status = BnPathSearchStatus.None;
        }

        private void Allocate(int capacity, Allocator allocator)
        {
            GScore = new NativeArray<float>(capacity, allocator, NativeArrayOptions.UninitializedMemory);
            CameFrom = new NativeArray<int>(capacity, allocator, NativeArrayOptions.UninitializedMemory);
            HeapPolygons = new NativeArray<int>(capacity + 1, allocator, NativeArrayOptions.UninitializedMemory);
            HeapKeys = new NativeArray<float>(capacity + 1, allocator, NativeArrayOptions.UninitializedMemory);
            HeapSlots = new NativeArray<int>(capacity, allocator, NativeArrayOptions.UninitializedMemory);
            Closed = new NativeArray<byte>(capacity, allocator, NativeArrayOptions.UninitializedMemory);
            Path = new NativeArray<int>(capacity, allocator, NativeArrayOptions.UninitializedMemory);
            VisitStamp = new NativeArray<uint>(capacity + 1, allocator, NativeArrayOptions.ClearMemory);

            Capacity = capacity;
            HeapCount = 0;
            PathCount = 0;
            Iterations = 0;
            SearchId = 0;
            PathCost = 0f;
            HeuristicScale = DefaultHeuristicScale;
            Status = BnPathSearchStatus.None;
        }
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct BnLinkFilter
    {
        public ushort BlockedTagMask;

        public ushort IgnoreTagged;

        public ushort AllowBlockedTags;

        public static BnLinkFilter PassThrough => default;

        public bool Allows(ushort tagBits)
        {
            var tagged = tagBits != 0;
            var blocked = (tagBits & BlockedTagMask) != 0;
            return (!tagged | (IgnoreTagged == 0)) & (!blocked | (AllowBlockedTags != 0));
        }
    }

    [BurstCompile]
    public static class BnAStar
    {
        [BurstCompile]
        public static bool FindPath(
            ref BnMeshGraphBlob graph,
            int startPoly,
            int targetPoly,
            ref BnPathScratchpad scratch,
            int maxIterations)
        {
            return FindPathCore(
                ref graph,
                startPoly,
                targetPoly,
                ref scratch,
                BnLinkFilter.PassThrough,
                default(NativeArray<ushort>),
                maxIterations);
        }

        [BurstCompile]
        public static bool FindPath(
            ref BnMeshGraphBlob graph,
            int startPoly,
            int targetPoly,
            ref BnPathScratchpad scratch,
            in BnLinkFilter filter,
            int maxIterations)
        {
            return FindPathCore(
                ref graph,
                startPoly,
                targetPoly,
                ref scratch,
                filter,
                default(NativeArray<ushort>),
                maxIterations);
        }

        [BurstCompile]
        public static bool FindPath(
            ref BnMeshGraphBlob graph,
            int startPoly,
            int targetPoly,
            ref BnPathScratchpad scratch,
            in BnLinkFilter filter,
            in NativeArray<ushort> linkTagMasks,
            int maxIterations)
        {
            return FindPathCore(
                ref graph, startPoly, targetPoly, ref scratch, filter, linkTagMasks, maxIterations);
        }

        private static bool FindPathCore(
            ref BnMeshGraphBlob graph,
            int startPoly,
            int targetPoly,
            ref BnPathScratchpad scratch,
            BnLinkFilter filter,
            NativeArray<ushort> linkTagMasks,
            int maxIterations)
        {
            var polygonCount = graph.PolygonCount;
            var nodesInRange =
                (startPoly >= 0) & (startPoly < polygonCount) &
                (targetPoly >= 0) & (targetPoly < polygonCount);

            scratch.PathCount = 0;
            scratch.Iterations = 0;
            scratch.PathCost = 0f;

            if (!((polygonCount > 0) & nodesInRange))
            {
                scratch.Status = BnPathSearchStatus.InvalidInput;
                return false;
            }

            if (scratch.Capacity < polygonCount)
            {
                scratch.Status = BnPathSearchStatus.CapacityExceeded;
                return false;
            }

            ClearSearchState(ref scratch);

            if (startPoly == targetPoly)
            {
                scratch.Path[0] = startPoly;
                scratch.PathCount = 1;
                scratch.PathCost = 0f;
                scratch.Status = BnPathSearchStatus.Found;
                return true;
            }

            var targetCenter = graph.Polygons[targetPoly].Center;
            scratch.VisitStamp[startPoly] = scratch.SearchId;
            scratch.Closed[startPoly] = 0;
            scratch.GScore[startPoly] = 0f;
            scratch.CameFrom[startPoly] = -1;
            scratch.HeapPolygons[0] = startPoly;
            scratch.HeapKeys[0] = Heuristic(graph.Polygons[startPoly].Center, targetCenter, scratch.HeuristicScale);
            scratch.HeapSlots[startPoly] = 0;
            scratch.HeapCount = 1;

            var found = false;
            var drained = false;
            var iterations = 0;

            for (var iteration = 0; iteration < maxIterations; iteration++)
            {
                if (scratch.HeapCount <= 0)
                {
                    drained = true;
                    break;
                }

                var current = PopMin(ref scratch);
                iterations = iteration + 1;
                scratch.Closed[current] = 1;

                if (current == targetPoly)
                {
                    found = true;
                    break;
                }

                RelaxNeighbours(ref graph, ref scratch, current, targetCenter, filter, linkTagMasks);
            }

            scratch.Iterations = iterations;

            if (found)
            {
                BuildPath(ref scratch, startPoly, targetPoly);
                scratch.PathCost = scratch.GScore[targetPoly];
                scratch.Status = BnPathSearchStatus.Found;
                return true;
            }

            scratch.PathCount = 0;
            scratch.PathCost = 0f;
            scratch.Status = drained ? BnPathSearchStatus.Unreachable : BnPathSearchStatus.BudgetExhausted;
            return false;
        }

        private static ushort TagMaskAt(NativeArray<ushort> linkTagMasks, int linkIndex)
        {
            var count = linkTagMasks.Length;
            if ((uint)linkIndex >= (uint)count)
            {
                return 0;
            }

            return linkTagMasks[linkIndex];
        }

        private static float Heuristic(float2 from, float2 to, float scale) =>
            scale * math.distance(from, to);

        private static void ClearSearchState(ref BnPathScratchpad scratch)
        {
            var stamp = scratch.VisitStamp;
            var counterIndex = stamp.Length - 1;
            var next = stamp[counterIndex] + 1u;
            var wrapping = next == 0u;

            if (wrapping)
            {
                for (var polygon = 0; polygon < counterIndex; polygon++)
                {
                    stamp[polygon] = 0;
                }

                next = 1u;
            }

            stamp[counterIndex] = next;
            scratch.SearchId = next;
            scratch.HeapCount = 0;
            scratch.PathCount = 0;
            scratch.Iterations = 0;
            scratch.PathCost = 0f;
            scratch.Status = BnPathSearchStatus.None;
        }

        private static int PopMin(ref BnPathScratchpad scratch)
        {
            var heapPolygons = scratch.HeapPolygons;
            var heapKeys = scratch.HeapKeys;
            var heapSlots = scratch.HeapSlots;

            var top = heapPolygons[0];
            var last = scratch.HeapCount - 1;
            var movedPoly = heapPolygons[last];
            var movedKey = heapKeys[last];

            heapPolygons[0] = movedPoly;
            heapKeys[0] = movedKey;
            heapSlots[movedPoly] = 0;
            scratch.HeapCount = last;
            SiftDown(ref scratch, 0, last);
            heapSlots[top] = -1;
            return top;
        }

        private static void RelaxNeighbours(
            ref BnMeshGraphBlob graph,
            ref BnPathScratchpad scratch,
            int current,
            float2 targetCenter,
            BnLinkFilter filter,
            NativeArray<ushort> linkTagMasks)
        {
            ref var polygons = ref graph.Polygons;
            ref var links = ref graph.Links;
            ref var linkOffsets = ref graph.PolyLinkOffsets;
            ref var linkCounts = ref graph.PolyLinkCounts;

            var gScore = scratch.GScore;
            var cameFrom = scratch.CameFrom;
            var closed = scratch.Closed;
            var heapPolygons = scratch.HeapPolygons;
            var heapKeys = scratch.HeapKeys;
            var heapSlots = scratch.HeapSlots;
            var visitStamp = scratch.VisitStamp;
            var searchId = scratch.SearchId;
            var heuristicScale = scratch.HeuristicScale;

            var currentCenter = polygons[current].Center;
            var currentCost = gScore[current];
            var linkBase = linkOffsets[current];
            var linkCount = linkCounts[current];
            var heapCount = scratch.HeapCount;

            for (var index = 0; index < linkCount; index++)
            {
                var linkIndex = linkBase + index;
                var link = links[linkIndex];
                var next = link.Target;
                var nextCenter = polygons[next].Center;

                var geometry = math.distance(currentCenter, nextCenter);
                var authored = link.Cost;
                var stepCost = math.select(geometry, authored, authored > 0f);
                var candidate = currentCost + stepCost;

                var visited = visitStamp[next] == searchId;
                var known = math.select(float.PositiveInfinity, gScore[next], visited);
                var tagMask = TagMaskAt(linkTagMasks, linkIndex);
                var traversable = filter.Allows(tagMask);
                var reexpanded = visited & (closed[next] != 0);
                var accept = traversable & (candidate < known) & !reexpanded;

                var updated = math.select(known, candidate, accept);
                gScore[next] = updated;
                cameFrom[next] = math.select(cameFrom[next], current, accept);
                visitStamp[next] = math.select(visitStamp[next], searchId, accept);
                closed[next] = (byte)math.select((int)closed[next], 0, accept);

                var key = math.mad(heuristicScale, math.distance(nextCenter, targetCenter), updated);

                var slot = math.select(-1, heapSlots[next], visited);
                var inHeap = slot >= 0;
                var insertSlot = heapCount;
                var push = accept & !inHeap;
                var existingSlot = math.max(slot, 0);

                heapPolygons[insertSlot] = math.select(heapPolygons[insertSlot], next, push);
                heapKeys[insertSlot] = math.select(heapKeys[insertSlot], key, push);
                heapSlots[next] = math.select(slot, insertSlot, push);
                heapCount += math.select(0, 1, push);

                var update = accept & inHeap;
                heapKeys[existingSlot] = math.select(heapKeys[existingSlot], key, update);

                var siftFrom = math.select(-1, math.select(existingSlot, insertSlot, push), accept);
                SiftUp(ref scratch, siftFrom, heapCount);
            }

            scratch.HeapCount = heapCount;
        }

        private static int SiftUp(ref BnPathScratchpad scratch, int slot, int count)
        {
            var heapPolygons = scratch.HeapPolygons;
            var heapKeys = scratch.HeapKeys;
            var heapSlots = scratch.HeapSlots;

            var position = slot;
            var running = (position > 0) & (position < count);

            while (running)
            {
                var parent = (position - 1) >> 1;
                var keyHere = heapKeys[position];
                var keyParent = heapKeys[parent];
                var swap = keyHere < keyParent;

                var polyHere = heapPolygons[position];
                var polyParent = heapPolygons[parent];

                heapPolygons[position] = math.select(polyHere, polyParent, swap);
                heapPolygons[parent] = math.select(polyParent, polyHere, swap);
                heapKeys[position] = math.select(keyHere, keyParent, swap);
                heapKeys[parent] = math.select(keyParent, keyHere, swap);
                heapSlots[polyHere] = math.select(position, parent, swap);
                heapSlots[polyParent] = math.select(parent, position, swap);

                position = math.select(position, parent, swap);
                running = swap & (position > 0);
            }

            return position;
        }

        private static int SiftDown(ref BnPathScratchpad scratch, int slot, int count)
        {
            var heapPolygons = scratch.HeapPolygons;
            var heapKeys = scratch.HeapKeys;
            var heapSlots = scratch.HeapSlots;

            var position = slot;
            var running = (position >= 0) & (position < count);

            while (running)
            {
                var left = position * 2 + 1;
                var right = left + 1;
                var last = count - 1;
                var keyHere = heapKeys[position];

                var leftValid = left <= last;
                var rightValid = right <= last;
                var leftKey = heapKeys[math.min(left, last)];
                var rightKey = heapKeys[math.min(right, last)];

                var takeLeft = leftValid & (leftKey < keyHere);
                var bestKey = math.select(keyHere, leftKey, takeLeft);
                var best = math.select(position, left, takeLeft);

                var takeRight = rightValid & (rightKey < bestKey);
                bestKey = math.select(bestKey, rightKey, takeRight);
                best = math.select(best, right, takeRight);

                var swap = best != position;

                var polyHere = heapPolygons[position];
                var polyBest = heapPolygons[best];

                heapPolygons[position] = math.select(polyHere, polyBest, swap);
                heapPolygons[best] = math.select(polyBest, polyHere, swap);
                heapKeys[position] = math.select(keyHere, bestKey, swap);
                heapKeys[best] = math.select(bestKey, keyHere, swap);
                heapSlots[polyHere] = math.select(position, best, swap);
                heapSlots[polyBest] = math.select(best, position, swap);

                position = math.select(position, best, swap);
                running = swap & (position < count);
            }

            return position;
        }

        private static void BuildPath(ref BnPathScratchpad scratch, int startPoly, int targetPoly)
        {
            var path = scratch.Path;
            var cameFrom = scratch.CameFrom;
            var capacity = scratch.Capacity;

            var length = 0;
            var polygon = targetPoly;

            while ((polygon >= 0) & (length < capacity))
            {
                path[length] = polygon;
                length++;
                var reachedStart = polygon == startPoly;
                polygon = math.select(cameFrom[polygon], -1, reachedStart);
            }

            var half = length >> 1;
            for (var index = 0; index < half; index++)
            {
                var mirrored = length - 1 - index;
                var head = path[index];
                path[index] = path[mirrored];
                path[mirrored] = head;
            }

            scratch.PathCount = length;
        }
    }
}
