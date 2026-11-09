using Unity.Burst;
using Unity.Burst.Intrinsics;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;

namespace BurstNav
{
    [BurstCompile]
    public struct BnPathfindJob : IJobChunk
    {
        public BnPathScratchpad Scratch;

        public EntityCommandBuffer Commands;

        public BlobAssetReference<BnMeshGraphBlob> Graph;

        public ComponentTypeHandle<BnAgentPathRequest> RequestHandle;

        public ComponentTypeHandle<BnAgentPath> PathHandle;

        public BufferTypeHandle<BnPathWaypoint> WaypointHandle;

        public EntityTypeHandle EntityHandle;

        public int MaxIterations;

        [BurstCompile]
        public void Execute(
            in ArchetypeChunk chunk,
            int unfilteredChunkIndex,
            bool useEnabledMask,
            in v128 chunkEnabledMask)
        {
            ref var graph = ref Graph.Value;

            var requests = chunk.GetNativeArray(ref RequestHandle);
            var paths = chunk.GetNativeArray(ref PathHandle);
            var waypoints = chunk.GetBufferAccessor(ref WaypointHandle);
            var entities = chunk.GetNativeArray(EntityHandle);

            var enumerator = new ChunkEntityEnumerator(useEnabledMask, chunkEnabledMask, chunk.Count);

            var scratchpad = Scratch;

            while (enumerator.NextEntityIndex(out var index))
            {
                var request = requests[index];
                var path = paths[index];

                BnPathWriter.ResolvePolygons(ref graph, ref request);

                var startPoly = request.StartPoly;
                var targetPoly = request.TargetPoly;

                var success = BnAStar.FindPath(
                    ref graph,
                    startPoly,
                    targetPoly,
                    ref scratchpad,
                    BnLinkFilter.PassThrough,
                    MaxIterations);

                if (success)
                {
                    path.Status = BnPathStatus.Ready;
                    path.WaypointCount = BnPathWriter.WritePath(
                        ref graph, ref request, ref scratchpad, waypoints[index]);
                }
                else
                {
                    path.Status = BnPathStatus.Failed;
                    path.WaypointCount = 0;
                    waypoints[index].Clear();
                }

                request.Status = path.Status;
                requests[index] = request;
                paths[index] = path;

                Commands.SetComponentEnabled<BnAgentPathRequestPending>(entities[index], false);
            }
        }
    }

    public static class BnPathWriter
    {
        public static void ResolvePolygons(ref BnMeshGraphBlob graph, ref BnAgentPathRequest request)
        {
            var polygonCount = graph.PolygonCount;

            if ((uint)request.StartPoly >= (uint)polygonCount)
            {
                request.StartPoly = NearestPolygon(ref graph, request.Start);
            }

            if ((uint)request.TargetPoly >= (uint)polygonCount)
            {
                request.TargetPoly = NearestPolygon(ref graph, request.Target);
            }
        }

        public static int WritePath(
            ref BnMeshGraphBlob graph,
            ref BnAgentPathRequest request,
            ref BnPathScratchpad scratch,
            DynamicBuffer<BnPathWaypoint> waypoints)
        {
            var count = scratch.PathCount;
            waypoints.ResizeUninitialized(count);

            var step = math.rcp(math.max(count - 1, 1));

            for (var index = 0; index < count; index++)
            {
                var polygon = scratch.Path[index];
                var last = index == count - 1;
                var centre = graph.Polygons[polygon].Center;
                var height = math.lerp(request.Start.z, request.Target.z, index * step);

                var position = new float3(centre.x, centre.y, height);
                position = math.select(position, request.Start, index == 0);
                position = math.select(position, request.Target, last);

                var linkIndex = -1;
                var exitPoint = position;

                if (!last)
                {
                    linkIndex = FindLink(ref graph, polygon, scratch.Path[index + 1]);
                    if (linkIndex >= 0)
                    {
                        var link = graph.Links[linkIndex];
                        var middle = (link.PortalLeft + link.PortalRight) * 0.5f;
                        exitPoint = new float3(middle.x, middle.y, position.z);
                    }
                }

                waypoints[index] = BnPathWaypoint.Create(position, exitPoint, polygon, linkIndex);
            }

            return count;
        }

        private static int NearestPolygon(ref BnMeshGraphBlob graph, float3 position)
        {
            var point = new float2(position.x, position.y);
            var polygonCount = graph.PolygonCount;

            var best = -1;
            var bestDistanceSq = float.PositiveInfinity;

            for (var polygon = 0; polygon < polygonCount; polygon++)
            {
                var distanceSq = math.distancesq(graph.Polygons[polygon].Center, point);
                var closer = distanceSq < bestDistanceSq;
                best = math.select(best, polygon, closer);
                bestDistanceSq = math.select(bestDistanceSq, distanceSq, closer);
            }

            return best;
        }

        private static int FindLink(ref BnMeshGraphBlob graph, int from, int to)
        {
            var offset = graph.PolyLinkOffsets[from];
            var linkCount = graph.PolyLinkCounts[from];

            var result = -1;
            for (var index = 0; index < linkCount; index++)
            {
                var linkIndex = offset + index;
                result = math.select(result, linkIndex, graph.Links[linkIndex].Target == to);
            }

            return result;
        }
    }

    [UpdateInGroup(typeof(SimulationSystemGroup))]
    public partial class BnPathfindingSystem : SystemBase
    {
        public const int DefaultMaxIterations = 4096;

        public int MaxIterations = DefaultMaxIterations;

        public bool HasScratchpad => _scratch.IsCreated;

        public int ScratchpadCapacity => _scratch.IsCreated ? _scratch.Capacity : 0;

        private EntityQuery _agentQuery;
        private EntityQuery _graphQuery;
        private EntityCommandBufferSystem _commandBufferSystem;
        private BnPathScratchpad _scratch;
        private JobHandle _pathfindHandle;

        protected override void OnCreate()
        {
            _commandBufferSystem = World.GetOrCreateSystemManaged<EndSimulationEntityCommandBufferSystem>();

            _agentQuery = GetEntityQuery(
                ComponentType.ReadWrite<BnAgentPathRequest>(),
                ComponentType.ReadWrite<BnAgentPath>(),
                ComponentType.ReadWrite<BnPathWaypoint>(),
                ComponentType.ReadWrite<BnAgentPathRequestPending>());

            _graphQuery = GetEntityQuery(ComponentType.ReadOnly<BnBlobMeshGraph>());

            _pathfindHandle = default;

            RequireForUpdate(_agentQuery);
            RequireForUpdate(_graphQuery);
        }

        protected override void OnUpdate()
        {
            if (!_graphQuery.TryGetSingleton<BnBlobMeshGraph>(out var graph) || !graph.Value.IsCreated)
            {
                return;
            }

            var polygonCount = graph.Value.Value.PolygonCount;
            if (!EnsureScratchpad(polygonCount))
            {
                return;
            }

            var commands = _commandBufferSystem.CreateCommandBuffer();

            var job = new BnPathfindJob
            {
                Scratch = _scratch,
                Commands = commands,
                Graph = graph.Value,
                RequestHandle = GetComponentTypeHandle<BnAgentPathRequest>(false),
                PathHandle = GetComponentTypeHandle<BnAgentPath>(false),
                WaypointHandle = GetBufferTypeHandle<BnPathWaypoint>(false),
                EntityHandle = GetEntityTypeHandle(),
                MaxIterations = MaxIterations,
            };

            var handle = job.Schedule(_agentQuery, Dependency);

            _commandBufferSystem.AddJobHandleForProducer(handle);
            _pathfindHandle = handle;
            Dependency = handle;
        }

        protected override void OnDestroy()
        {
            if (_scratch.IsCreated)
            {
                _scratch.Dispose(_pathfindHandle);
            }
        }

        private bool EnsureScratchpad(int polygonCount)
        {
            if (_scratch.IsCreated && _scratch.Capacity >= polygonCount)
            {
                return true;
            }

            var previous = _scratch;
            var relocated = BnPathScratchpad.Create(math.max(polygonCount, 1), Allocator.Persistent);

            _scratch = relocated;

            if (!previous.IsCreated)
            {
                return true;
            }

            previous.Dispose(_pathfindHandle);
            _pathfindHandle = default;
            return true;
        }
    }
}
