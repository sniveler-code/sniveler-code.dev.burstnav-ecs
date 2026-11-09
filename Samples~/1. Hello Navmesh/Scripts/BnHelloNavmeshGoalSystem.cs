using Unity.Burst;
using Unity.Burst.Intrinsics;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;

namespace BurstNav.Samples.HelloNavmesh
{
    [BurstCompile]
    public struct BnHelloNavmeshGoalJob : IJobChunk
    {
        public ComponentTypeHandle<BnAgentSteering> SteeringHandle;

        [ReadOnly] public ComponentTypeHandle<BnAgentPath> PathHandle;

        [ReadOnly] public ComponentTypeHandle<BnAgentPathRequest> RequestHandle;

        [ReadOnly] public EntityTypeHandle EntityHandle;

        public EntityCommandBuffer.ParallelWriter Commands;

        public float3 Near;

        public float3 Far;

        [BurstCompile]
        public void Execute(
            in ArchetypeChunk chunk,
            int unfilteredChunkIndex,
            bool useEnabledMask,
            in v128 chunkEnabledMask)
        {
            var states = chunk.GetNativeArray(ref SteeringHandle);
            var paths = chunk.GetNativeArray(ref PathHandle);
            var requests = chunk.GetNativeArray(ref RequestHandle);
            var entities = chunk.GetNativeArray(EntityHandle);
            var sortKeyBase = unfilteredChunkIndex * chunk.Capacity;

            var enumerator = new ChunkEntityEnumerator(useEnabledMask, chunkEnabledMask, chunk.Count);
            while (enumerator.NextEntityIndex(out var index))
            {
                var state = states[index];
                var path = paths[index];
                var request = requests[index];

                if (request.Status == BnPathStatus.Pending)
                {
                    continue;
                }

                var arrived = (state.Flags & BnAgentFlags.Arrived) != 0;
                var following = path.Status == BnPathStatus.Ready || path.Status == BnPathStatus.Partial;

                if (following && !arrived)
                {
                    continue;
                }

                var arrival = math.max(state.Radius, BnFunnel.DefaultArrivalRadius) + 0.25f;
                var atFar = math.distancesq(state.Position, Far) <= arrival * arrival;
                var target = math.select(Far, Near, atFar);

                request.Start = state.Position;
                request.Target = target;
                request.StartPoly = -1;
                request.TargetPoly = -1;
                request.Status = BnPathStatus.Pending;
                request.RequestId++;

                state.WaypointIndex = 0;
                state.PathProgress = 0f;
                state.PathLength = 0f;

                var sortKey = sortKeyBase + index;
                Commands.SetComponent(sortKey, entities[index], request);
                Commands.SetComponent(sortKey, entities[index], state);
                Commands.SetComponentEnabled<BnAgentPathRequestPending>(sortKey, entities[index], true);
            }
        }
    }

    [UpdateInGroup(typeof(SimulationSystemGroup))]
    [UpdateBefore(typeof(BnPathfindingSystem))]
    public partial class BnHelloNavmeshGoalSystem : SystemBase
    {
        private EndSimulationEntityCommandBufferSystem _commandBufferSystem;

        private EntityQuery _agents;

        private EntityQuery _graphs;

        private float3 _near;

        private float3 _far;

        private bool _configured;

        protected override void OnCreate()
        {
            _commandBufferSystem = World.GetOrCreateSystemManaged<EndSimulationEntityCommandBufferSystem>();

            _agents = GetEntityQuery(
                ComponentType.ReadWrite<BnAgentSteering>(),
                ComponentType.ReadWrite<BnAgentPath>(),
                ComponentType.ReadWrite<BnAgentPathRequest>(),
                ComponentType.ReadOnly<BnHelloNavmeshAgent>());

            _graphs = GetEntityQuery(ComponentType.ReadOnly<BnBlobMeshGraph>());
        }

        protected override void OnUpdate()
        {
            if (_agents.IsEmptyIgnoreFilter || _graphs.CalculateEntityCount() != 1)
            {
                return;
            }

            if (!_configured && !Configure())
            {
                return;
            }

            var handle = new BnHelloNavmeshGoalJob
            {
                SteeringHandle = GetComponentTypeHandle<BnAgentSteering>(false),
                PathHandle = GetComponentTypeHandle<BnAgentPath>(true),
                RequestHandle = GetComponentTypeHandle<BnAgentPathRequest>(true),
                EntityHandle = GetEntityTypeHandle(),
                Commands = _commandBufferSystem.CreateCommandBuffer().AsParallelWriter(),
                Near = _near,
                Far = _far,
            }.ScheduleParallel(_agents, Dependency);

            _commandBufferSystem.AddJobHandleForProducer(handle);
            Dependency = handle;
        }

        private bool Configure()
        {
            var reference = _graphs.GetSingleton<BnBlobMeshGraph>().Value;
            if (!reference.IsCreated)
            {
                return false;
            }

            ref var graph = ref reference.Value;
            var polygonCount = graph.PolygonCount;
            if (polygonCount <= 0)
            {
                return false;
            }

            var min = new float2(float.PositiveInfinity);
            var max = new float2(float.NegativeInfinity);

            for (var polygon = 0; polygon < polygonCount; polygon++)
            {
                var center = graph.Polygons[polygon].Center;
                min = math.min(min, center);
                max = math.max(max, center);
            }

            _near = new float3(min.x, min.y, 0f);
            _far = new float3(max.x, max.y, 0f);
            _configured = true;
            return true;
        }
    }
}
