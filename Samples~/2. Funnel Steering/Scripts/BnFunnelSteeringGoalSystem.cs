using Unity.Burst;
using Unity.Burst.Intrinsics;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;

namespace BurstNav.Samples.FunnelSteering
{
    [BurstCompile]
    public struct BnFunnelSteeringGoalJob : IJobChunk
    {
        public ComponentTypeHandle<BnAgentSteering> SteeringHandle;

        public ComponentTypeHandle<BnFunnelRoute> RouteHandle;

        [ReadOnly] public ComponentTypeHandle<BnAgentPath> PathHandle;

        [ReadOnly] public ComponentTypeHandle<BnAgentPathRequest> RequestHandle;

        [ReadOnly] public EntityTypeHandle EntityHandle;

        public EntityCommandBuffer.ParallelWriter Commands;

        [BurstCompile]
        public void Execute(
            in ArchetypeChunk chunk,
            int unfilteredChunkIndex,
            bool useEnabledMask,
            in v128 chunkEnabledMask)
        {
            var states = chunk.GetNativeArray(ref SteeringHandle);
            var routes = chunk.GetNativeArray(ref RouteHandle);
            var paths = chunk.GetNativeArray(ref PathHandle);
            var requests = chunk.GetNativeArray(ref RequestHandle);
            var entities = chunk.GetNativeArray(EntityHandle);
            var sortKeyBase = unfilteredChunkIndex * chunk.Capacity;

            var enumerator = new ChunkEntityEnumerator(useEnabledMask, chunkEnabledMask, chunk.Count);
            while (enumerator.NextEntityIndex(out var index))
            {
                var state = states[index];
                var route = routes[index];
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

                var target = route.Home;
                route.Home = route.Goal;
                route.Goal = target;

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
                Commands.SetComponent(sortKey, entities[index], route);
                Commands.SetComponentEnabled<BnAgentPathRequestPending>(sortKey, entities[index], true);
            }
        }
    }

    [UpdateInGroup(typeof(SimulationSystemGroup))]
    [UpdateBefore(typeof(BnPathfindingSystem))]
    public partial class BnFunnelSteeringGoalSystem : SystemBase
    {
        private EndSimulationEntityCommandBufferSystem _commandBufferSystem;

        private EntityQuery _agents;

        protected override void OnCreate()
        {
            _commandBufferSystem = World.GetOrCreateSystemManaged<EndSimulationEntityCommandBufferSystem>();

            _agents = GetEntityQuery(
                ComponentType.ReadWrite<BnAgentSteering>(),
                ComponentType.ReadWrite<BnAgentPath>(),
                ComponentType.ReadWrite<BnAgentPathRequest>(),
                ComponentType.ReadWrite<BnFunnelRoute>());
        }

        protected override void OnUpdate()
        {
            if (_agents.IsEmptyIgnoreFilter)
            {
                return;
            }

            var handle = new BnFunnelSteeringGoalJob
            {
                SteeringHandle = GetComponentTypeHandle<BnAgentSteering>(false),
                RouteHandle = GetComponentTypeHandle<BnFunnelRoute>(false),
                PathHandle = GetComponentTypeHandle<BnAgentPath>(true),
                RequestHandle = GetComponentTypeHandle<BnAgentPathRequest>(true),
                EntityHandle = GetEntityTypeHandle(),
                Commands = _commandBufferSystem.CreateCommandBuffer().AsParallelWriter(),
            }.ScheduleParallel(_agents, Dependency);

            _commandBufferSystem.AddJobHandleForProducer(handle);
            Dependency = handle;
        }
    }
}
