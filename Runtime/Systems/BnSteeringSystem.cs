using Unity.Burst;
using Unity.Burst.Intrinsics;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;

namespace BurstNav
{
    [BurstCompile]
    public struct BnFunnelJob : IJobChunk
    {
        public BlobAssetReference<BnMeshGraphBlob> Graph;

        public ComponentTypeHandle<BnAgentSteering> SteeringHandle;

        [ReadOnly] public ComponentTypeHandle<BnAgentPath> PathHandle;

        [ReadOnly] public BufferTypeHandle<BnPathWaypoint> WaypointHandle;

        public float DeltaTime;

        public void Execute(in ArchetypeChunk chunk, int unfilteredChunkIndex, bool useEnabledMask, in v128 chunkEnabledMask)
        {
            var steering = chunk.GetNativeArray(ref SteeringHandle);
            var paths = chunk.GetNativeArray(ref PathHandle);
            var waypoints = chunk.GetBufferAccessor(ref WaypointHandle);

            ref var graph = ref Graph.Value;

            var enumerator = new ChunkEntityEnumerator(useEnabledMask, chunkEnabledMask, chunk.Count);
            while (enumerator.NextEntityIndex(out var index))
            {
                var state = steering[index];
                var path = paths[index];
                BnFunnel.Solve(ref state, ref path, waypoints[index], ref graph, DeltaTime);
                steering[index] = state;
            }
        }
    }

    [BurstCompile]
    public struct BnSteeringStateJob : IJobChunk
    {
        public EntityCommandBuffer.ParallelWriter Commands;

        [ReadOnly] public ComponentTypeHandle<BnAgentPath> PathHandle;

        [ReadOnly] public ComponentTypeHandle<BnAgentSteeringActive> ActiveHandle;

        [ReadOnly] public EntityTypeHandle EntityHandle;

        public static bool IsSteerable(in BnAgentPath path) => path.IsFollowable;

        public void Execute(in ArchetypeChunk chunk, int unfilteredChunkIndex, bool useEnabledMask, in v128 chunkEnabledMask)
        {
            var paths = chunk.GetNativeArray(ref PathHandle);
            var entities = chunk.GetNativeArray(EntityHandle);
            var sortKeyBase = unfilteredChunkIndex * chunk.Capacity;

            var enumerator = new ChunkEntityEnumerator(useEnabledMask, chunkEnabledMask, chunk.Count);
            while (enumerator.NextEntityIndex(out var index))
            {
                var steerable = IsSteerable(paths[index]);
                var enabled = chunk.IsComponentEnabled(ref ActiveHandle, index);
                if (steerable != enabled)
                {
                    Commands.SetComponentEnabled<BnAgentSteeringActive>(sortKeyBase + index, entities[index], steerable);
                }
            }
        }
    }

    [BurstCompile]
    public struct KinematicBypassJob : IJobChunk
    {
        public ComponentTypeHandle<BnAgentSteering> SteeringHandle;

        public static BnAgentSteering Bypass(BnAgentSteering state)
        {
            state.Velocity = state.DesiredVelocity;
            return state;
        }

        public void Execute(in ArchetypeChunk chunk, int unfilteredChunkIndex, bool useEnabledMask, in v128 chunkEnabledMask)
        {
            var steering = chunk.GetNativeArray(ref SteeringHandle);

            var enumerator = new ChunkEntityEnumerator(useEnabledMask, chunkEnabledMask, chunk.Count);
            while (enumerator.NextEntityIndex(out var index))
            {
                steering[index] = Bypass(steering[index]);
            }
        }
    }

    [BurstCompile]
    public struct IntegrateJob : IJobChunk
    {
        public ComponentTypeHandle<BnAgentSteering> SteeringHandle;

        public float DeltaTime;

        public static BnAgentSteering Integrate(BnAgentSteering state, float deltaTime)
        {
            state.Position = math.mad(state.Velocity, new float3(deltaTime), state.Position);
            return state;
        }

        public void Execute(in ArchetypeChunk chunk, int unfilteredChunkIndex, bool useEnabledMask, in v128 chunkEnabledMask)
        {
            var steering = chunk.GetNativeArray(ref SteeringHandle);
            var deltaTime = DeltaTime;

            var enumerator = new ChunkEntityEnumerator(useEnabledMask, chunkEnabledMask, chunk.Count);
            while (enumerator.NextEntityIndex(out var index))
            {
                steering[index] = Integrate(steering[index], deltaTime);
            }
        }
    }

    [UpdateInGroup(typeof(SimulationSystemGroup))]
    [UpdateAfter(typeof(BnPathfindingSystem))]
    [UpdateBefore(typeof(EndSimulationEntityCommandBufferSystem))]
    public partial class BnSteeringSystem : SystemBase
    {
        private EntityQuery _steeringQuery;

        private EntityQuery _activeQuery;

        private EntityQuery _graphQuery;

        private EndSimulationEntityCommandBufferSystem _commandBufferSystem;

        protected override void OnCreate()
        {
            _steeringQuery = GetEntityQuery(new EntityQueryDesc
            {
                All = new[]
                {
                    ComponentType.ReadWrite<BnAgentSteering>(),
                    ComponentType.ReadOnly<BnAgentPath>(),
                    ComponentType.ReadOnly<BnPathWaypoint>(),
                    ComponentType.ReadOnly<BnAgentSteeringActive>(),
                },
                Options = EntityQueryOptions.IgnoreComponentEnabledState,
            });

            _activeQuery = GetEntityQuery(
                ComponentType.ReadWrite<BnAgentSteering>(),
                ComponentType.ReadOnly<BnAgentSteeringActive>());

            _graphQuery = GetEntityQuery(ComponentType.ReadOnly<BnBlobMeshGraph>());

            _commandBufferSystem = World.GetOrCreateSystemManaged<EndSimulationEntityCommandBufferSystem>();

            RequireForUpdate(_steeringQuery);
            RequireForUpdate(_graphQuery);
        }

        protected override void OnUpdate()
        {
            if (!_graphQuery.TryGetSingleton<BnBlobMeshGraph>(out var graphBlob) || !graphBlob.Value.IsCreated)
            {
                return;
            }

            var graph = graphBlob.Value;

            var deltaTime = World.Time.DeltaTime;
            var commandBufferSystem = _commandBufferSystem;
            var steeringHandle = GetComponentTypeHandle<BnAgentSteering>(false);

            var funnelHandle = new BnFunnelJob
            {
                Graph = graph,
                SteeringHandle = steeringHandle,
                PathHandle = GetComponentTypeHandle<BnAgentPath>(true),
                WaypointHandle = GetBufferTypeHandle<BnPathWaypoint>(true),
                DeltaTime = deltaTime,
            }.ScheduleParallel(_steeringQuery, Dependency);

            var stateHandle = new BnSteeringStateJob
            {
                Commands = commandBufferSystem.CreateCommandBuffer().AsParallelWriter(),
                PathHandle = GetComponentTypeHandle<BnAgentPath>(true),
                ActiveHandle = GetComponentTypeHandle<BnAgentSteeringActive>(true),
                EntityHandle = GetEntityTypeHandle(),
            }.ScheduleParallel(_steeringQuery, Dependency);

            var bypassHandle = new KinematicBypassJob
            {
                SteeringHandle = steeringHandle,
            }.ScheduleParallel(_activeQuery, JobHandle.CombineDependencies(funnelHandle, stateHandle));

            Dependency = new IntegrateJob
            {
                SteeringHandle = steeringHandle,
                DeltaTime = deltaTime,
            }.ScheduleParallel(_activeQuery, bypassHandle);

            commandBufferSystem.AddJobHandleForProducer(Dependency);
        }
    }
}
