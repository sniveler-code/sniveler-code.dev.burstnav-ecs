using Unity.Burst;
using Unity.Entities;
using Unity.Mathematics;
using Unity.Transforms;

namespace BurstNav.Samples.StressTest
{
    [BurstCompile]
    public partial struct BnStressTransformJob : IJobEntity
    {
        public void Execute(ref LocalTransform transform, in BnAgentSteering steering)
        {
            transform.Position = steering.Position;
        }
    }

    [UpdateInGroup(typeof(SimulationSystemGroup))]
    [UpdateAfter(typeof(BnSteeringSystem))]
    public partial class BnStressTransformSystem : SystemBase
    {
        private EntityQuery _agents;

        protected override void OnCreate()
        {
            _agents = GetEntityQuery(
                ComponentType.ReadWrite<LocalTransform>(),
                ComponentType.ReadOnly<BnAgentSteering>(),
                ComponentType.ReadOnly<BnStressRoute>());
            RequireForUpdate(_agents);
        }

        protected override void OnUpdate()
        {
            if (_agents.IsEmptyIgnoreFilter)
            {
                return;
            }

            Dependency = new BnStressTransformJob().ScheduleParallel(_agents, Dependency);
        }
    }
}
