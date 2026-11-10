using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;

namespace BurstNav.Samples.StressTest
{
    [UpdateInGroup(typeof(SimulationSystemGroup))]
    [UpdateBefore(typeof(BnPathfindingSystem))]
    public partial class BnStressSpawnSystem : SystemBase
    {
        private EntityQuery _configQuery;

        private NativeArray<Entity> _agents;

        private int _spawned;

        private uint _random = 0x9E3779B9u;

        protected override void OnCreate()
        {
            _configQuery = GetEntityQuery(ComponentType.ReadOnly<BnStressSpawnConfig>());
            RequireForUpdate(_configQuery);
        }

        protected override void OnDestroy()
        {
            if (_agents.IsCreated)
            {
                _agents.Dispose();
            }
        }

        protected override void OnUpdate()
        {
            var config = _configQuery.GetSingleton<BnStressSpawnConfig>();
            if (config.Prefab == Entity.Null || config.Count <= 0)
            {
                return;
            }

            if (!_agents.IsCreated)
            {
                _agents = new NativeArray<Entity>(config.Count, Allocator.Persistent);
            }

            if (_spawned >= _agents.Length)
            {
                return;
            }

            var batch = math.min(math.max(config.PerFrame, 1), _agents.Length - _spawned);
            var slice = _agents.GetSubArray(_spawned, batch);
            EntityManager.Instantiate(config.Prefab, slice);

            for (var index = 0; index < slice.Length; index++)
            {
                var agent = slice[index];
                var spawnCell = new int2(Next(BnStressField.SpawnWidth), Next(config.CellsY));
                var goalCell = new int2(math.max(config.CellsX - BnStressField.GoalWidth + Next(BnStressField.GoalWidth), 0), Next(config.CellsY));
                var position = Jitter(BnStressField.Center(spawnCell.x, spawnCell.y, config.CellSize, config.Origin), config.CellSize);
                var goal = Jitter(BnStressField.Center(goalCell.x, goalCell.y, config.CellSize, config.Origin), config.CellSize);

                var steering = EntityManager.GetComponentData<BnAgentSteering>(agent);
                steering.Position = position;
                steering.Velocity = float3.zero;
                steering.DesiredVelocity = float3.zero;
                steering.MaxSpeed = math.lerp(3f, 6f, Next(1000) * 0.001f);
                steering.PathProgress = 0f;
                steering.PathLength = 0f;
                steering.WaypointIndex = 0;
                steering.Flags = BnAgentFlags.None;

                EntityManager.SetComponentData(agent, steering);
                EntityManager.SetComponentData(agent, new BnAgentPath
                {
                    PathIndex = _spawned + index,
                    WaypointCount = 0,
                    Status = BnPathStatus.None,
                });
                EntityManager.SetComponentData(agent, BnAgentPathRequest.Create(position, goal, 1));
                EntityManager.SetComponentData(agent, new BnStressRoute { Home = position, Goal = goal });
                EntityManager.SetComponentEnabled<BnAgentSteeringActive>(agent, true);
                EntityManager.SetComponentEnabled<BnAgentPathRequestPending>(agent, true);
            }

            _spawned += slice.Length;
        }

        private int Next(int bound)
        {
            _random = _random * 1664525u + 1013904223u;
            return bound <= 0 ? 0 : (int)((_random >> 8) % (uint)bound);
        }

        private float3 Jitter(float3 center, float cellSize)
        {
            var offset = new float2(Next(1000) * 0.001f - 0.5f, Next(1000) * 0.001f - 0.5f) * (cellSize * 0.7f);
            return new float3(center.x + offset.x, center.y + offset.y, center.z);
        }
    }
}
