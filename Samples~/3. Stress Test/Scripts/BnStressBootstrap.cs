using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;

namespace BurstNav.Samples.StressTest
{
    [DisallowMultipleComponent]
    public sealed class BnStressBootstrap : MonoBehaviour
    {
        [Min(1)]
        public int CellsX = 80;

        [Min(1)]
        public int CellsY = 24;

        [Min(0.01f)]
        public float CellSize = 2f;

        public Vector2 Origin = Vector2.zero;

        [Min(1)]
        public int AgentCount = 10000;

        [Min(1)]
        public int SpawnPerFrame = 64;

        [Min(0f)]
        public float MaxSpeed = 4f;

        [Min(0f)]
        public float Radius = 0.3f;

        public bool StartActive = true;

        private NativeArray<Entity> _agents;

        private BlobAssetReference<BnMeshGraphBlob> _graph;

        private Entity _graphEntity;

        private EntityArchetype _archetype;

        private EntityManager _entityManager;

        private int _spawned;

        private uint _random = 0x9E3779B9u;

        private bool _created;

        public int Spawned => _spawned;

        private void Start()
        {
            var world = World.DefaultGameObjectInjectionWorld;
            if (world == null || !world.IsCreated || _created)
            {
                return;
            }

            _entityManager = world.EntityManager;
            var origin = new float2(Origin.x, Origin.y);
            _graph = BnStressField.Build(CellsX, CellsY, CellSize, origin);

            _graphEntity = _entityManager.CreateEntity(typeof(BnBlobMeshGraph));
            _entityManager.SetComponentData(_graphEntity, new BnBlobMeshGraph { Value = _graph });

            _archetype = BnAgentComponents.CreateAgentArchetype(_entityManager);
            _agents = new NativeArray<Entity>(math.max(AgentCount, 1), Allocator.Persistent);
            _spawned = 0;
            _created = true;
        }

        private void Update()
        {
            if (!_created || _spawned >= _agents.Length)
            {
                return;
            }

            var origin = new float2(Origin.x, Origin.y);
            var count = math.min(math.max(SpawnPerFrame, 1), _agents.Length - _spawned);

            for (var step = 0; step < count; step++)
            {
                var index = _spawned;
                var spawnCell = new int2(Next((uint)BnStressField.SpawnWidth), Next((uint)math.max(CellsY, 1)));
                var goalCell = new int2(math.max(CellsX - 1 - Next((uint)BnStressField.GoalWidth), 0), Next((uint)math.max(CellsY, 1)));
                var position = BnStressField.Center(spawnCell.x, spawnCell.y, CellSize, origin);
                var goal = BnStressField.Center(goalCell.x, goalCell.y, CellSize, origin);

                var agent = _entityManager.CreateEntity(_archetype);
                _entityManager.AddComponent<BnStressRoute>(agent);
                _entityManager.SetComponentData(agent, BnAgentSteering.Idle(position, MaxSpeed, Radius));
                _entityManager.SetComponentData(agent, new BnAgentPath
                {
                    PathIndex = index,
                    WaypointCount = 0,
                    Status = BnPathStatus.None,
                });
                _entityManager.SetComponentData(agent, BnAgentPathRequest.Create(position, goal, 1));
                _entityManager.SetComponentData(agent, new BnStressRoute { Home = position, Goal = goal });
                _entityManager.SetComponentEnabled<BnAgentSteeringActive>(agent, StartActive);
                _entityManager.SetComponentEnabled<BnAgentPathRequestPending>(agent, true);

                _agents[index] = agent;
                _spawned++;
            }
        }

        private void OnDestroy()
        {
            var world = World.DefaultGameObjectInjectionWorld;
            if (world != null && world.IsCreated)
            {
                var entityManager = world.EntityManager;

                if (_graphEntity != Entity.Null && entityManager.Exists(_graphEntity))
                {
                    entityManager.DestroyEntity(_graphEntity);
                }

                if (_agents.IsCreated)
                {
                    for (var index = 0; index < _spawned; index++)
                    {
                        var agent = _agents[index];
                        if (agent != Entity.Null && entityManager.Exists(agent))
                        {
                            entityManager.DestroyEntity(agent);
                        }
                    }
                }
            }

            if (_graph.IsCreated)
            {
                _graph.Dispose();
            }

            if (_agents.IsCreated)
            {
                _agents.Dispose();
            }
        }

        private int Next(uint bound)
        {
            _random = _random * 1664525u + 1013904223u;
            return (int)((_random >> 8) % math.max(bound, 1u));
        }
    }
}
