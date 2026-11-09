using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;

namespace BurstNav.Samples.FunnelSteering
{
    public struct BnFunnelRoute : IComponentData
    {
        public float3 Home;

        public float3 Goal;
    }

    [DisallowMultipleComponent]
    public sealed class BnFunnelSteeringBootstrap : MonoBehaviour
    {
        [Min(1)]
        public int AgentCount = 4;

        public BnMeshAuthoring Navmesh;

        public BnAgentAuthoring Agent;

        private NativeArray<Entity> _agents;

        private BlobAssetReference<BnMeshGraphBlob> _graph;

        private Entity _graphEntity;

        private bool _created;

        private void Start()
        {
            var world = World.DefaultGameObjectInjectionWorld;
            if (world == null || !world.IsCreated || _created)
            {
                return;
            }

            if (Navmesh == null || Agent == null)
            {
                Debug.LogError("BnFunnelSteeringBootstrap needs a BnMeshAuthoring and a BnAgentAuthoring reference.");
                return;
            }

            var entityManager = world.EntityManager;
            var origin = new float2(Navmesh.Origin.x, Navmesh.Origin.y);
            _graph = BnFunnelSteeringGraph.Build(Navmesh.CellsX, Navmesh.CellsY, Navmesh.CellSize, origin);

            _graphEntity = entityManager.CreateEntity(typeof(BnBlobMeshGraph));
            entityManager.SetComponentData(_graphEntity, new BnBlobMeshGraph { Value = _graph });

            var cells = new float2(math.max(Navmesh.CellsX, 1), math.max(Navmesh.CellsY, 1));
            var cellSize = math.max(Navmesh.CellSize, 0.01f);
            var size = cells * cellSize;
            var center = origin + size * 0.5f;
            var radius = size * 0.4f;

            var count = math.max(AgentCount, 1);
            var archetype = BnAgentComponents.CreateAgentArchetype(entityManager);
            _agents = new NativeArray<Entity>(count, Allocator.Persistent);

            for (var index = 0; index < count; index++)
            {
                var angle = math.PI2 * index / count;
                var offset = new float2(math.cos(angle) * radius.x, math.sin(angle) * radius.y);
                var position = SnapToCell(center + offset, origin, cellSize, cells);
                var goal = SnapToCell(center - offset, origin, cellSize, cells);

                var agent = entityManager.CreateEntity(archetype);
                entityManager.AddComponent<BnFunnelRoute>(agent);
                entityManager.SetComponentData(agent, BnAgentSteering.Idle(position, Agent.MaxSpeed, Agent.Radius));
                entityManager.SetComponentData(agent, new BnAgentPath
                {
                    PathIndex = 0,
                    WaypointCount = 0,
                    Status = BnPathStatus.None,
                });
                entityManager.SetComponentData(agent, BnAgentPathRequest.Create(position, goal, 1));
                entityManager.SetComponentData(agent, new BnFunnelRoute { Home = position, Goal = goal });
                entityManager.SetComponentEnabled<BnAgentSteeringActive>(agent, Agent.StartActive);
                entityManager.SetComponentEnabled<BnAgentPathRequestPending>(agent, true);

                _agents[index] = agent;
            }

            _created = true;
        }

        private static float3 SnapToCell(float2 point, float2 origin, float cellSize, float2 cells)
        {
            var cell = math.clamp(math.floor((point - origin) / cellSize), float2.zero, cells - 1f);
            return new float3(origin + (cell + 0.5f) * cellSize, 0f);
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
                    for (var index = 0; index < _agents.Length; index++)
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
    }
}
