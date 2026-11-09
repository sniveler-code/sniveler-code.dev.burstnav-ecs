using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;

namespace BurstNav.Samples.HelloNavmesh
{
    public struct BnHelloNavmeshAgent : IComponentData
    {
    }

    [DisallowMultipleComponent]
    public sealed class BnHelloNavmeshBootstrap : MonoBehaviour
    {
        public BnMeshAuthoring Navmesh;

        public BnAgentAuthoring Agent;

        private BlobAssetReference<BnMeshGraphBlob> _graph;

        private Entity _graphEntity;

        private Entity _agentEntity;

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
                Debug.LogError("BnHelloNavmeshBootstrap needs a BnMeshAuthoring and a BnAgentAuthoring reference.");
                return;
            }

            var entityManager = world.EntityManager;
            var origin = new float2(Navmesh.Origin.x, Navmesh.Origin.y);
            _graph = BnHelloNavmeshGraph.Build(Navmesh.CellsX, Navmesh.CellsY, Navmesh.CellSize, origin);

            _graphEntity = entityManager.CreateEntity(typeof(BnBlobMeshGraph));
            entityManager.SetComponentData(_graphEntity, new BnBlobMeshGraph { Value = _graph });

            var archetype = BnAgentComponents.CreateAgentArchetype(entityManager);
            var position = new float3(Agent.transform.position);
            var steering = BnAgentSteering.Idle(position, Agent.MaxSpeed, Agent.Radius);

            _agentEntity = entityManager.CreateEntity(archetype);
            entityManager.AddComponent<BnHelloNavmeshAgent>(_agentEntity);
            entityManager.SetComponentData(_agentEntity, steering);
            entityManager.SetComponentData(_agentEntity, new BnAgentPath
            {
                PathIndex = 0,
                WaypointCount = 0,
                Status = BnPathStatus.None,
            });

            var request = BnAgentPathRequest.Create(position, position, 1);
            request.Status = BnPathStatus.None;

            entityManager.SetComponentData(_agentEntity, request);
            entityManager.SetComponentEnabled<BnAgentSteeringActive>(_agentEntity, Agent.StartActive);
            entityManager.SetComponentEnabled<BnAgentPathRequestPending>(_agentEntity, false);

            _created = true;
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

                if (_agentEntity != Entity.Null && entityManager.Exists(_agentEntity))
                {
                    entityManager.DestroyEntity(_agentEntity);
                }
            }

            if (_graph.IsCreated)
            {
                _graph.Dispose();
            }
        }
    }
}
