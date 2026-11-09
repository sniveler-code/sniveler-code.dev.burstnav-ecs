using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;

namespace BurstNav.Samples.FunnelSteering
{
    public sealed class BnFunnelSteeringGizmos : MonoBehaviour
    {
        public BnMeshAuthoring Navmesh;

        public Color GridColor = new Color(0.35f, 0.4f, 0.45f, 1f);

        public Color AgentColor = new Color(0.2f, 0.8f, 1f, 1f);

        public Color PathColor = new Color(1f, 0.85f, 0.2f, 1f);

        private EntityQuery _agents;

        private World _queryWorld;

        private bool _hasQuery;

        private void OnDisable()
        {
            ReleaseQuery();
        }

        private void OnDrawGizmos()
        {
            DrawGrid();

            var world = World.DefaultGameObjectInjectionWorld;
            if (world == null || !world.IsCreated)
            {
                return;
            }

            EnsureQuery(world);

            using var entities = _agents.ToEntityArray(Allocator.Temp);
            using var states = _agents.ToComponentDataArray<BnAgentSteering>(Allocator.Temp);

            Gizmos.color = AgentColor;
            for (var index = 0; index < states.Length; index++)
            {
                var state = states[index];
                Gizmos.DrawSphere(new Vector3(state.Position.x, state.Position.y, state.Position.z), math.max(state.Radius, 0.1f));
                DrawCorridor(world.EntityManager, entities[index]);
            }
        }

        private void DrawGrid()
        {
            if (Navmesh == null)
            {
                return;
            }

            var cellsX = math.max(Navmesh.CellsX, 1);
            var cellsY = math.max(Navmesh.CellsY, 1);
            var size = math.max(Navmesh.CellSize, 0.01f);
            var origin = new Vector3(Navmesh.Origin.x, Navmesh.Origin.y, 0f);

            Gizmos.color = GridColor;

            for (var x = 0; x <= cellsX; x++)
            {
                var bottom = origin + new Vector3(x * size, 0f, 0f);
                var top = origin + new Vector3(x * size, cellsY * size, 0f);
                Gizmos.DrawLine(bottom, top);
            }

            for (var y = 0; y <= cellsY; y++)
            {
                var left = origin + new Vector3(0f, y * size, 0f);
                var right = origin + new Vector3(cellsX * size, y * size, 0f);
                Gizmos.DrawLine(left, right);
            }
        }

        private void DrawCorridor(EntityManager entityManager, Entity entity)
        {
            if (!entityManager.Exists(entity) || !entityManager.HasBuffer<BnPathWaypoint>(entity))
            {
                return;
            }

            var waypoints = entityManager.GetBuffer<BnPathWaypoint>(entity, true);
            Gizmos.color = PathColor;

            for (var index = 1; index < waypoints.Length; index++)
            {
                var previous = waypoints[index - 1].Position;
                var current = waypoints[index].Position;
                Gizmos.DrawLine(
                    new Vector3(previous.x, previous.y, previous.z),
                    new Vector3(current.x, current.y, current.z));
            }
        }

        private void EnsureQuery(World world)
        {
            if (_hasQuery && _queryWorld == world)
            {
                return;
            }

            ReleaseQuery();
            _agents = world.EntityManager.CreateEntityQuery(
                ComponentType.ReadOnly<BnAgentSteering>(),
                ComponentType.ReadOnly<BnFunnelRoute>());
            _queryWorld = world;
            _hasQuery = true;
        }

        private void ReleaseQuery()
        {
            if (!_hasQuery)
            {
                return;
            }

            if (_queryWorld != null && _queryWorld.IsCreated)
            {
                _agents.Dispose();
            }

            _agents = default;
            _queryWorld = null;
            _hasQuery = false;
        }
    }
}
