using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;

namespace BurstNav.Samples.StressTest
{
    public struct BnStressSpawnConfig : IComponentData
    {
        public Entity Prefab;

        public int Count;

        public int PerFrame;

        public int CellsX;

        public int CellsY;

        public float CellSize;

        public float2 Origin;
    }

    [DisallowMultipleComponent]
    public sealed class BnStressSpawnAuthoring : MonoBehaviour
    {
        public GameObject AgentPrefab;

        [Min(1)]
        public int AgentCount = 10000;

        [Min(1)]
        public int SpawnPerFrame = 64;

        [Min(1)]
        public int CellsX = 80;

        [Min(1)]
        public int CellsY = 24;

        [Min(0.01f)]
        public float CellSize = 2f;

        public Vector2 Origin = Vector2.zero;
    }

    public sealed class BnStressSpawnBaker : Baker<BnStressSpawnAuthoring>
    {
        public override void Bake(BnStressSpawnAuthoring authoring)
        {
            var entity = GetEntity(TransformUsageFlags.None);
            var prefab = authoring.AgentPrefab != null
                ? GetEntity(authoring.AgentPrefab, TransformUsageFlags.Dynamic)
                : Entity.Null;

            AddComponent(entity, new BnStressSpawnConfig
            {
                Prefab = prefab,
                Count = math.max(authoring.AgentCount, 1),
                PerFrame = math.max(authoring.SpawnPerFrame, 1),
                CellsX = math.max(authoring.CellsX, 1),
                CellsY = math.max(authoring.CellsY, 1),
                CellSize = math.max(authoring.CellSize, 0.01f),
                Origin = new float2(authoring.Origin.x, authoring.Origin.y),
            });
        }
    }
}
