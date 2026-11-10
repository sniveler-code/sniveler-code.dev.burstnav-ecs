using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;

namespace BurstNav.Samples.StressTest
{
    [DisallowMultipleComponent]
    public sealed class BnStressFieldAuthoring : MonoBehaviour
    {
        [Min(1)]
        public int CellsX = 80;

        [Min(1)]
        public int CellsY = 24;

        [Min(0.01f)]
        public float CellSize = 2f;

        public Vector2 Origin = Vector2.zero;
    }

    public sealed class BnStressFieldBaker : Baker<BnStressFieldAuthoring>
    {
        public override void Bake(BnStressFieldAuthoring authoring)
        {
            var entity = GetEntity(TransformUsageFlags.None);
            var origin = new float2(authoring.Origin.x, authoring.Origin.y);
            var graph = BnStressField.Build(authoring.CellsX, authoring.CellsY, authoring.CellSize, origin);

            AddBlobAsset(ref graph, out _);
            AddComponent(entity, new BnBlobMeshGraph { Value = graph });
        }
    }
}
