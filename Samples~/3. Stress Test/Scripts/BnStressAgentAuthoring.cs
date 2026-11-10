using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;

namespace BurstNav.Samples.StressTest
{
    [DisallowMultipleComponent]
    public sealed class BnStressAgentAuthoring : MonoBehaviour
    {
    }

    public sealed class BnStressAgentBaker : Baker<BnStressAgentAuthoring>
    {
        public override void Bake(BnStressAgentAuthoring authoring)
        {
            var entity = GetEntity(TransformUsageFlags.Dynamic);
            AddComponent<BnStressRoute>(entity);
        }
    }
}
