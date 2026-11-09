using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;

namespace BurstNav
{
    [DisallowMultipleComponent]
    public sealed class BnAgentAuthoring : MonoBehaviour
    {
        [Min(0f)]
        public float MaxSpeed = 5f;

        [Min(0f)]
        public float Radius = 0.5f;

        public bool StartActive = true;
    }

    public sealed class BnAgentAuthoringBaker : Baker<BnAgentAuthoring>
    {
        public override void Bake(BnAgentAuthoring authoring)
        {
            var entity = GetEntity(TransformUsageFlags.Dynamic);

            AddComponent(entity, BnAgentSteering.Idle(
                new float3(authoring.transform.position),
                authoring.MaxSpeed,
                authoring.Radius));

            AddComponent(entity, new BnAgentPath
            {
                PathIndex = 0,
                WaypointCount = 0,
                Status = BnPathStatus.None,
            });

            AddBuffer<BnPathWaypoint>(entity);

            AddComponent(entity, new BnAgentPathRequest
            {
                Start = new float3(authoring.transform.position),
                Target = new float3(authoring.transform.position),
                RequestId = 0,
                StartPoly = -1,
                TargetPoly = -1,
                Status = BnPathStatus.None,
            });

            AddComponent<BnAgentSteeringActive>(entity);
            AddComponent<BnAgentPathRequestPending>(entity);

            SetComponentEnabled<BnAgentSteeringActive>(entity, authoring.StartActive);
            SetComponentEnabled<BnAgentPathRequestPending>(entity, false);
        }
    }
}
