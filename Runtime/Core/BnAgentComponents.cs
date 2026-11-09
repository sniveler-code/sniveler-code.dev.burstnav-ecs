using System.Runtime.InteropServices;
using Unity.Entities;
using Unity.Mathematics;

namespace BurstNav
{
    public enum BnPathStatus : byte
    {
        None = 0,

        Pending = 1,

        Ready = 2,

        Partial = 3,

        Failed = 4,
    }

    [System.Flags]
    public enum BnAgentFlags : byte
    {
        None = 0,

        Following = 1 << 0,

        Arrived = 1 << 1,

        Blocked = 1 << 2,
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct BnAgentSteering : IComponentData
    {
        public float3 Position;

        public float3 Velocity;

        public float3 DesiredVelocity;

        public float MaxSpeed;

        public float Radius;

        public float PathProgress;

        public float PathLength;

        public int WaypointIndex;

        public BnAgentFlags Flags;

        public bool HasPath => PathLength > 0f && (Flags & BnAgentFlags.Following) != 0;

        public float NormalizedProgress => math.select(0f, math.saturate(PathProgress * math.rcp(PathLength)), PathLength > 0f);

        public static BnAgentSteering Idle(float3 position, float maxSpeed, float radius) => new BnAgentSteering
        {
            Position = position,
            Velocity = float3.zero,
            DesiredVelocity = float3.zero,
            MaxSpeed = maxSpeed,
            Radius = radius,
            PathProgress = 0f,
            PathLength = 0f,
            WaypointIndex = 0,
            Flags = BnAgentFlags.None,
        };
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct BnAgentPath : IComponentData
    {
        public int PathIndex;

        public int WaypointCount;

        public BnPathStatus Status;

        public bool IsFollowable => WaypointCount > 0 && (Status == BnPathStatus.Ready || Status == BnPathStatus.Partial);
    }

    [StructLayout(LayoutKind.Sequential)]
    [InternalBufferCapacity(BnAgentComponents.DefaultPathCapacity)]
    public struct BnPathWaypoint : IBufferElementData
    {
        public float3 Position;

        public float3 ExitPoint;

        public int PolyIndex;

        public int LinkIndex;

        public static BnPathWaypoint Create(float3 position, float3 exitPoint, int polyIndex, int linkIndex) => new BnPathWaypoint
        {
            Position = position,
            ExitPoint = exitPoint,
            PolyIndex = polyIndex,
            LinkIndex = linkIndex,
        };
    }

    [StructLayout(LayoutKind.Sequential)]
    public struct BnAgentPathRequest : IComponentData
    {
        public float3 Start;

        public float3 Target;

        public uint RequestId;

        public int StartPoly;

        public int TargetPoly;

        public BnPathStatus Status;

        public static BnAgentPathRequest Create(float3 start, float3 target, uint requestId) => new BnAgentPathRequest
        {
            Start = start,
            Target = target,
            RequestId = requestId,
            StartPoly = -1,
            TargetPoly = -1,
            Status = BnPathStatus.Pending,
        };
    }

    public struct BnAgentSteeringActive : IComponentData, IEnableableComponent
    {
    }

    public struct BnAgentPathRequestPending : IComponentData, IEnableableComponent
    {
    }

    public static class BnAgentComponents
    {
        public const int DefaultPathCapacity = 16;

        public static EntityArchetype CreateAgentArchetype(EntityManager entityManager) => entityManager.CreateArchetype(
            ComponentType.ReadWrite<BnAgentSteering>(),
            ComponentType.ReadWrite<BnAgentPath>(),
            ComponentType.ReadWrite<BnPathWaypoint>(),
            ComponentType.ReadWrite<BnAgentPathRequest>(),
            ComponentType.ReadWrite<BnAgentSteeringActive>(),
            ComponentType.ReadWrite<BnAgentPathRequestPending>());
    }
}
