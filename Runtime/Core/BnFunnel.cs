using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;

namespace BurstNav
{
    [BurstCompile]
    public static class BnFunnel
    {
        public const float DefaultArrivalRadius = 0.05f;

        private const float Epsilon = 1e-6f;

        private const float DegenerateDistSq = 1e-4f;


        [BurstCompile]
        public static void Solve(
            ref BnAgentSteering steering,
            ref BnAgentPath path,
            in DynamicBuffer<BnPathWaypoint> waypoints,
            ref BnMeshGraphBlob graph,
            float deltaTime)
        {
            var corridor = waypoints.AsNativeArray();
            Solve(ref steering, ref path, in corridor, ref graph, deltaTime);
        }

        [BurstCompile]
        public static void Solve(
            ref BnAgentSteering steering,
            ref BnAgentPath path,
            in NativeArray<BnPathWaypoint> waypoints,
            ref BnMeshGraphBlob graph,
            float deltaTime)
        {
            var count = ResolveWaypointCount(in waypoints, path.WaypointCount);
            if (!IsFollowable(path.Status, count))
            {
                steering.DesiredVelocity = float3.zero;
                steering.Flags = ClearFlags(steering.Flags, BnAgentFlags.Following | BnAgentFlags.Arrived);
                return;
            }

            path.WaypointCount = count;
            var arrivalRadius = math.max(steering.Radius, DefaultArrivalRadius);
            var index = Advance(ref steering, ref path, in waypoints, count, ref graph, arrivalRadius);

            var position = new float2(steering.Position.x, steering.Position.y);
            var lastWaypoint = waypoints[count - 1].Position;
            var waypoint = GetNextWaypoint(ref graph, in waypoints, index, steering.Position, lastWaypoint);
            var target = new float2(waypoint.x, waypoint.y);
            var offset = target - position;
            var distanceSq = math.lengthsq(offset);
            var invDistance = math.rsqrt(math.max(distanceSq, Epsilon));
            var distance = distanceSq * invDistance;
            var direction = offset * invDistance;

            var maxSpeed = math.max(steering.MaxSpeed, 0f);
            var clampedSpeed = math.min(maxSpeed, distance * math.rcp(math.max(deltaTime, Epsilon)));
            var speed = math.select(maxSpeed, clampedSpeed, deltaTime > 0f);

            var arrived = (index >= count - 1) & (distanceSq <= arrivalRadius * arrivalRadius);
            var desired = new float3(direction.x * speed, direction.y * speed, 0f);
            steering.DesiredVelocity = math.select(desired, float3.zero, arrived);

            var flags = (byte)(((byte)steering.Flags | (byte)BnAgentFlags.Following) & ~(byte)BnAgentFlags.Arrived);
            flags = (byte)(flags | math.select(0, (int)BnAgentFlags.Arrived, arrived));
            steering.Flags = (BnAgentFlags)flags;
        }

        [BurstCompile]
        public static void Solve(
            ref BnAgentSteering steering,
            ref BnAgentPath path,
            in DynamicBuffer<BnPathWaypoint> waypoints,
            ref BnMeshGraphBlob graph)
        {
            var corridor = waypoints.AsNativeArray();
            Solve(ref steering, ref path, in corridor, ref graph, 0f);
        }

        [BurstCompile]
        public static void Solve(
            ref BnAgentSteering steering,
            ref BnAgentPath path,
            in NativeArray<BnPathWaypoint> waypoints,
            ref BnMeshGraphBlob graph)
        {
            Solve(ref steering, ref path, in waypoints, ref graph, 0f);
        }


        public static int AdvancePathProgress(
            ref BnAgentSteering steering,
            ref BnAgentPath path,
            in DynamicBuffer<BnPathWaypoint> waypoints,
            ref BnMeshGraphBlob graph,
            float arrivalRadius)
        {
            var corridor = waypoints.AsNativeArray();
            return AdvancePathProgress(ref steering, ref path, in corridor, ref graph, arrivalRadius);
        }

        public static int AdvancePathProgress(
            ref BnAgentSteering steering,
            ref BnAgentPath path,
            in NativeArray<BnPathWaypoint> waypoints,
            ref BnMeshGraphBlob graph,
            float arrivalRadius)
        {
            var count = ResolveWaypointCount(in waypoints, path.WaypointCount);
            if (count == 0)
            {
                steering.WaypointIndex = 0;
                steering.PathProgress = 0f;
                return 0;
            }

            path.WaypointCount = count;
            return Advance(ref steering, ref path, in waypoints, count, ref graph, arrivalRadius);
        }


        public static float3 GetNextWaypoint(
            ref BnMeshGraphBlob graph,
            in DynamicBuffer<BnPathWaypoint> corridor,
            int startIndex,
            float3 agentPosition,
            float3 targetPosition)
        {
            var waypointArray = corridor.AsNativeArray();
            return GetNextWaypoint(ref graph, in waypointArray, startIndex, agentPosition, targetPosition);
        }

        public static float3 GetNextWaypoint(
            ref BnMeshGraphBlob graph,
            in NativeArray<BnPathWaypoint> corridor,
            int startIndex,
            float3 agentPosition,
            float3 targetPosition)
        {
            var count = corridor.Length;
            var last = count - 1;
            var index = math.clamp(startIndex, 0, math.max(last, 0));
            var apex = new float2(agentPosition.x, agentPosition.y);

            if ((count < 2) | (index >= last))
            {
                return targetPosition;
            }

            var skipping = true;
            while (skipping & (index < last))
            {
                ResolvePortal(ref graph, in corridor, index, out var portalLeft, out var portalRight);
                skipping = ApexOnPortal(apex, portalLeft, portalRight);
                index += math.select(0, 1, skipping);
            }

            if (index >= last)
            {
                return targetPosition;
            }

            var target = new float2(targetPosition.x, targetPosition.y);
            var waypoint = Sweep(ref graph, in corridor, index, last, apex, target);
            return new float3(waypoint.x, waypoint.y, targetPosition.z);
        }


        private static bool ApexOnPortal(float2 apex, float2 left, float2 right)
        {
            var edge = right - left;
            var edgeLengthSq = math.lengthsq(edge);
            var offset = apex - left;
            var along = math.select(0f, math.saturate(math.dot(offset, edge) * math.rcp(math.max(edgeLengthSq, Epsilon))), edgeLengthSq > Epsilon);
            var closest = left + edge * along;
            return math.distancesq(apex, closest) <= DegenerateDistSq;
        }

        private static int Advance(
            ref BnAgentSteering steering,
            ref BnAgentPath path,
            in NativeArray<BnPathWaypoint> waypoints,
            int count,
            ref BnMeshGraphBlob graph,
            float arrivalRadius)
        {
            var last = count - 1;
            var index = math.clamp(steering.WaypointIndex, 0, last);
            var position = new float2(steering.Position.x, steering.Position.y);

            var startIndex = index;
            var rewinding = index > 0;
            while (rewinding & (index > 0))
            {
                var behind = !Passed(ref graph, in waypoints, index - 1, position, arrivalRadius);
                index -= math.select(0, 1, behind);
                rewinding = behind;
            }

            var advancing = index == startIndex;
            while (advancing & (index < last))
            {
                var passed = Passed(ref graph, in waypoints, index, position, arrivalRadius);
                index += math.select(0, 1, passed);
                advancing = passed;
            }

            var totalLength = CompletedLength(in waypoints, last, index, out var progress);

            steering.WaypointIndex = index;
            steering.PathLength = totalLength;
            steering.PathProgress = math.min(progress, totalLength);
            return index;
        }

        private static float2 Sweep(
            ref BnMeshGraphBlob graph,
            in NativeArray<BnPathWaypoint> corridor,
            int startIndex,
            int lastIndex,
            float2 apex,
            float2 target)
        {
            ResolvePortal(ref graph, in corridor, startIndex, out var left, out var right);

            var corner = target;
            var found = false;
            var running = true;

            for (var index = startIndex + 1; running & (index <= lastIndex); index++)
            {
                var isLast = index == lastIndex;
                var portalIndex = math.select(index, lastIndex - 1, isLast);
                ResolvePortal(ref graph, in corridor, portalIndex, out var portalLeft, out var portalRight);
                var nextLeft = math.select(portalLeft, target, isLast);
                var nextRight = math.select(portalRight, target, isLast);

                var toLeft = left - apex;
                var toRight = right - apex;
                var toNextLeft = nextLeft - apex;
                var toNextRight = nextRight - apex;

                var crossRightRight = Cross(toRight, toNextRight);
                var crossLeftRight = Cross(toLeft, toNextRight);
                var crossLeftLeft = Cross(toLeft, toNextLeft);
                var crossRightLeft = Cross(toRight, toNextLeft);

                var pinchAtLeft = crossLeftRight > 0f;
                var pinchAtRight = crossRightLeft < 0f;
                var pinned = pinchAtLeft | pinchAtRight;

                var rightOnApex = math.distancesq(right, apex) < DegenerateDistSq;
                var leftOnApex = math.distancesq(left, apex) < DegenerateDistSq;
                var updateRight = ((crossRightRight < 0f) | rightOnApex) & !pinchAtLeft;
                var updateLeft = ((crossLeftLeft > 0f) | leftOnApex) & !pinchAtRight;

                corner = math.select(corner, math.select(right, left, pinchAtLeft), pinned);
                found |= pinned;
                running = !pinned;

                right = math.select(right, nextRight, updateRight & !pinned);
                left = math.select(left, nextLeft, updateLeft & !pinned);
            }

            return math.select(target, corner, found);
        }

        private static void ResolvePortal(
            ref BnMeshGraphBlob graph,
            in NativeArray<BnPathWaypoint> corridor,
            int index,
            out float2 left,
            out float2 right)
        {
            var waypoint = corridor[index];
            var nextIndex = math.min(index + 1, corridor.Length - 1);
            var next = corridor[nextIndex];

            var from = new float2(waypoint.Position.x, waypoint.Position.y);
            var exit = new float2(waypoint.ExitPoint.x, waypoint.ExitPoint.y);
            var nextPosition = new float2(next.Position.x, next.Position.y);
            var hasExit = (math.lengthsq(exit) > 0f) & (math.distancesq(exit, from) > 0f);
            var fallback = math.select(nextPosition, exit, hasExit);

            var linkCount = graph.LinkCount;
            var linkIndex = waypoint.LinkIndex;
            var useLink = (linkIndex >= 0) & (linkIndex < linkCount);

            if (linkCount > 0)
            {
                var safeIndex = math.select(0, math.clamp(linkIndex, 0, linkCount - 1), useLink);
                var link = graph.Links[safeIndex];
                left = math.select(fallback, link.PortalLeft, useLink);
                right = math.select(fallback, link.PortalRight, useLink);
            }
            else
            {
                left = fallback;
                right = fallback;
            }

            var travel = nextPosition - from;
            var center = (left + right) * 0.5f;
            var swap = Cross(travel, left - center) < 0f;
            var orientedLeft = math.select(left, right, swap);
            var orientedRight = math.select(right, left, swap);
            left = orientedLeft;
            right = orientedRight;
        }

        private static bool Passed(
            ref BnMeshGraphBlob graph,
            in NativeArray<BnPathWaypoint> waypoints,
            int index,
            float2 position,
            float arrivalRadius)
        {
            ResolvePortal(ref graph, in waypoints, index, out var left, out var right);
            var from = waypoints[index].Position;
            var to = waypoints[index + 1].Position;
            var travel = new float2(to.x - from.x, to.y - from.y);
            var midpoint = (left + right) * 0.5f;

            var edge = right - left;
            var normal = new float2(-edge.y, edge.x);
            var flip = math.dot(normal, travel) < 0f;
            normal = math.select(normal, -normal, flip);

            var degenerate = math.distancesq(left, right) < Epsilon;
            normal = math.select(normal, travel, degenerate);

            var ahead = (math.lengthsq(normal) >= Epsilon) & (math.dot(position - midpoint, normal) >= 0f);
            var onPortal = degenerate & (math.distancesq(position, midpoint) <= arrivalRadius * arrivalRadius);
            return ahead | onPortal;
        }

        private static float CompletedLength(
            in NativeArray<BnPathWaypoint> waypoints,
            int lastIndex,
            int progressIndex,
            out float progress)
        {
            var total = 0f;
            var completed = 0f;
            var last = math.min(lastIndex, waypoints.Length - 1);

            for (var segment = 1; segment <= last; segment++)
            {
                var length = SegmentLength(waypoints[segment - 1].Position, waypoints[segment].Position);
                total += length;
                completed += math.select(0f, length, segment <= progressIndex);
            }

            progress = completed;
            return total;
        }

        private static float SegmentLength(float3 from, float3 to) =>
            math.distance(new float2(from.x, from.y), new float2(to.x, to.y));

        private static float Cross(float2 a, float2 b) => a.x * b.y - a.y * b.x;

        private static int ResolveWaypointCount(in NativeArray<BnPathWaypoint> waypoints, int declared)
        {
            var count = waypoints.Length;
            return math.select(count, math.min(declared, count), declared > 0);
        }

        private static bool IsFollowable(BnPathStatus status, int count) =>
            (count > 0) & ((status == BnPathStatus.Ready) |
                           (status == BnPathStatus.Partial) |
                           (status == BnPathStatus.None));

        private static BnAgentFlags ClearFlags(BnAgentFlags value, BnAgentFlags flags) =>
            (BnAgentFlags)((byte)value & ~(byte)flags);
    }
}
