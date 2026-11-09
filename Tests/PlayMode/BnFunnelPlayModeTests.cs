using NUnit.Framework;
using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;

namespace BurstNav.Tests
{
    [Category("Smoke")]
    public class BnFunnelPlayModeTests
    {
        [Test]
        public void Solve_FollowsTheCorridor_AtRuntime()
        {
            using var graph = BuildCorridorGraph(out var waypoints);
            try
            {
                var steering = BnAgentSteering.Idle(new float3(1.2f, 0f, 0f), 4f, 0.2f);
                var path = new BnAgentPath
                {
                    WaypointCount = waypoints.Length,
                    Status = BnPathStatus.Ready,
                };

                BnFunnel.Solve(ref steering, ref path, waypoints, ref graph.Value, 1f / 60f);

                Assert.AreEqual(1, steering.WaypointIndex, "The funnel did not advance past the first portal.");
                Assert.AreEqual(1f, steering.PathProgress, 1e-4f);
                Assert.AreEqual(4f, steering.DesiredVelocity.x, 1e-3f);
                Assert.AreEqual(0f, steering.DesiredVelocity.z, 1e-6f);
            }
            finally
            {
                waypoints.Dispose();
            }
        }

        [Test]
        public void Solve_ReadsThePathBufferOfAnEntity()
        {
            using var graph = BuildCorridorGraph(out var source);
            var world = new World("BnFunnelPlayMode");
            try
            {
                var entity = world.EntityManager.CreateEntity(
                    typeof(BnAgentSteering),
                    typeof(BnAgentPath),
                    typeof(BnPathWaypoint));

                var buffer = world.EntityManager.GetBuffer<BnPathWaypoint>(entity);
                for (var index = 0; index < source.Length; index++)
                {
                    buffer.Add(source[index]);
                }

                world.EntityManager.SetComponentData(entity, BnAgentSteering.Idle(new float3(2.1f, 0f, 0f), 4f, 0.2f));
                world.EntityManager.SetComponentData(entity, new BnAgentPath
                {
                    WaypointCount = buffer.Length,
                    Status = BnPathStatus.Ready,
                });

                var steering = world.EntityManager.GetComponentData<BnAgentSteering>(entity);
                var path = world.EntityManager.GetComponentData<BnAgentPath>(entity);
                BnFunnel.Solve(ref steering, ref path, buffer, ref graph.Value);
                world.EntityManager.SetComponentData(entity, steering);
                world.EntityManager.SetComponentData(entity, path);

                var resolved = world.EntityManager.GetComponentData<BnAgentPath>(entity);
                Assert.AreEqual(2, steering.WaypointIndex);
                Assert.AreEqual(4f, world.EntityManager.GetComponentData<BnAgentSteering>(entity).DesiredVelocity.x, 1e-4f);
            }
            finally
            {
                source.Dispose();
                world.Dispose();
            }
        }

        [Test]
        public void Solve_RunsInsideABurstJob()
        {
            using var graph = BuildCorridorGraph(out var waypoints);
            var result = new NativeArray<float>(3, Allocator.TempJob);
            try
            {
                new FunnelJob
                {
                    Graph = graph,
                    Waypoints = waypoints,
                    Result = result,
                }.Schedule().Complete();

                Assert.AreEqual(4f, result[0], 1e-4f, "Burst-compiled funnel produced the wrong velocity.");
                Assert.AreEqual(0f, result[1], 1e-4f);
                Assert.AreEqual(0f, result[2], "Burst-compiled funnel changed the path index unexpectedly.");
            }
            finally
            {
                result.Dispose();
            }
        }

        private static BlobAssetReference<BnMeshGraphBlob> BuildCorridorGraph(out NativeArray<BnPathWaypoint> waypoints)
        {
            const int waypointCount = 5;

            var builder = new BlobBuilder(Allocator.Temp);
            try
            {
                ref var root = ref builder.ConstructRoot<BnMeshGraphBlob>();
                var links = builder.Allocate(ref root.Links, waypointCount - 1);
                for (var index = 0; index < waypointCount - 1; index++)
                {
                    var center = new float2(index + 0.5f, 0f);
                    links[index] = new BnLinkData
                    {
                        PortalLeft = center + new float2(0f, 1f),
                        PortalRight = center - new float2(0f, 1f),
                        Target = index + 1,
                        Cost = 1f,
                    };
                }

                waypoints = new NativeArray<BnPathWaypoint>(waypointCount, Allocator.TempJob);
                for (var index = 0; index < waypointCount; index++)
                {
                    waypoints[index] = BnPathWaypoint.Create(
                        new float3(index, 0f, 0f),
                        float3.zero,
                        index,
                        index < waypointCount - 1 ? index : -1);
                }

                return builder.CreateBlobAssetReference<BnMeshGraphBlob>(Allocator.Persistent);
            }
            finally
            {
                builder.Dispose();
            }
        }

        [BurstCompile(CompileSynchronously = true)]
        private struct FunnelJob : IJob
        {
            public BlobAssetReference<BnMeshGraphBlob> Graph;
            public NativeArray<BnPathWaypoint> Waypoints;
            public NativeArray<float> Result;

            public void Execute()
            {
                ref var graph = ref Graph.Value;
                var steering = BnAgentSteering.Idle(float3.zero, 4f, 0.2f);
                var path = new BnAgentPath
                {
                    WaypointCount = Waypoints.Length,
                    Status = BnPathStatus.Ready,
                };

                BnFunnel.Solve(ref steering, ref path, Waypoints, ref graph, 1f / 60f);

                Result[0] = steering.DesiredVelocity.x;
                Result[1] = steering.DesiredVelocity.y;
                Result[2] = steering.WaypointIndex;
            }
        }
    }
}
