using System;
using System.Collections.Generic;
using System.Linq;
using BurstNav;
using NUnit.Framework;
using Unity.Collections;
using Unity.Core;
using Unity.Entities;
using Unity.Mathematics;

namespace BurstNav.Tests
{
    [Category("Smoke")]
    public class BnIntegrationPlayModeTests
    {
        private const float DeltaTime = 0.05f;
        private const float ArrivalEpsilon = 1e-3f;

        [Test]
        public void Pipeline_ResolvesAPathAndFollowsItToTheTarget()
        {
            var target = new float3(5f, 0f, 0f);

            using var graph = BuildChainGraph(6, 0f, 1f);
            using var fixture = new Fixture(graph, DeltaTime);

            var agent = fixture.CreateAgent(float3.zero, target, 2f, 0.25f, 1u);

            var sawReadyPath = false;
            var sawSteering = false;
            var frames = 0;

            for (; frames < 600; frames++)
            {
                fixture.Step();

                var path = fixture.EntityManager.GetComponentData<BnAgentPath>(agent);
                sawReadyPath |= path.Status == BnPathStatus.Ready;

                Assert.IsTrue(
                    fixture.EntityManager.IsComponentEnabled<BnAgentSteeringActive>(agent),
                    "the agent must stay in the steering chain at frame " + frames + ".");

                var state = fixture.EntityManager.GetComponentData<BnAgentSteering>(agent);
                sawSteering |= math.lengthsq(state.DesiredVelocity) > 0f;

                Assert.IsFalse(float.IsNaN(state.Position.x) || float.IsNaN(state.Position.y));
                Assert.AreEqual(0f, state.Position.z, 1e-6f, "navigation is planar: Z stays height");

                if ((state.Flags & BnAgentFlags.Arrived) != 0)
                {
                    break;
                }
            }

            var finalPath = fixture.EntityManager.GetComponentData<BnAgentPath>(agent);
            var finalState = fixture.EntityManager.GetComponentData<BnAgentSteering>(agent);
            var waypoints = fixture.EntityManager.GetBuffer<BnPathWaypoint>(agent);

            Assert.IsTrue(sawReadyPath, "BnPathfindingSystem must resolve the request.");
            Assert.IsTrue(sawSteering, "BnSteeringSystem must produce a desired velocity.");
            Assert.Greater(frames, 1, "the agent needs more than one frame to reach its target");
            Assert.AreEqual(BnPathStatus.Ready, finalPath.Status);
            Assert.IsTrue((finalState.Flags & BnAgentFlags.Arrived) != 0, "the funnel must report arrival.");
            Assert.LessOrEqual(
                math.distance(finalState.Position, target),
                0.25f + ArrivalEpsilon,
                "arrival must respect the agent radius.");
            Assert.Greater(finalState.PathProgress, 4f, "the whole path must have been traversed.");
            Assert.Greater(waypoints.Length, 1, "the pathfinder must write a corridor, not a single point.");
            Assert.IsFalse(
                fixture.EntityManager.IsComponentEnabled<BnAgentPathRequestPending>(agent),
                "the request toggle must be consumed through deferred playback.");
        }

        [Test]
        public void Request_CanBeReArmedAfterASolve()
        {
            var target = new float3(5f, 0f, 0f);

            using var graph = BuildChainGraph(6, 0f, 1f);
            using var fixture = new Fixture(graph, DeltaTime);

            var agent = fixture.CreateAgent(float3.zero, target, 2f, 0.25f, 1u);

            for (var frame = 0; frame < 10; frame++)
            {
                fixture.Step();
                if (fixture.EntityManager.GetComponentData<BnAgentPath>(agent).Status == BnPathStatus.Ready)
                {
                    break;
                }
            }

            Assert.IsFalse(
                fixture.EntityManager.IsComponentEnabled<BnAgentPathRequestPending>(agent),
                "the first solve must consume the request.");

            for (var frame = 0; frame < 5; frame++)
            {
                fixture.Step();
            }

            var beforeRearm = fixture.EntityManager.GetComponentData<BnAgentSteering>(agent);
            Assert.Greater(beforeRearm.Position.x, 0.25f, "the agent must have moved before the re-arm.");

            fixture.ArmRequest(agent, target);
            Assert.IsTrue(
                fixture.EntityManager.IsComponentEnabled<BnAgentPathRequestPending>(agent),
                "gameplay must be able to re-arm a consumed request.");

            fixture.Step();

            var path = fixture.EntityManager.GetComponentData<BnAgentPath>(agent);
            var waypoints = fixture.EntityManager.GetBuffer<BnPathWaypoint>(agent);

            Assert.AreEqual(BnPathStatus.Ready, path.Status, "the re-armed request must be solved.");
            Assert.IsFalse(
                fixture.EntityManager.IsComponentEnabled<BnAgentPathRequestPending>(agent),
                "the re-armed request must be consumed again.");
            Assert.Greater(waypoints.Length, 1, "the re-armed request must rewrite the corridor.");
            Assert.AreEqual(
                beforeRearm.Position.x,
                waypoints[0].Position.x,
                1e-3f,
                "the second search must start from the agent's current position.");
        }

        [Test]
        public void Crowd_AgentsOnParallelLanes_FollowTheirPathsWithoutColliding()
        {
            const int laneCount = 6;
            const int laneTotal = 4;
            const float laneSpacing = 2f;
            const float radius = 0.4f;
            const float maxSpeed = 2f;

            using var graph = BuildLaneGraph(laneCount, laneTotal, laneSpacing, 1f);
            using var fixture = new Fixture(graph, DeltaTime);

            var agents = new NativeArray<Entity>(laneTotal, Allocator.Temp);
            var targets = new NativeArray<float3>(laneTotal, Allocator.Temp);
            try
            {
                for (var lane = 0; lane < laneTotal; lane++)
                {
                    var y = lane * laneSpacing;
                    targets[lane] = new float3(laneCount - 1, y, 0f);
                    agents[lane] = fixture.CreateAgent(
                        new float3(0f, y, 0f),
                        targets[lane],
                        maxSpeed,
                        radius,
                        (uint)(lane + 1));
                }

                for (var frame = 0; frame < 600; frame++)
                {
                    fixture.Step();
                    AssertNoOverlap(fixture.EntityManager, agents, radius, frame);

                    for (var index = 0; index < agents.Length; index++)
                    {
                        Assert.IsTrue(
                            fixture.EntityManager.IsComponentEnabled<BnAgentSteeringActive>(agents[index]),
                            "agent " + index + " left the steering chain at frame " + frame + ".");

                        var state = fixture.EntityManager.GetComponentData<BnAgentSteering>(agents[index]);
                        Assert.IsFalse(
                            float.IsNaN(state.Position.x) || float.IsNaN(state.Position.y),
                            "agent " + index + " produced a NaN position at frame " + frame + ".");
                        Assert.LessOrEqual(
                            math.length(state.Velocity.xy),
                            maxSpeed + 1e-3f,
                            "agent " + index + " exceeded its maximum speed at frame " + frame + ".");

                        if (frame == 0)
                        {
                            Assert.Greater(
                                math.lengthsq(state.DesiredVelocity),
                                0f,
                                "agent " + index + " was not steered on the first frame.");
                        }
                    }
                }

                for (var index = 0; index < agents.Length; index++)
                {
                    var state = fixture.EntityManager.GetComponentData<BnAgentSteering>(agents[index]);
                    Assert.IsTrue(
                        (state.Flags & BnAgentFlags.Arrived) != 0,
                        "agent " + index + " did not arrive.");
                    Assert.LessOrEqual(math.distance(state.Position, targets[index]), radius + ArrivalEpsilon);

                }
            }
            finally
            {
                agents.Dispose();
                targets.Dispose();
            }
        }

        private static void AssertNoOverlap(
            EntityManager entityManager,
            NativeArray<Entity> agents,
            float radius,
            int frame)
        {
            var minimum = radius * 2f * 0.75f;

            for (var first = 0; first < agents.Length; first++)
            {
                var firstPosition = entityManager.GetComponentData<BnAgentSteering>(agents[first]).Position.xy;
                for (var second = first + 1; second < agents.Length; second++)
                {
                    var secondPosition = entityManager.GetComponentData<BnAgentSteering>(agents[second]).Position.xy;
                    Assert.GreaterOrEqual(
                        math.distance(firstPosition, secondPosition),
                        minimum,
                        "agents " + first + " and " + second + " overlap at frame " + frame + ".");
                }
            }
        }


        private sealed class Fixture : IDisposable
        {
            private readonly EndSimulationEntityCommandBufferSystem _barrier;

            public readonly World World;
            public readonly EntityManager EntityManager;
            public readonly BnPathfindingSystem Pathfinding;
            public readonly BnSteeringSystem Steering;

            public Fixture(BlobAssetReference<BnMeshGraphBlob> graph, float deltaTime)
            {
                World = new World("BnIntegrationPlayMode");
                World.SetTime(new TimeData(deltaTime, deltaTime));
                EntityManager = World.EntityManager;

                var graphEntity = EntityManager.CreateEntity(typeof(BnBlobMeshGraph));
                EntityManager.SetComponentData(graphEntity, new BnBlobMeshGraph { Value = graph });

                Pathfinding = World.GetOrCreateSystemManaged<BnPathfindingSystem>();
                Steering = World.GetOrCreateSystemManaged<BnSteeringSystem>();
                _barrier = World.GetOrCreateSystemManaged<EndSimulationEntityCommandBufferSystem>();
            }

            public Entity CreateAgent(
                float3 start,
                float3 target,
                float maxSpeed,
                float radius,
                uint requestId)
            {
                var agent = EntityManager.CreateEntity(BnAgentComponents.CreateAgentArchetype(EntityManager));
                EntityManager.SetComponentData(agent, BnAgentSteering.Idle(start, maxSpeed, radius));
                EntityManager.SetComponentData(agent, new BnAgentPath
                {
                    PathIndex = -1,
                    WaypointCount = 0,
                    Status = BnPathStatus.None,
                });
                EntityManager.SetComponentData(agent, BnAgentPathRequest.Create(start, target, requestId));
                EntityManager.SetComponentEnabled<BnAgentPathRequestPending>(agent, true);
                return agent;
            }

            public void ArmRequest(Entity agent, float3 target)
            {
                var state = EntityManager.GetComponentData<BnAgentSteering>(agent);
                var request = EntityManager.GetComponentData<BnAgentPathRequest>(agent);
                request.Start = state.Position;
                request.Target = target;
                request.StartPoly = -1;
                request.TargetPoly = -1;
                request.RequestId++;
                request.Status = BnPathStatus.Pending;
                EntityManager.SetComponentData(agent, request);
                EntityManager.SetComponentEnabled<BnAgentPathRequestPending>(agent, true);
            }

            public void Step()
            {
                Pathfinding.Update();
                EntityManager.CompleteAllTrackedJobs();
                _barrier.Update();

                Steering.Update();
                EntityManager.CompleteAllTrackedJobs();
                _barrier.Update();
            }

            public void Dispose()
            {
                World.Dispose();
            }
        }


        private struct Edge
        {
            public int From;
            public int To;

            public Edge(int from, int to)
            {
                From = from;
                To = to;
            }
        }

        private static BlobAssetReference<BnMeshGraphBlob> BuildChainGraph(
            int count,
            float y,
            float portalHalfWidth)
        {
            var centers = new float2[count];
            for (var index = 0; index < count; index++)
            {
                centers[index] = new float2(index, y);
            }

            return BuildGraph(centers, ChainEdges(count, 0), portalHalfWidth);
        }

        private static BlobAssetReference<BnMeshGraphBlob> BuildLaneGraph(
            int count,
            int laneTotal,
            float laneSpacing,
            float portalHalfWidth)
        {
            var centers = new float2[count * laneTotal];
            var edges = new List<Edge>(laneTotal * (count - 1) * 2);

            for (var lane = 0; lane < laneTotal; lane++)
            {
                var offset = lane * count;
                for (var index = 0; index < count; index++)
                {
                    centers[offset + index] = new float2(index, lane * laneSpacing);
                }

                edges.AddRange(ChainEdges(count, offset));
            }

            return BuildGraph(centers, edges.ToArray(), portalHalfWidth);
        }

        private static Edge[] ChainEdges(int count, int offset)
        {
            var edges = new Edge[Math.Max(count - 1, 0) * 2];
            var cursor = 0;

            for (var index = 0; index < count - 1; index++)
            {
                edges[cursor++] = new Edge(offset + index, offset + index + 1);
                edges[cursor++] = new Edge(offset + index + 1, offset + index);
            }

            return edges;
        }

        private static BlobAssetReference<BnMeshGraphBlob> BuildGraph(
            float2[] centers,
            Edge[] edges,
            float portalHalfWidth)
        {
            var polygonCount = centers.Length;
            var ordered = edges.OrderBy(edge => edge.From).ThenBy(edge => edge.To).ToArray();

            var counts = new int[polygonCount];
            var offsets = new int[polygonCount];
            for (var index = 0; index < ordered.Length; index++)
            {
                counts[ordered[index].From]++;
            }

            var running = 0;
            for (var polygon = 0; polygon < polygonCount; polygon++)
            {
                offsets[polygon] = running;
                running += counts[polygon];
            }

            var builder = new BlobBuilder(Allocator.Temp);
            try
            {
                ref var root = ref builder.ConstructRoot<BnMeshGraphBlob>();

                var vertices = builder.Allocate(ref root.Vertices, polygonCount);
                var polygons = builder.Allocate(ref root.Polygons, polygonCount);
                var neighbours = builder.Allocate(ref root.NeighborPolygons, ordered.Length);
                var links = builder.Allocate(ref root.Links, ordered.Length);
                var linkOffsets = builder.Allocate(ref root.PolyLinkOffsets, polygonCount);
                var linkCounts = builder.Allocate(ref root.PolyLinkCounts, polygonCount);

                for (var polygon = 0; polygon < polygonCount; polygon++)
                {
                    vertices[polygon] = centers[polygon];
                    polygons[polygon] = new BnPolygon
                    {
                        Center = centers[polygon],
                        VertexOffset = polygon,
                        VertexCount = 1,
                        NeighborOffset = offsets[polygon],
                        NeighborCount = counts[polygon],
                    };
                    linkOffsets[polygon] = offsets[polygon];
                    linkCounts[polygon] = counts[polygon];
                }

                for (var index = 0; index < ordered.Length; index++)
                {
                    var edge = ordered[index];
                    var from = centers[edge.From];
                    var to = centers[edge.To];
                    var delta = to - from;
                    var middle = (from + to) * 0.5f;
                    var perpendicular = math.normalizesafe(new float2(-delta.y, delta.x)) * portalHalfWidth;

                    var link = new BnLinkData
                    {
                        PortalLeft = middle + perpendicular,
                        PortalRight = middle - perpendicular,
                        Target = edge.To,
                        Cost = math.distance(from, to),
                    };

                    links[index] = link;
                    neighbours[index] = edge.To;
                }

                return builder.CreateBlobAssetReference<BnMeshGraphBlob>(Allocator.Persistent);
            }
            finally
            {
                builder.Dispose();
            }
        }
    }
}
