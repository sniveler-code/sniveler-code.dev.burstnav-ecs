using System.Collections.Generic;
using System.Linq;
using BurstNav;
using NUnit.Framework;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;

namespace BurstNav.Tests
{
    [Category("Smoke")]
    public class BnPathfindingSystemPlayModeTests
    {
        private const int GridWidth = 6;
        private const int GridHeight = 4;

        private static readonly int[] WallPolygons = { 2, 8, 14 };

        private const int DetourPolygonCount = 12;

        [Test]
        public void System_ResolvesDistinctPaths_ForAgentsInTheSameChunk()
        {
            using var fixture = new Fixture(WallPolygons);
            var first = fixture.CreateAgent(0, 5, new float3(0f, 0f, 0f), new float3(5f, 1f, 0f));
            var second = fixture.CreateAgent(1, 22, new float3(1f, 0f, 0f), new float3(4f, 3f, 0f));

            fixture.AdvanceFrames(2);

            var entityManager = fixture.EntityManager;
            Assert.AreEqual(
                entityManager.GetChunk(first).Archetype,
                entityManager.GetChunk(second).Archetype,
                "both agents must share the dominant archetype and chunk");

            var firstWaypoints = entityManager.GetBuffer<BnPathWaypoint>(first);
            var secondWaypoints = entityManager.GetBuffer<BnPathWaypoint>(second);

            Assert.AreEqual(BnPathStatus.Ready, entityManager.GetComponentData<BnAgentPath>(first).Status);
            Assert.AreEqual(BnPathStatus.Ready, entityManager.GetComponentData<BnAgentPath>(second).Status);

            Assert.AreEqual(0, firstWaypoints[0].PolyIndex);
            Assert.AreEqual(5, firstWaypoints[firstWaypoints.Length - 1].PolyIndex);
            Assert.AreEqual(DetourPolygonCount, firstWaypoints.Length);

            Assert.AreEqual(1, secondWaypoints[0].PolyIndex);
            Assert.AreEqual(22, secondWaypoints[secondWaypoints.Length - 1].PolyIndex);
            Assert.AreNotEqual(firstWaypoints.Length, secondWaypoints.Length, "each agent keeps its own corridor");
        }

        [Test]
        public void ResolvedCorridor_IsFollowableByTheFunnel_AtRuntime()
        {
            using var fixture = new Fixture(WallPolygons);
            var entity = fixture.CreateAgent(0, 5, new float3(0f, 0f, 0f), new float3(5f, 1f, 0f));

            fixture.AdvanceFrames(2);

            var entityManager = fixture.EntityManager;
            var steering = BnAgentSteering.Idle(new float3(0f, 0f, 0f), 4f, 0.2f);
            var path = entityManager.GetComponentData<BnAgentPath>(entity);
            var waypoints = entityManager.GetBuffer<BnPathWaypoint>(entity);

            Assert.IsTrue(path.IsFollowable);

            ref var graph = ref fixture.GraphBlob.Value;
            BnFunnel.Solve(ref steering, ref path, waypoints, ref graph, 1f / 60f);

            Assert.Greater(
                math.length(steering.DesiredVelocity),
                0f,
                "the funnel must produce a desired velocity from the resolved corridor");
            Assert.AreEqual(0f, steering.DesiredVelocity.z, 1e-5f, "the desired velocity stays on the navigation plane");
            Assert.AreEqual(BnPathStatus.Ready, path.Status);
        }

        [Test]
        public void System_KeepsTheRequestPendingUntilCommandBufferPlayback()
        {
            using var fixture = new Fixture(WallPolygons);
            var entity = fixture.CreateAgent(0, 5, new float3(0f, 0f, 0f), new float3(5f, 1f, 0f));

            fixture.System.Update();
            fixture.EntityManager.CompleteAllTrackedJobs();

            Assert.AreEqual(
                BnPathStatus.Ready,
                fixture.EntityManager.GetComponentData<BnAgentPath>(entity).Status);
            Assert.IsTrue(
                fixture.EntityManager.IsComponentEnabled<BnAgentPathRequestPending>(entity),
                "the toggle must only flip on command buffer playback");

            fixture.AdvanceFrames(1);
            Assert.IsFalse(fixture.EntityManager.IsComponentEnabled<BnAgentPathRequestPending>(entity));
        }

        private sealed class Fixture : System.IDisposable
        {
            private readonly BlobAssetReference<BnMeshGraphBlob> _graph;

            public readonly World World;
            public readonly EntityManager EntityManager;
            public readonly BnPathfindingSystem System;

            public Fixture(int[] blockedPolygons)
            {
                _graph = BuildGridGraph(GridWidth, GridHeight, blockedPolygons);
                World = new World("BnPathfindingSystemPlayMode");
                EntityManager = World.EntityManager;
                var graphEntity = EntityManager.CreateEntity(typeof(BnBlobMeshGraph));
                EntityManager.SetComponentData(graphEntity, new BnBlobMeshGraph { Value = _graph });
                System = World.GetOrCreateSystemManaged<BnPathfindingSystem>();

                var simulationGroup = World.GetOrCreateSystemManaged<SimulationSystemGroup>();
                var commandBufferSystem = World.GetOrCreateSystemManaged<EndSimulationEntityCommandBufferSystem>();
                simulationGroup.AddSystemToUpdateList(System);
                simulationGroup.AddSystemToUpdateList(commandBufferSystem);
                simulationGroup.SortSystems();
            }

            public BlobAssetReference<BnMeshGraphBlob> GraphBlob => _graph;

            public Entity CreateAgent(int startPoly, int targetPoly, float3 start, float3 target)
            {
                var entity = EntityManager.CreateEntity(BnAgentComponents.CreateAgentArchetype(EntityManager));
                EntityManager.SetComponentData(entity, new BnAgentPath
                {
                    PathIndex = -1,
                    WaypointCount = 0,
                    Status = BnPathStatus.None,
                });

                var request = BnAgentPathRequest.Create(start, target, 1u);
                request.StartPoly = startPoly;
                request.TargetPoly = targetPoly;
                EntityManager.SetComponentData(entity, request);
                return entity;
            }

            public void AdvanceFrames(int frames)
            {
                for (var frame = 0; frame < frames; frame++)
                {
                    World.Update();
                    EntityManager.CompleteAllTrackedJobs();
                }
            }

            public void Dispose()
            {
                World.Dispose();
                if (_graph.IsCreated)
                {
                    _graph.Dispose();
                }
            }
        }

        private static BlobAssetReference<BnMeshGraphBlob> BuildGridGraph(int width, int height, int[] blocked)
        {
            var blockedFlags = new bool[width * height];
            foreach (var polygon in blocked)
            {
                blockedFlags[polygon] = true;
            }

            var centers = new float2[width * height];
            for (var y = 0; y < height; y++)
            {
                for (var x = 0; x < width; x++)
                {
                    centers[x + width * y] = new float2(x, y);
                }
            }

            var outgoing = new List<(int From, int To, float Cost)>();
            var steps = new[] { new int2(1, 0), new int2(-1, 0), new int2(0, 1), new int2(0, -1) };

            for (var y = 0; y < height; y++)
            {
                for (var x = 0; x < width; x++)
                {
                    var id = x + width * y;
                    if (blockedFlags[id])
                    {
                        continue;
                    }

                    foreach (var step in steps)
                    {
                        var nx = x + step.x;
                        var ny = y + step.y;
                        if (nx < 0 || ny < 0 || nx >= width || ny >= height)
                        {
                            continue;
                        }

                        var neighbor = nx + width * ny;
                        if (blockedFlags[neighbor])
                        {
                            continue;
                        }

                        outgoing.Add((id, neighbor, math.distance(centers[id], centers[neighbor])));
                    }
                }
            }

            return BuildGraph(centers, outgoing);
        }

        private static BlobAssetReference<BnMeshGraphBlob> BuildGraph(
            float2[] centers,
            List<(int From, int To, float Cost)> outgoing)
        {
            var count = centers.Length;
            var ordered = outgoing.OrderBy(link => link.From).ToList();

            var builder = new BlobBuilder(Allocator.Temp);
            try
            {
                ref var root = ref builder.ConstructRoot<BnMeshGraphBlob>();

                var vertices = builder.Allocate(ref root.Vertices, count);
                var polygons = builder.Allocate(ref root.Polygons, count);
                var neighborPolygons = builder.Allocate(ref root.NeighborPolygons, ordered.Count);
                var links = builder.Allocate(ref root.Links, ordered.Count);
                var linkOffsets = builder.Allocate(ref root.PolyLinkOffsets, count);
                var linkCounts = builder.Allocate(ref root.PolyLinkCounts, count);

                for (var polygon = 0; polygon < count; polygon++)
                {
                    vertices[polygon] = centers[polygon];
                    polygons[polygon] = new BnPolygon
                    {
                        Center = centers[polygon],
                        VertexOffset = polygon,
                        VertexCount = 1,
                        NeighborOffset = 0,
                        NeighborCount = 0,
                    };
                    linkOffsets[polygon] = 0;
                    linkCounts[polygon] = 0;
                }

                for (var index = 0; index < ordered.Count; index++)
                {
                    var link = ordered[index];
                    var from = centers[link.From];
                    var to = centers[link.To];
                    var middle = (from + to) * 0.5f;
                    var delta = to - from;
                    var perpendicular = math.normalizesafe(new float2(-delta.y, delta.x)) * 0.5f;

                    links[index] = new BnLinkData
                    {
                        PortalLeft = middle + perpendicular,
                        PortalRight = middle - perpendicular,
                        Target = link.To,
                        Cost = link.Cost,
                    };
                    neighborPolygons[index] = link.To;

                    if (linkCounts[link.From] == 0)
                    {
                        linkOffsets[link.From] = index;
                        var first = polygons[link.From];
                        first.NeighborOffset = index;
                        polygons[link.From] = first;
                    }

                    var updated = polygons[link.From];
                    linkCounts[link.From]++;
                    updated.NeighborCount++;
                    polygons[link.From] = updated;
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
