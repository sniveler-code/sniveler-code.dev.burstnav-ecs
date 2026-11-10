using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Reflection;
using BurstNav;
using NUnit.Framework;
using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;

namespace BurstNav.Tests
{
    public class BnPathfindingSystemTests
    {
        private const string SourceRelativePath = "Runtime/Systems/BnPathfindingSystem.cs";

        private const int GridWidth = 6;
        private const int GridHeight = 4;
        private const int GridPolygonCount = GridWidth * GridHeight;

        private static readonly int[] WallPolygons = { 2, 8, 14 };

        private static readonly int[] FullWallPolygons = { 2, 8, 14, 20 };

        private const int DetourPolygonCount = 12;

        [Test]
        public void System_ResolvesPendingRequests_AndWritesTheCorridorIntoThePathBuffer()
        {
            using var fixture = new Fixture(WallPolygons);
            var first = fixture.CreateAgent(0, 5, new float3(0f, 0f, 0f), new float3(5f, 1f, 0f));
            var second = fixture.CreateAgent(1, 22, new float3(1f, 0f, 0f), new float3(4f, 3f, 0f));

            fixture.RunFrame();
            fixture.Sync();
            fixture.RunFrame();
            fixture.Sync();

            var entityManager = fixture.EntityManager;

            var firstPath = entityManager.GetComponentData<BnAgentPath>(first);
            Assert.AreEqual(BnPathStatus.Ready, firstPath.Status, "the reachable request must resolve");
            Assert.AreEqual(DetourPolygonCount, firstPath.WaypointCount);
            Assert.IsTrue(firstPath.IsFollowable);

            var firstWaypoints = entityManager.GetBuffer<BnPathWaypoint>(first);
            Assert.AreEqual(DetourPolygonCount, firstWaypoints.Length);
            Assert.AreEqual(0, firstWaypoints[0].PolyIndex, "the corridor starts at the start polygon");
            Assert.AreEqual(5, firstWaypoints[DetourPolygonCount - 1].PolyIndex, "the corridor ends at the target polygon");
            Assert.AreEqual(new float3(0f, 0f, 0f), firstWaypoints[0].Position, "the first waypoint is the exact request start");
            Assert.AreEqual(new float3(5f, 1f, 0f), firstWaypoints[DetourPolygonCount - 1].Position, "the last waypoint is the exact request target");
            Assert.AreEqual(-1, firstWaypoints[DetourPolygonCount - 1].LinkIndex, "the last polygon has no outgoing corridor link");

            AssertCorridorIsConnected(fixture.GraphBlob, firstWaypoints);

            var secondPath = entityManager.GetComponentData<BnAgentPath>(second);
            Assert.AreEqual(BnPathStatus.Ready, secondPath.Status);
            Assert.Greater(secondPath.WaypointCount, 0);
            Assert.AreEqual(1, entityManager.GetBuffer<BnPathWaypoint>(second)[0].PolyIndex);
            Assert.AreEqual(
                22,
                entityManager.GetBuffer<BnPathWaypoint>(second)[secondPath.WaypointCount - 1].PolyIndex);
        }

        [Test]
        public void System_DisablesTheRequestToggleThroughTheEndOfFrameCommandBuffer()
        {
            using var fixture = new Fixture(WallPolygons);
            var entity = fixture.CreateAgent(0, 5, new float3(0f, 0f, 0f), new float3(5f, 1f, 0f));

            Assert.IsTrue(fixture.EntityManager.IsComponentEnabled<BnAgentPathRequestPending>(entity));

            fixture.RunFrame();
            fixture.Sync();
            fixture.RunFrame();
            fixture.Sync();

            Assert.IsFalse(
                fixture.EntityManager.IsComponentEnabled<BnAgentPathRequestPending>(entity),
                "the resolved request must be disabled by command buffer playback");
            Assert.IsTrue(fixture.EntityManager.Exists(entity), "disabling the toggle must not destroy the agent");
            Assert.AreEqual(0, fixture.PendingQueryCount, "the query must stop matching a resolved agent");

            var archetype = fixture.EntityManager.GetChunk(entity).Archetype;
            Assert.IsTrue(archetype.Valid);
        }

        [Test]
        public void System_LeavesAgentsWithoutAPendingRequestUntouched()
        {
            using var fixture = new Fixture(WallPolygons);
            var entity = fixture.CreateAgent(0, 5, new float3(0f, 0f, 0f), new float3(5f, 1f, 0f));

            fixture.EntityManager.SetComponentData(entity, new BnAgentPath
            {
                PathIndex = 3,
                WaypointCount = 1,
                Status = BnPathStatus.Ready,
            });
            fixture.EntityManager.SetComponentEnabled<BnAgentPathRequestPending>(entity, false);

            fixture.RunFrame();
            fixture.Sync();
            fixture.RunFrame();
            fixture.Sync();

            var path = fixture.EntityManager.GetComponentData<BnAgentPath>(entity);
            Assert.AreEqual(3, path.PathIndex);
            Assert.AreEqual(1, path.WaypointCount, "a disabled request must not be resolved");
            Assert.AreEqual(BnPathStatus.Ready, path.Status);
        }

        [Test]
        public void System_ReportsFailure_WhenTheTargetIsUnreachable()
        {
            using var fixture = new Fixture(FullWallPolygons);
            var entity = fixture.CreateAgent(0, 5, new float3(0f, 0f, 0f), new float3(5f, 1f, 0f));

            fixture.RunFrame();
            fixture.Sync();
            fixture.RunFrame();
            fixture.Sync();

            var path = fixture.EntityManager.GetComponentData<BnAgentPath>(entity);
            Assert.AreEqual(BnPathStatus.Failed, path.Status);
            Assert.AreEqual(0, path.WaypointCount);
            Assert.IsFalse(path.IsFollowable);
            Assert.AreEqual(0, fixture.EntityManager.GetBuffer<BnPathWaypoint>(entity).Length);
            Assert.IsFalse(fixture.EntityManager.IsComponentEnabled<BnAgentPathRequestPending>(entity));
        }

        [Test]
        public void System_RespectsTheIterationBudget()
        {
            using var fixture = new Fixture(WallPolygons);
            var entity = fixture.CreateAgent(0, 5, new float3(0f, 0f, 0f), new float3(5f, 1f, 0f));

            fixture.System.MaxIterations = 0;
            fixture.RunFrame();
            fixture.Sync();
            fixture.RunFrame();
            fixture.Sync();

            var path = fixture.EntityManager.GetComponentData<BnAgentPath>(entity);
            Assert.AreEqual(BnPathStatus.Failed, path.Status, "a zero budget cannot expand the start polygon");
            Assert.AreEqual(0, path.WaypointCount);
        }

        [Test]
        public void System_SnapsUnresolvedPolygonsFromTheRequestPositions()
        {
            using var fixture = new Fixture(WallPolygons);
            var entity = fixture.CreateAgent(-1, -1, new float3(0f, 0f, 0f), new float3(5f, 0f, 0f));

            fixture.RunFrame();
            fixture.Sync();
            fixture.RunFrame();
            fixture.Sync();

            var request = fixture.EntityManager.GetComponentData<BnAgentPathRequest>(entity);
            Assert.AreEqual(0, request.StartPoly, "the start position must snap to polygon 0");
            Assert.AreEqual(5, request.TargetPoly, "the target position must snap to polygon 5");
            Assert.AreEqual(BnPathStatus.Ready, request.Status);
            Assert.AreEqual(
                DetourPolygonCount,
                fixture.EntityManager.GetComponentData<BnAgentPath>(entity).WaypointCount);
        }

        [Test]
        public void System_ResnapsStalePolygonIdsThatAreOutOfRange()
        {
            using var fixture = new Fixture(WallPolygons);
            var entity = fixture.CreateAgent(999, -1, new float3(0f, 0f, 0f), new float3(5f, 0f, 0f));

            fixture.RunFrame();
            fixture.Sync();
            fixture.RunFrame();
            fixture.Sync();

            var request = fixture.EntityManager.GetComponentData<BnAgentPathRequest>(entity);
            Assert.AreEqual(0, request.StartPoly, "a stale start polygon id must be re-snapped from the position");
            Assert.AreEqual(5, request.TargetPoly);
            Assert.AreEqual(
                DetourPolygonCount,
                fixture.EntityManager.GetComponentData<BnAgentPath>(entity).WaypointCount);
        }

        [Test]
        public void System_ReusesOneScratchpad_AndSizesItToTheGraph()
        {
            using var fixture = new Fixture(WallPolygons);
            fixture.CreateAgent(0, 5, new float3(0f, 0f, 0f), new float3(5f, 1f, 0f));

            Assert.IsFalse(fixture.System.HasScratchpad, "the scratchpad is allocated on first use, not in OnCreate");

            fixture.RunFrame();
            fixture.Sync();

            Assert.IsTrue(fixture.System.HasScratchpad);
            Assert.AreEqual(GridPolygonCount, fixture.System.ScratchpadCapacity);

            fixture.CreateAgent(1, 22, new float3(1f, 0f, 0f), new float3(4f, 3f, 0f));
            fixture.RunFrame();
            fixture.Sync();
            fixture.RunFrame();
            fixture.Sync();

            Assert.AreEqual(GridPolygonCount, fixture.System.ScratchpadCapacity, "the scratchpad must not grow per frame");
        }

        [Test]
        public void System_GrowsThePathBufferForCorridorsLongerThanTheInlineCapacity()
        {
            const int corridorPolygons = 48;
            using var fixture = new Fixture(BuildCorridorGraph(corridorPolygons));
            var entity = fixture.CreateAgent(0, corridorPolygons - 1, new float3(0f, 0f, 0f), new float3(corridorPolygons - 1, 0f, 0f));

            fixture.RunFrame();
            fixture.Sync();
            fixture.RunFrame();
            fixture.Sync();

            var path = fixture.EntityManager.GetComponentData<BnAgentPath>(entity);
            var waypoints = fixture.EntityManager.GetBuffer<BnPathWaypoint>(entity);

            Assert.AreEqual(BnPathStatus.Ready, path.Status);
            Assert.AreEqual(
                corridorPolygons,
                path.WaypointCount,
                "the corridor must not be truncated by the inline buffer capacity");
            Assert.AreEqual(path.WaypointCount, waypoints.Length);
            Assert.Greater(path.WaypointCount, BnAgentComponents.DefaultPathCapacity);
            Assert.AreEqual(0, waypoints[0].PolyIndex);
            Assert.AreEqual(corridorPolygons - 1, waypoints[path.WaypointCount - 1].PolyIndex);
            AssertCorridorIsConnected(fixture.GraphBlob, waypoints);
        }

        [Test]
        public void Job_IsABurstCompiledIJobChunk_WithTheAuthoritativeFieldContract()
        {
            var type = typeof(BnPathfindJob);

            Assert.IsTrue(typeof(IJobChunk).IsAssignableFrom(type), "BnPathfindJob must be an IJobChunk");
            Assert.IsNotNull(
                type.GetCustomAttribute<BurstCompileAttribute>(),
                "BnPathfindJob must be marked [BurstCompile]");

            Assert.AreEqual(typeof(BlobAssetReference<BnMeshGraphBlob>), type.GetField("Graph").FieldType);
            Assert.AreEqual(typeof(BnPathScratchpad), type.GetField("Scratch").FieldType);
            Assert.AreEqual(typeof(EntityCommandBuffer), type.GetField("Commands").FieldType);
            Assert.AreEqual(typeof(int), type.GetField("MaxIterations").FieldType);

            var execute = type.GetMethod("Execute");
            Assert.IsNotNull(execute, "Execute is missing");
            Assert.AreEqual(
                new[]
                {
                    typeof(ArchetypeChunk).MakeByRefType(),
                    typeof(int),
                    typeof(bool),
                    typeof(Unity.Burst.Intrinsics.v128).MakeByRefType(),
                },
                execute.GetParameters().Select(parameter => parameter.ParameterType).ToArray());
        }

        [Test]
        public void Source_SchedulesWithoutCompletingAndUsesCommandBufferPlayback()
        {
            var source = ReadSource();

            Assert.IsFalse(source.Contains(".Complete("), "the system must never complete a job handle");
            Assert.IsFalse(source.Contains("CompleteAll"), "the system must never drain tracked jobs");
            Assert.IsFalse(source.Contains("Run("), "the job must be scheduled, not run inline");

            StringAssert.Contains("job.Schedule(", source);
            StringAssert.Contains("Dependency = handle", source, "the job handle must feed the next system");
            StringAssert.Contains("AddJobHandleForProducer", source);
            StringAssert.Contains("EntityCommandBuffer", source);
            StringAssert.Contains("SetComponentEnabled<BnAgentPathRequestPending>", source);
            StringAssert.Contains("EndSimulationEntityCommandBufferSystem", source);
            StringAssert.Contains("Dispose(_pathfindHandle)", source, "scratchpad growth must defer the free to the in-flight job");
        }

        [Test]
        public void Source_JobReadsChunksDirectly_WithoutPerEntityComponentLookups()
        {
            var source = ReadSource();
            var execute = ExtractMethodBody(source, "Execute");

            StringAssert.Contains("GetNativeArray", execute);
            StringAssert.Contains("GetBufferAccessor", execute);
            StringAssert.Contains("ChunkEntityEnumerator", execute);
            Assert.IsFalse(execute.Contains("GetComponentData"), "the job must not read components per entity");
            Assert.IsFalse(execute.Contains("SetComponentData"), "the job must not write components per entity");
            Assert.IsFalse(execute.Contains("EntityManager"), "the job must not touch the entity manager");
            StringAssert.Contains("Commands.SetComponentEnabled", execute, "the toggle must be deferred");
        }

        private static void AssertCorridorIsConnected(
            BlobAssetReference<BnMeshGraphBlob> graphReference,
            DynamicBuffer<BnPathWaypoint> waypoints)
        {
            for (var index = 0; index + 1 < waypoints.Length; index++)
            {
                var linkIndex = waypoints[index].LinkIndex;
                Assert.GreaterOrEqual(linkIndex, 0, $"waypoint {index} has no outgoing link");

                ref var blob = ref graphReference.Value;
                Assert.AreEqual(
                    waypoints[index + 1].PolyIndex,
                    blob.Links[linkIndex].Target,
                    $"the link of waypoint {index} must lead to waypoint {index + 1}");

                var expectedExit = (blob.Links[linkIndex].PortalLeft + blob.Links[linkIndex].PortalRight) * 0.5f;
                Assert.AreEqual(expectedExit.x, waypoints[index].ExitPoint.x, 1e-4f);
                Assert.AreEqual(expectedExit.y, waypoints[index].ExitPoint.y, 1e-4f);
            }
        }

        private static string ReadSource() => File.ReadAllText(AssetPath(SourceRelativePath));

        private static string ExtractMethodBody(string source, string methodName)
        {
            var declaration = new System.Text.RegularExpressions.Regex(
                @"(?:public|private|internal)\s+(?:static\s+)?(?:void|bool|int)\s+" + methodName + @"\s*\(");
            var match = declaration.Match(source);
            Assert.IsTrue(match.Success, methodName + " declaration not found.");

            var open = source.IndexOf('{', match.Index + match.Length);
            Assert.IsTrue(open > 0, methodName + " has no body.");

            var depth = 0;
            for (var index = open; index < source.Length; index++)
            {
                if (source[index] == '{')
                {
                    depth++;
                }
                else if (source[index] == '}')
                {
                    depth--;
                    if (depth == 0)
                    {
                        return source.Substring(open, index - open + 1);
                    }
                }
            }

            Assert.Fail(methodName + " body is not balanced.");
            return string.Empty;
        }

        private static string AssetPath(string relative) => BnTestPaths.Package(relative);

        private sealed class Fixture : IDisposable
        {
            private readonly BlobAssetReference<BnMeshGraphBlob> _graph;
            private readonly EntityQuery _pendingQuery;

            public readonly World World;
            public readonly EntityManager EntityManager;
            public readonly BnPathfindingSystem System;
            public readonly Entity GraphEntity;

            public Fixture(int[] blockedPolygons)
                : this(BuildGridGraph(GridWidth, GridHeight, blockedPolygons))
            {
            }

            public Fixture(BlobAssetReference<BnMeshGraphBlob> graph)
            {
                _graph = graph;
                World = new World("BnPathfindingSystemTests");
                EntityManager = World.EntityManager;
                GraphEntity = EntityManager.CreateEntity(typeof(BnBlobMeshGraph));
                EntityManager.SetComponentData(GraphEntity, new BnBlobMeshGraph { Value = _graph });

                System = World.GetOrCreateSystemManaged<BnPathfindingSystem>();

                var simulationGroup = World.GetOrCreateSystemManaged<SimulationSystemGroup>();
                var commandBufferSystem = World.GetOrCreateSystemManaged<EndSimulationEntityCommandBufferSystem>();
                simulationGroup.AddSystemToUpdateList(System);
                simulationGroup.AddSystemToUpdateList(commandBufferSystem);
                simulationGroup.SortSystems();

                _pendingQuery = EntityManager.CreateEntityQuery(
                    ComponentType.ReadWrite<BnAgentPathRequestPending>());
            }

            public BlobAssetReference<BnMeshGraphBlob> GraphBlob => _graph;

            public int PendingQueryCount => _pendingQuery.CalculateEntityCount();

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

            public void RunFrame() => World.Update();

            public void Sync() => EntityManager.CompleteAllTrackedJobs();

            public void Dispose()
            {
                _pendingQuery.Dispose();
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

        private static BlobAssetReference<BnMeshGraphBlob> BuildCorridorGraph(int polygonCount)
        {
            var centers = new float2[polygonCount];
            var outgoing = new List<(int From, int To, float Cost)>();

            for (var polygon = 0; polygon < polygonCount; polygon++)
            {
                centers[polygon] = new float2(polygon, 0f);
            }

            for (var polygon = 0; polygon + 1 < polygonCount; polygon++)
            {
                outgoing.Add((polygon, polygon + 1, 1f));
                outgoing.Add((polygon + 1, polygon, 1f));
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
