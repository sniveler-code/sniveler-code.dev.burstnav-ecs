using System;
using System.IO;
using System.Linq;
using System.Reflection;
using System.Text.RegularExpressions;
using NUnit.Framework;
using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;

namespace BurstNav.Tests
{
    public class BnFunnelTests
    {
        private const string SourceRelativePath = "Runtime/Core/BnFunnel.cs";

        [Test]
        public void GetNextWaypoint_StraightCorridor_ReturnsTheFarTarget()
        {
            using var graph = BuildGraph(StraightPortals(4));
            using var waypoints = StraightWaypoints(5);

            var next = BnFunnel.GetNextWaypoint(ref graph.Value, waypoints, 0, float3.zero, new float3(4f, 0f, 0f));

            Assert.AreEqual(4f, next.x, 1e-4f);
            Assert.AreEqual(0f, next.y, 1e-4f);
        }

        [Test]
        public void GetNextWaypoint_STurn_StopsAtTheFirstPortalCorner()
        {
            using var graph = BuildGraph(new[]
            {
                (Left: new float2(1f, 1f), Right: new float2(1f, -1f)),
                (Left: new float2(-1f, 2f), Right: new float2(0f, 2f)),
            });
            using var waypoints = Waypoints(
                (new float3(0f, 0f, 0f), float3.zero, 0),
                (new float3(1f, 0f, 0f), float3.zero, 1),
                (new float3(1f, 3f, 0f), float3.zero, -1));

            var next = BnFunnel.GetNextWaypoint(ref graph.Value, waypoints, 0, float3.zero, new float3(1f, 3f, 0f));

            Assert.AreEqual(1f, next.x, 1e-4f, "The funnel must steer at the portal corner, not through the wall.");
            Assert.AreEqual(1f, next.y, 1e-4f);
        }

        [Test]
        public void GetNextWaypoint_RestartsAfterPassedPortals()
        {
            using var graph = BuildGraph(StraightPortals(4));
            using var waypoints = StraightWaypoints(5);

            var next = BnFunnel.GetNextWaypoint(ref graph.Value, waypoints, 2, new float3(2f, 0f, 0f), new float3(4f, 0f, 0f));

            Assert.AreEqual(4f, next.x, 1e-4f, "Starting past the first portals must not pull the agent back.");
        }

        [Test]
        public void GetNextWaypoint_SkipsPortalCollapsedOntoTheApex()
        {
            using var graph = BuildGraph(new[]
            {
                (Left: new float2(0f, 0f), Right: new float2(0f, 0f)),
                (Left: new float2(1f, 1f), Right: new float2(1f, -1f)),
            });
            using var waypoints = Waypoints(
                (float3.zero, float3.zero, 0),
                (new float3(1f, 0f, 0f), float3.zero, 1),
                (new float3(2f, 0f, 0f), float3.zero, -1));

            var next = BnFunnel.GetNextWaypoint(ref graph.Value, waypoints, 0, float3.zero, new float3(2f, 0f, 0f));

            Assert.AreEqual(2f, next.x, 1e-4f, "A portal on the apex must not stall the funnel.");
            Assert.IsTrue(math.all(math.isfinite(next)), "The funnel returned a non-finite waypoint.");
        }

        [Test]
        public void GetNextWaypoint_SkipsPortalWhenTheApexSitsOnOneEndpoint()
        {
            using var graph = BuildGraph(new[]
            {
                (Left: new float2(0f, 0f), Right: new float2(0f, 2f)),
                (Left: new float2(2f, 0f), Right: new float2(2f, 2f)),
            });
            using var waypoints = Waypoints(
                (float3.zero, float3.zero, 0),
                (new float3(1f, 0f, 0f), float3.zero, 1),
                (new float3(3f, 0f, 0f), float3.zero, -1));

            var next = BnFunnel.GetNextWaypoint(ref graph.Value, waypoints, 0, float3.zero, new float3(3f, 0f, 0f));

            Assert.Greater(math.distance(next, float3.zero), 0.5f,
                "An apex on a portal endpoint must steer away instead of returning the apex as the corner.");
        }

        [Test]
        public void GetNextWaypoint_FallsBackToExitPoint_WhenTheLinkIsMissing()
        {
            using var graph = BuildGraph(StraightPortals(1));
            using var waypoints = Waypoints(
                (float3.zero, new float3(1f, 0.25f, 0f), -1),
                (new float3(2f, 0f, 0f), float3.zero, -1));

            var next = BnFunnel.GetNextWaypoint(ref graph.Value, waypoints, 0, float3.zero, new float3(2f, 0f, 0f));

            Assert.IsTrue(math.all(math.isfinite(next)), "The funnel returned a non-finite waypoint.");
            Assert.Greater(next.x, 0f, "The fallback exit point must still make progress.");
        }

        [Test]
        public void AdvancePathProgress_CrossesAPortalWhenTheAgentStandsOnItsCorner()
        {
            using var graph = BuildGraph(new[] { (new float2(3f, 3f), new float2(3f, 4f)) });
            using var waypoints = Waypoints(
                (new float3(3.36f, 3.36f, 0f), new float3(3f, 3.5f, 0f), 0),
                (new float3(2.5f, 3.5f, 0f), float3.zero, -1));

            var steering = BnAgentSteering.Idle(new float3(3f, 3f, 0f), 3f, 0.2f);
            var path = new BnAgentPath
            {
                PathIndex = 0,
                WaypointCount = 2,
                Status = BnPathStatus.Ready,
            };

            var index = BnFunnel.AdvancePathProgress(ref steering, ref path, waypoints, ref graph.Value, 0.2f);

            Assert.AreEqual(1, index, "An agent standing on a portal corner must count that portal as passed.");
            Assert.Greater(steering.PathProgress, 0f, "Crossing a portal must report completed length.");
        }

        [Test]
        public void AdvancePathProgress_DoesNotSkipADegenerateCorridor()
        {
            using var graph = BuildGraph(new (float2 Left, float2 Right)[0]);
            using var waypoints = Waypoints(
                (new float3(0f, 0f, 0f), float3.zero, -1),
                (new float3(1f, 0f, 0f), float3.zero, -1),
                (new float3(2f, 0f, 0f), float3.zero, -1),
                (new float3(3f, 0f, 0f), float3.zero, -1));

            var steering = BnAgentSteering.Idle(new float3(0f, 0f, 0f), 3f, 0.2f);
            var path = new BnAgentPath { PathIndex = 0, WaypointCount = 4, Status = BnPathStatus.Ready };

            var index = BnFunnel.AdvancePathProgress(ref steering, ref path, waypoints, ref graph.Value, 0.2f);

            Assert.AreEqual(0, index, "A corridor without graph portals must not be skipped in one step.");
            Assert.AreEqual(0f, steering.PathProgress, 1e-5f);
        }

        [Test]
        public void AdvancePathProgress_WalksAPointOnlyCorridorLikeAGraphCorridor()
        {
            using var graph = BuildGraph(new (float2 Left, float2 Right)[0]);
            using var waypoints = Waypoints(
                (new float3(0f, 0f, 0f), float3.zero, -1),
                (new float3(1f, 0f, 0f), float3.zero, -1),
                (new float3(2f, 0f, 0f), float3.zero, -1),
                (new float3(3f, 0f, 0f), float3.zero, -1));

            var steering = BnAgentSteering.Idle(new float3(2.25f, 0f, 0f), 3f, 0.2f);
            var path = new BnAgentPath { PathIndex = 0, WaypointCount = 4, Status = BnPathStatus.Ready };

            var index = BnFunnel.AdvancePathProgress(ref steering, ref path, waypoints, ref graph.Value, 0.2f);

            Assert.AreEqual(2, index, "A portal without a graph link must still be crossed along the travel direction.");
            Assert.AreEqual(2f, steering.PathProgress, 1e-4f);
        }

        [Test]
        public void AdvancePathProgress_RewindsAStaleIndexFromAPreviousPath()
        {
            using var graph = BuildGraph(StraightPortals(3));
            using var waypoints = StraightWaypoints(4);

            var steering = BnAgentSteering.Idle(new float3(0.1f, 0f, 0f), 3f, 0.2f);
            steering.WaypointIndex = 3;
            var path = new BnAgentPath { PathIndex = 0, WaypointCount = 4, Status = BnPathStatus.Ready };

            var index = BnFunnel.AdvancePathProgress(ref steering, ref path, waypoints, ref graph.Value, 0.2f);

            Assert.AreEqual(0, index, "An index left over from a previous corridor must rewind to the agent position.");
            Assert.AreEqual(0f, steering.PathProgress, 1e-5f);
        }

        [Test]
        public void Solve_StraightPath_OutputsMaxSpeedAlongTheCorridor()
        {
            using var graph = BuildGraph(StraightPortals(4));
            using var waypoints = StraightWaypoints(5);
            var steering = BnAgentSteering.Idle(float3.zero, 5f, 0.25f);
            var path = ReadyPath(waypoints.Length);

            BnFunnel.Solve(ref steering, ref path, waypoints, ref graph.Value);

            Assert.AreEqual(5f, steering.DesiredVelocity.x, 1e-4f);
            Assert.AreEqual(0f, steering.DesiredVelocity.y, 1e-4f);
            Assert.AreEqual(0f, steering.DesiredVelocity.z, 1e-6f, "Steering is planar: XY navigated, Z is height.");
            Assert.AreEqual(0, steering.WaypointIndex);
            Assert.AreEqual(0f, steering.PathProgress);
            Assert.AreEqual(4f, steering.PathLength, 1e-4f, "PathLength must cover the whole waypoint chain.");
            Assert.IsTrue((steering.Flags & BnAgentFlags.Following) != 0);
            Assert.IsTrue((steering.Flags & BnAgentFlags.Arrived) == 0);
        }

        [Test]
        public void Solve_WalksACorridorInAGoldenWaypointSequence()
        {
            using var graph = BuildGraph(StraightPortals(4));
            using var waypoints = StraightWaypoints(5);

            var positions = new[] { 0f, 0.6f, 1.6f, 2.6f, 3.6f, 4f };
            var goldenIndices = new[] { 0, 1, 2, 3, 4, 4 };
            var next = new float3(4f, 0f, 0f);

            for (var step = 0; step < positions.Length; step++)
            {
                var steering = BnAgentSteering.Idle(new float3(positions[step], 0f, 0f), 4f, 0.2f);
                var path = ReadyPath(waypoints.Length);

                BnFunnel.Solve(ref steering, ref path, waypoints, ref graph.Value);

                Assert.AreEqual(
                    goldenIndices[step],
                    steering.WaypointIndex,
                    "Unexpected waypoint index at corridor step " + step + ".");
                Assert.AreEqual(goldenIndices[step], steering.PathProgress, 1e-4f, "Progress must equal the completed corridor segments.");

                var waypoint = BnFunnel.GetNextWaypoint(
                    ref graph.Value,
                    waypoints,
                    steering.WaypointIndex,
                    steering.Position,
                    next);
                Assert.AreEqual(4f, waypoint.x, 1e-4f, "A straight corridor must aim at the far target.");
                Assert.AreEqual(0f, waypoint.y, 1e-4f);

                if (step < positions.Length - 1)
                {
                    Assert.AreEqual(4f, steering.DesiredVelocity.x, 1e-4f);
                    Assert.AreEqual(0f, steering.DesiredVelocity.y, 1e-4f);
                    Assert.IsTrue((steering.Flags & BnAgentFlags.Arrived) == 0);
                }
                else
                {
                    Assert.AreEqual(0f, math.length(steering.DesiredVelocity), 1e-5f);
                    Assert.IsTrue((steering.Flags & BnAgentFlags.Arrived) != 0);
                }
            }
        }

        [Test]
        public void Solve_IsDeterministic_AcrossRepeatedRuns()
        {
            using var graph = BuildGraph(StraightPortals(4));
            using var waypoints = StraightWaypoints(5);

            BnAgentSteering SolveOnce()
            {
                var steering = BnAgentSteering.Idle(new float3(1.2f, 0f, 0f), 4f, 0.2f);
                var path = ReadyPath(waypoints.Length);
                BnFunnel.Solve(ref steering, ref path, waypoints, ref graph.Value);
                return steering;
            }

            var baseline = SolveOnce();
            Assert.AreEqual(1, baseline.WaypointIndex, "The baseline walk must cross the first portal.");
            Assert.AreEqual(4f, baseline.DesiredVelocity.x, 1e-4f);

            for (var run = 0; run < 8; run++)
            {
                var repeat = SolveOnce();

                Assert.AreEqual(baseline.WaypointIndex, repeat.WaypointIndex, "WaypointIndex drifted on run " + run + ".");
                Assert.AreEqual(baseline.Flags, repeat.Flags, "Agent flags drifted on run " + run + ".");
                Assert.AreEqual(baseline.PathProgress, repeat.PathProgress, 0f, "PathProgress must be bit-identical.");
                Assert.AreEqual(baseline.PathLength, repeat.PathLength, 0f, "PathLength must be bit-identical.");
                Assert.AreEqual(baseline.DesiredVelocity.x, repeat.DesiredVelocity.x, 0f, "Desired X must be bit-identical.");
                Assert.AreEqual(baseline.DesiredVelocity.y, repeat.DesiredVelocity.y, 0f, "Desired Y must be bit-identical.");
                Assert.AreEqual(baseline.DesiredVelocity.z, repeat.DesiredVelocity.z, 0f, "Desired Z must be bit-identical.");
            }
        }

        [Test]
        public void Solve_AdvancesWaypointIndex_WhenTheAgentPassesAPortal()
        {
            using var graph = BuildGraph(StraightPortals(4));
            using var waypoints = StraightWaypoints(5);
            var steering = BnAgentSteering.Idle(new float3(1.2f, 0f, 0f), 4f, 0.2f);
            var path = ReadyPath(waypoints.Length);

            BnFunnel.Solve(ref steering, ref path, waypoints, ref graph.Value);

            Assert.AreEqual(1, steering.WaypointIndex, "Portal 0 at x = 0.5 has been crossed.");
            Assert.AreEqual(1f, steering.PathProgress, 1e-4f, "Progress must equal the completed segment.");
            Assert.AreEqual(4f, steering.DesiredVelocity.x, 1e-4f);
        }

        [Test]
        public void Solve_AdvancesAcrossEveryPassedPortal()
        {
            using var graph = BuildGraph(StraightPortals(4));
            using var waypoints = StraightWaypoints(5);
            var steering = BnAgentSteering.Idle(new float3(3.6f, 0f, 0f), 4f, 0.2f);
            var path = ReadyPath(waypoints.Length);

            BnFunnel.Solve(ref steering, ref path, waypoints, ref graph.Value);

            Assert.AreEqual(4, steering.WaypointIndex, "Portals at x = 0.5, 1.5, 2.5 and 3.5 have been crossed.");
            Assert.AreEqual(4f, steering.PathProgress, 1e-4f);
        }

        [Test]
        public void Solve_Arrival_ClearsDesiredVelocityAndRaisesArrived()
        {
            using var graph = BuildGraph(StraightPortals(4));
            using var waypoints = StraightWaypoints(5);
            var steering = BnAgentSteering.Idle(new float3(4f, 0f, 0f), 4f, 0.2f);
            var path = ReadyPath(waypoints.Length);

            BnFunnel.Solve(ref steering, ref path, waypoints, ref graph.Value);

            Assert.AreEqual(4, steering.WaypointIndex);
            Assert.AreEqual(0f, math.length(steering.DesiredVelocity), 1e-5f);
            Assert.IsTrue((steering.Flags & BnAgentFlags.Arrived) != 0);
        }

        [Test]
        public void Solve_ClampsSpeedSoTheAgentCannotOvershoot()
        {
            using var graph = BuildGraph(StraightPortals(4));
            using var waypoints = StraightWaypoints(5);
            var steering = BnAgentSteering.Idle(new float3(3.9f, 0f, 0f), 100f, 0.01f);
            var path = ReadyPath(waypoints.Length);

            BnFunnel.Solve(ref steering, ref path, waypoints, ref graph.Value, 0.1f);

            Assert.AreEqual(1f, steering.DesiredVelocity.x, 1e-3f);
        }

        [Test]
        public void Solve_NotFollowablePath_ClearsSteeringState()
        {
            using var graph = BuildGraph(StraightPortals(4));
            using var waypoints = StraightWaypoints(5);
            var steering = BnAgentSteering.Idle(float3.zero, 4f, 0.2f);
            steering.DesiredVelocity = new float3(9f, 9f, 0f);
            steering.Flags = BnAgentFlags.Following | BnAgentFlags.Arrived;
            var path = new BnAgentPath { WaypointCount = waypoints.Length, Status = BnPathStatus.Pending };

            BnFunnel.Solve(ref steering, ref path, waypoints, ref graph.Value);

            Assert.AreEqual(0f, math.length(steering.DesiredVelocity), 1e-6f);
            Assert.IsTrue((steering.Flags & BnAgentFlags.Following) == 0);
            Assert.IsTrue((steering.Flags & BnAgentFlags.Arrived) == 0);
        }

        [Test]
        public void Solve_EmptyBuffer_ProducesNoVelocity()
        {
            using var graph = BuildGraph(StraightPortals(4));
            using var waypoints = new NativeArray<BnPathWaypoint>(0, Allocator.TempJob);
            var steering = BnAgentSteering.Idle(float3.zero, 4f, 0.2f);
            var path = ReadyPath(0);

            BnFunnel.Solve(ref steering, ref path, waypoints, ref graph.Value);

            Assert.AreEqual(0f, math.length(steering.DesiredVelocity), 1e-6f);
        }

        [Test]
        public void AdvancePathProgress_ReturnsTheNewIndexAndProgress()
        {
            using var graph = BuildGraph(StraightPortals(4));
            using var waypoints = StraightWaypoints(5);
            var steering = BnAgentSteering.Idle(new float3(2.1f, 0f, 0f), 4f, 0.2f);
            var path = ReadyPath(waypoints.Length);

            var index = BnFunnel.AdvancePathProgress(ref steering, ref path, waypoints, ref graph.Value, 0.05f);

            Assert.AreEqual(2, index);
            Assert.AreEqual(2, steering.WaypointIndex);
            Assert.AreEqual(2f, steering.PathProgress, 1e-4f);
            Assert.AreEqual(4f, steering.PathLength, 1e-4f);
        }

        [Test]
        public void Solve_ReadsACorridorFromAnEcsBuffer()
        {
            using var graph = BuildGraph(StraightPortals(4));
            var world = new World("BnFunnelBufferTest");
            try
            {
                var entity = world.EntityManager.CreateEntity();
                var buffer = world.EntityManager.AddBuffer<BnPathWaypoint>(entity);
                var source = StraightWaypoints(5);
                try
                {
                    for (var index = 0; index < source.Length; index++)
                    {
                        buffer.Add(source[index]);
                    }
                }
                finally
                {
                    source.Dispose();
                }

                var steering = BnAgentSteering.Idle(new float3(1.2f, 0f, 0f), 4f, 0.2f);
                var path = ReadyPath(buffer.Length);
                BnFunnel.Solve(ref steering, ref path, buffer, ref graph.Value);

                Assert.AreEqual(1, steering.WaypointIndex);
                Assert.AreEqual(4f, steering.DesiredVelocity.x, 1e-4f);
            }
            finally
            {
                world.Dispose();
            }
        }

        [Test]
        public void Solve_DoesNotAllocateManagedMemory()
        {
            using var graph = BuildGraph(StraightPortals(4));
            using var waypoints = StraightWaypoints(5);

            for (var warmup = 0; warmup < 4; warmup++)
            {
                var warmSteering = BnAgentSteering.Idle(float3.zero, 4f, 0.2f);
                var warmPath = ReadyPath(waypoints.Length);
                BnFunnel.Solve(ref warmSteering, ref warmPath, waypoints, ref graph.Value);
            }

            var steering = BnAgentSteering.Idle(float3.zero, 4f, 0.2f);
            var path = ReadyPath(waypoints.Length);
            var allocatedBefore = GC.GetAllocatedBytesForCurrentThread();
            for (var iteration = 0; iteration < 64; iteration++)
            {
                BnFunnel.Solve(ref steering, ref path, waypoints, ref graph.Value, 1f / 60f);
            }

            var allocatedAfter = GC.GetAllocatedBytesForCurrentThread();
            Assert.AreEqual(0, allocatedAfter - allocatedBefore, "BnFunnel.Solve allocated managed memory.");
        }

        [Test]
        public void Solve_RunsUnderBurst()
        {
            using var graph = BuildGraph(StraightPortals(4));
            using var waypoints = StraightWaypoints(5);
            using var result = new NativeArray<float>(3, Allocator.TempJob);

            new FunnelJob
            {
                Graph = graph,
                Waypoints = waypoints,
                Result = result,
            }.Schedule().Complete();

            Assert.AreEqual(4f, result[0], 1e-4f, "Burst-compiled funnel produced the wrong desired velocity.");
            Assert.AreEqual(0f, result[1], 1e-4f);
            Assert.AreEqual(0f, result[2], "Burst-compiled funnel advanced the path incorrectly.");
        }

        [Test]
        public void Solve_RunsUnderBurst_WithAnEcsBuffer()
        {
            using var graph = BuildGraph(StraightPortals(4));
            var world = new World("BnFunnelBurstBufferTest");
            var result = new NativeArray<float>(3, Allocator.TempJob);
            try
            {
                var entity = world.EntityManager.CreateEntity();
                var buffer = world.EntityManager.AddBuffer<BnPathWaypoint>(entity);
                var source = StraightWaypoints(5);
                try
                {
                    for (var index = 0; index < source.Length; index++)
                    {
                        buffer.Add(source[index]);
                    }
                }
                finally
                {
                    source.Dispose();
                }

                new BufferFunnelJob
                {
                    Graph = graph,
                    Waypoints = buffer,
                    Result = result,
                }.Schedule().Complete();

                Assert.AreEqual(4f, result[0], 1e-4f, "The Burst-compiled ECS-buffer funnel produced the wrong velocity.");
                Assert.AreEqual(0f, result[1], 1e-4f);
                Assert.AreEqual(0f, result[2]);
            }
            finally
            {
                result.Dispose();
                world.Dispose();
            }
        }

        [Test]
        public void Contract_SolveIsStaticBurstCompiledAndTakesComponentReferences()
        {
            var solves = typeof(BnFunnel)
                .GetMethods(BindingFlags.Public | BindingFlags.Static)
                .Where(method => method.Name == "Solve")
                .ToArray();

            Assert.IsNotEmpty(solves, "BnFunnel.Solve is missing.");
            foreach (var method in solves)
            {
                Assert.AreEqual(typeof(void), method.ReturnType);
                Assert.IsNotNull(
                    method.GetCustomAttribute<BurstCompileAttribute>(),
                    "Every Solve overload must be marked [BurstCompile].");

                var parameters = method.GetParameters();
                Assert.AreEqual(typeof(BnAgentSteering).MakeByRefType(), parameters[0].ParameterType);
                Assert.AreEqual(typeof(BnAgentPath).MakeByRefType(), parameters[1].ParameterType);
                Assert.IsTrue(
                    parameters.Any(parameter => parameter.ParameterType == typeof(BnMeshGraphBlob).MakeByRefType()),
                    "Solve must take the graph blob by reference.");
            }

            Assert.IsTrue(
                typeof(BnFunnel)
                    .GetMethods(BindingFlags.Public | BindingFlags.Static)
                    .Any(method => method.Name == "GetNextWaypoint"),
                "BnFunnel.GetNextWaypoint is missing.");
        }

        [Test]
        public void Source_UsesOnlyUnityMathematics()
        {
            var source = ReadSource();

            foreach (var managed in new[] { "Mathf.", "Vector2", "Vector3", "UnityEngine", "System.Math" })
            {
                Assert.IsFalse(source.Contains(managed), "BnFunnel.cs must not use " + managed);
            }

            StringAssert.Contains("using Unity.Mathematics;", source);
            StringAssert.Contains("math.select", source);
        }

        [Test]
        public void Source_HotLoopBodiesAreBranchFree()
        {
            var source = ReadSource();

            foreach (var method in new[] { "Sweep", "Advance", "CompletedLength" })
            {
                var body = ExtractMethodBody(source, method);
                Assert.IsFalse(Regex.IsMatch(body, @"\bif\s*\("), method + " must not branch on data.");
                Assert.IsFalse(Regex.IsMatch(body, @"\bswitch\s*\("), method + " must not switch.");
                Assert.IsFalse(Regex.IsMatch(body, @"\belse\b"), method + " must not use else.");
                Assert.IsFalse(Regex.IsMatch(body, @"\?\s*\w+\s*:"), method + " must not use a ternary.");
            }

            var sweep = ExtractMethodBody(source, "Sweep");
            StringAssert.Contains("math.select", sweep);
            Assert.GreaterOrEqual(
                Regex.Matches(sweep, "math.select").Count,
                6,
                "The sweep must select every left/right/pinch decision instead of branching.");
        }

        [Test]
        public void Source_SolveNeverAllocates()
        {
            var source = ReadSource();

            foreach (var allocation in new[] { "new NativeArray", "Allocator.", "GC.", "ToArray", "List<" })
            {
                Assert.IsFalse(source.Contains(allocation), "BnFunnel.cs must not allocate (" + allocation + ").");
            }
        }

        [Test]
        public void Source_StaysTierAgnostic()
        {
            var source = ReadSource();

            Assert.IsFalse(Regex.IsMatch(source, @"^\s*#\s*(?:if|elif)", RegexOptions.Multiline), "BnFunnel.cs must not use conditionals.");
        }


        private static string ReadSource() => File.ReadAllText(AssetPath(SourceRelativePath));

        private static string AssetPath(string relative) => BnTestPaths.Package(relative);

        private static BnAgentPath ReadyPath(int waypointCount) => new BnAgentPath
        {
            PathIndex = 0,
            WaypointCount = waypointCount,
            Status = BnPathStatus.Ready,
        };

        private static (float2 Left, float2 Right)[] StraightPortals(int count)
        {
            var portals = new (float2 Left, float2 Right)[count];
            for (var index = 0; index < count; index++)
            {
                var center = new float2(index + 0.5f, 0f);
                portals[index] = (center + new float2(0f, 1f), center - new float2(0f, 1f));
            }

            return portals;
        }

        private static NativeArray<BnPathWaypoint> Waypoints(params (float3 Position, float3 Exit, int Link)[] entries)
        {
            var waypoints = new NativeArray<BnPathWaypoint>(entries.Length, Allocator.TempJob);
            for (var index = 0; index < entries.Length; index++)
            {
                waypoints[index] = BnPathWaypoint.Create(entries[index].Position, entries[index].Exit, index, entries[index].Link);
            }

            return waypoints;
        }

        private static NativeArray<BnPathWaypoint> StraightWaypoints(int count)
        {
            var waypoints = new NativeArray<BnPathWaypoint>(count, Allocator.TempJob);
            for (var index = 0; index < count; index++)
            {
                var position = new float3(index, 0f, 0f);
                var link = index < count - 1 ? index : -1;
                waypoints[index] = BnPathWaypoint.Create(position, float3.zero, index, link);
            }

            return waypoints;
        }

        private static BlobAssetReference<BnMeshGraphBlob> BuildGraph((float2 Left, float2 Right)[] portals)
        {
            var builder = new BlobBuilder(Allocator.Temp);
            try
            {
                ref var root = ref builder.ConstructRoot<BnMeshGraphBlob>();
                var links = builder.Allocate(ref root.Links, portals.Length);
                for (var index = 0; index < portals.Length; index++)
                {
                    links[index] = new BnLinkData
                    {
                        PortalLeft = portals[index].Left,
                        PortalRight = portals[index].Right,
                        Target = index + 1,
                        Cost = 1f,
                    };
                }

                return builder.CreateBlobAssetReference<BnMeshGraphBlob>(Allocator.Persistent);
            }
            finally
            {
                builder.Dispose();
            }
        }

        private static string ExtractMethodBody(string source, string methodName)
        {
            var declaration = new Regex(
                @"(?:public|private|internal)\s+static\s+[^\s]+\s+" + Regex.Escape(methodName) + @"\s*\(");
            var match = declaration.Match(source);
            Assert.IsTrue(match.Success, methodName + " declaration not found in BnFunnel.cs.");

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

        [BurstCompile(CompileSynchronously = true)]
        private struct BufferFunnelJob : IJob
        {
            public BlobAssetReference<BnMeshGraphBlob> Graph;
            public DynamicBuffer<BnPathWaypoint> Waypoints;
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
