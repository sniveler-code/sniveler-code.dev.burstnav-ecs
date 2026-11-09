using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Reflection;
using System.Text.RegularExpressions;
using NUnit.Framework;
using Unity.Burst;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;

namespace BurstNav.Tests
{
    public class BnAStarTests
    {
        private const string SourceRelativePath = "BurstNav/Free/Runtime/Core/BnAStar.cs";
        private const string ProSymbol = "BURSTNAV_" + "PRO";

        private const int GridWidth = 6;
        private const int GridHeight = 4;
        private const int GridPolygonCount = GridWidth * GridHeight;

        private static readonly int[] WallPolygons = { 2, 8, 14 };

        private static readonly int[] FullWallPolygons = { 2, 8, 14, 20 };

        [Test]
        public void FindPath_ReturnsOptimalPath_OnFixedTestGraph()
        {
            using var graph = BuildGridGraph(GridWidth, GridHeight, WallPolygons);
            using var lease = ScratchpadLease.Create(GridPolygonCount);

            ref var blob = ref graph.Value;
            ref var scratch = ref lease.Value;
            var found = BnAStar.FindPath(ref blob, 0, 5, ref scratch, 512);

            Assert.IsTrue(found, "A* failed to route around the wall.");
            Assert.AreEqual(BnPathSearchStatus.Found, scratch.Status);
            AssertPathIsOptimal(ref blob, 0, 5, ref scratch);
            Assert.AreEqual(12, scratch.PathCount, "The shortest detour walks 12 polygons / 11 steps.");
            Assert.AreEqual(11f, scratch.PathCost, 1e-4f, "The optimal detour costs 11 unit steps.");
            Assert.Greater(scratch.Iterations, 0);

            var first = CopyPath(ref scratch);
            Assert.IsTrue(BnAStar.FindPath(ref blob, 0, 5, ref scratch, 512));
            CollectionAssert.AreEqual(first, CopyPath(ref scratch));
        }

        [Test]
        public void FindPath_UsesOptimalRoute_WhenACheaperDetourExists()
        {
            using var graph = BuildRouteGraph(new[] { 10f, 2f, 2f, 2f });
            using var lease = ScratchpadLease.Create(4);

            ref var blob = ref graph.Value;
            ref var scratch = ref lease.Value;
            Assert.IsTrue(BnAStar.FindPath(ref blob, 0, 3, ref scratch, 64));

            CollectionAssert.AreEqual(new[] { 0, 1, 2, 3 }, CopyPath(ref scratch));
            Assert.AreEqual(6f, scratch.PathCost, 1e-4f);
        }

        [Test]
        public void FindPath_RespectsIterationBudget()
        {
            using var graph = BuildGridGraph(GridWidth, GridHeight, WallPolygons);
            using var lease = ScratchpadLease.Create(GridPolygonCount);

            ref var blob = ref graph.Value;
            ref var scratch = ref lease.Value;

            Assert.IsFalse(BnAStar.FindPath(ref blob, 0, 5, ref scratch, 0), "A zero budget cannot expand anything.");
            Assert.AreEqual(BnPathSearchStatus.BudgetExhausted, scratch.Status);
            Assert.AreEqual(0, scratch.PathCount);
            Assert.AreEqual(0, scratch.Iterations);

            Assert.IsFalse(BnAStar.FindPath(ref blob, 0, 5, ref scratch, 3), "Three expansions cannot reach an 11-step route.");
            Assert.AreEqual(BnPathSearchStatus.BudgetExhausted, scratch.Status);
            Assert.AreEqual(0, scratch.PathCount);
            Assert.LessOrEqual(scratch.Iterations, 3, "The search must never expand more nodes than the budget allows.");

            Assert.IsTrue(BnAStar.FindPath(ref blob, 0, 5, ref scratch, 512));
            var required = scratch.Iterations;
            Assert.IsTrue(
                BnAStar.FindPath(ref blob, 0, 5, ref scratch, required),
                "The reported iteration count must be a sufficient budget.");
            Assert.LessOrEqual(required, GridPolygonCount, "A* never expands a polygon twice.");
        }

        [Test]
        public void FindPath_ReportsUnreachableTarget()
        {
            using var graph = BuildGridGraph(GridWidth, GridHeight, FullWallPolygons);
            using var lease = ScratchpadLease.Create(GridPolygonCount);

            ref var blob = ref graph.Value;
            ref var scratch = ref lease.Value;

            Assert.IsFalse(BnAStar.FindPath(ref blob, 0, 5, ref scratch, 4096), "The full wall disconnects the graph.");
            Assert.AreEqual(BnPathSearchStatus.Unreachable, scratch.Status);
            Assert.AreEqual(0, scratch.PathCount);
        }

        [Test]
        public void FindPath_HandlesStartEqualsTarget()
        {
            using var graph = BuildGridGraph(GridWidth, GridHeight, WallPolygons);
            using var lease = ScratchpadLease.Create(GridPolygonCount);

            ref var blob = ref graph.Value;
            ref var scratch = ref lease.Value;

            Assert.IsTrue(BnAStar.FindPath(ref blob, 7, 7, ref scratch, 0));
            Assert.AreEqual(BnPathSearchStatus.Found, scratch.Status);
            Assert.AreEqual(1, scratch.PathCount);
            Assert.AreEqual(7, scratch.GetPathPolygon(0));
            Assert.AreEqual(0f, scratch.PathCost);
        }

        [Test]
        public void FindPath_RejectsInvalidNodesAndUndersizedScratchpad()
        {
            using var graph = BuildGridGraph(GridWidth, GridHeight, WallPolygons);
            using var exact = ScratchpadLease.Create(GridPolygonCount);
            using var small = ScratchpadLease.Create(3);

            ref var blob = ref graph.Value;
            ref var exactPad = ref exact.Value;

            Assert.IsFalse(BnAStar.FindPath(ref blob, -1, 5, ref exactPad, 64));
            Assert.AreEqual(BnPathSearchStatus.InvalidInput, exactPad.Status);

            Assert.IsFalse(BnAStar.FindPath(ref blob, 0, GridPolygonCount, ref exactPad, 64));
            Assert.AreEqual(BnPathSearchStatus.InvalidInput, exactPad.Status);

            ref var smallPad = ref small.Value;
            Assert.IsFalse(BnAStar.FindPath(ref blob, 0, 5, ref smallPad, 64), "The scratchpad must be grown before searching.");
            Assert.AreEqual(BnPathSearchStatus.CapacityExceeded, smallPad.Status);
        }

        [Test]
        public void Scratchpad_GrowsOnDemand_AndIsReusableAcrossGraphs()
        {
            using var small = BuildRouteGraph(new[] { 10f, 2f, 2f, 2f });
            using var large = BuildGridGraph(GridWidth, GridHeight, WallPolygons);
            using var lease = ScratchpadLease.Create(1);

            ref var pad = ref lease.Value;
            ref var smallBlob = ref small.Value;

            Assert.IsFalse(BnAStar.FindPath(ref smallBlob, 0, 3, ref pad, 64));
            Assert.AreEqual(BnPathSearchStatus.CapacityExceeded, pad.Status);

            pad.EnsureCapacity(GridPolygonCount, Allocator.Persistent);
            Assert.AreEqual(GridPolygonCount, pad.Capacity);

            ref var largeBlob = ref large.Value;
            Assert.IsTrue(BnAStar.FindPath(ref largeBlob, 0, 5, ref pad, 512));
            Assert.IsTrue(BnAStar.FindPath(ref smallBlob, 0, 3, ref pad, 64), "The same scratchpad serves any graph that fits.");
            CollectionAssert.AreEqual(new[] { 0, 1, 2, 3 }, CopyPath(ref pad));
        }

        [Test]
        public void FindPath_DoesNotAllocateManagedMemory_AndReusesTheScratchpad()
        {
            using var graph = BuildGridGraph(GridWidth, GridHeight, WallPolygons);
            using var lease = ScratchpadLease.Create(GridPolygonCount);

            ref var blob = ref graph.Value;
            ref var scratch = ref lease.Value;

            for (var warmup = 0; warmup < 4; warmup++)
            {
                Assert.IsTrue(BnAStar.FindPath(ref blob, 0, 5, ref scratch, 512));
            }

            var capacityBefore = scratch.Capacity;
            var gScoreBefore = scratch.GScore;
            var pathBefore = scratch.Path;
            var heapBefore = scratch.HeapKeys;

            var allocatedBefore = GC.GetAllocatedBytesForCurrentThread();
            var found = false;
            for (var iteration = 0; iteration < 64; iteration++)
            {
                found = BnAStar.FindPath(ref blob, 0, 5, ref scratch, 512);
            }

            var allocatedAfter = GC.GetAllocatedBytesForCurrentThread();

            Assert.IsTrue(found);
            Assert.AreEqual(0, allocatedAfter - allocatedBefore, "FindPath allocated managed memory.");
            Assert.AreEqual(capacityBefore, scratch.Capacity, "A search must not re-allocate the scratchpad.");
            Assert.IsTrue(scratch.GScore.Equals(gScoreBefore), "The g-score buffer was replaced.");
            Assert.IsTrue(scratch.Path.Equals(pathBefore), "The path buffer was replaced.");
            Assert.IsTrue(scratch.HeapKeys.Equals(heapBefore), "The heap buffer was replaced.");
        }

        [Test]
        public void FindPath_RunsUnderBurst()
        {
            using var graph = BuildGridGraph(GridWidth, GridHeight, WallPolygons);
            using var lease = ScratchpadLease.Create(GridPolygonCount);
            using var result = new NativeArray<int>(3, Allocator.TempJob);

            new AStarJob
            {
                Graph = graph,
                Scratch = lease.Value,
                Start = 0,
                Target = 5,
                MaxIterations = 512,
                Result = result,
            }.Schedule().Complete();

            Assert.AreEqual(1, result[0], "Burst-compiled FindPath reported failure.");
            Assert.AreEqual(12, result[1], "path length");
            Assert.AreEqual(11000, result[2], "path cost * 1000");
        }

        [Test]
        public void FindPath_RunsUnderBurst_WhenBudgetIsExhausted()
        {
            using var graph = BuildGridGraph(GridWidth, GridHeight, WallPolygons);
            using var lease = ScratchpadLease.Create(GridPolygonCount);
            using var result = new NativeArray<int>(3, Allocator.TempJob);

            new AStarJob
            {
                Graph = graph,
                Scratch = lease.Value,
                Start = 0,
                Target = 5,
                MaxIterations = 2,
                Result = result,
            }.Schedule().Complete();

            Assert.AreEqual(0, result[0], "The budget must be honoured inside Burst as well.");
            Assert.AreEqual(0, result[1]);
        }

        [Test]
        public void Contract_FindPathHasTheAuthoritativeSignature()
        {
            var method = typeof(BnAStar)
                .GetMethods(BindingFlags.Public | BindingFlags.Static)
                .Single(candidate =>
                {
                    var parameters = candidate.GetParameters();
                    return candidate.Name == "FindPath" && parameters.Length == 5 &&
                           parameters[4].ParameterType == typeof(int);
                });

            Assert.IsNotNull(method, "BnAStar.FindPath is missing.");
            Assert.AreEqual(typeof(bool), method.ReturnType);
            CollectionAssert.AreEqual(
                new[]
                {
                    typeof(BnMeshGraphBlob).MakeByRefType(),
                    typeof(int),
                    typeof(int),
                    typeof(BnPathScratchpad).MakeByRefType(),
                    typeof(int),
                },
                method.GetParameters().Select(parameter => parameter.ParameterType).ToArray());
            Assert.IsNotNull(
                method.GetCustomAttribute<BurstCompileAttribute>(),
                "FindPath must be marked [BurstCompile].");

            var filtered = typeof(BnAStar)
                .GetMethods(BindingFlags.Public | BindingFlags.Static)
                .Single(candidate =>
                {
                    var parameters = candidate.GetParameters();
                    return candidate.Name == "FindPath" && parameters.Length == 6 &&
                           parameters[4].ParameterType == typeof(BnLinkFilter);
                });

            Assert.IsNotNull(filtered.GetCustomAttribute<BurstCompileAttribute>());
            Assert.AreEqual(typeof(BnLinkFilter), filtered.GetParameters()[4].ParameterType);
            Assert.AreEqual(typeof(int), filtered.GetParameters()[5].ParameterType);

            var sidecar = typeof(BnAStar)
                .GetMethods(BindingFlags.Public | BindingFlags.Static)
                .Single(candidate =>
                {
                    var parameters = candidate.GetParameters();
                    return candidate.Name == "FindPath" && parameters.Length == 7 &&
                           parameters[4].ParameterType == typeof(BnLinkFilter) &&
                           parameters[5].ParameterType == typeof(NativeArray<ushort>);
                });

            Assert.IsNotNull(sidecar.GetCustomAttribute<BurstCompileAttribute>());
            Assert.AreEqual(typeof(int), sidecar.GetParameters()[6].ParameterType);
        }

        [Test]
        public void Scratchpad_IsAValueTypeWithoutManagedReferences_AndOrdersFieldsBySizeDescending()
        {
            var type = typeof(BnPathScratchpad);

            Assert.IsTrue(type.IsValueType);
            Assert.IsFalse(
                type.GetFields(BindingFlags.Public | BindingFlags.NonPublic | BindingFlags.Instance)
                    .Any(field => !field.FieldType.IsValueType),
                "BnPathScratchpad must not contain managed references.");

            var fields = type.GetFields(BindingFlags.Public | BindingFlags.Instance);
            for (var index = 1; index < fields.Length; index++)
            {
                var previous = UnsafeUtility.SizeOf(fields[index - 1].FieldType);
                var current = UnsafeUtility.SizeOf(fields[index].FieldType);
                Assert.GreaterOrEqual(
                    previous,
                    current,
                    $"{type.Name}.{fields[index].Name} ({current} B) is larger than {fields[index - 1].Name} ({previous} B).");
            }
        }

        [Test]
        public void LinkFilter_PassThroughKeepsEveryLink()
        {
            Assert.IsTrue(BnLinkFilter.PassThrough.Allows(0));
            Assert.IsTrue(BnLinkFilter.PassThrough.Allows(0x4));
            Assert.IsTrue(BnLinkFilter.PassThrough.Allows(ushort.MaxValue));
        }

        [Test]
        public void LinkFilter_HandBuiltBlockingFilterHonoursTheSeam()
        {
            const ushort doorTag = 0x4;
            var blocking = new BnLinkFilter
            {
                BlockedTagMask = doorTag,
                IgnoreTagged = 0,
                AllowBlockedTags = 0,
            };

            Assert.IsTrue(blocking.Allows(0), "Untagged links stay traversable.");
            Assert.IsFalse(blocking.Allows(doorTag), "A blocked tag closes the link.");
            Assert.IsTrue(blocking.Allows(0x8), "Tags outside the mask stay open.");

            var allowing = new BnLinkFilter
            {
                BlockedTagMask = doorTag,
                IgnoreTagged = 0,
                AllowBlockedTags = 1,
            };
            Assert.IsTrue(allowing.Allows(doorTag));

            var ignoreTagged = new BnLinkFilter
            {
                BlockedTagMask = 0,
                IgnoreTagged = 1,
                AllowBlockedTags = 0,
            };
            Assert.IsFalse(ignoreTagged.Allows(doorTag));
            Assert.IsTrue(ignoreTagged.Allows(0));
        }

        [Test]
        public void FindPath_FilteredOverloadSeesTheSameGraphAsPassThroughInFree()
        {
            using var graph = BuildGridGraph(GridWidth, GridHeight, WallPolygons);
            using var lease = ScratchpadLease.Create(GridPolygonCount);

            ref var blob = ref graph.Value;
            ref var scratch = ref lease.Value;

            Assert.IsTrue(BnAStar.FindPath(ref blob, 0, 5, ref scratch, 512));
            var plainPath = CopyPath(ref scratch);
            var plainCost = scratch.PathCost;

            Assert.IsTrue(BnAStar.FindPath(ref blob, 0, 5, ref scratch, BnLinkFilter.PassThrough, 512));
            CollectionAssert.AreEqual(plainPath, CopyPath(ref scratch));
            Assert.AreEqual(plainCost, scratch.PathCost, 1e-5f);

            var blocking = new BnLinkFilter
            {
                BlockedTagMask = ushort.MaxValue,
                IgnoreTagged = 1,
                AllowBlockedTags = 0,
            };

            Assert.IsTrue(BnAStar.FindPath(ref blob, 0, 5, ref scratch, blocking, 512));
            CollectionAssert.AreEqual(plainPath, CopyPath(ref scratch));
            Assert.AreEqual(BnPathSearchStatus.Found, scratch.Status);
        }

        [Test]
        public void FindPath_SidecarMasksReachTheSharedFilter()
        {
            using var graph = BuildGridGraph(GridWidth, GridHeight, WallPolygons);
            using var lease = ScratchpadLease.Create(GridPolygonCount);
            var masks = new NativeArray<ushort>(graph.Value.LinkCount, Allocator.Temp);

            try
            {
                for (var index = 0; index < masks.Length; index++)
                {
                    masks[index] = 0x4;
                }

                ref var blob = ref graph.Value;
                ref var scratch = ref lease.Value;
                var blocking = new BnLinkFilter { BlockedTagMask = 0x4 };

                Assert.IsFalse(BnAStar.FindPath(ref blob, 0, 5, ref scratch, blocking, masks, 512));
                Assert.AreEqual(BnPathSearchStatus.Unreachable, scratch.Status);
                Assert.AreEqual(0, scratch.PathCount);

                var allowing = new BnLinkFilter { BlockedTagMask = 0x4, AllowBlockedTags = 1 };
                Assert.IsTrue(BnAStar.FindPath(ref blob, 0, 5, ref scratch, allowing, masks, 512));
                Assert.AreEqual(12, scratch.PathCount, "The same relaxation serves a sidecar-enabled search.");
            }
            finally
            {
                masks.Dispose();
            }
        }

        [Test]
        public void FindPath_FilteredOverloadKeepsTheDiagnosticsContract()
        {
            using var graph = BuildGridGraph(GridWidth, GridHeight, WallPolygons);
            using var lease = ScratchpadLease.Create(GridPolygonCount);

            ref var blob = ref graph.Value;
            ref var scratch = ref lease.Value;

            Assert.IsFalse(BnAStar.FindPath(ref blob, 0, 5, ref scratch, BnLinkFilter.PassThrough, 0));
            Assert.AreEqual(BnPathSearchStatus.BudgetExhausted, scratch.Status);
            Assert.AreEqual(0, scratch.PathCount);

            Assert.IsFalse(BnAStar.FindPath(ref blob, -1, 5, ref scratch, BnLinkFilter.PassThrough, 8));
            Assert.AreEqual(BnPathSearchStatus.InvalidInput, scratch.Status);
        }

        [Test]
        public void Source_SearchUsesOnlyUnityMathematics()
        {
            var source = ReadSource();

            foreach (var managed in new[] { "Mathf.", "Vector2", "Vector3", "UnityEngine", "Math.", "System.Math" })
            {
                Assert.IsFalse(source.Contains(managed), "BnAStar.cs must not use " + managed);
            }

            StringAssert.Contains("using Unity.Mathematics;", source);
            StringAssert.Contains("math.select", source);
            StringAssert.Contains("math.mad", source);
        }

        [Test]
        public void Source_HotLoopBodiesAreBranchFree()
        {
            var source = ReadSource();

            foreach (var method in new[] { "RelaxNeighbours", "SiftUp", "SiftDown", "PopMin" })
            {
                var body = ExtractMethodBody(source, method);
                Assert.IsFalse(Regex.IsMatch(body, @"\bif\s*\("), method + " must not branch on data.");
                Assert.IsFalse(Regex.IsMatch(body, @"\bswitch\s*\("), method + " must not switch.");
                Assert.IsFalse(Regex.IsMatch(body, @"\belse\b"), method + " must not use else.");
                Assert.IsFalse(Regex.IsMatch(body, @"\?\s*\w+\s*:"), method + " must not use a ternary.");
            }

            var relax = ExtractMethodBody(source, "RelaxNeighbours");
            StringAssert.Contains("math.select", relax);
            StringAssert.Contains("filter.Allows", relax, "the portal filter is folded into the same loop.");
            Assert.IsFalse(relax.Contains("continue"), "Relaxation must not skip links with control flow.");
            Assert.GreaterOrEqual(
                Regex.Matches(relax, "math.select").Count,
                6,
                "Relaxation must select every decision instead of branching.");
        }

        [Test]
        public void Source_SearchNeverAllocates()
        {
            var source = ReadSource();
            var findPath = ExtractMethodBody(source, "FindPathCore");

            foreach (var allocation in new[] { "new NativeArray", "Allocator.", "new ", "GC.", "ToArray", "List<" })
            {
                Assert.IsFalse(findPath.Contains(allocation), "FindPath must not allocate (" + allocation + ").");
            }

            var overloads = ExtractMethodBodies(source, "FindPath");
            Assert.AreEqual(3, overloads.Length, "BnAStar exposes the plain, filtered and sidecar-filtered overloads.");
            StringAssert.Contains("BnLinkFilter.PassThrough", overloads[0], "the plain overload delegates with PassThrough.");
            StringAssert.Contains("FindPathCore", overloads[1], "the filtered overload runs the shared search.");
            StringAssert.Contains("FindPathCore", overloads[2], "the sidecar overload runs the shared search.");

            StringAssert.Contains("EnsureCapacity", source, "Allocation belongs to the growth API, not the search.");
        }

        [Test]
        public void Source_StaysTierAgnostic()
        {
            var source = ReadSource();

            Assert.IsFalse(source.Contains(ProSymbol), "BnAStar.cs must not carry Pro-only regions.");
            Assert.IsFalse(
                Regex.IsMatch(source, @"^\s*#\s*(?:if|elif|else|endif)\b", RegexOptions.Multiline),
                "BnAStar.cs must not use conditionals.");
            Assert.AreEqual(
                1,
                Regex.Matches(source, @"public\s+(?:unsafe\s+)?struct BnLinkFilter\b").Count,
                "BnLinkFilter is declared once.");
            Assert.IsFalse(source.Contains("partial struct BnLinkFilter"), "the filter is not a tier-gated extension.");
            Assert.IsFalse(source.Contains("FromFlags"), "the filter is built directly from field values.");
        }

        private static string ReadSource() => File.ReadAllText(AssetPath(SourceRelativePath));

        private static string[] ExtractMethodBodies(string source, string methodName)
        {
            var declaration = new Regex(
                @"(?:public|private|internal)\s+static\s+[^\s]+\s+" + Regex.Escape(methodName) + @"\s*\(");
            var matches = declaration.Matches(source);
            Assert.Greater(matches.Count, 0, methodName + " declaration not found in BnAStar.cs.");

            var bodies = new string[matches.Count];
            for (var match = 0; match < matches.Count; match++)
            {
                var open = source.IndexOf('{', matches[match].Index + matches[match].Length);
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
                            bodies[match] = source.Substring(open, index - open + 1);
                            break;
                        }
                    }
                }

                Assert.IsNotNull(bodies[match], methodName + " body is not balanced.");
            }

            return bodies;
        }

        private static string ExtractMethodBody(string source, string methodName)
        {
            var declaration = new Regex(
                @"(?:public|private|internal)\s+static\s+[^\s]+\s+" + Regex.Escape(methodName) + @"\s*\(");
            var match = declaration.Match(source);
            Assert.IsTrue(match.Success, methodName + " declaration not found in BnAStar.cs.");

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

        private static int[] CopyPath(ref BnPathScratchpad scratch)
        {
            var path = new int[scratch.PathCount];
            for (var index = 0; index < path.Length; index++)
            {
                path[index] = scratch.GetPathPolygon(index);
            }

            return path;
        }

        private static void AssertPathIsOptimal(
            ref BnMeshGraphBlob graph,
            int start,
            int target,
            ref BnPathScratchpad scratch)
        {
            Assert.AreEqual(start, scratch.GetPathPolygon(0), "The path must start at the start polygon.");
            Assert.AreEqual(
                target,
                scratch.GetPathPolygon(scratch.PathCount - 1),
                "The path must end at the target polygon.");

            var walked = 0f;
            for (var index = 1; index < scratch.PathCount; index++)
            {
                var from = scratch.GetPathPolygon(index - 1);
                var to = scratch.GetPathPolygon(index);

                var linked = false;
                var stepCost = 0f;
                for (var link = 0; link < graph.GetPolyLinkCount(from); link++)
                {
                    var data = graph.GetPolyLink(from, link);
                    if (data.Target != to)
                    {
                        continue;
                    }

                    linked = true;
                    stepCost = data.Cost > 0f
                        ? data.Cost
                        : math.distance(graph.GetPolygonCenter(from), graph.GetPolygonCenter(to));
                    break;
                }

                Assert.IsTrue(linked, $"Path step {from} -> {to} is not a graph link.");
                walked += stepCost;
            }

            Assert.AreEqual(walked, scratch.PathCost, 1e-3f, "PathCost must be the sum of the traversed links.");
            Assert.AreEqual(
                ReferenceOptimalCost(ref graph, start, target),
                scratch.PathCost,
                1e-3f,
                "A* must return the optimal cost.");
        }

        private static float ReferenceOptimalCost(ref BnMeshGraphBlob graph, int start, int target)
        {
            var count = graph.PolygonCount;
            var distance = new float[count];
            var done = new bool[count];

            for (var polygon = 0; polygon < count; polygon++)
            {
                distance[polygon] = float.PositiveInfinity;
            }

            distance[start] = 0f;

            for (var iteration = 0; iteration < count; iteration++)
            {
                var best = -1;
                var bestDistance = float.PositiveInfinity;
                for (var polygon = 0; polygon < count; polygon++)
                {
                    if (!done[polygon] && distance[polygon] < bestDistance)
                    {
                        best = polygon;
                        bestDistance = distance[polygon];
                    }
                }

                if (best < 0)
                {
                    break;
                }

                done[best] = true;
                if (best == target)
                {
                    return distance[best];
                }

                for (var link = 0; link < graph.GetPolyLinkCount(best); link++)
                {
                    var data = graph.GetPolyLink(best, link);
                    var stepCost = data.Cost > 0f
                        ? data.Cost
                        : math.distance(graph.GetPolygonCenter(best), graph.GetPolygonCenter(data.Target));
                    var candidate = distance[best] + stepCost;
                    if (candidate < distance[data.Target])
                    {
                        distance[data.Target] = candidate;
                    }
                }
            }

            return distance[target];
        }

        private static BlobAssetReference<BnMeshGraphBlob> BuildRouteGraph(float[] costs)
        {
            var centers = new[]
            {
                new float2(0f, 0f),
                new float2(1f, 1f),
                new float2(2f, 1f),
                new float2(3f, 0f),
            };

            var edges = new[]
            {
                (From: 0, To: 3, Cost: costs[0]),
                (From: 0, To: 1, Cost: costs[1]),
                (From: 1, To: 2, Cost: costs[2]),
                (From: 2, To: 3, Cost: costs[3]),
            };

            var outgoing = new List<(int From, int To, float Cost)>();
            foreach (var edge in edges)
            {
                outgoing.Add(edge);
                outgoing.Add((edge.To, edge.From, edge.Cost));
            }

            return BuildGraph(centers, outgoing);
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

        private static string AssetPath(string relative) =>
            Path.Combine(Application.dataPath, relative.Replace('/', Path.DirectorySeparatorChar));

        private sealed class ScratchpadLease : IDisposable
        {
            public BnPathScratchpad Value;

            public static ScratchpadLease Create(int polygonCapacity) => new ScratchpadLease
            {
                Value = BnPathScratchpad.Create(polygonCapacity, Allocator.Persistent),
            };

            public void Dispose() => Value.Dispose();
        }

        [BurstCompile(CompileSynchronously = true)]
        private struct AStarJob : IJob
        {
            public BlobAssetReference<BnMeshGraphBlob> Graph;
            public BnPathScratchpad Scratch;
            public int Start;
            public int Target;
            public int MaxIterations;
            public NativeArray<int> Result;

            public void Execute()
            {
                ref var graph = ref Graph.Value;
                var scratch = Scratch;
                var found = BnAStar.FindPath(ref graph, Start, Target, ref scratch, MaxIterations);

                Result[0] = math.select(0, 1, found);
                Result[1] = scratch.PathCount;
                Result[2] = (int)math.round(scratch.PathCost * 1000f);
            }
        }
    }
}
