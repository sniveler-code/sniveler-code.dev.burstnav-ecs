using System;
using System.IO;
using System.Linq;
using System.Reflection;
using System.Text.RegularExpressions;
using BurstNav;
using NUnit.Framework;
using Unity.Burst;
using Unity.Burst.Intrinsics;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;
using UnityEngine;

namespace BurstNav.Tests
{
    public class BnSteeringSystemTests
    {
        private const string SourceRelativePath = "Runtime/Systems/BnSteeringSystem.cs";

        private static readonly Type[] SteeringJobTypes =
        {
            typeof(BnFunnelJob),
            typeof(BnSteeringStateJob),
            typeof(KinematicBypassJob),
            typeof(IntegrateJob),
        };

        [Test]
        public void Source_LivesInSystemsFolderAndSchedulesTheFreeChain()
        {
            var path = AssetPath(SourceRelativePath);
            Assert.IsTrue(File.Exists(path), "Missing " + SourceRelativePath);
            Assert.IsTrue(File.Exists(path + ".meta"), "Every runtime asset must ship with its .meta (GUID stability).");

            var source = File.ReadAllText(path);

            Assert.IsFalse(Regex.IsMatch(source, @"^\s*#\s*(?:if|elif)", RegexOptions.Multiline), "The Free system must not use tier conditionals.");

            StringAssert.Contains("new KinematicBypassJob", source);
            StringAssert.Contains("BnFunnel.Solve", source);
            StringAssert.Contains("IntegrateJob", source);
            StringAssert.Contains("IJobChunk", source);
            StringAssert.Contains("[BurstCompile]", source);
            StringAssert.Contains("EntityCommandBuffer", source);
            StringAssert.Contains("SetComponentEnabled", source);
            StringAssert.Contains("CombineDependencies", source);
            StringAssert.Contains("ScheduleParallel", source);
            StringAssert.Contains("AddJobHandleForProducer", source);
        }

        [Test]
        public void Source_BranchRunsFunnelThenBypassThenIntegrate()
        {
            var source = File.ReadAllText(AssetPath(SourceRelativePath));
            var updateStart = source.IndexOf("protected override void OnUpdate", StringComparison.Ordinal);
            Assert.GreaterOrEqual(updateStart, 0, "The steering system must drive the chain from OnUpdate.");

            var update = ExtractMethodBody(source.Substring(updateStart), "OnUpdate");
            var funnel = update.IndexOf("BnFunnelJob", StringComparison.Ordinal);
            var bypass = update.IndexOf("new KinematicBypassJob", StringComparison.Ordinal);
            var integrate = update.IndexOf("new IntegrateJob", StringComparison.Ordinal);

            Assert.GreaterOrEqual(funnel, 0, "The chain must keep the funnel stage.");
            Assert.Greater(bypass, funnel, "The bypass must consume the funnel output.");
            Assert.Greater(integrate, bypass, "Integration must consume the bypass handle.");
            Assert.IsFalse(update.Contains(".Complete("), "OnUpdate must never complete a job handle.");
            StringAssert.Contains(
                "JobHandle.CombineDependencies(funnelHandle, stateHandle)",
                update,
                "The bypass must fan in the funnel and steering-state passes.");
        }

        [Test]
        public void Source_FreeSystemOwnsNoNativeBuffers()
        {
            var source = File.ReadAllText(AssetPath(SourceRelativePath));

            foreach (var forbidden in new[] { "Allocator.", "new NativeArray", "Dispose()", "OnDestroy" })
            {
                Assert.IsFalse(source.Contains(forbidden), SourceRelativePath + " must stay allocation-free: " + forbidden);
            }
        }

        [Test]
        public void Source_UpdateDefersMutationsAndNeverBlocksOnAJob()
        {
            var body = ExtractMethodBody(File.ReadAllText(AssetPath(SourceRelativePath)), "OnUpdate");

            foreach (var blocking in new[]
                     {
                         ".Complete(", ".Run(", "EntityManager.", "GetComponentData",
                         "SetComponentData", "GetBuffer<", "new NativeArray", "Allocator.",
                     })
            {
                Assert.IsFalse(body.Contains(blocking), "OnUpdate must not use " + blocking);
            }

            Assert.IsTrue(
                Regex.IsMatch(body, @"Dependency\s*=\s*new\s+IntegrateJob"),
                "Integration must be chained onto the system dependency chain.");
            Assert.IsTrue(
                Regex.IsMatch(body, @"JobHandle\.CombineDependencies\s*\(\s*funnelHandle\s*,\s*stateHandle\s*\)"),
                "The bypass must fan in the funnel and steering-state passes.");
            Assert.IsTrue(
                Regex.IsMatch(body, @"AddJobHandleForProducer\s*\(\s*Dependency\s*\)"),
                "The command buffer system must wait for the final job before playback.");
        }

        [Test]
        public void Jobs_AreBurstCompiledIJobChunkStages()
        {
            foreach (var type in SteeringJobTypes)
            {
                Assert.IsTrue(type.IsValueType, type.Name + " must be a struct.");
                Assert.IsTrue(typeof(IJobChunk).IsAssignableFrom(type), type.Name + " must implement IJobChunk.");
                Assert.IsNotNull(type.GetCustomAttribute<BurstCompileAttribute>(), type.Name + " must be [BurstCompile].");
            }

            Assert.AreEqual(
                typeof(BlobAssetReference<BnMeshGraphBlob>),
                typeof(BnFunnelJob).GetField("Graph").FieldType,
                "The funnel must consume the graph blob directly.");
            Assert.AreEqual(typeof(float), typeof(BnFunnelJob).GetField("DeltaTime").FieldType);

            Assert.AreEqual(
                typeof(ComponentTypeHandle<BnAgentSteering>),
                typeof(KinematicBypassJob).GetField("SteeringHandle").FieldType);
            Assert.AreEqual(
                typeof(ComponentTypeHandle<BnAgentSteering>),
                typeof(IntegrateJob).GetField("SteeringHandle").FieldType);
            Assert.AreEqual(typeof(float), typeof(IntegrateJob).GetField("DeltaTime").FieldType);

            Assert.AreEqual(
                typeof(EntityCommandBuffer.ParallelWriter),
                typeof(BnSteeringStateJob).GetField("Commands").FieldType,
                "Deferred enableable-component flips must go through a parallel command buffer.");
        }

        [Test]
        public void JobStructs_StoreOnlyBlittableFields()
        {
            foreach (var type in SteeringJobTypes)
            {
                foreach (var field in type.GetFields(BindingFlags.Public | BindingFlags.Instance))
                {
                    Assert.IsTrue(
                        field.FieldType.IsValueType,
                        type.Name + "." + field.Name + " must be a value type to stay Burst-compatible.");
                    Assert.IsFalse(
                        field.FieldType == typeof(string),
                        type.Name + "." + field.Name + " must not be a string.");
                }
            }
        }

        [Test]
        public void System_UpdatesBeforeCommandBufferPlaybackInsideSimulationGroup()
        {
            var type = typeof(BnSteeringSystem);

            Assert.IsTrue(typeof(SystemBase).IsAssignableFrom(type), "The steering system must be a SystemBase.");

            var group = type.GetCustomAttribute<UpdateInGroupAttribute>();
            Assert.IsNotNull(group);
            Assert.AreEqual(typeof(SimulationSystemGroup), group.GroupType);

            var before = type.GetCustomAttribute<UpdateBeforeAttribute>();
            Assert.IsNotNull(before, "The system must run before the deferred command buffer playback.");
            Assert.AreEqual(typeof(EndSimulationEntityCommandBufferSystem), before.SystemType);
        }

        [Test]
        public void Bypass_CopiesDesiredVelocityIntoVelocity()
        {
            var state = BnAgentSteering.Idle(new float3(1f, 2f, 3f), 6f, 0.3f);
            state.DesiredVelocity = new float3(2f, -4f, 0f);

            var result = KinematicBypassJob.Bypass(state);

            Assert.AreEqual(state.DesiredVelocity, result.Velocity, "Free avoidance is a pure kinematic bypass.");
            Assert.AreEqual(state.Position, result.Position, "The bypass must not move the agent.");
        }

        [Test]
        public void Integrate_AdvancesPositionByVelocityTimesDeltaTime()
        {
            var state = BnAgentSteering.Idle(new float3(1f, 2f, 3f), 6f, 0.3f);
            state.Velocity = new float3(2f, -4f, 0.5f);

            var result = IntegrateJob.Integrate(state, 0.5f);

            Assert.AreEqual(2f, result.Position.x, 1e-5f);
            Assert.AreEqual(0f, result.Position.y, 1e-5f);
            Assert.AreEqual(3.25f, result.Position.z, 1e-5f);
            Assert.AreEqual(state.Velocity, result.Velocity, "Integration must not change velocity.");
        }

        [Test]
        public void Integrate_WithZeroDeltaTime_LeavesPositionUntouched()
        {
            var state = BnAgentSteering.Idle(new float3(4f, -1f, 2f), 6f, 0.3f);
            state.Velocity = new float3(100f, 100f, 100f);

            var result = IntegrateJob.Integrate(state, 0f);

            Assert.AreEqual(state.Position, result.Position);
        }

        [Test]
        public void SteeringState_IsSteerableOnlyForFollowablePaths()
        {
            Assert.IsTrue(BnSteeringStateJob.IsSteerable(MakePath(2, BnPathStatus.Ready)));
            Assert.IsTrue(BnSteeringStateJob.IsSteerable(MakePath(1, BnPathStatus.Partial)));
            Assert.IsFalse(BnSteeringStateJob.IsSteerable(MakePath(2, BnPathStatus.Pending)));
            Assert.IsFalse(BnSteeringStateJob.IsSteerable(MakePath(2, BnPathStatus.Failed)));
            Assert.IsFalse(BnSteeringStateJob.IsSteerable(MakePath(0, BnPathStatus.Ready)));
            Assert.IsFalse(BnSteeringStateJob.IsSteerable(default));
        }

        [Test]
        public void Stages_CompileUnderBurst()
        {
            var integrate = BurstCompiler.CompileFunctionPointer<SteeringStageDelegate>(IntegrateProbe);
            Assert.IsTrue(integrate.IsCreated);

            var state = BnAgentSteering.Idle(new float3(1f, 1f, 0f), 4f, 0.2f);
            state.Velocity = new float3(3f, 0f, 0f);

            var result = integrate.Invoke(state, 0.5f);
            Assert.AreEqual(3f, result.Velocity.x, 1e-5f, "Burst-compiled steering did not keep the resolved velocity.");
            Assert.AreEqual(2.5f, result.Position.x, 1e-5f, "Burst-compiled integration did not advance the position.");
        }

        private delegate BnAgentSteering SteeringStageDelegate(BnAgentSteering state, float deltaTime);

        [BurstCompile(CompileSynchronously = true)]
        private static BnAgentSteering IntegrateProbe(BnAgentSteering state, float deltaTime) =>
            IntegrateJob.Integrate(state, deltaTime);

        private static BnAgentPath MakePath(int waypointCount, BnPathStatus status) => new BnAgentPath
        {
            PathIndex = 0,
            WaypointCount = waypointCount,
            Status = status,
        };

        private static string AssetPath(string relative) => BnTestPaths.Package(relative);

        private static string ExtractMethodBody(string source, string methodName)
        {
            var declaration = new Regex(
                @"(?:public|private|internal|protected)\s+(?:override\s+|static\s+|virtual\s+)*[\w<>\[\],\s\.]+\s+" +
                Regex.Escape(methodName) + @"\s*\(");
            var match = declaration.Match(source);
            Assert.IsTrue(match.Success, methodName + " declaration not found in " + SourceRelativePath + ".");

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
    }
}
