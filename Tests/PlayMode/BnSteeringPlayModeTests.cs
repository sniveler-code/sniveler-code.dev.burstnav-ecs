using BurstNav;
using NUnit.Framework;
using Unity.Collections;
using Unity.Core;
using Unity.Entities;
using Unity.Mathematics;

namespace BurstNav.Tests
{
    [Category("Smoke")]
    public class BnSteeringPlayModeTests
    {
        private const float DeltaTime = 0.5f;

        [Test]
        public void System_RunsFunnelBypassIntegrateInOneUpdate()
        {
            using var graph = BuildCorridorGraph(5);
            using var world = CreateWorld(graph);
            var entityManager = world.EntityManager;
            var commandBuffers = world.GetOrCreateSystemManaged<EndSimulationEntityCommandBufferSystem>();
            var steeringSystem = world.GetOrCreateSystemManaged<BnSteeringSystem>();

            var agent = CreateAgent(entityManager, float3.zero, 2f, 0.2f, 5);

            steeringSystem.Update();
            entityManager.CompleteAllTrackedJobs();
            commandBuffers.Update();

            var state = entityManager.GetComponentData<BnAgentSteering>(agent);

            Assert.AreEqual(2f, state.DesiredVelocity.x, 1e-4f, "The funnel did not produce a desired velocity.");
            Assert.AreEqual(0f, state.DesiredVelocity.y, 1e-4f);
            Assert.AreEqual(0f, state.DesiredVelocity.z, 1e-6f, "Steering is planar: XY navigated, Z is height.");
            Assert.AreEqual(2f, state.Velocity.x, 1e-4f, "The kinematic bypass did not copy desired velocity.");
            Assert.AreEqual(1f, state.Position.x, 1e-4f, "Position must advance by velocity × delta time.");
            Assert.AreEqual(0f, state.Position.z, 1e-6f);
        }

        [Test]
        public void System_KeepsSteppingAlongThePathAndStopsOnArrival()
        {
            using var graph = BuildCorridorGraph(5);
            using var world = CreateWorld(graph);
            var entityManager = world.EntityManager;
            var commandBuffers = world.GetOrCreateSystemManaged<EndSimulationEntityCommandBufferSystem>();
            var steeringSystem = world.GetOrCreateSystemManaged<BnSteeringSystem>();

            var agent = CreateAgent(entityManager, float3.zero, 2f, 0.2f, 5);

            for (var frame = 1; frame <= 4; frame++)
            {
                steeringSystem.Update();
                entityManager.CompleteAllTrackedJobs();
                commandBuffers.Update();

                var state = entityManager.GetComponentData<BnAgentSteering>(agent);
                Assert.AreEqual(frame, state.Position.x, 1e-4f, "Wrong distance after frame " + frame + ".");
                Assert.AreEqual(2f, state.Velocity.x, 1e-4f);
            }

            steeringSystem.Update();
            entityManager.CompleteAllTrackedJobs();
            commandBuffers.Update();

            var arrived = entityManager.GetComponentData<BnAgentSteering>(agent);
            Assert.AreEqual(4f, arrived.Position.x, 1e-4f);
            Assert.AreEqual(0f, math.length(arrived.DesiredVelocity), 1e-5f);
            Assert.AreEqual(0f, math.length(arrived.Velocity), 1e-5f);
            Assert.IsTrue((arrived.Flags & BnAgentFlags.Arrived) != 0, "The funnel must flag arrival.");
        }

        [Test]
        public void System_DeactivatesIdleAgentsThroughDeferredPlayback()
        {
            using var graph = BuildCorridorGraph(5);
            using var world = CreateWorld(graph);
            var entityManager = world.EntityManager;
            var commandBuffers = world.GetOrCreateSystemManaged<EndSimulationEntityCommandBufferSystem>();
            var steeringSystem = world.GetOrCreateSystemManaged<BnSteeringSystem>();

            var agent = CreateAgent(entityManager, float3.zero, 2f, 0.2f, 0);
            Assert.IsTrue(entityManager.IsComponentEnabled<BnAgentSteeringActive>(agent));

            steeringSystem.Update();
            entityManager.CompleteAllTrackedJobs();

            Assert.IsTrue(
                entityManager.IsComponentEnabled<BnAgentSteeringActive>(agent),
                "Enableable-component flips must be deferred: the job may not mutate chunk state.");

            commandBuffers.Update();
            Assert.IsFalse(
                entityManager.IsComponentEnabled<BnAgentSteeringActive>(agent),
                "An agent without a followable path must leave the steering chain.");

            var idle = entityManager.GetComponentData<BnAgentSteering>(agent);
            Assert.AreEqual(float3.zero, idle.Position, "An idle agent must not move.");
            Assert.AreEqual(0f, math.length(idle.Velocity), 1e-6f);
        }

        [Test]
        public void System_ReactivatesAnAgentWhenAPathIsAssigned()
        {
            using var graph = BuildCorridorGraph(5);
            using var world = CreateWorld(graph);
            var entityManager = world.EntityManager;
            var commandBuffers = world.GetOrCreateSystemManaged<EndSimulationEntityCommandBufferSystem>();
            var steeringSystem = world.GetOrCreateSystemManaged<BnSteeringSystem>();

            var agent = CreateAgent(entityManager, float3.zero, 2f, 0.2f, 0);

            steeringSystem.Update();
            entityManager.CompleteAllTrackedJobs();
            commandBuffers.Update();
            Assert.IsFalse(entityManager.IsComponentEnabled<BnAgentSteeringActive>(agent));

            WritePath(entityManager, agent, 5);

            steeringSystem.Update();
            entityManager.CompleteAllTrackedJobs();
            commandBuffers.Update();

            Assert.IsTrue(
                entityManager.IsComponentEnabled<BnAgentSteeringActive>(agent),
                "A followable path must bring the agent back into the steering chain.");

            var reactivated = entityManager.GetComponentData<BnAgentSteering>(agent);
            Assert.AreEqual(2f, reactivated.DesiredVelocity.x, 1e-4f, "The funnel must already steer the agent.");
            Assert.AreEqual(0f, reactivated.Position.x, 1e-5f, "Reactivation lands in the next frame's stages.");

            steeringSystem.Update();
            entityManager.CompleteAllTrackedJobs();
            commandBuffers.Update();

            var moving = entityManager.GetComponentData<BnAgentSteering>(agent);
            Assert.AreEqual(2f, moving.Velocity.x, 1e-4f);
            Assert.AreEqual(1f, moving.Position.x, 1e-4f);
        }


        [Test]
        public void System_DoesNothingWithoutANavigationGraph()
        {
            using var world = new World("BnSteeringNoGraph");
            var entityManager = world.EntityManager;
            var commandBuffers = world.GetOrCreateSystemManaged<EndSimulationEntityCommandBufferSystem>();
            var steeringSystem = world.GetOrCreateSystemManaged<BnSteeringSystem>();

            var agent = CreateAgent(entityManager, float3.zero, 2f, 0.2f, 5);

            Assert.DoesNotThrow(() => steeringSystem.Update());
            entityManager.CompleteAllTrackedJobs();
            commandBuffers.Update();

            var state = entityManager.GetComponentData<BnAgentSteering>(agent);
            Assert.AreEqual(float3.zero, state.Position, "Without topology the steering loop must not run.");
            Assert.AreEqual(0f, math.length(state.Velocity), 1e-6f);
            Assert.IsTrue(
                entityManager.IsComponentEnabled<BnAgentSteeringActive>(agent),
                "A missing graph must not disable the agent.");
        }

        [Test]
        public void System_WithoutAgents_DoesNotThrow()
        {
            using var graph = BuildCorridorGraph(5);
            using var world = CreateWorld(graph);
            var steeringSystem = world.GetOrCreateSystemManaged<BnSteeringSystem>();

            Assert.DoesNotThrow(() => steeringSystem.Update());
            world.EntityManager.CompleteAllTrackedJobs();
        }

        private static World CreateWorld(BlobAssetReference<BnMeshGraphBlob> graph)
        {
            var world = new World("BnSteeringPlayMode");
            world.SetTime(new TimeData(DeltaTime, DeltaTime));

            var graphEntity = world.EntityManager.CreateEntity(typeof(BnBlobMeshGraph));
            world.EntityManager.SetComponentData(graphEntity, new BnBlobMeshGraph { Value = graph });
            return world;
        }

        private static Entity CreateAgent(
            EntityManager entityManager,
            float3 position,
            float maxSpeed,
            float radius,
            int waypointCount)
        {
            var entity = entityManager.CreateEntity(BnAgentComponents.CreateAgentArchetype(entityManager));
            entityManager.SetComponentData(entity, BnAgentSteering.Idle(position, maxSpeed, radius));
            WritePath(entityManager, entity, waypointCount);
            return entity;
        }

        private static void WritePath(EntityManager entityManager, Entity agent, int waypointCount)
        {
            var waypoints = entityManager.GetBuffer<BnPathWaypoint>(agent);
            waypoints.Clear();

            for (var index = 0; index < waypointCount; index++)
            {
                var link = index < waypointCount - 1 ? index : -1;
                waypoints.Add(BnPathWaypoint.Create(new float3(index, 0f, 0f), float3.zero, index, link));
            }

            entityManager.SetComponentData(agent, new BnAgentPath
            {
                PathIndex = 0,
                WaypointCount = waypointCount,
                Status = waypointCount > 0 ? BnPathStatus.Ready : BnPathStatus.None,
            });
        }

        private static BlobAssetReference<BnMeshGraphBlob> BuildCorridorGraph(int waypointCount)
        {
            var linkCount = math.max(waypointCount - 1, 0);
            var builder = new BlobBuilder(Allocator.Temp);
            try
            {
                ref var root = ref builder.ConstructRoot<BnMeshGraphBlob>();
                var links = builder.Allocate(ref root.Links, linkCount);

                for (var index = 0; index < linkCount; index++)
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

                return builder.CreateBlobAssetReference<BnMeshGraphBlob>(Allocator.Persistent);
            }
            finally
            {
                builder.Dispose();
            }
        }
    }
}
