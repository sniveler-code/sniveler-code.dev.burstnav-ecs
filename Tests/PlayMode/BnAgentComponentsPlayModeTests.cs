using BurstNav;
using NUnit.Framework;
using Unity.Burst;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;

namespace BurstNav.Tests
{
    [Category("Smoke")]
    public class BnAgentComponentsPlayModeTests
    {
        [BurstCompile(CompileSynchronously = true)]
        private static float StepSteering(BnAgentSteering steering, float deltaTime)
        {
            var moving = math.lengthsq(steering.DesiredVelocity) > 0f;
            var velocity = math.select(steering.Velocity, steering.DesiredVelocity, moving);
            var position = steering.Position + velocity * deltaTime;
            var progress = math.mad(math.length(velocity), deltaTime, steering.PathProgress);
            var normalised = math.select(0f, math.saturate(progress * math.rcp(steering.PathLength)), steering.PathLength > 0f);
            return math.length(position) + normalised;
        }

        private delegate float StepSteeringDelegate(BnAgentSteering steering, float deltaTime);

        [Test]
        public void SteeringState_CompilesUnderBurst()
        {
            var function = BurstCompiler.CompileFunctionPointer<StepSteeringDelegate>(StepSteering);
            Assert.IsTrue(function.IsCreated);

            var steering = BnAgentSteering.Idle(new float3(1f, 0f, 2f), 5f, 0.5f);
            steering.DesiredVelocity = new float3(0f, 0f, 3f);
            steering.PathLength = 10f;

            Assert.AreEqual(3.6400549f + 0.15f, function.Invoke(steering, 0.5f), 1e-4f);
        }

        [Test]
        public void AgentArchetype_StaysSingleAcrossToggleFlips()
        {
            using (var world = new World("BnAgentComponentsPlayModeTests"))
            {
                var entityManager = world.EntityManager;
                var entities = new NativeArray<Entity>(512, Allocator.Temp);
                try
                {
                    entityManager.CreateEntity(BnAgentComponents.CreateAgentArchetype(entityManager), entities);
                    var archetype = entityManager.GetChunk(entities[0]).Archetype;

                    entityManager.SetComponentEnabled<BnAgentSteeringActive>(entities[17], false);
                    Assert.IsFalse(entityManager.IsComponentEnabled<BnAgentSteeringActive>(entities[17]));
                    Assert.AreEqual(archetype, entityManager.GetChunk(entities[17]).Archetype);

                    var query = entityManager.CreateEntityQuery(ComponentType.ReadWrite<BnAgentSteering>());
                    try
                    {
                        Assert.AreEqual(entities.Length, query.CalculateEntityCount());
                    }
                    finally
                    {
                        query.Dispose();
                    }
                }
                finally
                {
                    entities.Dispose();
                }
            }
        }

        [Test]
        public void SteeringJob_RunsOverAgentArchetype()
        {
            using (var world = new World("BnAgentComponentsPlayModeTests"))
            {
                var entityManager = world.EntityManager;
                var entities = new NativeArray<Entity>(512, Allocator.TempJob);
                var steeringArray = new NativeArray<BnAgentSteering>(512, Allocator.TempJob);
                try
                {
                    entityManager.CreateEntity(BnAgentComponents.CreateAgentArchetype(entityManager), entities);
                    for (var i = 0; i < entities.Length; i++)
                    {
                        var steering = BnAgentSteering.Idle(new float3(i, 0f, 0f), 4f, 0.5f);
                        steering.DesiredVelocity = new float3(0f, 0f, 1f);
                        steering.PathLength = 8f;
                        steering.Flags = BnAgentFlags.Following;
                        steeringArray[i] = steering;
                    }

                    var job = new IntegrateSteeringJob
                    {
                        Steering = steeringArray,
                        DeltaTime = 0.5f,
                    }.Schedule(steeringArray.Length, 64);
                    job.Complete();

                    Assert.AreEqual(new float3(511f, 0f, 0.5f), steeringArray[511].Position);
                    Assert.AreEqual(0.5f, steeringArray[123].PathProgress, 1e-5f);
                    Assert.AreEqual(0.0625f, steeringArray[123].NormalizedProgress, 1e-5f);
                    Assert.IsTrue(steeringArray[0].HasPath);
                }
                finally
                {
                    steeringArray.Dispose();
                    entities.Dispose();
                }
            }
        }

        [BurstCompile]
        private struct IntegrateSteeringJob : IJobParallelFor
        {
            public NativeArray<BnAgentSteering> Steering;
            public float DeltaTime;

            public void Execute(int index)
            {
                var steering = Steering[index];
                steering.Velocity = steering.DesiredVelocity;
                steering.Position += steering.Velocity * DeltaTime;
                steering.PathProgress = math.mad(math.length(steering.Velocity), DeltaTime, steering.PathProgress);
                Steering[index] = steering;
            }
        }
    }
}
