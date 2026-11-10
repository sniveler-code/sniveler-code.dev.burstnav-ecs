using System;
using System.IO;
using System.Linq;
using System.Reflection;
using System.Runtime.InteropServices;
using System.Text.RegularExpressions;
using BurstNav;
using NUnit.Framework;
using Unity.Collections;
using Unity.Collections.LowLevel.Unsafe;
using Unity.Entities;
using Unity.Mathematics;
using UnityEngine;

namespace BurstNav.Tests
{
    public class BnAgentComponentsTests
    {
        private static readonly Type[] AgentComponentTypes =
        {
            typeof(BnAgentSteering),
            typeof(BnAgentPath),
            typeof(BnPathWaypoint),
            typeof(BnAgentPathRequest),
            typeof(BnAgentSteeringActive),
            typeof(BnAgentPathRequestPending),
        };

        private static readonly (Type Type, int Size)[] ExpectedSizes =
        {
            (typeof(BnAgentSteering), 60),
            (typeof(BnAgentPath), 12),
            (typeof(BnPathWaypoint), 32),
            (typeof(BnAgentPathRequest), 40),
        };

        [Test]
        public void Components_MatchTheFreeContractFieldSets()
        {
            AssertFields<BnAgentSteering>(
                "Position",
                "Velocity",
                "DesiredVelocity",
                "MaxSpeed",
                "Radius",
                "PathProgress",
                "PathLength",
                "WaypointIndex",
                "Flags");
            AssertFields<BnAgentPath>("PathIndex", "WaypointCount", "Status");
            AssertFields<BnPathWaypoint>("Position", "ExitPoint", "PolyIndex", "LinkIndex");
            AssertFields<BnAgentPathRequest>("Start", "Target", "RequestId", "StartPoly", "TargetPoly", "Status");
            AssertFields<BnAgentSteeringActive>();
            AssertFields<BnAgentPathRequestPending>();
        }

        private static void AssertFields<T>(params string[] expected)
        {
            var actual = typeof(T)
                .GetFields(BindingFlags.Public | BindingFlags.Instance)
                .Select(field => field.Name)
                .ToArray();
            CollectionAssert.AreEqual(expected, actual, typeof(T).Name + " drifted from the Free contract.");
        }

        [Test]
        public void Components_AreBlittable()
        {
            Assert.IsTrue(UnsafeUtility.IsBlittable<BnAgentSteering>());
            Assert.IsTrue(UnsafeUtility.IsBlittable<BnAgentPath>());
            Assert.IsTrue(UnsafeUtility.IsBlittable<BnPathWaypoint>());
            Assert.IsTrue(UnsafeUtility.IsBlittable<BnAgentPathRequest>());
            Assert.IsTrue(UnsafeUtility.IsBlittable<BnAgentSteeringActive>());
            Assert.IsTrue(UnsafeUtility.IsBlittable<BnAgentPathRequestPending>());
        }

        [Test]
        public void Source_HasNoTierConditionalsOrPadFields()
        {
            var source = File.ReadAllText(BnTestPaths.Package("Runtime/Core/BnAgentComponents.cs"));

            Assert.IsFalse(source.Contains("BURSTNAV_" + "PRO"));
            Assert.IsFalse(Regex.IsMatch(source, @"^[ \t]*#(?:if|elif|else|endif)", RegexOptions.Multiline));
            Assert.IsFalse(source.Contains("_pad"));
        }

        private static int SizeOf(Type type)
        {
            if (type.IsEnum)
            {
                return Marshal.SizeOf(Enum.GetUnderlyingType(type));
            }

            return Marshal.SizeOf(type);
        }

        [Test]
        public void Components_HaveNoManagedFields()
        {
            foreach (var type in AgentComponentTypes)
            {
                foreach (var field in type.GetFields(BindingFlags.Public | BindingFlags.Instance))
                {
                    Assert.IsTrue(field.FieldType.IsValueType, type.Name + "." + field.Name + " must be a value type");
                    Assert.IsFalse(field.FieldType == typeof(string), type.Name + "." + field.Name + " must not be a string");
                }
            }
        }

        [Test]
        public void Components_HaveExpectedUnmanagedLayout()
        {
            foreach (var (type, expected) in ExpectedSizes)
            {
                var actual = Marshal.SizeOf(type);
                Assert.AreEqual(expected, actual, type.Name + " has an unexpected size");
                Assert.AreEqual(actual, UnsafeUtility.SizeOf(type), type.Name + " has implicit padding");
            }
        }

        [Test]
        public void Components_AreStructsImplementingComponentData()
        {
            foreach (var type in AgentComponentTypes)
            {
                Assert.IsTrue(type.IsValueType, type.Name + " must be a struct");
                Assert.IsTrue(type.IsLayoutSequential, type.Name + " must be sequentially laid out");
                var isComponent = type == typeof(BnPathWaypoint)
                    ? typeof(IBufferElementData).IsAssignableFrom(type)
                    : typeof(IComponentData).IsAssignableFrom(type);
                Assert.IsTrue(isComponent, type.Name + " must be an ECS component type");
            }
        }

        [Test]
        public void Components_OrderFieldsBySizeDescending()
        {
            foreach (var (type, _) in ExpectedSizes)
            {
                var previous = int.MaxValue;
                foreach (var field in type.GetFields(BindingFlags.Public | BindingFlags.Instance))
                {
                    var size = SizeOf(field.FieldType);
                    Assert.LessOrEqual(size, previous, type.Name + "." + field.Name + " breaks size-descending order");
                    previous = size;
                }
            }
        }

        [Test]
        public void Steering_MatchesPublicContract()
        {
            var steering = typeof(BnAgentSteering);
            Assert.AreEqual(typeof(float3), steering.GetField("Position").FieldType);
            Assert.AreEqual(typeof(float3), steering.GetField("Velocity").FieldType);
            Assert.AreEqual(typeof(float3), steering.GetField("DesiredVelocity").FieldType);

            foreach (var progress in new[] { "PathProgress", "PathLength" })
            {
                Assert.AreEqual(typeof(float), steering.GetField(progress).FieldType, progress + " must be a float");
            }

            Assert.AreEqual(typeof(int), steering.GetField("WaypointIndex").FieldType);
        }

        [Test]
        public void Steering_NormalisedProgress_IsBranchFreeAndClamped()
        {
            var steering = BnAgentSteering.Idle(float3.zero, 4f, 0.5f);
            Assert.AreEqual(0f, steering.NormalizedProgress, 1e-6f, "no path => zero progress");

            steering.PathLength = 10f;
            steering.PathProgress = 5f;
            Assert.AreEqual(0.5f, steering.NormalizedProgress, 1e-6f);

            steering.PathProgress = 25f;
            Assert.AreEqual(1f, steering.NormalizedProgress, 1e-6f);
        }

        [Test]
        public void PathComponent_UsesStableIntegerIndex()
        {
            Assert.AreEqual(typeof(int), typeof(BnAgentPath).GetField("PathIndex").FieldType);

            var path = new BnAgentPath { PathIndex = 7, WaypointCount = 3, Status = BnPathStatus.Ready };
            Assert.AreEqual(7, path.PathIndex);
            Assert.IsTrue(path.IsFollowable);

            path.Status = BnPathStatus.Failed;
            Assert.IsFalse(path.IsFollowable);
        }

        [Test]
        public void PathWaypoint_IsBufferElementWithInlineCapacity()
        {
            Assert.IsTrue(typeof(IBufferElementData).IsAssignableFrom(typeof(BnPathWaypoint)));

            var attribute = typeof(BnPathWaypoint)
                .GetCustomAttributes(typeof(InternalBufferCapacityAttribute), false)
                .Cast<InternalBufferCapacityAttribute>()
                .Single();
            Assert.AreEqual(BnAgentComponents.DefaultPathCapacity, attribute.Capacity);
            Assert.Greater(attribute.Capacity, 0);
        }

        [Test]
        public void Toggles_AreEnableableComponents()
        {
            Assert.IsTrue(typeof(IEnableableComponent).IsAssignableFrom(typeof(BnAgentSteeringActive)));
            Assert.IsTrue(typeof(IEnableableComponent).IsAssignableFrom(typeof(BnAgentPathRequestPending)));

            foreach (var dataType in new[] { typeof(BnAgentSteering), typeof(BnAgentPath), typeof(BnAgentPathRequest), typeof(BnPathWaypoint) })
            {
                Assert.IsFalse(typeof(IEnableableComponent).IsAssignableFrom(dataType), dataType.Name + " must not be enableable");
            }
        }

        [Test]
        public void AgentSet_FormsSingleDominantArchetype()
        {
            using (var world = new World("BnAgentComponentsTests"))
            {
                var entityManager = world.EntityManager;
                var archetype = BnAgentComponents.CreateAgentArchetype(entityManager);
                Assert.IsTrue(archetype.Valid);

                var entities = new NativeArray<Entity>(256, Allocator.Temp);
                try
                {
                    entityManager.CreateEntity(archetype, entities);

                    var expected = entityManager.GetChunk(entities[0]).Archetype;
                    foreach (var type in AgentComponentTypes)
                    {
                        Assert.IsTrue(entityManager.HasComponent(entities[0], type), "agent is missing " + type.Name);
                    }

                    for (var i = 1; i < entities.Length; i++)
                    {
                        Assert.AreEqual(expected, entityManager.GetChunk(entities[i]).Archetype, "agent " + i + " drifted to another archetype");
                    }

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
        public void Toggles_FlipWithoutMovingEntitiesBetweenArchetypes()
        {
            using (var world = new World("BnAgentComponentsTests"))
            {
                var entityManager = world.EntityManager;
                var entities = new NativeArray<Entity>(4, Allocator.Temp);
                try
                {
                    entityManager.CreateEntity(BnAgentComponents.CreateAgentArchetype(entityManager), entities);
                    var archetype = entityManager.GetChunk(entities[0]).Archetype;

                    var query = entityManager.CreateEntityQuery(
                        ComponentType.ReadWrite<BnAgentSteering>(),
                        ComponentType.ReadWrite<BnAgentSteeringActive>(),
                        ComponentType.ReadWrite<BnAgentPathRequestPending>());
                    try
                    {
                        Assert.AreEqual(4, query.CalculateEntityCount());

                        entityManager.SetComponentEnabled<BnAgentSteeringActive>(entities[0], false);
                        entityManager.SetComponentEnabled<BnAgentPathRequestPending>(entities[0], false);

                        Assert.IsFalse(entityManager.IsComponentEnabled<BnAgentSteeringActive>(entities[0]));
                        Assert.IsFalse(entityManager.IsComponentEnabled<BnAgentPathRequestPending>(entities[0]));
                        Assert.AreEqual(3, query.CalculateEntityCount());

                        Assert.IsTrue(entityManager.Exists(entities[0]), "disabling a toggle must not destroy the entity");
                        Assert.AreEqual(archetype, entityManager.GetChunk(entities[0]).Archetype, "toggle flip must not change archetype");

                        entityManager.SetComponentEnabled<BnAgentSteeringActive>(entities[0], true);
                        entityManager.SetComponentEnabled<BnAgentPathRequestPending>(entities[0], true);
                        Assert.AreEqual(4, query.CalculateEntityCount());
                        Assert.AreEqual(archetype, entityManager.GetChunk(entities[0]).Archetype);
                    }
                    finally
                    {
                        query.Dispose();
                    }

                    var steeringQuery = entityManager.CreateEntityQuery(ComponentType.ReadWrite<BnAgentSteering>());
                    try
                    {
                        Assert.AreEqual(4, steeringQuery.CalculateEntityCount());
                    }
                    finally
                    {
                        steeringQuery.Dispose();
                    }
                }
                finally
                {
                    entities.Dispose();
                }
            }
        }

        [Test]
        public void PathBuffer_StoresPolySequenceAndWaypoints()
        {
            using (var world = new World("BnAgentComponentsTests"))
            {
                var entityManager = world.EntityManager;
                var entity = entityManager.CreateEntity(BnAgentComponents.CreateAgentArchetype(entityManager));
                try
                {
                    entityManager.SetComponentData(entity, new BnAgentPath
                    {
                        PathIndex = 42,
                        WaypointCount = 3,
                        Status = BnPathStatus.Ready,
                    });

                    var buffer = entityManager.GetBuffer<BnPathWaypoint>(entity);
                    buffer.Add(BnPathWaypoint.Create(new float3(0f, 0f, 0f), new float3(1f, 0f, 0f), 10, 2));
                    buffer.Add(BnPathWaypoint.Create(new float3(2f, 0f, 0f), new float3(3f, 0f, 0f), 11, 3));
                    buffer.Add(BnPathWaypoint.Create(new float3(4f, 0f, 0f), new float3(4f, 0f, 0f), 12, -1));

                    var path = entityManager.GetComponentData<BnAgentPath>(entity);
                    Assert.AreEqual(42, path.PathIndex, "path index must stay stable across buffer writes");
                    Assert.AreEqual(3, path.WaypointCount);
                    Assert.AreEqual(10, buffer[0].PolyIndex);
                    Assert.AreEqual(11, buffer[1].PolyIndex);
                    Assert.AreEqual(12, buffer[2].PolyIndex);
                    Assert.AreEqual(3, buffer[1].LinkIndex);
                    Assert.AreEqual(-1, buffer[2].LinkIndex);

                    entityManager.SetComponentEnabled<BnAgentPathRequestPending>(entity, false);
                    Assert.AreEqual(42, entityManager.GetComponentData<BnAgentPath>(entity).PathIndex);
                }
                finally
                {
                    if (entityManager.Exists(entity))
                    {
                        entityManager.DestroyEntity(entity);
                    }
                }
            }
        }

        [Test]
        public void PathRequest_CarriesStartAndTarget()
        {
            var request = BnAgentPathRequest.Create(new float3(1f, 0f, 2f), new float3(9f, 0f, 7f), 5u);

            Assert.AreEqual(new float3(1f, 0f, 2f), request.Start);
            Assert.AreEqual(new float3(9f, 0f, 7f), request.Target);
            Assert.AreEqual(5u, request.RequestId);
            Assert.AreEqual(-1, request.StartPoly);
            Assert.AreEqual(-1, request.TargetPoly);
            Assert.AreEqual(BnPathStatus.Pending, request.Status);
        }
    }
}
