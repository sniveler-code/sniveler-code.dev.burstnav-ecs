# Free Cookbook

These recipes use only the Free API: a flat navigation graph, flat A* and the
kinematic funnel. Every sample schedules jobs through systems and keeps native
containers alive until their owner disposes them. Start with
[getting-started.md](../getting-started.md) if you have not baked a graph
yet, and read [concepts.md](../concepts.md) for the data flow behind the
recipes.

## Bake a static navigation graph

Free graphs are immutable at runtime. Bake once from a static description, then
read the blob from every pathfinding and steering job. The authoring steps,
the field table and the singleton rule are in
[getting-started.md section 2](../getting-started.md#2-bake-a-static-navigation-graph).
This page adds the fixture helper and the bake checks used by tests.

### Bake and inspect a fixture graph

Use the static baker helper for a runtime-built level or a test fixture. The
returned reference belongs to your code, so teardown is your job; the authoring
path hands that ownership to the baking system instead. The plain build and
teardown path is in
[Build the blob without a SubScene](../getting-started.md#build-the-blob-without-a-subscene);
the helper below adds the counts a fixture asserts on.

```csharp
using Unity.Entities;
using Unity.Mathematics;

namespace BurstNav.Cookbook
{
    public struct BnGraphReport
    {
        public int Polygons;
        public int Links;
        public bool Created;
    }

    public static class BnGraphFixture
    {
        public static Entity Build(EntityManager entityManager, int2 cells, float cellSize, float2 origin)
        {
            var blob = BnMeshAuthoringBaker.BuildGraph(cells.x, cells.y, cellSize, origin);
            var entity = entityManager.CreateEntity();
            entityManager.AddComponentData(entity, new BnBlobMeshGraph { Value = blob });
            return entity;
        }

        public static BnGraphReport Describe(EntityManager entityManager, Entity entity)
        {
            var reference = entityManager.GetComponentData<BnBlobMeshGraph>(entity).Value;

            return new BnGraphReport
            {
                Created = reference.IsCreated,
                Polygons = reference.IsCreated ? reference.Value.PolygonCount : 0,
                Links = reference.IsCreated ? reference.Value.LinkCount : 0,
            };
        }
    }
}
```

Teardown is a one-shot path: dispose the reference before you destroy the
entity, and only after the pathfinding and steering chains have finished. The
guide's `BnRuntimeGraph.Destroy` shows the order; never dispose a blob while a
job still reads it.

### Validate the bake

- `Describe(...).Created` is true for exactly one `BnBlobMeshGraph` entity.
- `Polygons` matches `cells.x * cells.y`.
- `Links` is greater than zero for a grid with at least two polygons.
- The graph lies on the XY plane; Z is height and is not searched.

## Spawn agents and follow a path

A cookbook agent needs the dominant BurstNav archetype plus a gameplay goal and
a re-path trigger. Keep every component in one archetype so all agents share
chunks; adding a component after spawn would move that agent to a second
archetype and split the steering chunks.

### Spawn one dominant archetype

```csharp
using Unity.Entities;
using Unity.Mathematics;

namespace BurstNav.Cookbook
{
    public struct BnAgentGoal : IComponentData
    {
        public float3 Target;
    }

    public struct BnRepathRequested : IComponentData, IEnableableComponent
    {
    }

    public static class BnAgentSpawn
    {
        public static EntityArchetype CreateArchetype(EntityManager entityManager) => entityManager.CreateArchetype(
            ComponentType.ReadWrite<BnAgentSteering>(),
            ComponentType.ReadWrite<BnAgentPath>(),
            ComponentType.ReadWrite<BnPathWaypoint>(),
            ComponentType.ReadWrite<BnAgentPathRequest>(),
            ComponentType.ReadWrite<BnAgentSteeringActive>(),
            ComponentType.ReadWrite<BnAgentPathRequestPending>(),
            ComponentType.ReadWrite<BnAgentGoal>(),
            ComponentType.ReadWrite<BnRepathRequested>());

        public static void Spawn(
            EntityManager entityManager,
            EntityArchetype archetype,
            int count,
            float3 origin,
            float3 target,
            float spacing)
        {
            for (var index = 0; index < count; index++)
            {
                var position = origin + new float3((index % 16) * spacing, (index / 16) * spacing, 0f);
                var agent = entityManager.CreateEntity(archetype);

                entityManager.SetComponentData(agent, BnAgentSteering.Idle(position, 5f, 0.5f));
                entityManager.SetComponentData(agent, new BnAgentPath
                {
                    PathIndex = index,
                    WaypointCount = 0,
                    Status = BnPathStatus.None,
                });
                entityManager.SetComponentData(agent, new BnAgentPathRequest
                {
                    Start = position,
                    Target = target,
                    RequestId = 0,
                    StartPoly = -1,
                    TargetPoly = -1,
                    Status = BnPathStatus.None,
                });
                entityManager.SetComponentData(agent, new BnAgentGoal { Target = target });

                entityManager.SetComponentEnabled<BnAgentSteeringActive>(agent, false);
                entityManager.SetComponentEnabled<BnAgentPathRequestPending>(agent, false);
                entityManager.SetComponentEnabled<BnRepathRequested>(agent, true);
            }
        }
    }
}
```

The first request is armed by the enabled `BnRepathRequested` toggle. On the
following frames the funnel writes `DesiredVelocity`, the Free velocity stage
copies it, and `IntegrateJob` advances `Position`. Do not write
`BnAgentSteering.Position` from gameplay; the steering chain owns it.

## Issue a path request

A request is a `BnAgentPathRequest` plus the enabled
`BnAgentPathRequestPending` toggle. The pathfinding system only visits agents
whose request toggle is enabled, so writing the component alone does not start a
search.

### Request payload

| Field | Set it to | Why |
| --- | --- | --- |
| `Start` | `BnAgentSteering.Position` at request time | `WritePath` places waypoint 0 at `Start`, so a stale spawn point breaks every re-path |
| `Target` | World-space target on the XY plane | Z is preserved as height |
| `StartPoly` | `-1` | `ResolvePolygons` reuses a valid id, so reset it to force a fresh snap |
| `TargetPoly` | `-1` | Same reuse rule as `StartPoly` |
| `Status` | `BnPathStatus.Pending` | `BnPathfindJob` overwrites it; the enabled toggle is what gates the visit |
| `RequestId` | Previous value plus one | Caller-owned counter; the Free pipeline copies it but never compares it |

### Arm the first request and every re-path

This job runs only for agents whose `BnRepathRequested` toggle is enabled. It
copies the live steering position, so the first request and an on-demand re-path
take the same path. The command buffer disables the trigger and enables the
pathfinding toggle; the barrier plays both back once at frame end.

```csharp
using Unity.Burst;
using Unity.Burst.Intrinsics;
using Unity.Collections;
using Unity.Entities;
using Unity.Jobs;
using Unity.Mathematics;

namespace BurstNav.Cookbook
{
    [BurstCompile]
    public struct BnAgentRequestJob : IJobChunk
    {
        [ReadOnly] public ComponentTypeHandle<BnAgentSteering> SteeringHandle;
        [ReadOnly] public ComponentTypeHandle<BnAgentGoal> GoalHandle;
        public ComponentTypeHandle<BnAgentPathRequest> RequestHandle;
        public EntityTypeHandle EntityHandle;
        public EntityCommandBuffer Commands;

        [BurstCompile]
        public void Execute(in ArchetypeChunk chunk, int unfilteredChunkIndex, bool useEnabledMask, in v128 chunkEnabledMask)
        {
            var steering = chunk.GetNativeArray(ref SteeringHandle);
            var goals = chunk.GetNativeArray(ref GoalHandle);
            var requests = chunk.GetNativeArray(ref RequestHandle);
            var entities = chunk.GetNativeArray(EntityHandle);
            var enumerator = new ChunkEntityEnumerator(useEnabledMask, chunkEnabledMask, chunk.Count);

            while (enumerator.NextEntityIndex(out var index))
            {
                var state = steering[index];
                var request = requests[index];

                request.Start = state.Position;
                request.Target = goals[index].Target;
                request.StartPoly = -1;
                request.TargetPoly = -1;
                request.Status = BnPathStatus.Pending;
                request.RequestId++;

                requests[index] = request;
                Commands.SetComponentEnabled<BnAgentPathRequestPending>(entities[index], true);
                Commands.SetComponentEnabled<BnRepathRequested>(entities[index], false);
            }
        }
    }

    [UpdateInGroup(typeof(SimulationSystemGroup))]
    [UpdateBefore(typeof(BnPathfindingSystem))]
    public partial class BnPathRequestSystem : SystemBase
    {
        private EndSimulationEntityCommandBufferSystem _barrier;
        private EntityQuery _query;

        protected override void OnCreate()
        {
            _barrier = World.GetOrCreateSystemManaged<EndSimulationEntityCommandBufferSystem>();
            _query = GetEntityQuery(
                ComponentType.ReadWrite<BnAgentPathRequest>(),
                ComponentType.ReadWrite<BnRepathRequested>(),
                ComponentType.ReadOnly<BnAgentSteering>(),
                ComponentType.ReadOnly<BnAgentGoal>());
        }

        protected override void OnUpdate()
        {
            if (_query.IsEmptyIgnoreFilter)
            {
                return;
            }

            var handle = new BnAgentRequestJob
            {
                SteeringHandle = GetComponentTypeHandle<BnAgentSteering>(true),
                GoalHandle = GetComponentTypeHandle<BnAgentGoal>(true),
                RequestHandle = GetComponentTypeHandle<BnAgentPathRequest>(false),
                EntityHandle = GetEntityTypeHandle(),
                Commands = _barrier.CreateCommandBuffer(),
            }.Schedule(_query, Dependency);

            _barrier.AddJobHandleForProducer(handle);
            Dependency = handle;
        }
    }
}
```

The command buffer plays back once at the end of the frame. On the next update,
`BnPathfindingSystem` searches toward the polygon closest to the target, writes
the corridor, and clears the request toggle. The steering system then re-enables
`BnAgentSteeringActive` when the path becomes followable.

### Re-path from gameplay

Ask for a new path only when a goal actually changes, not once per frame per
agent. Write the goal and enable the trigger through a command buffer so the
change lands at frame end.

```csharp
using Unity.Entities;
using Unity.Mathematics;

namespace BurstNav.Cookbook
{
    public static class BnGoals
    {
        public static void SetGoal(EntityCommandBuffer commands, Entity agent, float3 target)
        {
            commands.SetComponent(agent, new BnAgentGoal { Target = target });
            commands.SetComponentEnabled<BnRepathRequested>(agent, true);
        }
    }
}
```

`SetGoal` runs on the main thread once per goal change. If goal events come from
a job, write through an `EntityCommandBuffer.ParallelWriter` registered with the
same barrier instead.

The request system resets `StartPoly`, `TargetPoly` and `Status` and overwrites
`Start` on every pass. Never write only `Target` and `RequestId`: `StartPoly`
stays valid and `Start` keeps the old position, so the corridor would begin at
the old point. Do not clear `BnPathWaypoint` yourself either; the pathfinding
system owns the buffer write.

## Consume the path

Read the result through chunk arrays. The path slot carries the lifecycle state,
the steering value carries progress and arrival flags, and the waypoint buffer
carries the corridor.

### Read path state in a job

Add one small readout component to your agent archetype and update it after
pathfinding. This keeps gameplay reads out of the hot steering loop and uses no
per-entity component access.

```csharp
using Unity.Burst;
using Unity.Burst.Intrinsics;
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;

namespace BurstNav.Cookbook
{
    public struct BnPathReadout : IComponentData
    {
        public float Progress;
        public int WaypointCount;
        public BnPathStatus Status;
        public BnAgentFlags Flags;
    }

    [BurstCompile]
    public struct BnPathReadoutJob : IJobChunk
    {
        [ReadOnly] public ComponentTypeHandle<BnAgentPath> PathHandle;
        [ReadOnly] public ComponentTypeHandle<BnAgentSteering> SteeringHandle;
        [ReadOnly] public BufferTypeHandle<BnPathWaypoint> WaypointHandle;
        public ComponentTypeHandle<BnPathReadout> ReadoutHandle;

        [BurstCompile]
        public void Execute(in ArchetypeChunk chunk, int unfilteredChunkIndex, bool useEnabledMask, in v128 chunkEnabledMask)
        {
            var paths = chunk.GetNativeArray(ref PathHandle);
            var steering = chunk.GetNativeArray(ref SteeringHandle);
            var waypoints = chunk.GetBufferAccessor(ref WaypointHandle);
            var readouts = chunk.GetNativeArray(ref ReadoutHandle);
            var enumerator = new ChunkEntityEnumerator(useEnabledMask, chunkEnabledMask, chunk.Count);

            while (enumerator.NextEntityIndex(out var index))
            {
                var path = paths[index];
                var state = steering[index];
                var count = math.min(path.WaypointCount, waypoints[index].Length);

                readouts[index] = new BnPathReadout
                {
                    Progress = state.NormalizedProgress,
                    WaypointCount = count,
                    Status = path.Status,
                    Flags = state.Flags,
                };
            }
        }
    }

    [UpdateInGroup(typeof(SimulationSystemGroup))]
    [UpdateAfter(typeof(BnPathfindingSystem))]
    public partial class BnPathReadoutSystem : SystemBase
    {
        private EntityQuery _query;

        protected override void OnCreate()
        {
            _query = GetEntityQuery(
                ComponentType.ReadOnly<BnAgentPath>(),
                ComponentType.ReadOnly<BnAgentSteering>(),
                ComponentType.ReadOnly<BnPathWaypoint>(),
                ComponentType.ReadWrite<BnPathReadout>());
        }

        protected override void OnUpdate()
        {
            if (_query.IsEmptyIgnoreFilter)
            {
                return;
            }

            Dependency = new BnPathReadoutJob
            {
                PathHandle = GetComponentTypeHandle<BnAgentPath>(true),
                SteeringHandle = GetComponentTypeHandle<BnAgentSteering>(true),
                WaypointHandle = GetBufferTypeHandle<BnPathWaypoint>(true),
                ReadoutHandle = GetComponentTypeHandle<BnPathReadout>(false),
            }.ScheduleParallel(_query, Dependency);
        }
    }
}
```

`BnPathReadout` is gameplay state, not part of the BurstNav agent archetype. Add
it once when you spawn or author the agent; do not add or remove it per frame.

### Interpret the result

| State | Meaning | Action |
| --- | --- | --- |
| `BnPathStatus.None` | No request has completed | Wait |
| `BnPathStatus.Pending` | A request is armed or in flight | Wait for a later frame |
| `BnPathStatus.Ready` | A corridor is available | Follow it or read the waypoint buffer |
| `BnPathStatus.Failed` | No corridor was written | Check the graph, endpoints and budget |
| `BnAgentFlags.Arrived` | The funnel reached the final waypoint | Stop or issue the next goal |
| `BnAgentFlags.Blocked` | Reserved for gameplay; Free never sets or reads it | Use it as your own flag if a custom system needs it |

`BnPathStatus` also declares `Partial`, but `BnPathfindJob` never writes it: a
search that runs out of budget reports `Failed` with an empty corridor. Do not
wait for `Partial`; use the budget diagnostics below.

The buffer holds one waypoint per corridor polygon, starting at the request
origin and ending at the exact target. Read `Position`, `ExitPoint`, `PolyIndex`
and `LinkIndex` from `BnPathWaypoint`; treat the buffer as read-only gameplay
data.

## Tune funnel following

Work through these steps in order. Each one removes a class of complaint before
you consider a code change.

1. Settle speed and clearance at spawn. Write `MaxSpeed` and `Radius` once, when
   the agent entity is created or loaded. The authoritative description of both
   fields, including the arrival tolerance, is in
   [Free configuration](../configuration.md#speed-and-arrival).
2. Confirm the agent stops where you expect. Mirror `BnAgentSteering.Flags` into
   a readout and test `BnAgentFlags.Arrived`; if the stop point is too far away,
   reduce `Radius` down to the funnel floor instead of comparing positions.
3. Remove jerky starts without touching the steering loop. Speed caps and goal
   spacing are the cheap levers; a replacement acceleration stage has to sit
   between your velocity decision and `IntegrateJob`. Never add a per-frame
   `SetComponentData` call to the existing steering system.
4. Keep gameplay reads out of the hot path. `WaypointIndex`, `PathProgress`,
   `PathLength`, `Velocity` and `Flags` are funnel-owned outputs; read them from
   `BnPathReadout` rather than from the steering value in a per-frame loop.

Use the tuning checklist in
[configuration.md](../configuration.md#speed-and-arrival) to confirm a
smoothness change actually helped before you keep it.

## Detect flat A* budget exhaustion

A failed search reaches gameplay as `BnPathStatus.Failed`, with no corridor and
no reason attached. Work from the observable state inward.

1. Read the path slot. A failed path leaves the waypoint buffer empty, so
   `BnPathReadout.Status` and `WaypointCount` separate "no corridor" from
   "corridor still in flight".
2. Check the request before the budget. A start or target outside the graph, or
   two polygons that no link chain joins, fails no matter how large the budget
   is.
3. Reproduce the search with a probe. Run the same flat call with a
   caller-owned scratchpad and read its status.

### Probe the search directly

Call the probe from a test, an editor window or load-time validation. Never run
it per frame next to the pathfinding system: that doubles the search work and
defeats the batch pipeline.

```csharp
using Unity.Collections;

namespace BurstNav.Cookbook
{
    public static class BnBudgetProbe
    {
        public static BnPathSearchStatus Measure(
            ref BnMeshGraphBlob meshGraph,
            int start,
            int target,
            int maxIterations,
            out int iterations,
            out int pathCount)
        {
            var scratch = BnPathScratchpad.Create(meshGraph.PolygonCount, Allocator.Temp);

            try
            {
                BnAStar.FindPath(ref meshGraph, start, target, ref scratch, maxIterations);
                iterations = scratch.Iterations;
                pathCount = scratch.PathCount;
                return scratch.Status;
            }
            finally
            {
                scratch.Dispose();
            }
        }
    }
}
```

### Read the probe result

| `BnPathSearchStatus` | Meaning for your request | Next step |
| --- | --- | --- |
| `Found` | The route exists inside the budget | Compare `pathCount` with the corridor you expected |
| `BudgetExhausted` | The probe stopped at `maxIterations` | Raise the configured budget, or split the level |
| `Unreachable` | The frontier drained without the target | The graph has no route; fix the level or the endpoints |
| `InvalidInput` | A polygon id is outside the graph | Validate ids before the request |
| `CapacityExceeded` | The scratchpad is smaller than the graph | Allocate for the whole graph, not for one path |

### Change the budget only after the probe

Set `BnPathfindingSystem.MaxIterations` once at startup, as shown in
[Free configuration](../configuration.md#search-budget). A per-frame
assignment buys nothing because the system reads the value when it builds the
job.

Measure the result: a larger budget bounds a bad request less tightly, so a
misconfigured graph gets more expensive rather than correct. If long routes
still exhaust the budget on a large graph, the package boundary is the problem, not
the value. See [limitations.md](../limitations.md) for the flat-search scaling
limits.

## See also

- [Free getting started](../getting-started.md) - the first graph and agent.
- [Free configuration](../configuration.md) - the authoritative tuning fields.
- [Free concepts](../concepts.md) - graph, portal, funnel and path slot.
- [Free API reference](../api/Free.md) - complete Free signatures.
- [Free limitations](../limitations.md) - search scaling and missing stages.
- [Free overview](../README.md) - the page map and requirements.
