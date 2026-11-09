# Getting started with BurstNav Free

This page takes you from an empty Unity project to one agent that follows a
path across a baked 4 x 4 navigation grid. Every type and method used here is
part of the Free API.

## 1. Install the package

Add the pinned DOTS packages to Packages/manifest.json:

```json
{
  "dependencies": {
    "com.unity.burst": "1.8.12",
    "com.unity.collections": "2.4.0",
    "com.unity.entities": "1.2.4",
    "com.unity.mathematics": "1.3.1"
  }
}
```

Add `com.burstnav.free` with **Window > Package Manager > Add package from
disk** and select the `package.json` in this package folder, or add the
equivalent local, registry or embedded package entry. Keep every `.meta` file:
it carries the GUIDs that keep serialized references stable.

The runtime assembly is `BurstNav.Free` with root namespace `BurstNav`. It is
auto-referenced and references only Unity DOTS assemblies.

Wait for Unity to compile, then confirm that the console has no errors before
you author a scene.

## 2. Bake a static navigation graph

The navigation graph is immutable at runtime. In Free you bake it once from a
static mesh description and never mutate it while agents run.

1. Create a SubScene and open it.
2. Add an empty GameObject named Navmesh.
3. Add the BnMeshAuthoring component.

BnMeshAuthoring builds a flat grid of square polygons with directed portals
between axis neighbours. The baker writes one BnBlobMeshGraph component whose
Value points at a BlobAssetReference of BnMeshGraphBlob.

| Field | Meaning |
| --- | --- |
| CellsX | Polygon count along X. |
| CellsY | Polygon count along Y. |
| CellSize | World-space edge length of one square polygon. |
| Origin | Lower-left corner of the grid on the XY navigation plane. |

Polygon ids are row-major: id = y * CellsX + x. A 4 x 4 grid with
CellSize 1 at Origin (0, 0) covers (0, 0) to (4, 4) and has 16 polygons.

The world must contain exactly one BnBlobMeshGraph. BnPathfindingSystem and
BnSteeringSystem both read a graph singleton and skip their work while no valid
singleton exists.

## 3. Spawn an agent

1. In the same SubScene, create a GameObject named Agent at (0.5, 0.5, 0).
2. Add the BnAgentAuthoring component.
3. Set MaxSpeed to 5, Radius to 0.5 and StartActive to true.

The baker creates the dominant agent archetype in one step:

| Component | Purpose |
| --- | --- |
| BnAgentSteering | Position, velocity, desired velocity, speed, radius and path progress. |
| BnAgentPath | Path slot state and the waypoint count. |
| BnPathWaypoint | The agent's path buffer: one element per polygon in the corridor, so n polygons give n waypoints. |
| BnAgentPathRequest | The pending search request slot. |
| BnAgentSteeringActive | Enableable toggle for the steering chain. |
| BnAgentPathRequestPending | Enableable toggle that arms a search request. |

The baker sets Start and Target to the spawn position and leaves the request
toggle disabled. The agent stays still until you write a target and enable the
request, as shown in step 5.

## 4. Add the systems to the world

The default world discovers both systems automatically, so there is nothing to
register:

| System | Group | Job |
| --- | --- | --- |
| BnPathfindingSystem | SimulationSystemGroup | BnPathfindJob |
| BnSteeringSystem | SimulationSystemGroup | BnFunnelJob, BnSteeringStateJob, KinematicBypassJob, IntegrateJob |

BnSteeringSystem declares [UpdateAfter(typeof(BnPathfindingSystem))] and
[UpdateBefore(typeof(EndSimulationEntityCommandBufferSystem))], so it consumes
the corridors the pathfinding job produced and its deferred toggles are
recorded before the barrier runs. BnPathfindingSystem declares only
[UpdateInGroup(typeof(SimulationSystemGroup))]; its command buffer playback is
ordered by AddJobHandleForProducer, which makes the barrier wait for the
pathfind job instead of by an UpdateBefore attribute. Either way the toggles
play back once per frame at end of simulation. If you build a custom World,
create both systems inside its SimulationSystemGroup; do not call OnUpdate
manually and do not complete their job handles from gameplay code.

## 5. Request the first path

Add the following system to your gameplay assembly. It runs once, writes a
target into every baked agent request, and arms the request toggle through the
end-of-frame command buffer.

```csharp
using Unity.Collections;
using Unity.Entities;
using Unity.Mathematics;

namespace BurstNav.Guide
{
    [UpdateInGroup(typeof(SimulationSystemGroup))]
    [UpdateBefore(typeof(BnPathfindingSystem))]
    public partial class BnFirstTargetSystem : SystemBase
    {
        private EndSimulationEntityCommandBufferSystem _commandBufferSystem;
        private EntityQuery _agents;
        private bool _issued;

        protected override void OnCreate()
        {
            _commandBufferSystem = World.GetOrCreateSystemManaged<EndSimulationEntityCommandBufferSystem>();
            _agents = GetEntityQuery(ComponentType.ReadWrite<BnAgentPathRequest>());
        }

        protected override void OnUpdate()
        {
            if (_issued || _agents.IsEmptyIgnoreFilter)
            {
                return;
            }

            _issued = true;

            using var entities = _agents.ToEntityArray(Allocator.Temp);
            using var requests = _agents.ToComponentDataArray<BnAgentPathRequest>(Allocator.Temp);
            var commands = _commandBufferSystem.CreateCommandBuffer();

            for (var index = 0; index < entities.Length; index++)
            {
                var request = requests[index];
                if (request.Status == BnPathStatus.Ready)
                {
                    continue;
                }

                request.Target = new float3(3.5f, 3.5f, 0f);
                request.TargetPoly = -1;
                request.Status = BnPathStatus.Pending;
                request.RequestId++;
                commands.SetComponent(entities[index], request);
                commands.SetComponentEnabled<BnAgentPathRequestPending>(entities[index], true);
            }
        }
    }
}
```

The command buffer plays back at the end of frame 0. On frame 1,
BnPathfindingSystem sees the enabled request, runs flat A* to the nearest
polygon of the target, and writes the corridor into the waypoint buffer. The
steering system then re-enables BnAgentSteeringActive and starts moving the
agent.

Use a request payload that matches the baked agent:

- Start stays at the spawn position for the first request.
- Target is in world space on the XY plane; Z is height.
- TargetPoly = -1 asks the pathfinder to snap the target to the nearest polygon.

## 6. Verify the first moving agent

Enter Play mode and inspect the Agent entity in the Entities Hierarchy.

| Frame | What you should see |
| --- | --- |
| 0 | BnAgentPathRequestPending turns on at the end of the frame. |
| 1 | BnAgentPath.Status becomes Ready and WaypointCount is greater than zero. |
| 2 and later | BnAgentSteering.Position moves toward (3.5, 3.5, 0). WaypointIndex and PathProgress grow. |
| Arrival | BnAgentSteering.Flags contains Arrived and DesiredVelocity returns to zero. |

If the agent does not move, check these points in order:

1. The world has exactly one BnBlobMeshGraph with a created blob.
2. The agent carries all six components from the baker and its request toggle is
   enabled.
3. The graph is not empty, a route exists, and the search stays within
   MaxIterations. Both StartPoly and TargetPoly are snapped to the nearest
   polygon when the id is out of range, so an off-graph start behaves exactly
   like an off-graph target. A failure means an empty graph (no polygons, so
   the snap yields -1) or no route between the resolved polygons; a budget
   cut-off is the third case and only happens when MaxIterations is smaller
   than the number of polygons the search expands. In every case
   BnAgentPath.Status becomes Failed.
4. BnPathfindingSystem and BnSteeringSystem are present and enabled.
5. Burst compilation is enabled and the console shows no job safety errors.

## Build the blob without a SubScene

You can build the same graph from code with the static baker helper. Call this
one-shot setup before agents run, and dispose the blob when the graph entity is
destroyed.

```csharp
using Unity.Entities;
using Unity.Mathematics;

namespace BurstNav.Guide
{
    public static class BnRuntimeGraph
    {
        public static Entity Create(EntityManager entityManager, int cellsX, int cellsY, float cellSize, float2 origin)
        {
            var blob = BnMeshAuthoringBaker.BuildGraph(cellsX, cellsY, cellSize, origin);
            var entity = entityManager.CreateEntity(typeof(BnBlobMeshGraph));
            entityManager.SetComponentData(entity, new BnBlobMeshGraph { Value = blob });
            return entity;
        }

        public static void Destroy(EntityManager entityManager, Entity entity)
        {
            var blob = entityManager.GetComponentData<BnBlobMeshGraph>(entity).Value;
            if (blob.IsCreated)
            {
                blob.Dispose();
            }

            entityManager.DestroyEntity(entity);
        }
    }
}
```

The helper returns a persistent blob owned by the caller, unlike the authoring
baker, whose blob is owned by the baking system. Continue with
[concepts.md](concepts.md) to learn how the graph, portal and funnel fit
together, or go straight to [configuration.md](configuration.md) for tuning.
