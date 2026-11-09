# Configuring BurstNav Free

The Free package has few tuning knobs on purpose. The search budget bounds worst
case pathfinding, and the agent steering fields define speed and stopping
distance. Everything else is fixed by the zero-allocation job pipeline.

## Search budget

BnPathfindingSystem.MaxIterations is the maximum number of A* node expansions
per request. The default is BnPathfindingSystem.DefaultMaxIterations, which is
4096.

| Value | Effect |
| --- | --- |
| Too low | Long paths end as BnPathStatus.Failed and the corridor is cleared. |
| Default 4096 | Covers most levels up to a few thousand polygons. |
| Larger | Tolerates longer or more obstructed paths, but bounds a bad request less tightly. |

One expansion pops one polygon and relaxes its outgoing links. A budget only
needs to exceed the number of polygons the search expands before it reaches the
target. An unreachable target drains the open set and stops early, so it costs
at most one expansion per reachable polygon; a budget cut-off is a separate
outcome that happens when MaxIterations is smaller than that count. Size the
value to your graph rather than to the largest path you can imagine.

Set the field before agents start requesting paths:

```csharp
using Unity.Entities;

namespace BurstNav.Guide
{
    [UpdateInGroup(typeof(InitializationSystemGroup))]
    public partial class BnTuningSystem : SystemBase
    {
        private bool _applied;

        protected override void OnUpdate()
        {
            if (_applied)
            {
                return;
            }

            _applied = true;
            var pathfinding = World.GetOrCreateSystemManaged<BnPathfindingSystem>();
            pathfinding.MaxIterations = 8192;
        }
    }
}
```

BnPathfindingSystem owns one scratchpad sized to the graph's polygon count. It
reallocates only when a bound graph is larger than the current capacity, then
releases the old arrays with `Dispose(JobHandle)` on the in-flight search handle
instead of completing it on the main thread. Do not resize or dispose that
scratchpad from gameplay code.

The scratchpad keeps eight arrays indexed by polygon id. Budget roughly
29 bytes per polygon plus 12 bytes for the spare heap and visit-stamp slots,
allocated once per graph size, not once per frame.

## Speed and arrival

Velocity comes from BnAgentSteering on the agent entity. The two fields you tune
are baked by BnAgentAuthoring:

| Field | Effect |
| --- | --- |
| MaxSpeed | Upper bound of DesiredVelocity and Velocity in world units per second. |
| Radius | Agent radius; also the effective arrival tolerance with the funnel. |

The funnel computes desired speed as Min(MaxSpeed, distance / deltaTime), so an
agent cannot overshoot the waypoint at the end of a frame. The arrival test uses
max(Radius, BnFunnel.DefaultArrivalRadius): a larger Radius stops the agent
further from the target, a Radius below 0.05 still stops at 0.05.

There is no acceleration or deceleration model in the kinematic bypass. Velocity
changes from zero to MaxSpeed in one frame. To make motion feel smoother, lower
MaxSpeed, or replace the steering system with your own chain that limits
acceleration before integration.

To change the profile after baking, write the component once during setup:

```csharp
using Unity.Entities;

namespace BurstNav.Guide
{
    public static class BnAgentProfile
    {
        public static void Apply(EntityManager entityManager, Entity agent, float maxSpeed, float radius)
        {
            var steering = entityManager.GetComponentData<BnAgentSteering>(agent);
            steering.MaxSpeed = maxSpeed;
            steering.Radius = radius;
            entityManager.SetComponentData(agent, steering);
        }
    }
}
```

Call Apply outside the frame loop. The hot path never reads or writes components
one entity at a time: the steering jobs access chunk arrays.

Path progress is read-only for tuning. BnAgentSteering.PathProgress and
PathLength give you a 0..1 completion ratio through
BnAgentSteering.NormalizedProgress; BnAgentSteering.WaypointIndex tells you
which corridor element the funnel is steering toward. Do not write these fields
from gameplay: the funnel owns them.

## Burst and Jobs

BurstNav ships Burst-compiled jobs: BnPathfindJob, BnFunnelJob,
BnSteeringStateJob, KinematicBypassJob and IntegrateJob. The systems schedule
them and chain JobHandle values through SystemBase.Dependency. No Free system
calls Complete in Update or FixedUpdate, and no Free job allocates.

Recommended editor settings while developing:

- Jobs > Burst > Enable Compilation: on.
- Jobs > Burst > Synchronous Compilation: on only when you debug Burst code.
- Jobs > Safety Checks: on in the editor and in development builds.
- Jobs > Leak Detection: on in development builds.

Runtime rules that keep the pipeline valid:

- Do not call Complete on the pathfinding or steering handles from gameplay.
  Read BnAgentPath and BnAgentSteering on a later frame instead.
- Play back EntityCommandBuffer content once, at frame end. The systems already
  use EndSimulationEntityCommandBufferSystem.
- Do not write the system-owned BnPathScratchpad.
- Keep one dominant agent archetype. Adding optional components per agent
  splits chunks and costs the chunk iteration path.
- Keep one BnBlobMeshGraph singleton per world.

The pathfinding job schedules single-threaded because every agent shares one A*
scratchpad. The steering jobs schedule in parallel because each agent owns its
own state and buffer. Neither choice changes your code: both are internal to the
systems.

## System order

| Group | System | Work |
| --- | --- | --- |
| InitializationSystemGroup | Your BnTuningSystem | One-shot field assignment. |
| SimulationSystemGroup | Your request system, ordered before BnPathfindingSystem | Write target and arm the request. |
| SimulationSystemGroup | BnPathfindingSystem | Flat A* and corridor write. |
| SimulationSystemGroup | BnSteeringSystem | Funnel, bypass and integration. |
| EndSimulationEntityCommandBufferSystem | Command playback | Enable or disable request and steering toggles. |

Keep gameplay systems that edit requests or agent profiles before
BnPathfindingSystem, and do not depend on a request being resolved in the same
frame. The command buffer may have armed it at the end of the previous frame.

## Assembly references

Put gameplay code in its own assembly definition and reference
`SnivelerCode.BurstNav.Runtime`. A minimal runtime assembly looks like this:

```json
{
  "name": "Game.Navigation",
  "rootNamespace": "Game.Navigation",
  "references": [
    "SnivelerCode.BurstNav.Runtime",
    "Unity.Entities",
    "Unity.Burst",
    "Unity.Collections",
    "Unity.Jobs",
    "Unity.Mathematics"
  ],
  "includePlatforms": [],
  "excludePlatforms": [],
  "allowUnsafeCode": false,
  "autoReferenced": true
}
```

Add Unity.Entities.Hybrid and Unity.Transforms when the same assembly also
contains authoring components and bakers. The `SnivelerCode.BurstNav.Runtime` assembly already
references all of them, but assembly references are not transitive for your own
code.

Keep the `SnivelerCode.BurstNav.Runtime.asmdef` assembly name unchanged. Unity resolves
assembly references by name, and a renamed assembly breaks every reference in
this page.
