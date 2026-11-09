# BurstNav Free concepts

BurstNav Free is a small, explicit pipeline: a polygon graph feeds a flat A*
search, the search fills a waypoint buffer, and a kinematic funnel turns that
corridor into a desired velocity. This page defines the data and the invariants
you rely on when you read or extend the code.

## Runtime flow

```text
BnBlobMeshGraph (singleton)
  -> BnMeshGraphBlob (blob asset)
  -> BnAStar.FindPath + BnPathScratchpad
  -> BnAgentPath + BnPathWaypoint buffer
  -> BnFunnel.Solve
  -> BnAgentSteering.DesiredVelocity
  -> KinematicBypassJob
  -> BnAgentSteering.Velocity
  -> IntegrateJob
  -> BnAgentSteering.Position
```

## Navigation graph and polygons

A navigation graph is an immutable polygon soup stored in a blob asset. Every
polygon is convex, and two polygons that share an edge are neighbours.

BnMeshGraphBlob stores four flat pools plus per-polygon index tables:

| Field | Contents |
| --- | --- |
| Vertices | All polygon vertices as float2, in graph space on the XY plane. |
| Polygons | One BnPolygon per polygon: centre, vertex offset and count, neighbour offset and count. |
| NeighborPolygons | Flat list of neighbour polygon ids, grouped per polygon. |
| Links | Flat list of directed portals between neighbours. |
| PolyLinkOffsets | First link index owned by each polygon. |
| PolyLinkCounts | Number of links leaving each polygon. |

Polygon id is a stable integer. Free never reorders or reuses ids, so gameplay
can store a polygon id and compare it later. Use the accessors instead of
walking the pools directly:

| Accessor | Returns |
| --- | --- |
| PolygonCount | Number of polygons. |
| LinkCount | Number of directed portals. |
| GetPolygonCenter(id) | Polygon centre. |
| GetPolyLinkOffset(id) | First link index of the polygon. |
| GetPolyLinkCount(id) | Number of links leaving the polygon. |
| GetPolyLink(id, index) | One BnLinkData. |
| GetNeighborCount(id) | Neighbour count. |
| GetNeighbor(id, index) | Neighbour polygon id. |

A branch-free closest-polygon query looks like this:

```csharp
using Unity.Mathematics;

namespace BurstNav.Guide
{
    public static class BnPolygonLookup
    {
        public static int NearestPolygon(ref BnMeshGraphBlob graph, float3 position)
        {
            var point = new float2(position.x, position.y);
            var best = -1;
            var bestDistanceSq = float.PositiveInfinity;

            for (var polygon = 0; polygon < graph.PolygonCount; polygon++)
            {
                var distanceSq = math.distancesq(graph.Polygons[polygon].Center, point);
                var closer = distanceSq < bestDistanceSq;
                best = math.select(best, polygon, closer);
                bestDistanceSq = math.select(bestDistanceSq, distanceSq, closer);
            }

            return best;
        }
    }
}
```

The scan is sequential over the polygon table and branch-free inside the loop.
It is suitable for one-shot queries, not for a per-entity hot loop; the
pathfinding system resolves world points to polygons once per request.

## Links and portals

A link is one directed portal from a polygon to one of its neighbours.
BnLinkData carries:

| Field | Meaning |
| --- | --- |
| PortalLeft | Left endpoint of the shared edge, in graph space. |
| PortalRight | Right endpoint of the shared edge, in graph space. |
| Target | Polygon id reached through this portal. |
| Cost | Traversal cost. |

A* uses Cost when it is positive and the centre-to-centre distance otherwise,
so an un-costed graph still produces shortest geometric paths. Links are
directed even when the same edge exists in both directions: each direction is
a separate BnLinkData entry with its own cost.

The funnel reads the portal endpoints, not the polygon centres. That is why the
path corridor stores both a waypoint position (usually a centre) and an exit
point (the midpoint of the portal used to leave that polygon).

## Flat A* and the path scratchpad

BnAStar.FindPath has one authoritative signature:

```text
bool BnAStar.FindPath(
    ref BnMeshGraphBlob graph,
    int startPoly,
    int targetPoly,
    ref BnPathScratchpad scratch,
    int maxIterations)
```

The search is a single flat A* over the polygon graph. Its heuristic is the
Euclidean distance between polygon centres, scaled by
BnPathScratchpad.HeuristicScale. The default scale is 1, which is admissible
for portal costs. Set it to 0 to degrade to Dijkstra when your cost metric is
arbitrary.

The scratchpad is reusable working memory indexed by polygon id. Nothing inside
FindPath allocates: a search only clears and overwrites the scratchpad arrays.
Allocate it once with capacity greater than or equal to graph.PolygonCount,
reuse it for every query, and dispose it when the owner is destroyed.

```csharp
using Unity.Collections;

namespace BurstNav.Guide
{
    public static class BnFlatSearch
    {
        public static BnPathSearchStatus Run(
            ref BnMeshGraphBlob graph,
            int startPoly,
            int targetPoly,
            int maxIterations,
            out float pathCost)
        {
            var scratch = BnPathScratchpad.Create(graph.PolygonCount, Allocator.Temp);
            try
            {
                var found = BnAStar.FindPath(ref graph, startPoly, targetPoly, ref scratch, maxIterations);
                pathCost = found ? scratch.PathCost : 0f;
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

The scratchpad exposes the result through stable fields:

| Field | Meaning |
| --- | --- |
| Status | Outcome of the last search. |
| Path | Polygon sequence, start polygon first. |
| PathCount | Number of valid polygon ids in Path. |
| PathCost | Total cost of the found path. |
| Iterations | Number of expanded polygons. |
| HeuristicScale | Multiplier applied to the Euclidean heuristic. |

BnPathSearchStatus distinguishes the outcomes that matter to gameplay:

| Status | Meaning |
| --- | --- |
| Found | A path is available in the scratchpad. |
| Unreachable | The open set drained without reaching the target. |
| BudgetExhausted | maxIterations ran out before the target was expanded. |
| InvalidInput | A polygon id is outside the graph. |
| CapacityExceeded | The scratchpad is smaller than the graph. |

The pathfinding system maps a failed search to BnPathStatus.Failed and clears
the agent corridor. A budget cut-off is therefore a failure, not a partial path.

## Path slot and waypoints

BnAgentPath is the per-agent path slot:

| Field | Meaning |
| --- | --- |
| PathIndex | Stable integer identity of the slot. |
| WaypointCount | Number of valid waypoints in the buffer. |
| Status | None, Pending, Ready, Partial or Failed. |
| IsFollowable | True when the count is positive and the status is Ready or Partial. |

BnPathWaypoint is the path buffer element:

| Field | Meaning |
| --- | --- |
| Position | Waypoint position on the navigation plane. The last element is the exact target. |
| ExitPoint | Portal midpoint used to leave PolyIndex. It equals Position for the last element. |
| PolyIndex | Polygon that owns this waypoint. |
| LinkIndex | Index into the graph link table, or -1 for the final waypoint. |

The pathfinder fills the buffer with one element per polygon in the polygon
sequence. The first element is the start point and the last element is the
target point, so a corridor with n polygons has n waypoints.

## Waypoint progress

BnAgentSteering tracks how far the agent has advanced:

| Field | Meaning |
| --- | --- |
| WaypointIndex | Cursor into the path buffer: the waypoint the funnel steers toward. |
| PathProgress | Distance travelled along completed path segments. |
| PathLength | Total length of the current path. |
| Flags | Following, Arrived and Blocked bits. |
| NormalizedProgress | Saturated PathProgress / PathLength, or 0 without a path. |

BnFunnel.AdvancePathProgress moves WaypointIndex forward when the agent crosses
the portal that separates the current waypoint from the next one. It also
rewrites PathProgress to the sum of completed segment lengths. The funnel calls
this pass automatically at the start of Solve, so you usually only read these
fields.

## Kinematic funnel

BnFunnel is a string-pulling solver over the portal corridor. For each frame it:

1. Advances the cursor past every portal the agent has already crossed.
2. Tightens a left/right cone from the agent position through the remaining
   portals.
3. Returns the first funnel corner, or the final waypoint when the string stays
   straight.

BnFunnel.Solve then writes a planar desired velocity:

```text
direction = normalize(waypoint - position)
DesiredVelocity = direction * Min(MaxSpeed, distance / deltaTime)
```

Z is height and is not steered; DesiredVelocity.z is always zero. When the
agent is on the last waypoint within max(Radius, BnFunnel.DefaultArrivalRadius),
Solve writes a zero desired velocity and sets BnAgentFlags.Arrived.

The arrival tolerance is therefore not a separate component field. It is the
larger of the agent Radius and the constant 0.05, which keeps a zero-radius
agent from jittering on the final point.

## Kinematic bypass and why there is no avoidance

Free resolves velocity with KinematicBypassJob. The job is intentionally a
direct copy:

```csharp
using Unity.Entities;

namespace BurstNav.Guide
{
    public static class BnFreeVelocity
    {
        public static BnAgentSteering Resolve(BnAgentSteering state) =>
            KinematicBypassJob.Bypass(state);
    }
}
```

There is no neighbour query, no broadphase and no velocity-obstacle solve in
this stage. An agent follows its funnel exactly and treats every other agent as
non-existent. Two agents on crossing paths can overlap, and a stationary agent
never pushes a moving one aside.

This is a deliberate package boundary, not a tuning gap:

- The Free contract is deterministic waypoint following with zero allocation.
- Avoidance needs neighbour data and a shared solver, which Free does not
  allocate or schedule.
- Gameplay that needs separation must provide it at the design level, for
  example by spacing spawn points, using one-way lanes, or staggering start
  times.

If agents must stay apart, or must react to moving blockers, the Free pipeline
has no stage where that decision can be made without replacing the steering
system. See [limitations.md](limitations.md) for the exact scope.

## Static graph, one owner

The graph is a blob asset and is immutable. Changing polygon topology at
runtime means building a new blob and replacing the singleton. Exactly one
BnBlobMeshGraph must exist in a world; both systems skip their work when there
is no singleton or when more than one exists.

For the next step, read [configuration.md](configuration.md) to tune the search
budget and the steering profile, or [getting-started.md](getting-started.md) for
the end-to-end setup.
