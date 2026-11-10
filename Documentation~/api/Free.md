# Free API reference

This page lists the Free API surface. Every type lives in the `BurstNav`
namespace. Free uses flat A* pathfinding, kinematic funnel steering and a static
navigation graph; there is no hierarchical search, crowd avoidance or dynamic
obstacle stage in this package. For task-oriented usage, read the [Free user
guide](../README.md). The `BnLinkFilter` type and the filtered `FindPath`
overloads are documented on this page, because Free compiles them too.

## Conventions

- Package: every row on this page is available in Free.
- Allocation: components, jobs and static APIs own no native memory unless the
  table says so. Native containers are borrowed handles owned by the caller or
  by the system that created them.
- Threading: jobs are scheduled by systems. Completion is deferred to the
  system and its command-buffer barrier; the main thread never waits on a job
  handle inside `OnUpdate`.
- Free does not expose package-specific fields. The field lists below are the
  complete Free layout of each type.
- Parameter tables use `In` for values read by the callee, `Out` for values
  written by the callee, and `In/out` for values read and rewritten. A `ref`
  parameter never transfers ownership; the caller keeps the storage alive for
  the duration of the call.

## Graph types

| Type | Namespace | Kind | Package | Fields (declaration order) | Allocation owner |
| --- | --- | --- | --- | --- | --- |
| `BnBlobMeshGraph` | `BurstNav` | `IComponentData` | Free | `Value: BlobAssetReference<BnMeshGraphBlob>` | World entity owns the component; the blob asset owner owns `Value`. |
| `BnMeshGraphBlob` | `BurstNav` | Blob struct | Free | `Vertices: BlobArray<float2>`, `Polygons: BlobArray<BnPolygon>`, `NeighborPolygons: BlobArray<int>`, `Links: BlobArray<BnLinkData>`, `PolyLinkOffsets: BlobArray<int>`, `PolyLinkCounts: BlobArray<int>` | Blob asset owner; immutable after bake. |
| `BnPolygon` | `BurstNav` | Blob struct | Free | `Center: float2`, `VertexOffset: int`, `VertexCount: int`, `NeighborOffset: int`, `NeighborCount: int` | Blob asset owner; stored inside `BnMeshGraphBlob.Polygons`. |
| `BnLinkData` | `BurstNav` | Blob struct | Free | `PortalLeft: float2`, `PortalRight: float2`, `Target: int`, `Cost: float` | Blob asset owner; stored inside `BnMeshGraphBlob.Links`. |

### BnBlobMeshGraph

The component grants one entity ownership of an immutable navigation graph.
The blob reference keeps topology out of the component, so a job reads the
graph without copying arrays or resolving components per agent. The world owns
the component; the code that created the blob asset owns and disposes the
reference.

### BnMeshGraphBlob

The blob stores vertices, polygons, neighbour ids, directed links and two
per-polygon link indices as dense pools. Per-polygon data is addressed with
offset/count pairs, so a search walks memory sequentially. `PolygonCount`,
`LinkCount`, `VertexCount`, `GetPolygonCenter`, `GetPolyLinkOffset`,
`GetPolyLinkCount`, `GetPolyLink`, `GetNeighborCount`, `GetNeighbor`,
`GetVertexCount` and `GetVertex` are read-only accessors. The struct is
immutable and safe to read from parallel jobs. Free compiles only the fields
listed above.

### BnPolygon

`BnPolygon` describes one navigation polygon. `Center` is the heuristic anchor,
`VertexOffset` and `VertexCount` address `BnMeshGraphBlob.Vertices`, and
`NeighborOffset` and `NeighborCount` address
`BnMeshGraphBlob.NeighborPolygons`. The struct owns no memory and is read-only
after the bake.

### BnLinkData

`BnLinkData` is one directed portal. `PortalLeft` and `PortalRight` are the
shared edge endpoints, `Target` is the destination polygon id and `Cost` is the
authored traversal cost. A non-positive cost makes the search use the
centre-to-centre distance. Free has no portal tag field, so every Free link is
unfiltered. The struct owns no memory.

## Agent types

| Type | Namespace | Kind | Package | Fields (declaration order) | Allocation owner |
| --- | --- | --- | --- | --- | --- |
| `BnAgentSteering` | `BurstNav` | `IComponentData` | Free | `Position: float3`, `Velocity: float3`, `DesiredVelocity: float3`, `MaxSpeed: float`, `Radius: float`, `PathProgress: float`, `PathLength: float`, `WaypointIndex: int`, `Flags: BnAgentFlags`; members: `Idle(float3, float, float)`, `HasPath`, `NormalizedProgress` | Entity chunk; no owned allocation. |
| `BnAgentPath` | `BurstNav` | `IComponentData` | Free | `PathIndex: int`, `WaypointCount: int`, `Status: BnPathStatus`; member: `IsFollowable` | Entity chunk; waypoints live in the agent buffer. |
| `BnPathWaypoint` | `BurstNav` | `IBufferElementData` | Free | `Position: float3`, `ExitPoint: float3`, `PolyIndex: int`, `LinkIndex: int`; member: `Create(float3, float3, int, int)` | Entity buffer; inline capacity `BnAgentComponents.DefaultPathCapacity`. |
| `BnAgentPathRequest` | `BurstNav` | `IComponentData` | Free | `Start: float3`, `Target: float3`, `RequestId: uint`, `StartPoly: int`, `TargetPoly: int`, `Status: BnPathStatus`; member: `Create(float3, float3, uint)` | Entity chunk; no owned allocation. |
| `BnAgentSteeringActive` | `BurstNav` | `IComponentData`, `IEnableableComponent` | Free | none | Entity chunk enable mask. |
| `BnAgentPathRequestPending` | `BurstNav` | `IComponentData`, `IEnableableComponent` | Free | none | Entity chunk enable mask. |
| `BnAgentComponents` | `BurstNav` | Static class | Free | `DefaultPathCapacity: const int`; member: `CreateAgentArchetype(EntityManager)` | None. |
| `BnPathStatus` | `BurstNav` | Enum (`byte`) | Free | `None = 0`, `Pending = 1`, `Ready = 2`, `Partial = 3`, `Failed = 4` | None. |
| `BnAgentFlags` | `BurstNav` | Flags enum (`byte`) | Free | `None = 0`, `Following = 1 << 0`, `Arrived = 1 << 1`, `Blocked = 1 << 2` | None. |

### BnAgentSteering

`BnAgentSteering` is the kinematic state consumed by the funnel, bypass and
integration jobs. XY is the navigation plane and Z is height. `DesiredVelocity`
is the requested velocity, `Velocity` is the resolved velocity and `Position`
is integrated from it. `PathProgress`, `PathLength` and `WaypointIndex` are
written by the funnel. The struct owns no native containers; read and write it
through chunk arrays, never through per-entity component access in a hot loop.

`HasPath` is true when `PathLength > 0` and `BnAgentFlags.Following` is set.
`NormalizedProgress` returns `PathProgress / PathLength` saturated to the 0..1
range, or zero when no path is assigned. Both are branch-free read-only
properties.

#### BnAgentSteering.Idle

```text
public static BnAgentSteering Idle(float3 position, float maxSpeed, float radius)
```

| Overload | Parameter | Type | Direction/ownership | Precondition |
| --- | --- | --- | --- | --- |
| `BnAgentSteering.Idle` | `position` | `float3` | In; copied into the returned value. | Any world position; XY is the navigation plane. |
| `BnAgentSteering.Idle` | `maxSpeed` | `float` | In; copied. | Non-negative speed in units per second. |
| `BnAgentSteering.Idle` | `radius` | `float` | In; copied. | Non-negative agent radius. |

The returned value has zero velocity, `BnAgentFlags.None` and no path. It owns
no memory.

### BnAgentPath

`BnAgentPath` is the per-agent path slot. `PathIndex` is a stable integer
identity assigned once, `WaypointCount` is the number of valid elements in the
agent's `BnPathWaypoint` buffer, and `Status` is the lifecycle state.
`IsFollowable` is true for a non-empty `Ready` or `Partial` path. The component
owns no memory; the entity owns the buffer.

### BnPathWaypoint

One `BnPathWaypoint` is one corridor element: the waypoint position, the portal
exit used by the funnel, the polygon id and the link index used to leave that
polygon. The element is blittable and stored in the agent's dynamic buffer,
which the entity owns.

#### BnPathWaypoint.Create

```text
public static BnPathWaypoint Create(float3 position, float3 exitPoint, int polyIndex, int linkIndex)
```

| Overload | Parameter | Type | Direction/ownership | Precondition |
| --- | --- | --- | --- | --- |
| `BnPathWaypoint.Create` | `position` | `float3` | In; copied into the returned element. | Waypoint position on the navigation plane. |
| `BnPathWaypoint.Create` | `exitPoint` | `float3` | In; copied. | Portal exit towards the next waypoint. |
| `BnPathWaypoint.Create` | `polyIndex` | `int` | In; copied. | Valid polygon id for the corridor element. |
| `BnPathWaypoint.Create` | `linkIndex` | `int` | In; copied. | Valid link id, or `-1` when there is no outgoing link. |

`Create` copies the four arguments unchanged. The returned element owns no
memory.

### BnAgentPathRequest

`BnAgentPathRequest` is a gameplay-authored search request. `RequestId` is a
caller-owned counter: `BnPathfindJob` copies it through untouched and never
compares it, so the Free pipeline cannot detect a stale result. `StartPoly` and
`TargetPoly` are `-1` until the pathfinding job resolves them. The component
owns no memory and is written only through the chunk array in the job.

#### BnAgentPathRequest.Create

```text
public static BnAgentPathRequest Create(float3 start, float3 target, uint requestId)
```

| Overload | Parameter | Type | Direction/ownership | Precondition |
| --- | --- | --- | --- | --- |
| `BnAgentPathRequest.Create` | `start` | `float3` | In; copied into the returned request. | World-space search start. |
| `BnAgentPathRequest.Create` | `target` | `float3` | In; copied. | World-space search target. |
| `BnAgentPathRequest.Create` | `requestId` | `uint` | In; copied. | Monotonic id, normally greater than the previous request. |

The returned request has `Status = BnPathStatus.Pending`, `StartPoly = -1` and
`TargetPoly = -1`. It owns no memory.

### BnAgentSteeringActive and BnAgentPathRequestPending

Both are empty enableable components used as toggles. Disabling one flips a bit
in the chunk enable mask and moves no entity between archetypes. The pathfinding
job records the request-toggle change, and the steering state job records the
steering-toggle change, for end-of-frame playback.

### BnAgentComponents

`DefaultPathCapacity` is the inline `BnPathWaypoint` capacity before a buffer
spills to heap storage. The class owns no memory and is safe to call during
world setup, not per frame.

#### BnAgentComponents.CreateAgentArchetype

```text
public static EntityArchetype CreateAgentArchetype(EntityManager entityManager)
```

| Overload | Parameter | Type | Direction/ownership | Precondition |
| --- | --- | --- | --- | --- |
| `BnAgentComponents.CreateAgentArchetype` | `entityManager` | `EntityManager` | In; borrowed, not stored. | Valid world entity manager on the main thread. |

The created archetype contains `BnAgentSteering`, `BnAgentPath`,
`BnPathWaypoint`, `BnAgentPathRequest`, `BnAgentSteeringActive` and
`BnAgentPathRequestPending`. Call it during world setup, not per frame.

### BnPathStatus and BnAgentFlags

`BnPathStatus` tracks a path from request to result. `BnAgentFlags` is a
`[System.Flags]` bit set: `Following` marks active path following, `Arrived`
marks the final waypoint, and `Blocked` is reserved for gameplay. No Free code
sets or reads `Blocked`, so a custom system owns both the meaning and the
reaction. Both enums are backed by `byte` so the containing structs stay
blittable and compact.

## Search types

| Type | Namespace | Kind | Package | Fields (declaration order) | Allocation owner |
| --- | --- | --- | --- | --- | --- |
| `BnAStar` | `BurstNav` | Static class | Free | none | None; the caller supplies the scratchpad. |
| `BnPathScratchpad` | `BurstNav` | Struct, `IDisposable` | Free | `GScore: NativeArray<float>`, `CameFrom: NativeArray<int>`, `HeapPolygons: NativeArray<int>`, `HeapKeys: NativeArray<float>`, `HeapSlots: NativeArray<int>`, `Closed: NativeArray<byte>`, `Path: NativeArray<int>`, `VisitStamp: NativeArray<uint>`, `Capacity: int`, `HeapCount: int`, `PathCount: int`, `Iterations: int`, `SearchId: uint`, `PathCost: float`, `HeuristicScale: float`, `Status: BnPathSearchStatus`, `DefaultHeuristicScale: const float`; properties: `IsCreated`, `PathLength` | Caller; allocated by `Create` or `EnsureCapacity`, released by `Dispose` or `Dispose(JobHandle)`. |
| `BnLinkFilter` | `BurstNav` | Struct | Free | `BlockedTagMask: ushort`, `IgnoreTagged: ushort`, `AllowBlockedTags: ushort`; members: `PassThrough`, `Allows(ushort)` | None; value type. |
| `BnPathSearchStatus` | `BurstNav` | Enum (`byte`) | Free | `None = 0`, `Found = 1`, `Unreachable = 2`, `BudgetExhausted = 3`, `InvalidInput = 4`, `CapacityExceeded = 5` | None. |

### BnAStar

`BnAStar` is a branch-free flat A* over a `BnMeshGraphBlob`. It allocates
nothing: the search reuses the caller's `BnPathScratchpad`, clears state in
place and reports the result through the scratchpad. The Free signature is:

```text
bool BnAStar.FindPath(ref BnMeshGraphBlob graph, int startPoly, int targetPoly, ref BnPathScratchpad scratch, int maxIterations)
```

| Overload | Parameter | Type | Direction/ownership | Precondition |
| --- | --- | --- | --- | --- |
| `BnAStar.FindPath` | `graph` | `ref BnMeshGraphBlob` | In; borrowed for the call. | Blob must outlive the call. |
| `BnAStar.FindPath` | `startPoly` | `int` | In. | Stable polygon id inside `graph.PolygonCount`. |
| `BnAStar.FindPath` | `targetPoly` | `int` | In. | Stable polygon id inside `graph.PolygonCount`. |
| `BnAStar.FindPath` | `scratch` | `ref BnPathScratchpad` | In/out; caller owns the native arrays. | Must already address `graph.PolygonCount` polygons. |
| `BnAStar.FindPath` | `maxIterations` | `int` | In. | Positive expansion budget. |

The call returns `true` when a path was found and writes the polygon sequence to
`scratch.Path` with its length in `scratch.PathCount` and cost in
`scratch.PathCost`. A `false` result leaves `scratch.Status` explaining invalid
input, insufficient capacity, an unreachable target or an exhausted budget. The
function is Burst-compatible and safe to call from a job.

Free compiles three overloads:

```text
bool FindPath(ref BnMeshGraphBlob graph, int startPoly, int targetPoly, ref BnPathScratchpad scratch, int maxIterations)
bool FindPath(ref BnMeshGraphBlob graph, int startPoly, int targetPoly, ref BnPathScratchpad scratch, in BnLinkFilter filter, int maxIterations)
bool FindPath(ref BnMeshGraphBlob graph, int startPoly, int targetPoly, ref BnPathScratchpad scratch, in BnLinkFilter filter, in NativeArray<ushort> linkTagMasks, int maxIterations)
```

`filter` and `linkTagMasks` are `in` parameters, not values. Burst rejects structs passed by value to an external `[BurstCompile]` function (BC1064), so the reference form is part of the contract: call with a local, `BnLinkFilter.PassThrough` or `default`.

`BnLinkFilter` is a branch-free link predicate. `PassThrough` (the default
value) allows every link. A non-zero `IgnoreTagged` rejects every link that has
any tag bit set. With `IgnoreTagged` zero, a link is allowed unless one of its
tag bits is in `BlockedTagMask`; a non-zero `AllowBlockedTags` waives that block.
`Allows(ushort tagBits)` evaluates one link and is what the search calls per
outgoing link. `linkTagMasks` is optional: an index outside the array, or the
default `NativeArray`, means tag bits `0`. `BnPathfindJob` always uses the
unfiltered overload, so filtering only applies to searches you run yourself.

### BnPathScratchpad

`BnPathScratchpad` is reusable working memory for one flat search. The storage
is not uniformly indexed by polygon id:

- `GScore`, `CameFrom`, `HeapSlots`, `Closed` and `Path` are indexed by stable
  polygon id and have length `Capacity`.
- `HeapPolygons` and `HeapKeys` are indexed by heap slot, not by polygon id.
  `HeapPolygons[slot]` stores the polygon id in that slot and `HeapKeys[slot]`
  stores its f-score. Both arrays have length `Capacity + 1`; the extra slot
  lets the branch-free speculative push write one past the current heap count
  without a bounds check.
- `VisitStamp` is the per-polygon generation counter and also has length
  `Capacity + 1`. Its last slot holds `SearchId`, so a new search invalidates
  the previous state by incrementing the stamp instead of clearing five arrays;
  when the counter wraps, the rest of the array is cleared once.

`Create(polygonCapacity, allocator)` allocates once;
`EnsureCapacity(polygonCount, allocator)` grows only when the graph outgrows the
current capacity. `GetPathPolygon`, `CopyPathTo` and the `PathLength` property
read the last successful path. `DefaultHeuristicScale` is the default heuristic
multiplier used by newly allocated scratchpads. `IsCreated` reports whether the
native arrays are allocated. `Dispose` releases every native array and must run
exactly once per created scratchpad; `Dispose(JobHandle)` does the same after the
handle completes, which is how `BnPathfindingSystem` retires a resized
scratchpad without stalling the main thread. The caller owns the memory; the
search never allocates, and the scratchpad is not thread-safe while a search
mutates it.

#### BnPathScratchpad.Create and EnsureCapacity

```text
public static BnPathScratchpad Create(int polygonCapacity, Allocator allocator)
public void EnsureCapacity(int polygonCount, Allocator allocator)
```

| Overload | Parameter | Type | Direction/ownership | Precondition |
| --- | --- | --- | --- | --- |
| `BnPathScratchpad.Create` | `polygonCapacity` | `int` | In; sizes the allocation. | At least 1; normally `graph.PolygonCount`. |
| `BnPathScratchpad.Create` | `allocator` | `Allocator` | In; input enum. The caller owns the returned native arrays; `Dispose` releases them. | `Allocator.Persistent` for system-owned memory. |
| `BnPathScratchpad.EnsureCapacity` | `polygonCount` | `int` | In; minimum capacity to address. | At least 1; normally `graph.PolygonCount`. |
| `BnPathScratchpad.EnsureCapacity` | `allocator` | `Allocator` | In; input enum. The caller owns the reallocated native arrays; `Dispose` releases them. | Same allocator family used to dispose. |

`EnsureCapacity` is a no-op when the current capacity is already sufficient.
It reallocates in place otherwise and preserves `HeuristicScale`. Both calls
must run outside the search.

#### BnPathScratchpad path access and disposal

```text
public int GetPathPolygon(int index)
public int CopyPathTo(NativeArray<int> destination)
public void Dispose()
public void Dispose(JobHandle handle)
```

| Overload | Parameter | Type | Direction/ownership | Precondition |
| --- | --- | --- | --- | --- |
| `BnPathScratchpad.GetPathPolygon` | `index` | `int` | In. | `0 <= index < PathCount` after a successful search. |
| `BnPathScratchpad.CopyPathTo` | `destination` | `NativeArray<int>` | Out; caller owns the array. | Created array; the call copies at most its length. |
| `BnPathScratchpad.Dispose` | none | none | Releases every owned native array. | Call exactly once per created scratchpad. |
| `BnPathScratchpad.Dispose` | `handle` | `JobHandle` | In; the handle that must complete first. | The caller must not use the scratchpad or the handle afterwards. |

`CopyPathTo` returns the number of polygon ids written, which is
`min(PathCount, destination.Length)`.

### BnPathSearchStatus

`BnPathSearchStatus` is the outcome stored on the scratchpad. `Found` means the
path buffer is valid, `Unreachable` means the open set drained, and
`BudgetExhausted` means `maxIterations` was reached. `InvalidInput` and
`CapacityExceeded` are caller errors that no retry with the same arguments can
fix.

## Steering type

| Type | Namespace | Kind | Package | Fields (declaration order) | Allocation owner |
| --- | --- | --- | --- | --- | --- |
| `BnFunnel` | `BurstNav` | Static class | Free | `DefaultArrivalRadius: const float` | None. |

### BnFunnel

`BnFunnel` is the kinematic funnel (string-pulling) solver. `Solve` writes
`BnAgentSteering.DesiredVelocity`, `WaypointIndex`, `PathProgress`, `PathLength`
and `Flags` from the agent's `BnPathWaypoint` corridor and the graph portals.
The `DynamicBuffer` overloads and `NativeArray` overloads have identical
semantics. `AdvancePathProgress` advances the cursor over crossed portals, and
`GetNextWaypoint` returns the next funnel corner for a given corridor window.
`Solve` is the only Burst-compiled entry point; `AdvancePathProgress` and
`GetNextWaypoint` are plain static methods. All of them allocate nothing and
own no memory. `Solve` writes the steering value and also writes
`BnAgentPath.WaypointCount`, so the path slot is `In/out`; only the
`BnPathWaypoint` buffer and the graph are read-only. `AdvancePathProgress` has
the same write contract. The overload without `deltaTime` uses `0` as the frame
delta, so the desired speed is the full `MaxSpeed`.

#### BnFunnel.Solve overloads

```text
public static void Solve(ref BnAgentSteering steering, ref BnAgentPath path, in DynamicBuffer<BnPathWaypoint> waypoints, ref BnMeshGraphBlob graph, float deltaTime)
public static void Solve(ref BnAgentSteering steering, ref BnAgentPath path, in NativeArray<BnPathWaypoint> waypoints, ref BnMeshGraphBlob graph, float deltaTime)
public static void Solve(ref BnAgentSteering steering, ref BnAgentPath path, in DynamicBuffer<BnPathWaypoint> waypoints, ref BnMeshGraphBlob graph)
public static void Solve(ref BnAgentSteering steering, ref BnAgentPath path, in NativeArray<BnPathWaypoint> waypoints, ref BnMeshGraphBlob graph)
```

| Overload | Parameter | Type | Direction/ownership | Precondition |
| --- | --- | --- | --- | --- |
| `BnFunnel.Solve` (DynamicBuffer, with deltaTime) | `steering` | `ref BnAgentSteering` | In/out; caller owns the component storage. | Component must be writable. |
| `BnFunnel.Solve` (DynamicBuffer, with deltaTime) | `path` | `ref BnAgentPath` | In/out; caller owns the component storage. | Component must be writable. |
| `BnFunnel.Solve` (DynamicBuffer, with deltaTime) | `waypoints` | `DynamicBuffer<BnPathWaypoint>` | In; borrowed from the entity. | Buffer length must cover `path.WaypointCount`. |
| `BnFunnel.Solve` (DynamicBuffer, with deltaTime) | `graph` | `ref BnMeshGraphBlob` | In; borrowed for the call. | Blob must outlive the call. |
| `BnFunnel.Solve` (DynamicBuffer, with deltaTime) | `deltaTime` | `float` | In. | Positive frame delta; `0` means no speed clamp. |
| `BnFunnel.Solve` (NativeArray, with deltaTime) | `waypoints` | `NativeArray<BnPathWaypoint>` | In; borrowed native view. | Created array; length covers the corridor. |
| `BnFunnel.Solve` (DynamicBuffer, no deltaTime) | `deltaTime` | none | Implicit `0`. | Uses the full `MaxSpeed`. |
| `BnFunnel.Solve` (NativeArray, no deltaTime) | `deltaTime` | none | Implicit `0`. | Uses the full `MaxSpeed`. |

The overloads without `deltaTime` take the same `steering`, `path`, `waypoints`
and `graph` parameters with the same direction and preconditions as the
matching overload with `deltaTime`. All four write the steering value and
update `path.WaypointCount`; the path slot is `In/out` and only the `waypoints`
buffer and `graph` are read-only.

#### BnFunnel.AdvancePathProgress overloads

```text
public static int AdvancePathProgress(ref BnAgentSteering steering, ref BnAgentPath path, in DynamicBuffer<BnPathWaypoint> waypoints, ref BnMeshGraphBlob graph, float arrivalRadius)
public static int AdvancePathProgress(ref BnAgentSteering steering, ref BnAgentPath path, in NativeArray<BnPathWaypoint> waypoints, ref BnMeshGraphBlob graph, float arrivalRadius)
```

| Overload | Parameter | Type | Direction/ownership | Precondition |
| --- | --- | --- | --- | --- |
| `BnFunnel.AdvancePathProgress` (DynamicBuffer) | `steering` | `ref BnAgentSteering` | In/out; caller owns the component storage. | Component must be writable. |
| `BnFunnel.AdvancePathProgress` (DynamicBuffer) | `path` | `ref BnAgentPath` | In/out; caller owns the component storage. | Component must be writable. |
| `BnFunnel.AdvancePathProgress` (DynamicBuffer) | `waypoints` | `DynamicBuffer<BnPathWaypoint>` | In; borrowed from the entity. | Buffer length must cover `path.WaypointCount`. |
| `BnFunnel.AdvancePathProgress` (DynamicBuffer) | `graph` | `ref BnMeshGraphBlob` | In; borrowed for the call. | Blob must outlive the call. |
| `BnFunnel.AdvancePathProgress` (DynamicBuffer) | `arrivalRadius` | `float` | In. | Non-negative tolerance for degenerate portals. |
| `BnFunnel.AdvancePathProgress` (NativeArray) | `waypoints` | `NativeArray<BnPathWaypoint>` | In; borrowed native view. | Created array; length covers the corridor. |

The `NativeArray` overload takes the same `steering`, `path`, `graph` and
`arrivalRadius` parameters. Both overloads write the steering value and update
`path.WaypointCount`, then return the new waypoint index.

#### BnFunnel.GetNextWaypoint overloads

```text
public static float3 GetNextWaypoint(ref BnMeshGraphBlob graph, in DynamicBuffer<BnPathWaypoint> corridor, int startIndex, float3 agentPosition, float3 targetPosition)
public static float3 GetNextWaypoint(ref BnMeshGraphBlob graph, in NativeArray<BnPathWaypoint> corridor, int startIndex, float3 agentPosition, float3 targetPosition)
```

| Overload | Parameter | Type | Direction/ownership | Precondition |
| --- | --- | --- | --- | --- |
| `BnFunnel.GetNextWaypoint` (DynamicBuffer) | `graph` | `ref BnMeshGraphBlob` | In; borrowed for the call. | Blob must outlive the call. |
| `BnFunnel.GetNextWaypoint` (DynamicBuffer) | `corridor` | `DynamicBuffer<BnPathWaypoint>` | In; borrowed from the entity. | Ordered path buffer. |
| `BnFunnel.GetNextWaypoint` (DynamicBuffer) | `startIndex` | `int` | In. | Clamped to the corridor bounds internally. |
| `BnFunnel.GetNextWaypoint` (DynamicBuffer) | `agentPosition` | `float3` | In. | Current agent position. |
| `BnFunnel.GetNextWaypoint` (DynamicBuffer) | `targetPosition` | `float3` | In. | Final path target; its Z is preserved. |
| `BnFunnel.GetNextWaypoint` (NativeArray) | `corridor` | `NativeArray<BnPathWaypoint>` | In; borrowed native view. | Created array; ordered path buffer. |

The `NativeArray` overload takes the same `graph`, `startIndex`,
`agentPosition` and `targetPosition` parameters. Both overloads return the next
funnel corner as a `float3` and allocate nothing.

## Pathfinding job and system

| Type | Namespace | Kind | Package | Fields (declaration order) | Allocation owner |
| --- | --- | --- | --- | --- | --- |
| `BnPathfindJob` | `BurstNav` | `IJobChunk` | Free | `Scratch: BnPathScratchpad`, `Commands: EntityCommandBuffer`, `Graph: BlobAssetReference<BnMeshGraphBlob>`, `RequestHandle: ComponentTypeHandle<BnAgentPathRequest>`, `PathHandle: ComponentTypeHandle<BnAgentPath>`, `WaypointHandle: BufferTypeHandle<BnPathWaypoint>`, `EntityHandle: EntityTypeHandle`, `MaxIterations: int` | None; every container is borrowed from `BnPathfindingSystem`. |
| `BnPathfindingSystem` | `BurstNav` | `SystemBase` | Free | `DefaultMaxIterations: const int`, `MaxIterations: int`; `HasScratchpad: bool`, `ScratchpadCapacity: int` properties | System owns one persistent `BnPathScratchpad`; graph blob is borrowed. |

### BnPathfindJob

`BnPathfindJob` is a `[BurstCompile]` `IJobChunk`. Required input components:
`BnAgentPathRequest`, `BnAgentPath`, the `BnPathWaypoint` buffer and an enabled
`BnAgentPathRequestPending` toggle; the graph comes from the
`BnBlobMeshGraph` singleton. Written components: `BnAgentPathRequest`,
`BnAgentPath` and the `BnPathWaypoint` buffer. The job also records a
`SetComponentEnabled<BnAgentPathRequestPending>` command. It expands one flat
A* search per pending agent into the shared scratchpad and writes the corridor
directly through chunk accessors. The job schedules single-threaded because all
agents share one scratchpad. It never completes its own handle: completion is
deferred to `BnPathfindingSystem` and the end-of-frame command-buffer barrier.

### BnPathfindingSystem

`BnPathfindingSystem` schedules `BnPathfindJob` in `SimulationSystemGroup`. It
resolves the graph singleton once per update, creates the command buffer,
ensures its persistent scratchpad is large enough and chains the job handle into
`Dependency`. `MaxIterations` bounds each flat search.

When the graph outgrows the scratchpad, `EnsureScratchpad` allocates a new one,
swaps it in and calls `Dispose(JobHandle)` on the old arrays with the in-flight
search handle. The update is never skipped and the system never calls `Complete`
on that handle on the main thread. The system owns and disposes the scratchpad;
it owns no graph memory.

## Steering jobs and system

| Type | Namespace | Kind | Package | Fields (declaration order) | Allocation owner |
| --- | --- | --- | --- | --- | --- |
| `BnFunnelJob` | `BurstNav` | `IJobChunk` | Free | `Graph: BlobAssetReference<BnMeshGraphBlob>`, `SteeringHandle: ComponentTypeHandle<BnAgentSteering>`, `PathHandle: ComponentTypeHandle<BnAgentPath>`, `WaypointHandle: BufferTypeHandle<BnPathWaypoint>`, `DeltaTime: float` | None; handles are borrowed from `BnSteeringSystem`. |
| `BnSteeringStateJob` | `BurstNav` | `IJobChunk` | Free | `Commands: EntityCommandBuffer.ParallelWriter`, `PathHandle: ComponentTypeHandle<BnAgentPath>`, `ActiveHandle: ComponentTypeHandle<BnAgentSteeringActive>`, `EntityHandle: EntityTypeHandle` | None; command buffer is borrowed from the barrier. |
| `KinematicBypassJob` | `BurstNav` | `IJobChunk` | Free | `SteeringHandle: ComponentTypeHandle<BnAgentSteering>` | None; handle is borrowed. |
| `IntegrateJob` | `BurstNav` | `IJobChunk` | Free | `SteeringHandle: ComponentTypeHandle<BnAgentSteering>`, `DeltaTime: float` | None; handle is borrowed. |
| `BnSteeringSystem` | `BurstNav` | `SystemBase` | Free | none | Free owns no native memory. |

### BnFunnelJob

`BnFunnelJob` is a `[BurstCompile]` `IJobChunk`. Required input components:
`BnAgentSteering`, `BnAgentPath`, the `BnPathWaypoint` buffer and
`BnAgentSteeringActive`. The system schedules it on the steering query, which
includes `BnAgentSteeringActive` with
`EntityQueryOptions.IgnoreComponentEnabledState`, so disabled agents are still
visited and a newly assigned path can reactivate them. Written component:
`BnAgentSteering`; the job calls `BnFunnel.Solve` on a local copy of the path
slot and does not write the path slot or the waypoint buffer back. The job is
scheduled in parallel and never completes its handle; `BnSteeringSystem` owns
chaining.

### BnSteeringStateJob

`BnSteeringStateJob` is a `[BurstCompile]` `IJobChunk`. Required input
components: `BnAgentPath`, `BnAgentSteeringActive` and the entity handle. The
system schedules it on the same steering query as `BnFunnelJob`, which includes
`BnAgentSteeringActive` with `EntityQueryOptions.IgnoreComponentEnabledState`,
so the job can read the current toggle for every agent. Written state: the
enableable toggle, recorded as command-buffer commands. `IsSteerable` is the
pure predicate derived from `BnAgentPath.IsFollowable`. The job runs in
parallel with the funnel job and leaves completion to the system. Command
playback happens once, at frame end.

### KinematicBypassJob

`KinematicBypassJob` is a `[BurstCompile]` `IJobChunk` and the Free velocity
stage. Required and written component: `BnAgentSteering`; it copies
`DesiredVelocity` to `Velocity` through the pure `Bypass` helper. It runs on the
active query (`BnAgentSteering` plus `BnAgentSteeringActive`) and skips disabled
agents through `ChunkEntityEnumerator`. It allocates nothing and never completes
its handle; the system fans in its dependency before integration.

### IntegrateJob

`IntegrateJob` is a `[BurstCompile]` `IJobChunk`. Required and written
component: `BnAgentSteering`; it advances `Position` by `Velocity` multiplied by
`DeltaTime` with `math.mad`. It runs on the active query (`BnAgentSteering` plus
`BnAgentSteeringActive`) and skips disabled agents through
`ChunkEntityEnumerator`. The job is branch-free, allocates nothing and never
completes its handle.

### BnSteeringSystem

`BnSteeringSystem` schedules the Free steering chain in
`SimulationSystemGroup` and is ordered before
`EndSimulationEntityCommandBufferSystem`. The funnel job and the state job fan
out from the incoming dependency; their handles are combined with
`JobHandle.CombineDependencies` before `KinematicBypassJob`, and `IntegrateJob`
consumes the resolved handle. The system registers the final handle with the
barrier so enableable-component commands play back once at frame end. It uses
chunk arrays only: no per-entity component access in the loop. The system owns
no native memory on Free.

## Package tier

`BnTier` and `BnTierInfo` describe which package a build contains:

| Type | Namespace | Kind | Package | Fields | Allocation owner |
| --- | --- | --- | --- | --- | --- |
| `BnTier` | `BurstNav` | Enum (`byte`) | Free | `Free = 0`, `Pro = 1` | None. |
| `BnTierInfo` | `BurstNav` | Static class | Free | `Current: const BnTier`, `IsPro: const bool`, `Name: const string` | None. |

In this package `Current` is `BnTier.Free`, `IsPro` is `false` and `Name` is
`"Free"`. The enum keeps the `Pro` value so a future package can share it, but
no Free code reads the tier at runtime.

## Package boundary

Free does not expose package-specific fields or algorithms. Every public type and
member listed above is available in this package: paths come from flat A*,
velocity resolution is the kinematic bypass, and obstacles must be static in
the baked graph. Filtered searches exist at the `BnAStar.FindPath` level only;
the systems always run the unfiltered search.

## See also

- [Free user guide](../README.md) - tasks, recipes and tuning.
- [Package overview](../../README.md) - repository root README.
