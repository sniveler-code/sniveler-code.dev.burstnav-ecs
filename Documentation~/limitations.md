# BurstNav Free limitations

Free is built for small static levels with a modest number of agents. The
limits below follow directly from the architecture, so plan around them rather
than trying to tune them away.

## Flat search scaling

Every request runs one flat A* over the whole polygon graph. Cost grows with the
number of expanded polygons, and every agent searches the same graph from
scratch. There is no cluster decomposition, no abstract graph and no precomputed
long-distance route table.

The practical consequences are:

| Graph size | Behaviour |
| --- | --- |
| Tens to a few thousand polygons | The default 4096-expansion budget covers typical paths. |
| Several thousand polygons | Raise MaxIterations or paths may fail on long routes. |
| Tens of thousands of polygons | Each request becomes expensive and the scratchpad grows with the graph. |

The scratchpad is O(polygon count) in memory (roughly 29 bytes per polygon plus
12 bytes of spare heap and visit-stamp slots). It is reused, so it does not
allocate per frame, but it must be large enough for the whole graph. A large
graph therefore has a fixed memory floor even when agents only travel short
distances.

Pathfinding is also all-or-nothing in the systems: when the budget runs out, the
request becomes BnPathStatus.Failed and the corridor is cleared. Free does not
emit a partial path for the agent to follow toward the target.

## No crowd avoidance

Free has no neighbour query and no local avoidance solve. KinematicBypassJob
copies DesiredVelocity into Velocity unchanged, so every agent follows its
funnel as if no other agent existed.

Observable effects:

- Agents on crossing paths can overlap.
- A stopped agent does not block or deflect a moving agent.
- Dense groups do not separate, queue or spread out around corners.
- Throughput through a narrow portal is not regulated.

The funnel still produces smooth single-agent paths through portal edges. The
missing behaviour is social: nothing in Free reads another agent's position or
velocity during steering.

If your design needs separation, you must provide it outside the runtime, for
example with spaced spawn points, one-way lanes, or staggered start times. A
custom post-steering separation job would replace the Free steering contract and
is not part of the supported Free pipeline.

## No dynamic obstacles

The navigation graph is an immutable blob asset. Free has no runtime obstacle
store, no obstacle query and no broadphase to update when a blocker moves.

Consequences:

- Moving a wall, crate or vehicle does not change the graph or the agent paths.
- An agent keeps following a corridor even if something now occupies it.
- Adding or removing blocked regions requires rebuilding the blob and replacing
  the graph singleton.
- There is no built-in push-out when an agent overlaps a static or moving
  blocker.

You can bake a different graph for a different level state, but swapping graphs
at runtime invalidates existing corridors and can force every agent to repath.
That is not the dynamic-obstacle workflow Free supports.

## No door tags

Free links carry no door or area tags, and the Free pathfinding system always
calls the unfiltered A* overload. Every portal in the graph is traversable.

Consequences:

- You cannot close a doorway for one agent and leave it open for another.
- You cannot route around a locked area through the Free systems.
- Access control must be enforced by gameplay before a request is issued, and
  the graph itself must not contain the forbidden region.
- A closed door is a graph change, which means a rebake and a repath, not a
  per-link flag.

`BnAStar.FindPath` does expose filtered overloads that take a `BnLinkFilter` and
an optional per-link tag mask array, and `BnLinkFilter` is part of this
assembly. Using them requires your own job or system: `BnPathfindJob` rewrites
it with `BnLinkFilter.PassThrough` and no tag masks, so a filtered search is not
available through `BnPathfindingSystem`.

## Static graph and one owner

Free assumes one static graph per world:

- Exactly one BnBlobMeshGraph singleton. Both systems skip their work when there
  is no singleton or when more than one exists.
- A graph must be created before agents request paths. BnPathfindingSystem
  returns early while graph.Value is not created.
- Graph topology is fixed for the lifetime of the blob. Replacing the blob while
  searches are in flight is not supported by the Free systems.

## Extending beyond Free

Free stops at the architecture described on this page. If a level needs more,
the options are:

- Replace or wrap the steering system when agents must stay apart in crowds,
  queues or narrow corridors. Free has no separation stage to tune.
- Replace the pathfinding job when the flat search is the frame-time bottleneck,
  long routes hit MaxIterations, or paths need a different filter.
- Rebuild and swap the graph blob when obstacles move or the level changes
  without a full rebake. Replacing the blob while searches are in flight is not
  supported by the Free systems.
- Enforce doors and areas before a request is issued, or write a filtered search
  job around `BnAStar.FindPath`.

Each option replaces a Free pipeline stage rather than changing a field, because
the package deliberately has no extension points: the six agent components, the
five jobs and the two systems are the whole contract.

To revisit the Free architecture, return to [concepts.md](concepts.md) or
[configuration.md](configuration.md). For setup, see
[getting-started.md](getting-started.md) and the [Free overview](README.md).
