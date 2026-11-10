# Stress Test

## What it shows

Ten thousand agents cross a 160 x 48 metre open field with scattered obstacles
and then walk back. The sample exercises the Free pipeline at scale: one baked
blob graph, one flat A* search per request, kinematic funnel steering for every
agent, and rendering through Entities Graphics with no GameObject per agent.

The level is authored in a SubScene:

| Object | Authoring | What it bakes |
| --- | --- | --- |
| `Field Graph` | `BnStressFieldAuthoring` | The `BnBlobMeshGraph` singleton with the obstacle mask. |
| `Walls` | Quad with `MeshFilter` and `MeshRenderer` | One static render entity for the obstacle quads. |
| `Floor` | Quad with `MeshRenderer` | One static render entity. |
| `Spawner` | `BnStressSpawnAuthoring` | The `BnStressSpawnConfig` singleton with the agent prefab reference, the agent count and the spawn rate. |
| `AgentPrefab` (prefab asset) | `BnAgentAuthoring`, `BnStressAgentAuthoring`, `MeshFilter`, `MeshRenderer` | The dominant agent archetype plus `BnStressRoute` and Entities Graphics render components. |

`BnStressField` scatters 2 x 1 to 3 x 2 cell obstacles over the whole field at
about 16 percent density: 311 blocked cells, 1609 free polygons and 5524
directed links. Obstacles reach the field edges, so no free highway runs along a
border, and any straight free run longer than 10 cells is split by one more
block. The generator flood-fills from the spawn band after every placement, so
the field always keeps a route from the western band to the eastern band while
leaving many alternative routes.

`BnStressSpawnSystem` instantiates the prefab in batches of `SpawnPerFrame`
behind the pathfinding system, gives each agent a random point inside a free
spawn cell, a random goal point in the far band and an individual speed between
3 and 6 m/s. Agents therefore spread over several routes instead of following
one line.

On arrival `BnStressGoalSystem` swaps `BnStressRoute.Home` and
`BnStressRoute.Goal` and re-arms the search request, so the crowd returns the
same way it came. `BnStressTransformSystem` copies `BnAgentSteering.Position`
into `LocalTransform` so Entities Graphics draws the current position.

## Requirements

- Unity 2022.3 or newer with Entities, Entities Graphics and URP.
- The Free package (`sniveler-code.dev.burstnav-ecs`).
- URP asset assigned in the project; the sample materials are URP/Unlit.

## How to run

1. Add `sniveler-code.dev.burstnav-ecs` through Window > Package Manager >
   Add package from disk and select the package's `package.json`.
2. Open Window > Package Manager, select BurstNav Free, open the Samples tab and
   import Stress Test.
3. Open `Stress Test.unity` and press Play.
4. The HUD shows the live agent count, frame rate and frame time. The crowd
   fills the field within a few seconds and returns after reaching the far band.
5. Select the `Stress Field` object and open the SubScene if you want to inspect
   or edit the field and the spawn settings.

## Knobs

| Field on `Spawner` | Default | Effect |
| --- | --- | --- |
| `AgentPrefab` | `Art/StressAgent.prefab` | Prefab every agent is instantiated from. |
| `AgentCount` | 10000 | Total agents spawned. |
| `SpawnPerFrame` | 64 | Agents instantiated per frame; also the number of A* searches per frame while the crowd fills. |
| `CellsX`, `CellsY`, `CellSize` | 80, 24, 2 | Field size; the obstacle rule follows the same values through `BnStressField`. |

`BnAgentAuthoring` on the prefab sets the base speed and radius; the spawn
system overrides the speed with a random 3 to 6 m/s per agent.

## Expected load

- Graph: 80 x 24 cells with 311 blocked cells, 1609 free polygons and 5524
  directed links.
- Pathfinding: one single-threaded A* per pending agent, throttled to the spawn
  batch and the arrival retry budget.
- Steering: funnel, bypass and integration run in parallel over the crowd.
- Rendering: Entities Graphics instanced quads, one entity per agent and no
  GameObject per agent.

Measured in Unity 6000.5.2f1 with URP and Entities Graphics 6.5.0: 10000
agents, 150-165 fps in the Play mode game view (about 6 ms per frame), zero
failed paths.

## Free API used

| Type | Role in the sample |
| --- | --- |
| `BnMeshGraphBlob`, `BnPolygon`, `BnLinkData` | Graph layout the field builder fills into a blob asset with `BlobBuilder`. |
| `BnBlobMeshGraph` | Singleton component that owns the blob. |
| `BnAgentComponents.CreateAgentArchetype` | Archetype contract the prefab extends with `BnStressRoute`. |
| `BnAgentSteering`, `BnAgentPath`, `BnAgentPathRequest` | Per-agent steering state, path slot and search request. |
| `BnPathWaypoint` | Corridor storage the funnel sweeps through. |
| `BnPathfindingSystem` | Runs flat A* and writes one corridor per request. |
| `BnSteeringSystem` | Runs `BnFunnelJob`, the kinematic bypass and integration. |
| `BnFunnel` | Turns the corridor portals into a desired velocity. |
| `BnStressSpawnSystem` | Streams prefab instances and seeds route and request state. |
| `BnStressGoalSystem` | Swaps the route and re-arms the request on arrival. |
| `BnStressTransformSystem` | Copies steering positions into `LocalTransform` for Entities Graphics. |

The sample uses no crowd avoidance: agents overlap, and the demo shows
throughput and steering cost rather than separation. Add a post-steering stage
yourself if the crowd must stay apart.
