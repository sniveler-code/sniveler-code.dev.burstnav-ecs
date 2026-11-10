# Stress Test

## What it shows

Ten thousand agents cross a 160 x 48 metre serpentine field made of 80 x 24
cells of 2 metres and then walk back. The sample exercises the Free pipeline at
scale: one blob graph with roughly 1.4 thousand polygons, one flat A* search per
request, kinematic funnel steering for every agent and instanced rendering with
no GameObject per agent.

The field is generated, not authored. `BnStressField` walls off every sixth
column and leaves a two-cell gap that alternates between the bottom and the top,
so the only route through the level is a long serpentine corridor. Single-cell
pillars sit inside the corridors and cannot disconnect them. The first three
columns are the spawn band and the last three columns are the goal band; both
stay free of walls. The field yields 1574 free polygons and 5276 directed links.

Agents spawn in batches from the four-column spawn band, path to a random cell in
the goal band, and on arrival swap `BnStressRoute.Home` and `BnStressRoute.Goal`
and request the return path. The demo runs forever without a fixed end.

The scene has no SubScene and no bake step. `BnStressBootstrap` builds the graph
blob with `BlobBuilder` from the Free graph data types, creates the
`BnBlobMeshGraph` singleton, and spawns `SpawnPerFrame` agents from the dominant
archetype every frame until `AgentCount` is reached. `BnStressRender` reads the
steering positions and draws instanced quads with `Graphics.RenderMeshInstanced`,
so the cost of the crowd is the simulation, not ten thousand GameObjects.

## How to run

1. Add `sniveler-code.dev.burstnav-ecs` through Window > Package Manager >
   Add package from disk and select the package's `package.json`.
2. Open Window > Package Manager, select BurstNav Free, open the Samples tab and
   import Stress Test.
3. Open `Stress Test.unity` and press Play.
4. Watch the counter and the crowd. The field fills over a few seconds because
   agents are spawned and pathfound in batches.
5. Select the `Gizmos` object to draw the walls, the spawn band and the goal band
   in the Scene view.

## Knobs

| Field on `Bootstrap` | Default | Effect |
| --- | --- | --- |
| `AgentCount` | 10000 | Total agents spawned. |
| `SpawnPerFrame` | 64 | Agents added per frame; also the number of A* searches per frame while the crowd is filling. |
| `MaxSpeed` | 4 | Speed of every agent in metres per second. |
| `Radius` | 0.3 | Steering radius and arrival tolerance per agent. |
| `CellsX`, `CellsY`, `CellSize` | 80, 24, 2 | Field dimensions; the maze pattern follows them. |
| `StartActive` | true | Initial `BnAgentSteeringActive` toggle. |

## Expected load

- Graph: 80 x 24 cells, 1574 free polygons and 5276 directed links.
- Pathfinding: one single-threaded A* per pending agent. The batch is
  `SpawnPerFrame` while filling and the arriving agents share the same retry
  throttle, so a frame never schedules the whole crowd at once.
- Steering: funnel, bypass and integration run in parallel over the crowd.
- Rendering: instanced quads in batches of 1023, no per-agent GameObject.

Lower `SpawnPerFrame` if the first seconds are too spiky on a small machine, and
raise it when the editor has headroom.

## Free API used

| Type | Role in the sample |
| --- | --- |
| `BnMeshGraphBlob`, `BnPolygon`, `BnLinkData` | Graph layout the field builder fills into a blob asset with `BlobBuilder`. |
| `BnBlobMeshGraph` | Singleton component that owns the blob. |
| `BnAgentComponents.CreateAgentArchetype` | Dominant archetype every agent is spawned from. |
| `BnAgentSteering`, `BnAgentPath`, `BnAgentPathRequest` | Per-agent steering state, path slot and search request. |
| `BnPathWaypoint` | Corridor storage the funnel sweeps through. |
| `BnPathfindingSystem` | Runs flat A* and writes one corridor per request. |
| `BnSteeringSystem` | Runs `BnFunnelJob`, the kinematic bypass and integration. |
| `BnFunnel` | Turns the corridor portals into a desired velocity. |
| `BnStressBootstrap` | Builds the field and streams agents into the world. |
| `BnStressGoalSystem` | Swaps the route and re-arms the request on arrival. |
| `BnStressRender` | Instanced crowd rendering without GameObjects. |
| `BnStressGizmos` | Draws the walls and the two bands in the Scene view. |

The sample uses no crowd avoidance: agents overlap, and the demo shows
throughput and steering cost rather than separation. Add a post-steering stage
yourself if the crowd must stay apart.
