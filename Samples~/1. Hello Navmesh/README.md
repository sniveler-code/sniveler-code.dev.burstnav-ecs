# Hello Navmesh

## What it shows

A single agent that walks a navigation grid from one corner to the opposite
corner and keeps patrolling back and forth. The sample shows the whole Free
pipeline in one scene: a polygon graph stored in a blob asset, a flat A* search
that fills the agent corridor, and kinematic funnel steering that turns the
corridor into a velocity each frame.

The two authoring MonoBehaviours in the scene are inspector parameter holders.
This sample has no SubScene, so `BnHelloNavmeshBootstrap` reads `BnMeshAuthoring`
and `BnAgentAuthoring` when Play starts, builds the graph blob with
`BnHelloNavmeshGraph` from the Free graph data types and spawns the agent from
`BnAgentComponents.CreateAgentArchetype`; no baker runs and nothing is baked in
the editor.

While Play mode runs, the Scene view draws the grid, the agent sphere and the
current corridor as gizmos. In the Entities Hierarchy, `BnAgentPath.Status`
turns `Ready`, `BnAgentSteering.WaypointIndex` and `PathProgress` grow, and
`BnAgentSteering.Flags` gains `Arrived` on arrival, after which the agent
requests the opposite corner again.

## How to run

1. Add `sniveler-code.dev.burstnav-ecs` through Window > Package Manager > Add package from
   disk and select the package's `package.json`.
2. Open Window > Package Manager, select BurstNav.Free, open the Samples tab and
   import Hello Navmesh.
3. Open `Hello Navmesh.unity` and press Play.
4. Select the `Gizmos` object to draw the grid, the corridor and the agent in
   the Scene view, and inspect the agent entity in the Entities Hierarchy.

## Free API used

| Type | Role in the sample |
| --- | --- |
| `BnMeshAuthoring` | Inspector parameter holder for the 4 x 4 grid; the bootstrap reads its fields at Play start. |
| `BnAgentAuthoring` | Inspector parameter holder for speed, radius and the initial steering toggle. |
| `BnMeshGraphBlob`, `BnPolygon`, `BnLinkData` | Free graph layout the helper fills into a blob asset with `BlobBuilder`. |
| `BnHelloNavmeshGraph` | Sample helper that allocates the grid blob and returns it to the bootstrap. |
| `BnAgentComponents.CreateAgentArchetype` | Creates the dominant agent archetype the bootstrap spawns from. |
| `BnAgentSteering`, `BnAgentPath`, `BnAgentPathRequest` | Per-agent steering state, path slot and search request. |
| `BnHelloNavmeshAgent` | Sample marker that scopes the goal system and the gizmos to this demo's agent. |
| `BnPathWaypoint` | Buffer that holds the corridor polygon by polygon. |
| `BnPathfindingSystem` | Runs flat A* inside `BnPathfindJob` and writes the corridor. |
| `BnSteeringSystem` | Runs the funnel, the kinematic bypass and the integration jobs. |
| `BnFunnel` | Turns the corridor portals into a desired velocity. |
