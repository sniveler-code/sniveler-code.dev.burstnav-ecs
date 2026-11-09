# Funnel Steering

## What it shows

Four agents cross a navigation grid on diagonals that pass through each other.
Flat A* produces a stair-step corridor polygon by polygon; the kinematic funnel
sweeps the corridor portals and replaces the staircase with one smooth desired
velocity, so each agent cuts the corners of its own corridor instead of visiting
every polygon centre.

Each agent also carries `BnFunnelRoute`, the sample component that stores the two
endpoints of its crossing. The goal system flips the route when the agent
arrives, so the demo keeps running without a fixed end.

The agents follow the kinematic model of the Free package: velocity equals
desired velocity, so motion starts and stops in a single frame and agents do not
avoid each other. The two authoring MonoBehaviours in the scene are inspector
parameter holders: `BnFunnelSteeringBootstrap` reads them when Play starts,
builds the graph blob with `BnFunnelSteeringGraph` from the Free graph data
types and spawns every agent from `BnAgentComponents.CreateAgentArchetype`. The
scene has no SubScene, so no baker runs and nothing is baked in the editor.

While Play mode runs, the Scene view draws the grid, the agent spheres and their
corridors as gizmos.

## How to run

1. Add `com.burstnav.free` through Window > Package Manager > Add package from
   disk and select the package's `package.json`.
2. Open Window > Package Manager, select BurstNav.Free, open the Samples tab and
   import Funnel Steering.
3. Open `Funnel Steering.unity` and press Play.
4. Select the `Gizmos` object to draw the grid, the corridors and the agents in
   the Scene view, and inspect the agent entities in the Entities Hierarchy.
5. Change `AgentCount` on the `Bootstrap` object to spawn more agents on the same
   crossing routes.

## Free API used

| Type | Role in the sample |
| --- | --- |
| `BnMeshAuthoring` | Inspector parameter holder for the 6 x 6 grid; the bootstrap reads its fields at Play start. |
| `BnAgentAuthoring` | Inspector parameter holder for speed, radius and the initial steering toggle. |
| `BnMeshGraphBlob`, `BnPolygon`, `BnLinkData` | Free graph layout the helper fills into a blob asset with `BlobBuilder`. |
| `BnFunnelSteeringGraph` | Sample helper that allocates the grid blob and returns it to the bootstrap. |
| `BnAgentComponents.CreateAgentArchetype` | Creates the dominant agent archetype every agent is spawned from. |
| `BnAgentSteering`, `BnAgentPath`, `BnAgentPathRequest` | Per-agent steering state, path slot and search request. |
| `BnPathWaypoint` | Corridor storage the funnel sweeps through. |
| `BnPathfindingSystem` | Runs flat A* and writes one corridor per agent. |
| `BnSteeringSystem` | Runs `BnFunnelJob`, the kinematic bypass and integration. |
| `BnFunnel` | Advances waypoint progress and produces the corner-cutting velocity. |
| `BnFunnelRoute` | Sample component with the two route endpoints the goal system swaps on arrival. |
