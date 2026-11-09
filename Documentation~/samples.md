# BurstNav Free samples

`com.burstnav.free` ships its own sample set under `Samples~` in the package
root. The folder is hidden from the asset database until you import a sample
through the Package Manager Samples tab. Every sample is
self-contained: one entry scene, a `Scripts` folder with its own assembly
definition, a `README.md`, and a paired `.meta` file for every file and folder.

## What ships with Free

| Folder | What it teaches | Key APIs |
| --- | --- | --- |
| `1. Hello Navmesh` (see [Hello Navmesh README](../Samples~/1.%20Hello%20Navmesh/README.md)) | Build a 4 x 4 static navigation grid at Play start, spawn one agent and patrol it between opposite corners with flat A* and the kinematic funnel. | `BnMeshAuthoring`, `BnAgentAuthoring`, `BnMeshGraphBlob`, `BnPolygon`, `BnLinkData`, `BnAgentComponents.CreateAgentArchetype`, `BnAgentSteering`, `BnAgentPath`, `BnAgentPathRequest`, `BnPathWaypoint`, `BnPathfindingSystem`, `BnSteeringSystem`, `BnFunnel` |
| `2. Funnel Steering` (see [Funnel Steering README](../Samples~/2.%20Funnel%20Steering/README.md)) | Send four agents across a 6 x 6 grid on crossing diagonals and show how the funnel replaces a stair-step corridor with corner-cutting velocity. | `BnMeshAuthoring`, `BnAgentAuthoring`, `BnMeshGraphBlob`, `BnPolygon`, `BnLinkData`, `BnAgentComponents.CreateAgentArchetype`, `BnAgentSteering`, `BnPathWaypoint`, `BnPathfindingSystem`, `BnSteeringSystem`, `BnFunnel`, sample route component `BnFunnelRoute` |

Folder numbering is contiguous from 1 inside the package. The sample assembly
definitions are `BurstNav.Free.Samples.HelloNavmesh` and
`BurstNav.Free.Samples.FunnelSteering`; each one references `BurstNav.Free` and
no other sample assembly.

## Import a sample

1. Open the project in Unity 2022.3 or newer and import the DOTS packages from
   [Free requirements](README.md#requirements).
2. Open **Window > Package Manager**.
3. Select **BurstNav Free**.
4. Open the **Samples** tab.
5. Import **Hello Navmesh** or **Funnel Steering**.
6. Wait for compilation. Unity copies the sample into
   `Assets/Samples/BurstNav Free/<version>/` and keeps the package copy under
   `Samples~` unchanged.

If you previously imported samples from an older package copy, delete the old
`Assets/Samples/BurstNav Free` folder before importing again: Unity matches the
copied assets by path and GUID, so a stale copy can shadow the new one.

## Run a sample

1. Open the scene named after the sample folder, for example
   `Hello Navmesh.unity`.
2. Enter Play mode.
3. Select the `Gizmos` object in the Hierarchy to draw the grid, the agent
   spheres and the active corridors in the Scene view.
4. Inspect the agent entities in the Entities Hierarchy.

Hello Navmesh patrols a single agent around a 4 x 4 grid; Funnel Steering sends
four agents across a 6 x 6 grid on crossing diagonals. The linked sample READMEs
own the per-frame outcomes and the per-sample tweaks.

Neither Free sample uses a SubScene. The bootstrap MonoBehaviour reads the
authoring components when Play starts, builds the graph blob from the Free graph
data types and spawns the agents, so the scene needs no bake step.

## Troubleshooting

| Symptom | Check |
| --- | --- |
| The sample is missing from the Samples tab | Confirm that `com.burstnav.free` is installed at version 1.1.0 and that `package.json` lists the sample under `samples`. Refresh the Package Manager window. |
| Imported scripts report missing references | Keep the generated `Scripts` folder and its `.asmdef` intact. The assembly references `BurstNav.Free` plus the DOTS assemblies; a manual move can break the reference. |
| The scene opens with missing scripts | Remove the old `Assets/Samples` copy, reimport the sample from the Samples tab and open the scene again. Two copies with different GUIDs can shadow each other. |
| Agents stay still in Play mode | Enter Play mode, check the Console for job or Burst errors, and confirm that the default world is running. The Free samples create their graph and agents at Play start and need an active DOTS world. |
| Gizmos are not visible | Select the `Gizmos` object and enable Gizmos in the Scene view toolbar. Gizmo drawing is a scene aid only; the simulation runs without it. |
| The sample scenes are not under `Assets` | That is expected before import. `Samples~` is hidden by design. Import through the Samples tab rather than copying the folder. |

Run the `BurstNav.Free.Tests.EditMode` and `BurstNav.Free.Tests.PlayMode`
assemblies from **Window > General > Test Runner** to exercise the coverage the
samples rely on. Like any package test set, they are compiled only when
`com.burstnav.free` is listed under `testables` in `Packages/manifest.json`.

## Related pages

- [Free getting started](getting-started.md) builds the same graph and agent
  path without the sample bootstrap.
- [Free concepts](concepts.md) explains polygons, portals, waypoints and the
  funnel.
- [Free API](api/Free.md) lists the complete Free surface used by the samples.
- [Free requirements](README.md#requirements) covers package and DOTS setup.

## Package scope

These two samples are the whole sample set of this package. There is no second
package with its own samples, and nothing outside `Samples~` is imported. When
you change a sample, edit the copy under `Samples~`: the `Assets/Samples/...`
copy is generated and is overwritten on the next import.
