# BurstNav Free

`sniveler-code.dev.burstnav-ecs` is the static navigation package for Unity DOTS. It resolves
paths with flat A* over a baked polygon graph and moves agents with a kinematic
funnel. The steady-state pipeline allocates nothing: the graph is a blob asset,
the search reuses one fixed scratchpad, and the steering jobs write chunk arrays
only.

## What this package includes

| Layer | Implementation | Main types |
| --- | --- | --- |
| Navigation graph | Static polygon graph in a blob asset | `BnMeshGraphBlob`, `BnPolygon`, `BnLinkData` |
| Graph owner | Singleton component pointing at the blob | `BnBlobMeshGraph` |
| Search | Flat A* with a fixed iteration budget | `BnAStar`, `BnPathScratchpad`, `BnLinkFilter` |
| Path storage | Per-agent path slot and waypoint buffer | `BnAgentPath`, `BnPathWaypoint` |
| Steering | Kinematic funnel over portal edges | `BnFunnel` |
| Velocity | Kinematic bypass: desired velocity becomes velocity | `KinematicBypassJob` |
| Integration | Position advanced by velocity and delta time | `IntegrateJob` |
| Environment | Static navmesh only | `BnMeshAuthoring` |
| Agent authoring | One dominant agent archetype | `BnAgentAuthoring`, `BnAgentSteering` |

The package has no local avoidance stage, no moving obstacle store and no
tag-gated links. Read [limitations.md](Documentation~/limitations.md) before you
choose the package for a level.

## Runtime flow

```text
BnAgentPathRequest
  -> BnPathfindingSystem / BnPathfindJob
  -> flat A* over BnMeshGraphBlob
  -> BnAgentPath + BnPathWaypoint buffer
  -> BnSteeringSystem / BnFunnel
  -> BnAgentSteering.DesiredVelocity
  -> KinematicBypassJob
  -> BnAgentSteering.Velocity
  -> IntegrateJob
  -> BnAgentSteering.Position
```

## Samples

`sniveler-code.dev.burstnav-ecs` owns its samples under `Samples~`. Open the Package
Manager, select **BurstNav Free**, open the **Samples** tab and import Hello
Navmesh or Funnel Steering. The generated copy lands under
`Assets/Samples/BurstNav Free/<version>/`.

The full sample guide, including the per-sample API table, run steps and
troubleshooting, is in [Documentation~/samples.md](Documentation~/samples.md).
The samples are copied out of `Samples~`, so the package copy stays untouched.

## Documentation

| Page | Use it for |
| --- | --- |
| [Samples](Documentation~/samples.md) | Import the Free samples, run them and fix common import problems. |
| [Getting started](Documentation~/getting-started.md) | Import the package, bake a graph, spawn an agent and move it. |
| [Concepts](Documentation~/concepts.md) | Polygons, portals, scratchpads, waypoint progress and the kinematic bypass. |
| [Configuration](Documentation~/configuration.md) | Search budget, speed and arrival tuning, Burst settings and assembly references. |
| [Limitations](Documentation~/limitations.md) | Flat-search limits, missing crowd avoidance and how to extend beyond Free. |
| [API reference](Documentation~/api/Free.md) | Complete Free signatures and field layouts. |
| [Cookbook](Documentation~/cookbook/Free.md) | End-to-end Free recipes. |
| [Documentation home](Documentation~/README.md) | Free overview and page map. |

## Requirements

- Unity 2022.3, as declared in `package.json`.
- `com.unity.entities` 1.2.4, `com.unity.burst` 1.8.12,
  `com.unity.collections` 2.4.0 and `com.unity.mathematics` 1.3.1, as declared
  in `package.json`. A host project may pin newer DOTS releases; the package is
  verified in Unity 6000.5.2f1 with `com.unity.entities` 6.5.0.
- A project that references the `BurstNav.Free` assembly.
- No scripting define and no editor setting.

## Assembly

The runtime assembly is `BurstNav.Free` with root namespace `BurstNav`. It has
`allowUnsafeCode` enabled and is auto-referenced. It references only Unity DOTS
assemblies. Gameplay code that uses Free types adds `BurstNav.Free` to its own
assembly definition.

## Package scope

This repository contains the Free package only. There is no Pro package, no
shared documentation folder and no editor tooling: everything a project needs
is under `Runtime`, `Samples~` and `Tests`. Each package release is a
self-contained version (`1.1.0` in `package.json`), and the sample import path
is `Assets/Samples/BurstNav Free/<version>/`.
