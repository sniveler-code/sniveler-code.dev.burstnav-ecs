# BurstNav Free

BurstNav Free is the static navigation package for Unity DOTS. It resolves paths
with flat A* over a baked polygon graph and moves agents with a kinematic
funnel. The package stays zero-allocation in the steady state: the graph is a
blob asset, the search reuses one fixed scratchpad, and the steering jobs write
chunk arrays only.

## What Free includes

| Layer | Free implementation | Main types |
| --- | --- | --- |
| Navigation graph | Static polygon graph stored in a blob asset | BnMeshGraphBlob, BnPolygon, BnLinkData |
| Graph owner | Singleton component pointing at the blob | BnBlobMeshGraph |
| Search | Flat A* with a fixed iteration budget | BnAStar, BnPathScratchpad, BnLinkFilter |
| Path storage | Per-agent path slot and waypoint buffer | BnAgentPath, BnPathWaypoint |
| Steering | Kinematic funnel over portal edges | BnFunnel |
| Velocity | Kinematic bypass: desired velocity becomes velocity | KinematicBypassJob |
| Integration | Position advanced by velocity and delta time | IntegrateJob |
| Environment | Static navmesh only | BnMeshAuthoring |
| Agent authoring | One dominant agent archetype | BnAgentAuthoring, BnAgentSteering |

Free has no local avoidance stage, no moving obstacle store and no tag-gated
links. Read [limitations.md](limitations.md) before you choose the package for a
level.

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

## Page map

| Page | Use it for |
| --- | --- |
| [getting-started.md](getting-started.md) | Import the package, bake a graph, spawn an agent and move it. |
| [concepts.md](concepts.md) | Polygons, portals, scratchpads, waypoint progress and the kinematic bypass. |
| [configuration.md](configuration.md) | Search budget, speed and arrival tuning, Burst settings and assembly references. |
| [limitations.md](limitations.md) | Flat-search limits, missing crowd avoidance and how to extend beyond Free. |
| [api/Free.md](api/Free.md) | Complete Free signatures and field layouts. |
| [cookbook/Free.md](cookbook/Free.md) | End-to-end Free recipes. |
| [Documentation home](../README.md) | Package overview at the repository root. |

## Requirements

- Unity 2022.3, as declared in `package.json`.
- com.unity.entities 1.2.4, com.unity.burst 1.8.12,
  com.unity.collections 2.4.0 and com.unity.mathematics 1.3.1, as declared in
  `package.json`. A host project may pin newer DOTS releases; the package is
  verified in Unity 6000.5.2f1 with com.unity.entities 6.5.0.
- A project that references the `SnivelerCode.BurstNav.Runtime` assembly.
- No scripting define and no editor setting.

## Assembly and namespace

The runtime assembly is `SnivelerCode.BurstNav.Runtime` with root namespace `BurstNav`. It
enables unsafe code and is auto-referenced. It references only Unity DOTS
assemblies.

## Package scope

The Free package in this repository is standalone: `Runtime`, `Samples~` and
`Tests` are the whole package surface. There is no Pro package, no shared
documentation folder and no editor tooling, so every type and page referenced
here exists in this package. The version in `package.json` is `1.1.0`, which is
also the folder name Unity uses under `Assets/Samples/BurstNav Free/`.
