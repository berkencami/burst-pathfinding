# Burst Path Finding

A high-performance, Job System–based A\* pathfinding framework for **Unity 6**, supporting both square and hexagonal grids with dynamic obstacle detection and multi-agent parallelism.

---
![ScreenShot](https://github.com/berkencami/burst-pathfinding/blob/main/Assets/sample.gif)

## Features

- **Burst-compiled parallel A\*** — multiple agents pathfind simultaneously via `IJobParallelFor`
- **Two grid types** — Square (8-directional) and Hexagon flat-top odd-q (6-directional)
- **Dynamic obstacle detection** — periodic `BoxcastCommand` scans update the grid in real time
- **Automatic re-pathing** — agents re-request paths automatically when obstacles change
- **Editor tooling** — custom inspectors, gizmo overlays, and a debug window

---

## How It Works

### Request Flow

1. `PathfindingAgent.SetDestination(target)` queues a request with `PathfindingManager`
2. Each `LateUpdate`, pending requests are batched into a single `BatchAStarJob` / `HexBatchAStarJob`
3. The job runs on worker threads (Burst-compiled); the main thread continues unblocked
4. The following `LateUpdate`, completed paths are delivered via callback on the main thread
5. `AgentMovementController` receives the waypoint list and moves the agent

### Dynamic Obstacles

`DynamicObstacleScanner` fires a grid-aligned `BoxcastCommand` batch at a configurable interval (default 0.25 s). When the scan finishes, `PathfindingManager.InvalidatePaths()` is called, queuing a re-path for all active agents.

### Grid Types

| Type | Directions | Heuristic |
|---|---|---|
| Square | 8 (with diagonal cost) | Octile distance |
| Hexagon (flat-top odd-q) | 6 | Cube distance |

---

## Quick Start

1. Add `PathfindingManager` (+ `PathfindingGrid`) to a GameObject in your scene
2. Configure grid size, node size, and grid type in the Inspector
3. Add `PathfindingAgent` and `AgentMovementController` to each agent GameObject
4. (Optional) Add `DynamicObstacleScanner` for real-time obstacle updates
5. Call `agent.SetDestination(worldPosition)` from any script

```csharp
// Request a path manually
PathfindingManager.Instance.RequestPath(
    transform.position,
    targetPosition,
    agentId,
    path => { /* use waypoints */ }
);
```

---

## Configuration

| Parameter | Default | Description |
|---|---|---|
| Grid Width / Height | 50 × 50 | Number of nodes |
| Node Size | 1 m | World-space size of each cell |
| Max Agents | 128 | Max simultaneous path requests per batch |
| Max Path Length | 512 | Max waypoints per path |
| Scan Interval | 0.25 s | Obstacle re-scan frequency |
| Agent Speed | 5 m/s | Movement speed |
| Stopping Distance | 0.15 m | Distance to consider waypoint reached |

---

## Editor Tools

- **Pathfinding Debugger Window** (`Window > Pathfinding Debugger`) — live grid stats, agent list, manual scan trigger
- **PathfindingGrid Inspector** — gizmo color controls, cost overlay, node index labels
- **Gizmos** — walkable/obstacle nodes, agent paths with arrowheads, destination markers
