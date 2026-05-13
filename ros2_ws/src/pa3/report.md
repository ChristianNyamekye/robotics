# PA3 Report — Path Planning and Global Navigation

**Christian Nyamekye** · COSC 81/281 · 2026-05-04

## Implementation

`path_planner.py` subscribes to `/map`, publishes `/map_smoothed` (5×5 Gaussian, `max(raw, smoothed)` to keep walls at 100), and builds a planning grid by morphological dilation of obstacles by 7 cells (35 cm clearance for the rosbot's 25 cm half-width — the C-space construction from Lec 10). BFS uses `collections.deque`; DFS uses a stack; A\* uses `heapq` keyed by `f(n) = g(n) + h(n)` with **Manhattan** heuristic, lazy closed-set, goal-test on dequeue. All three return `(path, expansions)` with `came_from` history. The path is subsampled at stride 6 (~30 cm waypoints), start cell dropped, each pose's yaw set toward the next waypoint, and published as `PoseArray` on `/pose_sequence`. Robot pose comes from `/odom` + cached `map → odom` static transform (avoids TF staleness). Motion is rotate-then-drive with P control and 3 s stuck-detection.

## Comparison

Start `(2, 2)`, goal `(8, 8)`, same planning grid.

| Algorithm | Path length (cells) | Nodes expanded | Planning time |
|---|---|---|---|
| BFS | 241 | 15,397 | 8.1 ms |
| DFS | **3,919** | 8,958 | 6.4 ms |
| A\* | 241 | **6,895** | 17.1 ms |

**BFS vs A\*.** A\* expanded **55 % fewer cells** than BFS while finding the same-length path. Manhattan filters the search directionally — BFS expands an isotropic ring around the start until it touches the goal; A\* only admits nodes whose `g + h` lower-bound beats every alternative. Wall-time is slightly higher for A\* (heap ops + arithmetic) but expansion count dominates on real maps.

**DFS pathology.** 3,919 cells — **16× longer** than BFS for the same goal. DFS commits to the top-of-stack neighbor first, snaking into dead ends before backtracking. The demo cuts at ~20 of 654 waypoints; the full traverse would take hours and wouldn't change the lesson.

## Heuristic admissibility

Manhattan is admissible for unweighted 4-connected grids: each step costs 1, so `h(n) = |Δcol| + |Δrow| ≤ true_cost`. Smoothing/dilation relabel blocked cells but don't change per-step cost between free cells — Manhattan remains admissible and A\* remains optimal. Under weighted A\* (below) step costs grow above 1; Manhattan is no longer a lower bound and optimality is sacrificed for safer paths.

## Extra credit

**8-connectivity** (`eight_connected:=true`) adds diagonal neighbors with `√2` step cost. A\* finds shorter paths in open space; Manhattan is no longer strictly admissible — Octile (`max(d) + (√2−1)·min(d)`) would be the correct heuristic.

**Weighted A\*** (`weighted_astar:=true`) multiplies step cost by `1 + gain · w/100` where `w` is the smoothed occupancy of the destination cell (`gain = 5`). Cells near walls become expensive, so the path hugs corridor centerlines instead of skimming the dilation boundary. Manhattan over-estimates near walls so A\* expands more nodes than unweighted.

## Credits

Lec 10 (occupancy grids, C-space / Minkowski) · Lec 12 (BFS / DFS / A\*, Gaussian smoothing) · `nav2_map_server` · PA1 polygon planner for tf2 + rotate-then-drive.
