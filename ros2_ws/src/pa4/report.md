# PA4 Report — Occupancy Grid Mapping

**Christian Nyamekye** · COSC 81/281 · 2026-05-11

## Description

`occupancy_mapper.py` is a single ROS 2 node that subscribes to `sensor_msgs/LaserScan` on `/base_scan` and `nav_msgs/Odometry` on `/odom`, builds a 2D occupancy grid in the odom frame, and publishes `nav_msgs/OccupancyGrid` on `/map` at 1 Hz with `TRANSIENT_LOCAL` durability. Pose is read directly from `/odom` rather than via tf2 lookup, matching the approach from PA3 and avoiding stale-buffer issues on long runs.

**Beam processing.** For each beam: skip if `r` is NaN/Inf or below `range_min`; if `r ≥ range_max` treat as max-range; compute the endpoint in odom coords as `(rx + r·cos(ryaw + θ), ry + r·sin(ryaw + θ))`; convert robot and endpoint to grid cells via `floor((x − origin)/resolution)`. Bresenham walks the integer ray from robot to endpoint; intermediates are marked free, and the endpoint is marked occupied (or free, if max-range).

**Update rule.** Three states per ROS spec — `−1` unknown, `0` free, `100` occupied. Default is recursive Bayesian log-odds: each cell stores `l` initialised to 0; free hits add `l_free = log(0.35/0.65)`; occupied hits add `l_occ = log(0.65/0.35)`. Values clamp to `[−5, +5]` so cells stay responsive to change. Output: `l > 0.85 → 100`, `l < −0.85 → 0`, intermediate `l` maps to the linearised probability `100 / (1 + e^−l)`; unobserved cells stay −1. A `use_log_odds:=false` parameter switches to a binary update with no temporal smoothing (base credit path).

**Origin and growth.** Grid initialises 400×400 cells (20 m × 20 m) at origin `(−10, −10)`. Each scan computes the endpoint bounding box; if it falls outside the current grid, the array reallocates with 200 cells of padding in the needed direction, the old data copies into the offset position, and the origin shifts. At most one reallocation per scan. Growth count is logged on shutdown.

**Frame id handling.** Stage prefixes the odom frame with the vehicle name (`rosbot1/odom`) regardless of `enforce_prefixes`. The mapper reads `frame_id` from the first `/odom` message it receives and publishes the map header in that frame so RViz can transform consistently.

## Evaluation

In the PA2 corridor (`2017-02-11-00-31-57`), a clean 90 s autonomous wall-following run grows the grid 2–3 times (final ~850×940) and produces a recognisable corridor outline with crisp wall edges. Switching `use_log_odds:=false` produces visibly noisier walls — a single spurious far-range return flips a binary cell to occupied, whereas log-odds requires ~10 confirming hits to push past threshold.

**Resolution/compute trade-off.** 0.05 m/cell balances detail and cost. 0.025 m quadruples bookkeeping for no visible gain at Stage's wall thickness; 0.10 m collapses the ~10 cm walls and blocks doorways. 0.05 m also matches PA3's `maze.yml` resolution.

**Max-range handling.** Stage emits `range_max` (20 m) when a beam hits nothing. The mapper clears cells along the ray but does **not** mark the endpoint occupied. Ignoring max-range entirely leaves donuts of unknown in open rooms; marking the endpoint paints phantom walls at the sensor edge.

**Line algorithm choice.** Bresenham over a DDA variant because it's integer-only (no per-cell float ops) and produces a connected 4-/8-neighbour line with no gaps.

**Noise / transient obstacles.** Log-odds handles both: clamp + symmetric increments mean an occupied cell can be re-cleared by enough free observations, so a person walking through doesn't burn a permanent obstacle into the map.

**Limitations.** Pose is dead-reckoning. When the robot collides and command velocity continues, Stage's simulated encoders integrate phantom motion, and the map smears — drift produces tilted walls and duplicate hallways. A long crash run grew the grid to 1453×1551 over 9 reallocations. Loop closure (lec16) would fix this; PA4's scope is mapping with known pose, so the drift is reported rather than corrected. The mapper also assumes the LIDAR is at the robot's base and treats cells as independent given pose — the lec16 cell-independence approximation.

## Extra credit — log-odds

Implemented as the default. `P_occ = 0.65, P_free = 0.35` is symmetric around 0.5 and mildly confident per measurement so single noisy returns don't flip a cell. Clamping `[−5, +5]` corresponds to probability `[0.0067, 0.993]`; cells stop responding to repeated identical evidence at the clamp but can still flip given contradicting evidence.

## Credits

Lec 14 state estimation framing · lec 16 occupancy grids and log-odds update · Bresenham's line algorithm · `nav_msgs/OccupancyGrid` ROS spec · PA2 `wall_follower.py` as the autonomous explorer.
